"""Car-side pre-AP pedal behavior: feedback rules mirror the panda, the wire is silent while idle and follows the
sequence logged on the owner's interceptor, NAP's brake gate, stock-cruise cancel and the steering table clamp."""
import struct
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.pedal_contract import AGREE_OFFSET, AGREE_SLOPE, M1, PedalCalibration, PedalFeedback, checksum
from opendbc.car.tesla.preap.pedal_control import PreAPPedalControl
from opendbc.car.tesla.preap.stock import apply_rate_table

CAL = PedalCalibration(1., 0., 2)
REST = 450


def two_for(one, residual_units=0.):
  return int(round((one - (AGREE_SLOPE * one + AGREE_OFFSET) - residual_units / M1) / 2))


def frame(counter, *, one=REST, two=None, state=0, bad_checksum=False):
  prefix = struct.pack('>HHB', one, two_for(one) if two is None else two, (state << 4) | (counter & 15))
  return prefix + bytes([(checksum(0x552, prefix) + int(bad_checksum)) & 255])


def healthy_feedback(state=0):
  fb = PedalFeedback(CAL)
  for n in range(3):
    fb.observe(frame(n, state=state), 2, 1_000_000 + n * 20_000_000)
  assert fb.healthy
  return fb, 3, 1_000_000 + 3 * 20_000_000


@pytest.mark.parametrize('state', [0, 5])
def test_idle_state_is_healthy_but_only_state_0_is_commandable(state):
  fb, _, now = healthy_feedback(state)
  assert fb.ready(now) and fb.commandable(now) == (state == 0)


@pytest.mark.parametrize('kw', [{'state': 1}, {'state': 3}, {'state': 6}, {'bad_checksum': True}, {'one': 0, 'two': two_for(REST)}, {'two': 0},
                                {'one': 4095, 'two': two_for(REST)}, {'two': 4095}, {'one': 65535, 'two': two_for(REST)}, {'two': 65535},
                                {'two': two_for(REST, 4.2)}, {'two': two_for(REST, -4.2)},
                                {'one': 0, 'two': 0}, {'one': 299, 'two': 157}, {'one': 320, 'two': 149}])
def test_feedback_faults_clear_health(kw):
  fb, n, now = healthy_feedback()
  assert not fb.observe(frame(n, **kw), 2, now) and not fb.ready(now) and not fb.commandable(now)


@pytest.mark.parametrize('residual', [-3.8, -2.98, 0., 2.98, 3.8])
def test_logged_channel_scatter_is_accepted(residual):
  fb, n, now = healthy_feedback()
  assert fb.observe(frame(n, two=two_for(REST, residual)), 2, now)


@pytest.mark.parametrize('advance,ok', [(0, False), (1, True), (2, True), (3, False), (15, False)])
def test_counter_advance_one_or_two(advance, ok):
  fb, n, now = healthy_feedback()
  assert fb.observe(frame(n - 1 + advance), 2, now) == ok


def test_gas_is_channel_one_only():
  fb, n, now = healthy_feedback()
  fb.observe(frame(n, two=two_for(REST, -3.5)), 2, now)       # channel 2 reads 3.5 units high: not a press
  assert not fb.gas_pressed()
  pressed = REST + int(3.0 / M1)
  fb.observe(frame(n + 1, one=pressed), 2, now + 20_000_000)
  assert fb.gas_pressed()


class Car:
  """Minimal CS/CC pair plus a model of the interceptor's STATE: 0 from the first command, 5 again 200 ms after the last."""
  def __init__(self):
    self.pc = PreAPPedalControl(CAL)
    self.fb = PedalFeedback(CAL)
    self.counter, self.frame_no, self.now, self.last_tx = 0, 0, 1_000_000_000, None
    self.out = SimpleNamespace(canValid=True, canTimeout=False, gasPressed=False, brakePressed=False, accFaulted=False,
                               doorOpen=False, steerFaultPermanent=False, steeringDisengage=False, vEgo=20., aEgo=0.)
    self.cs = SimpleNamespace(pedal_feedback=self.fb, cruiseEnabled=False, enableLongControl=False, out=self.out,
                              cruise_buttons=0, prev_cruise_buttons=0, preap_cc_cancel_needed=False, preap_accel_max=1.,
                              pccEvent=None)
    self.long_active = False
    self.wire = []

  def step(self, n=1):
    sent = []
    for _ in range(n):
      if self.frame_no % 2 == 0:
        state = 0 if self.last_tx is not None and self.now - self.last_tx <= 200_000_000 else 5
        self.fb.observe(frame(self.counter, state=state), 2, self.now)
        self.counter += 1
      cc = SimpleNamespace(enabled=True, longActive=self.long_active, orientationNED=[0., 0., 0.],
                           actuators=SimpleNamespace(accel=0.5))
      out = self.pc.update(cc, self.cs, self.frame_no, self.now)
      for _, data, _ in out:
        self.last_tx = self.now
        sent.append('en' if data[4] & 0x80 else 'dis')
        if not data[4] & 0x80:
          assert data[:4] == bytes(4)
      self.frame_no += 1
      self.now += 10_000_000
    self.wire += sent
    return sent


def test_wire_is_silent_while_idle_and_follows_the_logged_sequence():
  car = Car()
  assert car.step(100) == [], 'nothing on 0x551 while not engaged'
  car.cs.cruiseEnabled = car.cs.enableLongControl = True       # double pull
  first = car.step(4)
  assert first == ['dis', 'dis'], 'disabled frames first: the interceptor leaves its timeout, the panda can grant'
  car.long_active = True                                        # panda granted, controls active
  assert set(car.step(100)) == {'en'}
  car.cs.enableLongControl = False                              # brake or cancel
  assert car.step(2) == ['dis'], 'one disabled frame to release'
  assert car.step(100) == []


def test_no_enabled_command_while_the_interceptor_reports_its_timeout_state():
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  assert car.step(1) == ['dis'], 'feedback still STATE 5 on the first frame'
  assert 'en' in car.step(6)


def test_driver_gas_or_brake_silences_the_pedal_and_stock_cruise_is_cancelled_on_every_stalk_press():
  car = Car()
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.step(20)
  car.out.gasPressed = True
  assert car.step(2) == ['dis'] and car.step(50) == []
  car.out.gasPressed = False
  assert car.step(2) == ['dis'] and 'en' in car.step(6)
  car.cs.preap_cc_cancel_needed = False
  car.cs.prev_cruise_buttons, car.cs.cruise_buttons = 0, 2      # any non-idle stalk press
  car.step(2)
  assert car.cs.preap_cc_cancel_needed
  car.cs.preap_cc_cancel_needed = False
  car.cs.prev_cruise_buttons, car.cs.cruise_buttons = 2, 0      # release is not a press
  car.step(2)
  assert not car.cs.preap_cc_cancel_needed


def pull(fsm, t, brake):
  fsm.process_buttons(2, 0, t, 20., "MPH", True, True, True, brake)
  fsm.process_buttons(0, 2, t + 50, 20., "MPH", True, True, True, brake)


def test_double_pull_window_brake_gate_and_brake_level():
  fsm = PreAPEngagement(True, 400)
  pull(fsm, 1000, False)
  pull(fsm, 1450, False)
  assert fsm.cruiseEnabled and not fsm.enableLongControl, '450 ms apart is two single pulls'
  pull(fsm, 3000, True)
  pull(fsm, 3300, True)
  assert fsm.cruiseEnabled and not fsm.enableLongControl and fsm.pedal_speed_kph == 0., 'brake held: lateral only'
  for t in range(3400, 5000, 100):
    fsm.process_buttons(0, 0, t, 20., "MPH", True, True, True, False)
  assert not fsm.enableLongControl, 'releasing the brake does not start pedal cruise'
  pull(fsm, 6000, False)
  pull(fsm, 6300, False)
  assert fsm.enableLongControl and fsm.pedal_speed_kph > 0.
  fsm.process_buttons(0, 0, 6400, 20., "MPH", True, True, True, True)
  assert fsm.cruiseEnabled and not fsm.enableLongControl and fsm.enableJustCC, 'brake drops pedal cruise, keeps lateral'


@pytest.mark.parametrize('speed,up,down', [(0., 8., 9.), (2., 8., 9.), (4.5, 6., 7.), (7., 4., 5.), (17., 2.5, 4.5), (40., 2.5, 4.5)])
def test_steering_table_clamp(speed, up, down):
  assert apply_rate_table(100., 10., speed) == pytest.approx(10. + up)
  assert apply_rate_table(-100., 10., speed) == pytest.approx(10. - down)
  assert apply_rate_table(-100., -10., speed) == pytest.approx(-10. - up)
  assert apply_rate_table(100., -10., speed) == pytest.approx(-10. + down)
  assert apply_rate_table(11., 10., speed) == 11.
  assert apply_rate_table(365., 359., 0.) == 360. and apply_rate_table(-365., -359., 0.) == -360.


def test_button_event_types_unchanged():
  fsm = PreAPEngagement(True, 400)
  events = fsm.process_buttons(2, 0, 1000, 20., "MPH", True, True, True, False)
  assert [e.type for e in events] == [structs.CarState.ButtonEvent.Type.setCruise]
