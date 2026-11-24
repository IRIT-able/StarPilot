"""Car-side pre-AP pedal behavior: feedback rules mirror the panda, the wire is silent while idle and follows the
sequence logged on the owner's interceptor, NAP's brake gate, stock-cruise cancel and the steering table clamp."""
import struct
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.pedal_contract import AGREE_OFFSET, AGREE_SLOPE, D, M1, PedalCalibration, PedalFeedback, checksum
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
                               doorOpen=False, steerFaultPermanent=False, steeringDisengage=False, vEgo=20., vEgoRaw=20.,
                               aEgo=0.)
    self.cs = SimpleNamespace(pedal_feedback=self.fb, cruiseEnabled=False, enableLongControl=False, out=self.out,
                              cruise_buttons=0, prev_cruise_buttons=0, preap_cc_cancel_needed=False, preap_accel_max=1.,
                              pccEvent=None, pedal_command_failed=False)
    self.long_active = False
    self.echo = True
    self.stuck_idle = False
    self.never_times_out = False   # interceptor model that stays in STATE 0 while idle (NAP's test fixture)
    self.accel, self.orientation = 0.5, [0., 0., 0.]
    self.fb_one = REST
    self.wire = []
    self.commands = []             # (frame, enabled, di) of every frame sent

  def step(self, n=1):
    sent = []
    for _ in range(n):
      if self.frame_no % 2 == 0:
        recent = self.last_tx is not None and self.now - self.last_tx <= 200_000_000
        state = 0 if (recent or self.never_times_out) and not self.stuck_idle else 5
        self.fb.observe(frame(self.counter, one=self.fb_one, state=state), 2, self.now)
        self.counter += 1
      cc = SimpleNamespace(enabled=True, longActive=self.long_active, orientationNED=list(self.orientation),
                           actuators=SimpleNamespace(accel=self.accel))
      out = self.pc.update(cc, self.cs, self.frame_no, self.now)
      for _, data, _ in out:
        self.last_tx = self.now
        if self.echo:
          self.fb.note_echo(self.now)   # the panda returns every frame it accepted
        sent.append('en' if data[4] & 0x80 else 'dis')
        self.commands.append((self.frame_no, bool(data[4] & 0x80), CAL.di(struct.unpack('>H', data[:2])[0] * M1 + D)))
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
  assert car.step(2) == ['dis'], 'a disabled frame first after every override: the panda needs it again'
  assert 'en' in car.step(6)
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


def engaged_car():
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.step(20)
  assert car.wire[-1] == 'en'
  return car


def test_interceptor_that_never_leaves_its_timeout_state_fails_the_takeover_and_latches():
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.stuck_idle = True
  sent, unhealthy = [], False
  for _ in range(60):
    sent += car.step(1)
    unhealthy = unhealthy or not car.fb.ready(car.now) or car.cs.pedal_command_failed
    if unhealthy:
      car.cs.cruiseEnabled = car.cs.enableLongControl = False   # what carstate does on lost feedback or the failure flag
  # STATE 5 with a command stream running for more than 100 ms is unhealthy feedback, exactly as in the panda.
  assert 'en' not in sent and unhealthy and 0 < sent.count('dis') <= 12
  assert car.step(100) == []


def test_state5_after_an_enabled_command_is_unhealthy_on_the_car_side_too():
  car = engaged_car()
  car.last_tx = None                                             # interceptor falls back to its timeout state while commanded
  car.step(2)
  assert not car.fb.healthy and not car.fb.ready(car.now)


def test_blocked_commands_are_noticed_through_the_missing_echo():
  car = engaged_car()
  car.echo = False
  sent = car.step(40)
  assert car.cs.pedal_command_failed
  assert sent.count('en') <= 8 and sent[-1] == 'dis' and car.step(50) == []


def test_command_error_latches_instead_of_retrying():
  car = engaged_car()
  car.pc.vdas.update = lambda *a, **k: (_ for _ in ()).throw(RuntimeError('boom'))
  assert car.step(2) == ['dis']
  assert car.cs.pedal_command_failed and car.step(50) == []


def test_non_finite_input_latches():
  car = engaged_car()
  car.out.aEgo = float('nan')
  assert car.step(2) == ['dis'] and car.cs.pedal_command_failed and car.step(20) == []


def test_enabled_command_never_exceeds_the_panda_cap_and_first_command_stays_near_the_measured_pedal():
  from opendbc.car.tesla.preap.pedal_contract import MAX_ENABLED_RAW, PedalCodec
  codec = PedalCodec(PedalCalibration(1., 30., 2))                # 90 DI would be raw 2812
  for di in (0., 50., 80., 90.):
    _, data, _ = codec.command(di, enabled=True)
    one, two = struct.unpack('>HH', data[:4])
    assert one <= MAX_ENABLED_RAW and 0 <= one - 2 * two <= 1
  car = Car()
  car.step(20)
  car.pc.zero_torque.get = lambda v: 30.                          # learned coast point far above the resting pedal
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  seen = []
  real = car.pc.codec.command
  car.pc.codec.command = lambda di=0., *, enabled=False: (seen.append(di) if enabled else None) or real(di, enabled=enabled)
  car.step(8)
  assert seen and seen[0] == pytest.approx(car.fb.value + 1.5), 'coast point far above: capped at measured + 1.5, unramped'


@pytest.mark.parametrize('coast,expected', [(0., 0.), (0.8, 0.8), (-3., 0.)])
def test_first_command_never_exceeds_nap_seed(coast, expected):
  # NAP: max(measured pedal, coast point). Below 5 mph or before learning, the coast point is 0: the measured pedal.
  car = Car()
  car.step(20)
  car.pc.zero_torque.get = lambda v: coast
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  seen = []
  real = car.pc.codec.command
  car.pc.codec.command = lambda di=0., *, enabled=False: (seen.append(di) if enabled else None) or real(di, enabled=enabled)
  car.step(8)
  assert seen and seen[0] == pytest.approx(car.fb.value + expected)


def test_non_finite_raw_speed_alone_latches():
  car = engaged_car()
  car.out.vEgoRaw = float('nan')
  assert car.step(2) == ['dis'] and car.cs.pedal_command_failed and car.step(20) == []


def test_any_brake_status_other_than_released_drops_pedal_cruise():
  fsm = PreAPEngagement(True, 400)
  pull(fsm, 1000, False)
  pull(fsm, 1300, False)
  assert fsm.enableLongControl
  # stock.py passes the published brakePressed: DI_brakePedal, or a brake status that is anything but "not applied".
  fsm.process_buttons(0, 0, 1400, 20., "MPH", True, True, True, True)
  assert not fsm.enableLongControl


def test_failure_stays_visible_for_a_second_and_needs_the_request_gone():
  car = engaged_car()
  car.echo = False
  car.step(40)
  assert car.cs.pedal_command_failed
  car.cs.cruiseEnabled = car.cs.enableLongControl = False       # carstate resets the engagement at once
  car.step(50)
  assert car.cs.pedal_command_failed, 'still a fault half a second later'
  car.step(60)
  assert not car.cs.pedal_command_failed and not car.pc.failed
  car.echo = True
  assert car.step(50) == [], 'and nothing resumes without a new pull'


# *** every gate in `wanted`, one at a time (safety review U4) ***

GATES = [('canValid', False), ('canTimeout', True), ('accFaulted', True), ('doorOpen', True), ('steerFaultPermanent', True),
         ('steeringDisengage', True), ('gasPressed', True), ('brakePressed', True)]


@pytest.mark.parametrize('gate,value', GATES)
def test_each_car_side_gate_alone_releases_and_silences_the_pedal(gate, value):
  car = engaged_car()
  setattr(car.out, gate, value)
  assert car.step(2) == ['dis'], 'one disabled frame to release'
  assert car.step(60) == [], 'then nothing while the gate holds, with pedal cruise still requested'
  setattr(car.out, gate, not value)
  after = car.step(10)
  assert after[:1] == ['dis'] and 'en' in after, 'a disabled frame again before any enabled one'


@pytest.mark.parametrize('gate,value', GATES)
def test_each_car_side_gate_alone_blocks_an_engage(gate, value):
  car = Car()
  car.step(20)
  setattr(car.out, gate, value)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  assert car.step(60) == []


# *** acquisition: the two rules that end a takeover, each on its own ***

def test_the_state5_stream_rule_alone_ends_a_takeover(monkeypatch):
  import opendbc.car.tesla.preap.pedal_control as pedal_control
  monkeypatch.setattr(pedal_control, 'ACQUIRE_MAX_FRAMES', 10_000)
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.stuck_idle = True
  sent = []
  for _ in range(40):
    sent += car.step(1)
    if not car.fb.ready(car.now):
      break
  assert not car.fb.ready(car.now), 'STATE 5 more than 100 ms into a command stream is unhealthy feedback'
  assert 'en' not in sent and 5 <= sent.count('dis') <= 7, sent


def test_the_acquire_limit_alone_ends_a_takeover(monkeypatch):
  import opendbc.car.tesla.preap.pedal_contract as pedal_contract
  monkeypatch.setattr(pedal_contract, 'STATE_SETTLE_NS', 10_000_000_000)   # STATE 5 stays "healthy" for 10 s
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.stuck_idle = True
  sent = car.step(40)
  assert car.fb.ready(car.now) and car.cs.pedal_command_failed and car.pc.failed
  assert 'en' not in sent and 11 <= sent.count('dis') <= 13, sent
  assert car.step(40) == []


def test_echo_watchdog_fires_after_100_ms_of_unanswered_enabled_frames():
  car = engaged_car()
  car.echo = False
  sent = car.step(40)
  assert car.cs.pedal_command_failed
  assert 4 <= sent.count('en') <= 6 and sent[sent.count('en')] == 'dis', sent


# *** grace cap and handoff slew on the real controller (review U5), NAP grace start (NAP :561) ***

def test_grace_cap_and_handoff_slew_on_the_real_controller():
  from opendbc.car.tesla.preap.pedal_control import ENGAGE_GRACE_FRAMES, ENGAGE_GRACE_PEDAL_RAMP_RATE_UP
  from opendbc.car.tesla.preap.pedal_dynamics import PEDAL_RAMP_RATE_UP
  car = Car()
  car.step(20)
  car.cs.preap_accel_max = 0.8
  car.accel = 2.0                                               # request far above the envelope at the engage speed
  calls = []
  real = car.pc.vdas.update

  def spy(demand, v_ego, previous, **kw):
    calls.append((car.frame_no, demand, kw['accel_effort_limits'], kw['pedal_ramp_rate_up'], kw['freeze_integrator']))
    return real(demand, v_ego, previous, **kw)
  car.pc.vdas.update = spy
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.step(400)
  first_enabled = next(f for f, enabled, _ in car.commands if enabled)
  assert car.pc.engage_frame == first_enabled == calls[0][0], 'the grace starts on the first enabled frame'
  ramps_after = []
  for f, demand, limits, ramp, freeze in calls:
    elapsed = f - first_enabled
    if elapsed < ENGAGE_GRACE_FRAMES:
      cap = elapsed / ENGAGE_GRACE_FRAMES * 0.8
      assert 0. <= demand <= cap + 1e-9 and limits[0] == 0. and limits[1] == pytest.approx(cap), (elapsed, demand)
      assert ramp == ENGAGE_GRACE_PEDAL_RAMP_RATE_UP and freeze
    else:
      assert demand == 2.0 and limits is None and not freeze
      ramps_after.append(ramp)
  assert ramps_after[0] == ENGAGE_GRACE_PEDAL_RAMP_RATE_UP, 'the gentle take-up holds after the grace while it limits'
  assert ramps_after[-1] == PEDAL_RAMP_RATE_UP and not car.pc.handoff_slew_active, 'and ends once the ramp stops limiting'


# *** commanded ceiling from the raw speed the panda uses, with margin (review N4) ***

@pytest.mark.parametrize('v_raw', [3., 10., 25., 38.])
def test_command_ceiling_follows_the_raw_speed_less_the_margin(v_raw):
  from opendbc.car.tesla.preap.pedal_contract import pedal_max
  from opendbc.car.tesla.preap.pedal_control import SPEED_CAP_MARGIN
  car = engaged_car()
  car.out.vEgo, car.out.vEgoRaw = v_raw + 3., v_raw          # filtered speed lagging above the raw sample
  car.pc.vdas.update = lambda *a, **k: 500.                     # demand far above any ceiling
  car.step(40)
  top = max(di for _, enabled, di in car.commands[-10:] if enabled)
  ceiling = pedal_max(v_raw - SPEED_CAP_MARGIN)
  assert ceiling - 0.1 <= top <= ceiling + 1e-6, (top, ceiling)
  assert top < pedal_max(v_raw) - 0.3, 'below the table at the raw speed itself, let alone at the filtered one'


# *** failure latch (NAP :287) ***

def test_failed_takeover_cannot_late_acquire_and_rearms_only_after_the_request_drops(monkeypatch):
  import opendbc.car.tesla.preap.pedal_contract as pedal_contract
  monkeypatch.setattr(pedal_contract, 'STATE_SETTLE_NS', 10_000_000_000)
  car = Car()
  car.step(20)
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  car.stuck_idle = True
  car.step(40)
  assert car.pc.failed
  car.stuck_idle = False                                        # the interceptor would answer now
  assert car.step(300) == [] and car.cs.pedal_command_failed, 'no late acquisition while the failed request stands'
  car.cs.cruiseEnabled = car.cs.enableLongControl = False
  car.step(2)
  assert not car.pc.failed, 'more than a second since the failure and no request: cleared'
  car.cs.cruiseEnabled = car.cs.enableLongControl = True       # a new request (a new pull in carstate)
  sent = car.step(10)
  assert sent[:1] == ['dis'] and 'en' in sent


# *** NAP: no launch after a gas override is released (NAP test_pedal_authority.py:575 and :610) ***

def _hold_gas_override(car):
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.out.gasPressed = True
  car.out.aEgo = 1.618
  car.long_active = False
  car.accel = 0.34
  car.orientation = [0., 0.05, 0.]
  car.step(460)
  assert car.commands == [], 'a pedal cruise request during a gas override is silent (NAP :552)'


def _release_gas(car, accel):
  car.out.gasPressed = False
  car.out.aEgo = 0.1
  car.long_active = True
  car.accel = accel


@pytest.mark.parametrize('engage_a_max', (0.8, 0.9, 1.0))
def test_gas_override_release_has_no_launch_for_1p5_seconds(engage_a_max):
  car = Car()
  car.never_times_out = True                                    # NAP's case: the interceptor is still in STATE 0
  car.pc.zero_torque.get = lambda v: 0.                         # NAP's fixture: coast point PEDAL_DI_ZERO
  car.cs.preap_accel_max = engage_a_max
  _hold_gas_override(car)
  _release_gas(car, 0.67)
  release = car.frame_no
  limited = {}
  for _ in range(156):
    car.step(1)
    limited[car.frame_no - 1 - release] = car.pc.vdas.jerk_limiter.a_limited
  assert car.commands[0][1] is False, 'a disabled frame before the first enabled one'
  enabled = [c for c in car.commands if c[1]]
  assert enabled and all(c[1] for c in car.commands[1:])
  assert enabled[0][2] < car.fb.value + 0.1, 'NAP :603: no coast point above the pedal, so the first command is the pedal'
  steps = [b[2] - a[2] for a, b in zip(enabled, enabled[1:])]
  assert max(steps) < 1.0, steps
  assert limited[48] < 0.4
  assert limited[154] == pytest.approx(0.67, abs=1e-3)
  assert not car.pc.handoff_slew_active


def test_gas_override_release_after_the_interceptor_timed_out_has_no_launch():
  car = Car()                                                   # the interceptor returns to STATE 5 200 ms after the last command
  _hold_gas_override(car)
  _release_gas(car, 0.34)
  assert car.step(2) == ['dis']
  assert car.pc.engage_frame < 0, 'nothing engaged while the interceptor is still in its timeout state'
  sent = car.step(2)
  first = next(c for c in car.commands if c[1])
  assert sent == ['en'] and car.pc.engage_frame == first[0]
  from opendbc.car.tesla.preap.pedal_dynamics import PEDAL_RAMP_RATE_UP
  assert first[2] < car.fb.value + PEDAL_RAMP_RATE_UP - 1.0
  assert car.pc.vdas.jerk_limiter.a_limited < 0.2
  assert car.pc.vdas.grade_estimator.pitch_lp.x > 0.02, 'the grade estimate stayed warm through the override'


# *** wire counter (NAP :143, :167) ***

def test_command_counter_wraps_from_fifteen_to_zero():
  from opendbc.car.tesla.preap.pedal_contract import PedalCodec
  codec = PedalCodec(CAL)
  codec.counter = 15
  assert [codec.command(0., enabled=True)[1][4] & 15 for _ in range(3)] == [15, 0, 1]


@pytest.mark.parametrize('initial', range(16))
def test_any_initial_interceptor_counter_is_acquired_without_an_early_enabled_frame(initial):
  # The interceptor takes a command only when its counter follows the last one it saw (rejected ones are remembered),
  # and leaves its timeout state only on an accepted disabled frame (NAP's firmware model).
  car = Car()
  car.step(20)
  firmware = SimpleNamespace(last=initial, state=5)

  def interceptor_state():
    return firmware.state
  real_observe = car.fb.observe
  car.fb.observe = lambda payload, bus, ns: real_observe(frame(payload[4] & 15, state=interceptor_state()), bus, ns)
  real_update = car.pc.update

  def update(cc, cs, frame_no, now):
    out = real_update(cc, cs, frame_no, now)
    for _, data, _ in out:
      counter, enabled = data[4] & 15, bool(data[4] & 0x80)
      accepted = counter == (firmware.last + 1) & 15
      firmware.last = counter
      assert not enabled or firmware.state == 0, 'enabled frame before the interceptor left its timeout'
      if accepted and not enabled:
        firmware.state = 0
    return out
  car.pc.update = update
  car.cs.cruiseEnabled = car.cs.enableLongControl = True
  car.long_active = True
  sent = car.step(12)
  assert 'en' in sent and not car.cs.pedal_command_failed
  assert sent.index('en') <= 3
