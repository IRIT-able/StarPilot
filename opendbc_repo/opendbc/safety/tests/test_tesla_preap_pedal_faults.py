"""Mode 39 pedal interceptor fault detection on the registered hooks.

Every relaxed check (channel agreement around the measured relationship, driver demand from channel 1, counter advance
1-2, interceptor STATE 5 while idle) is paired here with the faults it must still catch: an open, shorted or stuck
sensor on either channel, a stalled or repeated counter, a bad checksum and a wrong STATE each withdraw control and need
a fresh physical pull. Also covers the tightenings taken from the references: DI_brakePedal as a second brake source,
hands-on level 2, BogPilot's steering rate table, 360 deg cap and absolute pedal cap.
"""
import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.preap.can import TeslaCANPreAP
from opendbc.car.tesla.preap.pedal_contract import D, M1, PedalCalibration, PedalCodec
from opendbc.car.tesla.tests.preap_stream import PreAPStream
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.tesla.tests.pedal_fixture import configure_native, feedback_frame

SLOPE, OFFSET = 0.00844, -14.17      # measured (one - 2*two) vs one, raw counts
TOLERANCE_UNITS = 4.0
REST_RAW = 450                        # 0 DI with factor 1, zero 0


def expected_two(one, residual_units=0.):
  return int(round((one - (SLOPE * one + OFFSET) - residual_units / M1) / 2))


class Rig:
  def __init__(self, *, bus=2, word=1, factor=1., zero=0., speed=80., silent=True):
    self.safety = libsafety_py.libsafety
    self.cal = PedalCalibration(factor, zero, bus)
    configure_native(self.safety, self.cal, word, b'pedal-fault-tests')
    self.stream = PreAPStream()
    self.codec = PedalCodec(self.cal)
    self.can = TeslaCANPreAP(CANPacker("tesla_can"))
    self.tick, self.now, self.fb, self.speed = 0, 1_000_000, 0, speed
    self.state = 5 if silent else 0
    self.silent = silent
    self.armed = False
    self.stream_kw = {}
    self.steer_counter = 0

  def rx(self, frame):
    address, data, bus = frame
    return bool(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))

  def tx(self, frame):
    address, data, bus = frame
    return bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)))

  def cmd(self, di=0., enabled=False):
    # A rejected frame must not consume the codec counter.
    probe = PedalCodec(self.cal)
    probe.counter = self.codec.counter
    accepted = self.tx(probe.command(di, enabled=enabled))
    if accepted:
      self.codec.counter = probe.counter
    return accepted

  def feedback(self, advance=1, **kw):
    self.fb = (self.fb + advance) & 15
    kw.setdefault('state', self.state)
    return self.rx(feedback_frame(self.cal, self.fb, **kw))

  def advance(self, microseconds):
    self.now += microseconds
    self.safety.set_timer(self.now)

  def step(self, n=1, *, lever=0, feedback=True, command=None, **fb):
    for _ in range(n):
      self.tick += 1
      self.advance(10000)
      for frame in self.stream.frames(self.tick, lever=lever, cruise=1, speed=self.speed, **self.stream_kw):
        self.rx(frame)
      if feedback and self.tick % 2 == 0:
        self.feedback(**fb)
      self.safety.safety_tick()
      self.safety.aol_set_host_request(3 if self.armed else 0)
      if command is not None and self.tick % 2 == 0:
        assert self.cmd(*command[:1], enabled=command[1]) == command[2]

  def arm(self):
    """Double pull. With a silent idle the result is lateral only until the interceptor is acquired."""
    self.step(64, command=None if self.silent else (0., False, True))
    self.safety.set_aol_test_heartbeat(True)
    self.armed = True
    for tick in range(65, 91):
      self.step(lever=2 if tick in (65, 75) else 0, command=None if self.silent else (0., False, True))
    assert self.safety.safety_config_valid() and self.safety.get_controls_allowed()
    assert self.mask() == (1 if self.silent else 3)
    return self

  def acquire(self):
    """The sequence logged on the car: disabled frames, interceptor leaves its timeout, then enabled commands."""
    assert self.cmd(enabled=False)
    self.state = 0
    self.step(2, command=(0., False, True))
    assert self.mask() == 3
    return self

  def engaged(self):
    self.arm()
    if self.silent:
      self.acquire()
    assert self.cmd(0., enabled=True)
    return self

  def mask(self):
    return self.safety.aol_get_permission_mask()

  def assert_withdrawn(self, *, feedback_frames=6):
    assert not self.safety.get_controls_allowed()
    assert self.mask() == 0
    assert not self.cmd(0., enabled=True)
    # Healthy feedback returning does not give anything back: a fresh physical pull is needed.
    self.state = 0 if not self.silent else self.state
    for _ in range(feedback_frames):
      self.step(2)
    assert not self.safety.get_controls_allowed()
    assert self.mask() == 0
    assert not self.cmd(0., enabled=True)

  def steer(self, can_angle, enabled=True):
    self.steer_counter = (self.steer_counter + 1) % 16
    return self.tx(self.can.create_steering_control(self.steer_counter, -can_angle / 10., enabled))


# *** silent idle ***

def test_silent_idle_keeps_lateral_and_engages_with_the_logged_sequence():
  rig = Rig().arm()
  assert rig.mask() == 1, 'STATE 5 with nothing commanded is the healthy idle state: lateral stays'
  assert not rig.cmd(0., enabled=True), 'no enabled command before a disabled frame'
  assert rig.cmd(enabled=False)
  rig.step(2, state=5)                       # interceptor not out of its timeout yet
  assert rig.safety.get_controls_allowed()
  assert rig.cmd(enabled=False)
  assert not rig.cmd(0., enabled=True), 'no enabled command while the interceptor still reports the timeout state'
  rig.step(2, state=0)
  assert rig.cmd(0., enabled=True)
  assert rig.mask() == 3


def test_state5_while_a_command_stream_is_running_is_a_fault():
  rig = Rig().arm()
  assert rig.cmd(enabled=False)
  for _ in range(4):                         # 80 ms into the stream: still inside the settle time
    rig.step(2, command=(0., False, True))
  assert rig.safety.get_controls_allowed() and rig.mask() & 1
  rig.step(4, command=(0., False, True))     # 120 ms+: the interceptor should have answered
  rig.assert_withdrawn()


def test_state5_after_an_enabled_command_is_a_fault():
  rig = Rig().engaged()
  rig.step(2, state=5)
  rig.assert_withdrawn()


def test_state5_returning_after_the_stream_stops_is_healthy():
  rig = Rig().arm().acquire()
  rig.step(22)                               # 220 ms without a command, interceptor still in STATE 0
  rig.state = 5
  rig.step(20)
  assert rig.safety.get_controls_allowed() and rig.mask() == 1


@pytest.mark.parametrize('state', [1, 2, 3, 4, 6, 7, 15])
@pytest.mark.parametrize('silent', [True, False])
def test_any_other_interceptor_state_is_a_fault(state, silent):
  rig = Rig(silent=silent).arm()
  rig.step(2, state=state)
  rig.assert_withdrawn()


# *** channel agreement (W1) with channel 1 as driver demand (W2) ***

@pytest.mark.parametrize('channel', [1, 2])
@pytest.mark.parametrize('raw', [0, 4095, 65534, 65535])
@pytest.mark.parametrize('engaged', [False, True])
def test_open_or_shorted_channel_on_either_side_disengages(channel, raw, engaged):
  rig = Rig().engaged() if engaged else Rig().arm()
  rig.step(2, **({'raw_one': raw} if channel == 1 else {'raw_two': raw}))
  rig.assert_withdrawn()


@pytest.mark.parametrize('stuck', [1, 2])
@pytest.mark.parametrize('engaged', [False, True])
def test_stuck_channel_on_either_side_disengages_within_the_tolerance(stuck, engaged):
  rig = Rig().engaged() if engaged else Rig().arm()
  rest_one, rest_two = REST_RAW, expected_two(REST_RAW)
  moved = 0.
  while rig.safety.get_controls_allowed() and moved < 10.:
    moved += 0.25                            # the healthy channel follows the pedal, the stuck one stays at rest
    one = rest_one if stuck == 1 else rest_one + int(moved / M1)
    two = rest_two if stuck == 2 else expected_two(rest_one + int(moved / M1))
    rig.step(2, raw_one=one, raw_two=two)
  assert TOLERANCE_UNITS - 0.3 <= moved <= TOLERANCE_UNITS + 0.3, moved
  rig.assert_withdrawn()


@pytest.mark.parametrize('pair', [(0, 0), (150, 82), (199, 107), (REST_RAW // 2, 116), (299, 157), (320, 149)])
@pytest.mark.parametrize('engaged', [False, True])
def test_both_channels_low_together_is_a_fault(pair, engaged):
  # Lost or sagging sensor supply: the two readings still agree with each other.
  rig = Rig().engaged() if engaged else Rig().arm()
  rig.step(2, raw_one=pair[0], raw_two=pair[1])
  rig.assert_withdrawn()


@pytest.mark.parametrize('stuck', [1, 2])
@pytest.mark.parametrize('edge', [-3.5, 3.5])
def test_stuck_channel_from_anywhere_in_the_band_is_caught_within_twice_the_tolerance(stuck, edge):
  # Worst case: the pair sits at one edge of the agreement band when a channel sticks.
  rig = Rig().arm()
  rest_one, rest_two = REST_RAW, expected_two(REST_RAW, edge)
  moved = 0.
  while rig.safety.get_controls_allowed() and moved < 12.:
    moved += 0.25
    one = rest_one if stuck == 1 else rest_one + int(moved / M1)
    two = rest_two if stuck == 2 else expected_two(rest_one + int(moved / M1), edge)
    rig.step(2, raw_one=one, raw_two=two)
  assert moved <= 2 * TOLERANCE_UNITS + 0.3, moved
  rig.assert_withdrawn()


@pytest.mark.parametrize('di,pressed', [(1.8, False), (2.2, True)])
def test_gas_threshold_is_two_di_on_channel_one(di, pressed):
  rig = Rig().engaged()
  one = int((di - D) / M1)
  rig.step(2, raw_one=one, raw_two=expected_two(one))
  assert rig.safety.get_controls_allowed()
  assert (rig.mask() == 1) == pressed and rig.cmd(0., enabled=True) == (not pressed)


def test_state5_settle_time_boundary():
  for frames, healthy in ((4, True), (5, False)):   # last STATE 5 frame 90 ms / 110 ms into a running stream
    rig = Rig().arm()
    rig.step(1)                                      # commands on odd ticks, feedback on even ones
    for _ in range(frames):
      assert rig.cmd(enabled=False)
      rig.step(2)
    assert rig.cmd(enabled=False)
    rig.step(1)
    assert rig.safety.get_controls_allowed() == healthy, frames


@pytest.mark.parametrize('one', [REST_RAW, 1200, 2400])
@pytest.mark.parametrize('sign', [-1, 1])
def test_agreement_boundary_around_the_measured_relationship(one, sign):
  rig = Rig().arm()
  for residual in (0., 2.98, 3.8):           # 2.98 is the worst frame logged
    rig.step(2, raw_one=one, raw_two=expected_two(one, sign * residual))
    assert rig.safety.get_controls_allowed(), residual
  rig.step(2, raw_one=one, raw_two=expected_two(one, sign * 4.2))
  rig.assert_withdrawn()


def test_channel2_offset_is_not_driver_demand_but_channel1_is():
  rig = Rig().engaged()
  # Channel 2 reading 3 units above channel 1 (inside the agreement band): not a gas press.
  for _ in range(5):
    rig.step(2, raw_one=REST_RAW, raw_two=expected_two(REST_RAW, -3.0))
    assert rig.cmd(0., enabled=True)
  assert rig.mask() == 3
  # Channel 1 above 2 DI: the driver is on the pedal. Longitudinal is withdrawn, lateral and the engagement stay.
  pressed = int((3.0 - D) / M1)
  rig.step(2, raw_one=pressed, raw_two=expected_two(pressed))
  assert rig.safety.get_controls_allowed()
  assert rig.mask() == 1
  assert not rig.cmd(0., enabled=True)
  assert rig.cmd(enabled=False)


# *** counter (W3) and checksum ***

def test_single_dropped_feedback_frame_is_tolerated():
  rig = Rig().engaged()
  for _ in range(3):
    rig.step(2, feedback=False)
    rig.step(2, advance=2)
    assert rig.safety.get_controls_allowed()
    assert rig.cmd(0., enabled=True)
    rig.step(2)
  assert rig.mask() == 3


@pytest.mark.parametrize('advance', [0, 3, 4, 8, 15])
@pytest.mark.parametrize('engaged', [False, True])
def test_repeated_or_jumping_counter_disengages(advance, engaged):
  rig = Rig().engaged() if engaged else Rig().arm()
  rig.step(2, advance=advance)
  rig.assert_withdrawn()


def test_stalled_interceptor_withdraws_longitudinal_and_lateral():
  # StarPilot's stream freshness rule for a 50 Hz message is 200 ms; it is unchanged.
  rig = Rig().engaged()
  rig.step(19, feedback=False)
  assert rig.cmd(0., enabled=True)
  rig.step(3, feedback=False)
  assert not rig.cmd(0., enabled=True)
  assert rig.mask() == 0
  assert rig.cmd(enabled=False)
  rig.step(20)
  assert rig.mask() == 0 and not rig.cmd(0., enabled=True), 'the stream returning does not re-arm'


def test_frozen_frame_replayed_forever_disengages():
  rig = Rig().engaged()
  rig.step(2, advance=0)
  assert not rig.safety.get_controls_allowed()
  for _ in range(10):
    rig.step(2, advance=0)
    assert rig.mask() == 0 and not rig.cmd(0., enabled=True)


@pytest.mark.parametrize('engaged', [False, True])
def test_bad_feedback_checksum_disengages(engaged):
  rig = Rig().engaged() if engaged else Rig().arm()
  rig.step(2, bad_checksum=True)
  rig.assert_withdrawn()


# *** caps and brake ***

def raw_command(rig, one, enabled=True):
  """An enabled 0x551 frame built by hand: the car's own encoder never goes above raw 2450."""
  import struct
  prefix = struct.pack('>HHB', one, one // 2, (int(enabled) << 7) | rig.codec.counter)
  accepted = rig.tx((0x551, prefix + bytes([(0x51 + 0x05 + sum(prefix)) & 255]), rig.cal.bus))
  if accepted:
    rig.codec.counter = (rig.codec.counter + 1) & 15
  return accepted


def test_enabled_command_above_raw_2500_is_rejected_whatever_the_calibration_allows():
  rig = Rig(zero=60.).engaged()              # 0 DI = raw 1631, so the calibrated bound at 80 kph (75 DI) is above raw 2500
  one = int((rig.cal.physical(0.) - D) / M1)
  while one + 70 <= 2500:
    one += 70                                # about 3.5 DI per frame, inside the rate limit
    assert raw_command(rig, one), one
  assert raw_command(rig, 2500)
  assert not raw_command(rig, 2501)
  assert not raw_command(rig, 2560)
  assert raw_command(rig, 2500), 'a rejected frame leaves the accepted state untouched'


def test_the_car_encoder_stays_under_the_panda_cap():
  rig = Rig(zero=60.).engaged()
  di = 0.
  while di < 72.:
    di += 4.
    assert rig.cmd(di, enabled=True), di     # clamped at raw 2450 by the encoder, so never rejected for the cap


@pytest.mark.parametrize('source', ['message', 'di', 'both'])
def test_either_brake_source_withdraws_and_one_releasing_does_not_clear_the_other(source):
  rig = Rig().engaged()
  rig.stream_kw = {'brake': 2 if source in ('message', 'both') else 1, 'di_brake': int(source in ('di', 'both'))}
  rig.step(2)
  assert rig.safety.get_brake_pressed_prev()
  assert not rig.safety.get_controls_allowed()
  assert not rig.cmd(0., enabled=True)
  if source == 'both':
    rig.stream_kw = {'brake': 1, 'di_brake': 1}
    rig.step(2)
    assert rig.safety.get_brake_pressed_prev()
    rig.stream_kw = {'brake': 2, 'di_brake': 0}
    rig.step(2)
    assert rig.safety.get_brake_pressed_prev()
  rig.stream_kw = {}
  rig.step(4)
  assert not rig.safety.get_brake_pressed_prev()
  assert not rig.cmd(0., enabled=True), 'releasing the brake does not bring the pedal back without a new pull'


@pytest.mark.parametrize('hands,dropped', [(0, False), (1, False), (2, True), (3, True)])
def test_hands_on_level_2_disengages(hands, dropped):
  rig = Rig().engaged()
  rig.stream_kw = {'hands': hands}
  rig.step(4)
  assert rig.safety.get_controls_allowed() == (not dropped)
  assert (rig.mask() == 0) == dropped


# *** steering: BogPilot's table and cap on top of the vehicle-model limits ***

def test_low_speed_steering_rate_follows_the_table():
  rig = Rig(speed=5.).arm()                  # 1.39 m/s: table 8 deg up / 9 deg down per frame, the model allows far more
  assert rig.steer(0, enabled=False)
  assert rig.steer(0)
  assert rig.steer(80)
  assert not rig.steer(80 + 90), '9 deg in one frame away from center'
  assert rig.steer(0, enabled=False)
  assert rig.steer(0)
  assert rig.steer(80)
  assert rig.steer(160)
  assert rig.steer(160 - 90), '9 deg toward center is inside the down table'
  assert not rig.steer(70 - 100)


def test_absolute_angle_cap_is_360_deg():
  rig = Rig(speed=5.)
  rig.stream_kw = {'angle': 359.5}
  rig.arm()
  measured = rig.safety.get_angle_meas_max()
  sign = 1 if measured > 0 else -1
  assert abs(measured) == 3595
  assert rig.steer(measured, enabled=False)
  assert rig.steer(measured)
  assert rig.steer(sign * 3600)
  assert not rig.steer(sign * 3605)


# *** the car's own controller against this panda: nothing it sends may be rejected ***

class CarOnPanda:
  """PreAPPedalControl and PedalFeedback driven by the same frames as the registered safety hooks, with a model of the
  interceptor's STATE (0 from the first accepted command until 200 ms after the last, else 5)."""
  def __init__(self, zero_torque=None):
    from types import SimpleNamespace
    from opendbc.car.tesla.preap.pedal_contract import PedalFeedback
    from opendbc.car.tesla.preap.pedal_control import PreAPPedalControl
    self.rig = Rig().arm()
    self.fb = PedalFeedback(self.rig.cal)
    self.pc = PreAPPedalControl(self.rig.cal)
    if zero_torque is not None:
      self.pc.zero_torque.get = lambda v: zero_torque
    self.out = SimpleNamespace(canValid=True, canTimeout=False, gasPressed=False, brakePressed=False, accFaulted=False,
                               doorOpen=False, steerFaultPermanent=False, steeringDisengage=False, vEgo=80 / 3.6, aEgo=0.)
    self.cs = SimpleNamespace(pedal_feedback=self.fb, cruiseEnabled=True, enableLongControl=True, out=self.out,
                              cruise_buttons=0, prev_cruise_buttons=0, preap_cc_cancel_needed=False, preap_accel_max=1.,
                              pccEvent=None, pedal_command_failed=False)
    self.SimpleNamespace = SimpleNamespace
    self.last_accept, self.sent, self.rejected, self.gas_di = None, [], [], 0.

  def run(self, ticks):
    rig = self.rig
    for _ in range(ticks):
      rig.state = 0 if self.last_accept is not None and rig.now + 10000 - self.last_accept <= 200_000 else 5
      rig.tick += 1
      rig.advance(10000)
      for frame in rig.stream.frames(rig.tick, cruise=1, speed=rig.speed):
        rig.rx(frame)
      if rig.tick % 2 == 0:
        rig.fb = (rig.fb + 1) & 15
        frame = feedback_frame(rig.cal, rig.fb, di=self.gas_di, state=rig.state)
        rig.rx(frame)
        self.fb.observe(frame[1], frame[2], rig.now * 1000)
      rig.safety.safety_tick()
      rig.safety.aol_set_host_request(3)
      self.out.gasPressed = self.fb.gas_pressed()
      cc = self.SimpleNamespace(enabled=True, longActive=bool(rig.mask() & 2), orientationNED=[0., 0., 0.],
                                actuators=self.SimpleNamespace(accel=0.8))
      for frame in self.pc.update(cc, self.cs, rig.tick, rig.now * 1000):
        kind = 'en' if frame[1][4] & 0x80 else 'dis'
        self.sent.append(kind)
        if rig.tx(frame):
          self.last_accept = rig.now
          self.fb.note_echo(rig.now * 1000)
        else:
          self.rejected.append((rig.tick, kind, frame[1].hex()))
    return self


@pytest.mark.parametrize('zero_torque', [None, 30.])
def test_everything_the_car_controller_sends_is_accepted_by_its_panda(zero_torque):
  car = CarOnPanda(zero_torque).run(300)
  assert car.sent[:1] == ['dis'] and car.sent.count('en') > 120
  assert not car.rejected, car.rejected[:3]
  assert not car.cs.pedal_command_failed and car.rig.safety.get_controls_allowed()
  # A short driver press: silence, then a disabled frame before the pedal is commanded again.
  car.gas_di = 6.
  car.run(6)
  car.gas_di = 0.
  mark = len(car.sent)
  car.run(100)
  assert not car.rejected, car.rejected[:3]
  after = car.sent[mark:]
  assert after and after[0] == 'dis' and 'en' in after
  assert not car.cs.pedal_command_failed and car.rig.safety.get_controls_allowed()
