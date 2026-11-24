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
  def __init__(self, zero_torque=None, factor=1.):
    from types import SimpleNamespace
    from opendbc.car.tesla.preap.pedal_contract import PedalFeedback
    from opendbc.car.tesla.preap.pedal_control import PreAPPedalControl
    self.rig = Rig(factor=factor).arm()
    self.fb = PedalFeedback(self.rig.cal)
    self.pc = PreAPPedalControl(self.rig.cal)
    if zero_torque is not None:
      self.pc.zero_torque.get = lambda v: zero_torque
    self.out = SimpleNamespace(canValid=True, canTimeout=False, gasPressed=False, brakePressed=False, accFaulted=False,
                               doorOpen=False, steerFaultPermanent=False, steeringDisengage=False, vEgo=80 / 3.6,
                               vEgoRaw=80 / 3.6, aEgo=0.)
    self.speed_fn = None          # tick -> kph on ESP_B; the car sees the raw sample 4 ticks (40 ms) old, vEgo 3 m/s above
    self.speed_history = []
    self.noise_fn = None          # tick -> raw counts added to channel 1 of 0x552
    self.car_frame_lag = False    # the car gets each 0x552 one frame after the panda
    self.logged = None            # [(one, two)] raw 0x552 channels replayed in order
    self.pending = None
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
      if self.speed_fn is not None:
        rig.speed = self.speed_fn(rig.tick)
        self.speed_history.append(rig.speed)
        self.out.vEgoRaw = self.speed_history[max(0, len(self.speed_history) - 5)] / 3.6
        self.out.vEgo = self.out.vEgoRaw + 3.
      for frame in rig.stream.frames(rig.tick, cruise=1, speed=rig.speed):
        rig.rx(frame)
      if rig.tick % 2 == 0:
        rig.fb = (rig.fb + 1) & 15
        if self.logged is not None:
          one, two = self.logged[(rig.tick // 2) % len(self.logged)]
          frame = feedback_frame(rig.cal, rig.fb, state=rig.state, raw_one=one, raw_two=two)
        elif self.noise_fn is None:
          frame = feedback_frame(rig.cal, rig.fb, di=self.gas_di, state=rig.state)
        else:
          one = int((rig.cal.physical(self.gas_di) - D) / M1) + self.noise_fn(rig.tick)
          frame = feedback_frame(rig.cal, rig.fb, state=rig.state, raw_one=one, raw_two=expected_two(one))
        rig.rx(frame)
        if self.car_frame_lag:
          if self.pending is not None:
            self.fb.observe(*self.pending)
          self.pending = (frame[1], frame[2], rig.now * 1000)
        else:
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


@pytest.mark.parametrize('phase', [0, 1])
def test_first_enabled_frame_survives_the_largest_logged_channel1_jump_with_the_panda_one_frame_ahead(phase):
  # Logged on this interceptor at rest: channel 1 moves up to 67 counts between consecutive frames. Worst case for
  # channel 1: it alternates by 67 counts every frame and the panda judges the first enabled frame against a newer one.
  car = CarOnPanda(zero_torque=30.)
  car.noise_fn = lambda tick: -33 if (tick // 2 + phase) % 2 else 34
  car.car_frame_lag = True
  for _ in range(8):                         # repeated engagements: run, a short driver press, release
    car.run(80)
    car.gas_di = 6.
    car.run(8)
    car.gas_di = 0.
  car.run(80)
  assert car.sent.count('en') > 300 and not car.rejected, car.rejected[:3]
  assert not car.cs.pedal_command_failed and car.rig.safety.get_controls_allowed()


def test_commands_at_the_ceiling_while_decelerating_hard_are_never_rejected():
  # The panda bounds enabled frames by its newest raw ESP_B speed; the car's raw sample is older and its
  # filtered vEgo lags higher still. 10 m/s^2 from 90 kph, the car's raw sample 40 ms old, the command pinned at the
  # speed-table ceiling (40 ms at 10 m/s^2 is 0.4 m/s, inside the 0.5 m/s margin).
  car = CarOnPanda()
  car.rig.speed = 90.
  car.speed_fn = lambda tick: max(10., 90. - 0.36 * max(0, tick - 120))
  real = car.pc.vdas.update
  car.pc.vdas.update = lambda demand, v, previous, **kw: (real(demand, v, previous, **kw), previous + 4.)[1]
  car.run(380)
  assert car.sent.count('en') > 120 and not car.rejected, car.rejected[:3]
  assert not car.cs.pedal_command_failed


# *** remaining untested paths from the safety review ***

@pytest.mark.parametrize('stuck', [1, 2])
@pytest.mark.parametrize('position', [1200, 2400])
def test_channel_stuck_away_from_rest_is_caught_within_the_tolerance(stuck, position):
  # The driver holds the pedal down, a channel sticks there, the pedal comes back up.
  rig = Rig().arm()
  for one in range(REST_RAW, position + 1, 40):
    rig.step(2, raw_one=one, raw_two=expected_two(one))
  assert rig.safety.get_controls_allowed()
  moved = 0.
  while rig.safety.get_controls_allowed() and moved < 12.:
    moved += 0.25
    one = position if stuck == 1 else position - int(moved / M1)
    two = expected_two(position) if stuck == 2 else expected_two(position - int(moved / M1))
    rig.step(2, raw_one=one, raw_two=two)
  assert TOLERANCE_UNITS - 0.3 <= moved <= TOLERANCE_UNITS + 0.3, moved   # the pair sat on the fit when it stuck
  rig.assert_withdrawn()


def test_state5_in_a_running_stream_is_a_fault_after_the_enabled_state_was_cleared():
  # A gas press clears the enabled state but keeps the engagement, so STATE 5 is the only thing that can drop it here.
  # (Brake, gear, door and cancel already withdraw control and need a fresh pull, so the rule is unobservable there.)
  rig = Rig().engaged()
  pressed = int((6.0 - D) / M1)
  rig.step(2, raw_one=pressed, raw_two=expected_two(pressed))
  assert rig.safety.get_controls_allowed() and not rig.cmd(0., enabled=True), 'enabled state cleared, still engaged'
  for _ in range(8):                         # the car keeps a disabled stream going for 160 ms
    assert rig.cmd(enabled=False)
    rig.step(2)
  rig.step(2, state=5)
  assert not rig.safety.get_controls_allowed() and rig.mask() == 0
  assert not rig.cmd(0., enabled=True)


def test_timer_wrap_neither_faults_a_healthy_stream_nor_hides_state5():
  rig = Rig()
  rig.now = 2 ** 32 - 1_400_000              # the wrap falls about 0.5 s after engagement

  def advance(microseconds, rig=rig):
    rig.now = (rig.now + microseconds) & 0xFFFFFFFF
    rig.safety.set_timer(rig.now)
  rig.advance = advance
  rig.safety.set_timer(rig.now)
  rig.engaged()
  for _ in range(60):                        # 1.2 s of enabled commands across the wrap
    rig.step(1)
    assert rig.cmd(0., enabled=True)
    rig.step(1)
  assert rig.now < 1_000_000 and rig.safety.get_controls_allowed() and rig.mask() == 3
  rig.step(2, state=5)
  rig.assert_withdrawn()


def test_counter_advancing_by_two_on_every_frame_is_accepted_but_a_repeat_is_not():
  rig = Rig().engaged()
  for _ in range(20):
    rig.step(2, advance=2)
    assert rig.cmd(0., enabled=True)
  rig.step(2, advance=0)
  rig.assert_withdrawn()


def test_a_steering_table_violation_resets_the_reference_to_the_measured_angle():
  rig = Rig(speed=5.).arm()                  # table: 8 deg up per frame
  assert rig.steer(0, enabled=False)
  assert rig.steer(0)
  assert rig.steer(80)
  assert not rig.steer(80 + 90)
  # 16 deg is one table step from the last accepted 8 deg, but the reference is now the measured 0 deg (9 deg step).
  assert not rig.steer(160)
  assert rig.steer(80)


@pytest.mark.parametrize('raw', [424, 450, 500, 550, 600, 633])
def test_logged_rest_noise_range_is_not_gas_with_the_onset_calibrated_above_it(raw):
  # NAP's rest-noise regression (raw 424-633 while not pressing): with the gas onset saved just above the highest
  # rest reading (the zero handed to the panda is onset - 1/factor), nothing in that range reads as a press.
  from opendbc.car.tesla.preap.pedal_contract import float32
  onset = 633 * M1 + D + 0.1
  rig = Rig(zero=float32(onset - 1.)).engaged()
  rig.step(2, raw_one=raw, raw_two=expected_two(raw))
  assert rig.safety.get_controls_allowed() and rig.mask() == 3
  assert not rig.safety.get_gas_pressed_prev()
  assert rig.cmd(0., enabled=True), 'and enabled commands keep being accepted (NAP :693)'


@pytest.mark.parametrize('source', ['none', 'message', 'di'])
def test_brake_received_then_enabled_command_in_the_same_tick_is_rejected(source):
  # BogPilot test_tesla_preap_pedal_brake.py:64: the brake frame and the next enabled command in one 10 ms tick.
  rig = Rig().engaged()
  assert rig.cmd(0., enabled=True)
  rig.tick += 1
  rig.advance(10000)
  for frame in rig.stream.frames(rig.tick, cruise=1, speed=rig.speed, brake=2 if source == 'message' else 1,
                                 di_brake=int(source == 'di')):
    rig.rx(frame)
  assert rig.cmd(0., enabled=True) == (source == 'none')


# *** mode 39 with the pedal but without AOL (alternative experience 0; card always selects 32 for the pedal) ***

def _non_aol_engaged(monkeypatch):
  import opendbc.safety.tests.test_tesla_preap_pedal_faults as this
  from openpilot.starpilot.car.tesla.tests.pedal_fixture import native_words

  def configure(safety, calibration, word, cp_bytes):
    safety.init_tests()
    assert safety.set_safety_hooks(0, 0) == 0
    safety.set_aol_test_heartbeat(False)
    safety.set_alternative_experience(0)
    for i, value in enumerate(native_words(calibration, word, cp_bytes)):
      assert safety.tesla_preap_test_config_word(i, value)
    assert safety.tesla_preap_test_config_word(0xFFFF, 0xC051)
    assert safety.set_safety_hooks(39, word) == 0
  monkeypatch.setattr(this, 'configure_native', configure)
  rig = Rig()
  rig.step(64)
  rig.safety.set_aol_test_heartbeat(True)
  rig.armed = True
  for tick in range(65, 91):
    rig.step(lever=2 if tick in (65, 75) else 0)
  assert rig.safety.get_controls_allowed()
  assert not rig.cmd(0., enabled=True), 'no enabled frame before a disabled one in this mode either'
  assert rig.cmd(enabled=False)
  rig.state = 0
  rig.step(2, command=(0., False, True))
  assert rig.cmd(0., enabled=True)
  return rig


def test_without_aol_a_stalled_interceptor_blocks_enabled_frames_after_500_ms(monkeypatch):
  # Neither reference has a pedal-specific timeout (both rely on the 1 s RX-check lag); the port's is 500 ms, and with
  # AOL the stream rule cuts it to 200 ms.
  rig = _non_aol_engaged(monkeypatch)
  accepted = []
  for _ in range(60):
    rig.step(1, feedback=False)
    accepted.append(rig.cmd(0., enabled=True))
  assert all(accepted[:45]) and not any(accepted[51:]), [i for i, ok in enumerate(accepted) if ok][-1:]


@pytest.mark.parametrize('fault', ['state5', 'checksum', 'counter', 'stuck'])
def test_without_aol_feedback_faults_still_withdraw_control(monkeypatch, fault):
  rig = _non_aol_engaged(monkeypatch)
  if fault == 'state5':
    rig.step(2, state=5)
  elif fault == 'checksum':
    rig.step(2, bad_checksum=True)
  elif fault == 'counter':
    rig.step(2, advance=0)
  else:
    rig.step(2, raw_one=REST_RAW, raw_two=expected_two(REST_RAW + int(4.5 / M1)))
  assert not rig.safety.get_controls_allowed() and not rig.cmd(0., enabled=True)



@pytest.mark.parametrize('factor,coast', [(1., 30.), (1.25, 30.), (1., 0.)])
def test_first_enabled_frames_over_logged_rest_frames_with_the_panda_one_frame_ahead(factor, coast):
  # 1,790 consecutive logged 0x552 frames at rest (00000021 seg 5, working hardware), replayed in order. About 35
  # engagements, each judged by the panda against a frame one newer than the car's. Coast 30: first frame at
  # measured + 1.5 DI; coast 0: at the measured pedal (NAP's seed at low speed).
  import json
  import os
  with open(os.path.join(os.path.dirname(__file__), 'tesla_preap_pedal_rest_frames.json')) as f:
    logged = json.load(f)
  car = CarOnPanda(zero_torque=coast, factor=factor)
  car.logged = logged
  car.car_frame_lag = True
  engagements = 0
  while car.rig.tick < 2 * len(logged) - 120:
    car.cs.cruiseEnabled = car.cs.enableLongControl = True
    car.run(60)
    engagements += 1
    car.cs.cruiseEnabled = car.cs.enableLongControl = False
    car.run(40)                              # released long enough for the interceptor to time out again
  assert engagements >= 30
  assert car.sent.count('en') > 25 * engagements and not car.rejected, car.rejected[:3]
