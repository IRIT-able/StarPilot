"""NAP test_pedal_authority.py, the zero-torque learner and regen-prompt cases only.

The PedalAuthority FSM, pedal-unavailable handling and wire-counter cases are not ported (StarPilot keeps its own
gating in PreAPPedalControl; see test_preap_pedal_car.py). Test bodies are NAP's; the fixture builds StarPilot's
PreAPPedalControl behind NAP's PreAPLongController call shape.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.pedal_contract import D as PEDAL_D, M1 as PEDAL_M1, PedalCalibration
from opendbc.car.tesla.preap.pedal_control import (
  REGEN_DECEL_PROMPT_DWELL_UPDATES,
  PreAPPedalControl,
  RegenDecelMonitor,
)
from opendbc.car.tesla.preap.pedal_dynamics import (
  PEDAL_DI_ZERO,
  PEDAL_RAMP_RATE_UP,
  PedalZeroTorque,
)
from opendbc.car.tesla.preap.pedal_profile import accel_max

GAS_COMMAND_ID = 0x551


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: PEDAL_DI_ZERO,
    update=lambda *_args, **_kwargs: None,
  )


class _Controller:
  """StarPilot's PreAPPedalControl driven through NAP's PreAPLongController.update() call shape."""

  def __init__(self, zero_torque):
    self.control = PreAPPedalControl(PedalCalibration(1.0, 0.0, 2))
    self.set_zero_torque(zero_torque)

  def set_zero_torque(self, zero_torque):
    # NAP patches the module singleton used by both the controller and VirtualDAS.
    self.control.zero_torque = zero_torque
    self.control.vdas.zero_torque = zero_torque

  @property
  def vdas(self):
    return self.control.vdas

  @property
  def prev_pedal_di(self):
    return self.control.previous_di

  @property
  def regen_decel_monitor(self):
    return self.control.regen_decel_monitor

  def update(self, cc, cs, frame, tesla_can=None, can_bus_party=0):
    sends = self.control.update(cc, cs, frame, 0)
    # NAP publishes the monitor level as CS.pedal_brake_required; StarPilot as CS.pccEvent = 'pedalMaxRegen'.
    cs.pedal_brake_required = cs.pccEvent == 'pedalMaxRegen'
    return sends


@pytest.fixture
def controller_env():
  # Healthy, commandable interceptor feedback (NAP: available and STATE == 0).
  feedback = SimpleNamespace(ready=lambda _now_ns: True, commandable=lambda _now_ns: True, state=0,
      note_tx=lambda _enabled, _now_ns: None, echo_ns=2 ** 62,
                             value=0.0, torque_level=0.0)
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False, brakePressed=False, canValid=True,
                        canTimeout=False, accFaulted=False, doorOpen=False, steerFaultPermanent=False,
                        steeringDisengage=False),
    cruise_buttons=0,
    prev_cruise_buttons=0,
    pedal=feedback,
    pedal_feedback=feedback,
    # NAP get_preap_accel_limits(vEgo) at the default personality; CarState keeps this current in StarPilot.
    preap_accel_max=accel_max(15.0, 1),
    pccEvent=None,
    preap_cc_cancel_needed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.0),
    enabled=True,
    longActive=False,
    orientationNED=[],
  )
  return _Controller(_zero_torque()), cc, cs, None


def _decode_pedal_command(command):
  address, data, _bus = command
  assert address == GAS_COMMAND_ID
  raw_command = (data[0] << 8) | data[1]
  return SimpleNamespace(
    enabled=bool(data[4] & 0x80),
    command=raw_command * PEDAL_M1 + PEDAL_D,
    raw_command=raw_command,
    idx=data[4] & 0x0F,
  )


def _activate_longitudinal(cc, cs):
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cc.longActive = True


def test_zero_torque_anchor_converges_without_a_command_step():
  zero_torque = PedalZeroTorque()

  # The configured actuator delay exceeds 0.48s, so no candidate may be
  # accepted before this observation window settles.
  for _ in range(24):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)

  zero_torque.update(
    torque_level=-0.5,
    current_pedal_di=6.0,
    v_ego=17.0,
    control_active=True,
    accel_command=0.0,
  )
  assert zero_torque.value == pytest.approx(0.1)

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(6.0)


def test_zero_torque_anchor_ignores_inactive_pedal_feedback():
  zero_torque = PedalZeroTorque()

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=False,
      accel_command=0.0,
    )

  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)


@pytest.mark.parametrize(
  ("control_active", "accel_command"),
  ((False, 0.0), (True, 0.3)),
)
def test_zero_torque_anchor_freezes_across_invalid_observations(control_active, accel_command):
  zero_torque = PedalZeroTorque()

  for _ in range(25):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  frozen_value = zero_torque.value
  assert frozen_value == pytest.approx(0.1)

  for _ in range(100):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=control_active,
      accel_command=accel_command,
    )
  assert zero_torque.value == pytest.approx(frozen_value)

  # A newly valid observation must settle again before adaptation resumes.
  for _ in range(24):
    zero_torque.update(
      torque_level=-0.5,
      current_pedal_di=6.0,
      v_ego=17.0,
      control_active=True,
      accel_command=0.0,
    )
  assert zero_torque.value == pytest.approx(frozen_value)

  zero_torque.update(
    torque_level=-0.5,
    current_pedal_di=6.0,
    v_ego=17.0,
    control_active=True,
    accel_command=0.0,
  )
  assert zero_torque.value == pytest.approx(frozen_value + 0.1)


def test_acceleration_trace_cannot_reanchor_zero_torque_or_hit_backstop(controller_env):
  controller, cc, cs, tesla_can = controller_env
  zero_torque = PedalZeroTorque()
  controller.set_zero_torque(zero_torque)

  _activate_longitudinal(cc, cs)
  cc.actuators.accel = 0.31
  commands = []
  for update_index, frame in enumerate(range(0, 360, 2)):
    cs.pedal.torque_level = -0.5 if update_index == 150 else -20.0
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    commands.extend(_decode_pedal_command(command).command for command in sent)

  command_steps = [current - previous for previous, current in zip(commands, commands[1:], strict=False)]
  consecutive_backstop_steps = any(
    first >= PEDAL_RAMP_RATE_UP - 0.1 and second >= PEDAL_RAMP_RATE_UP - 0.1
    for first, second in zip(command_steps, command_steps[1:], strict=False)
  )
  assert zero_torque.value == pytest.approx(PEDAL_DI_ZERO)
  assert not consecutive_backstop_steps


def test_max_regen_does_not_prompt_when_requested_decel_is_delivered(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5

  max_regen_prompted = False
  for frame in range(0, 400, 2):
    # Model delivered deceleration as a one-update response to the controller's
    # previous jerk-limited command, beginning from the vehicle's current zero.
    cs.out.aEgo = controller.vdas.jerk_limiter.a_limited
    # CarController clears edge events before each PreAP controller update.
    cs.pccEvent = None
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    max_regen_prompted |= cs.pccEvent == "pedalMaxRegen"

  assert controller.vdas.jerk_limiter.a_limited == pytest.approx(cc.actuators.accel)
  assert controller.prev_pedal_di < -4.75
  assert not max_regen_prompted


def test_controller_prompts_when_full_regen_under_delivers_while_moving(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5
  cs.out.aEgo = -0.5

  for frame in range(0, 400, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if cs.pedal_brake_required:
      break

  assert controller.prev_pedal_di <= -4.5
  assert controller.regen_decel_monitor.active
  assert cs.pedal_brake_required


def _update_regen_monitor(monitor, *, actual_accel=-0.5, v_ego=15.0,
                          pedal_di=-5.0, pedal_control_active=True):
  return monitor.update(
    pedal_control_active=pedal_control_active,
    in_engage_grace=False,
    pedal_di=pedal_di,
    limited_accel=-1.5,
    actual_accel=actual_accel,
    v_ego=v_ego,
  )


def test_regen_prompt_requires_sustained_unmet_decel_while_moving():
  monitor = RegenDecelMonitor()

  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES - 1):
    assert not _update_regen_monitor(monitor)

  assert _update_regen_monitor(monitor)


def test_regen_prompt_does_not_fire_at_standstill():
  monitor = RegenDecelMonitor()

  for _ in range(2 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    assert not _update_regen_monitor(monitor, v_ego=0.0)


def test_regen_prompt_uses_hysteresis_and_clears_when_decel_recovers():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  # These values have crossed back over the trigger thresholds, but remain
  # inside the clear thresholds so sensor noise cannot chatter the prompt.
  assert _update_regen_monitor(monitor, actual_accel=-1.25, v_ego=1.5, pedal_di=-4.25)

  assert not _update_regen_monitor(monitor, actual_accel=-1.35)
  assert not monitor.active


def test_regen_prompt_clears_as_soon_as_pedal_control_releases():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  assert not _update_regen_monitor(monitor, pedal_control_active=False)


def test_regen_prompt_survives_brief_shortfall_dropouts():
  # Acceleration-estimate noise dips the shortfall below the trigger for a
  # single update every so often. Accumulated evidence must survive those
  # dropouts instead of restarting from zero.
  monitor = RegenDecelMonitor()
  fired = False
  for _ in range(3 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    for _ in range(9):
      fired = _update_regen_monitor(monitor) or fired
    # shortfall 0.3 m/s²: below the trigger, above the clear threshold
    fired = _update_regen_monitor(monitor, actual_accel=-1.2) or fired
    if fired:
      break
  assert fired


def test_regen_prompt_requires_deep_regen_command():
  # A command still in the shallow-regen range has authority left; the
  # driver does not need the brake yet.
  monitor = RegenDecelMonitor()
  for _ in range(3 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    assert not _update_regen_monitor(monitor, pedal_di=-1.5)


def _replay_fixture(name):
  with open(Path(__file__).parent / 'data' / name) as f:
    return json.load(f)['samples']


def _replay_monitor(samples):
  # Drive the monitor the way PreAPLongController wires it: driver pedal
  # input or authority loss resets, everything else goes through update().
  monitor = RegenDecelMonitor()
  first_active = None
  for i, (v_ego, a_ego, limited_accel, pedal_di, brake, gas, long_active) in enumerate(samples):
    if not long_active or brake or gas:
      monitor.reset()
      continue
    active = monitor.update(
      pedal_control_active=True,
      in_engage_grace=False,
      pedal_di=pedal_di,
      limited_accel=limited_accel,
      actual_accel=a_ego,
      v_ego=v_ego,
    )
    if active and first_active is None:
      first_active = i
  return first_active


def test_regen_prompt_fires_on_captured_weak_regen_drive():
  # 2026-07-22 field capture, minutes after supercharging: regen delivered
  # only -0.3 m/s² of a ~-1.0 m/s² request for several seconds while the
  # command walked down to the regen rail and the driver never braked.
  samples = _replay_fixture('regen_underdelivery_50hz.json')
  assert _replay_monitor(samples) is not None


def test_regen_prompt_stays_silent_on_captured_grade_decel():
  # Same drive, mountain uphill: deceleration requests are delivered through
  # grade compensation with the pedal still in the propulsion range.
  samples = _replay_fixture('grade_decel_positive_di_50hz.json')
  assert _replay_monitor(samples) is None
