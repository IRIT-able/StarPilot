"""Dom VirtualDAS pedal control with frozen calibration and current axis authority."""
import math

import numpy as np

from opendbc.car.tesla.preap.pedal_contract import STATE_OK, PedalCodec, pedal_max
from opendbc.car.tesla.preap.pedal_dynamics import PEDAL_RAMP_RATE_UP, PedalZeroTorque
from opendbc.car.tesla.preap.virtual_das import VirtualDAS

# Grace period after engage: ramp accel limit from 0 → full over this window.
# Prevents both regen spike (negative) and pedal stab (MPC requesting high
# positive accel on frame 1). Inspired by Tinkla's proportional ramp.
ENGAGE_GRACE_FRAMES = 50  # 0.5s at 100Hz
ENGAGE_GRACE_PEDAL_RAMP_RATE_UP = 0.9  # DI/update at 50Hz

# The pedal controller updates at 50 Hz. Prompt when a sustained deceleration
# request is not being delivered while the command is in the regen range.
# Available regen varies with SOC and battery temperature, so proximity to
# the DI rail is not a reliable "at the limit" signal: on a full battery the
# rail delivers a fraction of nominal regen, and the integrator only reaches
# it near the end of an under-delivery event. Field capture (2026-07-22,
# minutes after supercharging) showed the command within 0.5 DI of the rail
# for just 0.3 s of a multi-second under-delivery, so a rail gate plus a
# consecutive dwell can never complete. Evidence instead accumulates in a
# saturating up/down counter, which also rides through single-update
# shortfall noise. Trigger and clear thresholds are separated so the visible
# prompt cannot chatter.
REGEN_DECEL_PROMPT_DWELL_UPDATES = 40  # 0.8s of net evidence at 50Hz
REGEN_DECEL_PROMPT_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DECEL_PROMPT_CLEAR_SPEED = 1.0  # m/s
REGEN_DECEL_SHORTFALL_TRIGGER = 0.35  # m/s²
REGEN_DECEL_SHORTFALL_CLEAR = 0.15  # m/s²
REGEN_COMMAND_TRIGGER_DI = -2.0  # command deep in the regen range
REGEN_COMMAND_CLEAR_DI = -1.0
REGEN_DECEL_REQUEST_TRIGGER = -0.5  # m/s²; a meaningful deceleration request
REGEN_DECEL_REQUEST_CLEAR = -0.2  # m/s²


class RegenDecelMonitor:
  """Detect when the regen path cannot deliver the requested deceleration."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

  def update(self, *, pedal_control_active, in_engage_grace, pedal_di,
             limited_accel, actual_accel, v_ego):
    values_are_finite = all(np.isfinite((pedal_di, limited_accel, actual_accel, v_ego)))
    decel_shortfall = actual_accel - limited_accel
    monitoring_allowed = (
      pedal_control_active
      and not in_engage_grace
      and values_are_finite
      and limited_accel < 0.0
    )

    if self.active:
      keep_prompting = (
        monitoring_allowed
        and v_ego > REGEN_DECEL_PROMPT_CLEAR_SPEED
        and pedal_di <= REGEN_COMMAND_CLEAR_DI
        and limited_accel <= REGEN_DECEL_REQUEST_CLEAR
        and decel_shortfall > REGEN_DECEL_SHORTFALL_CLEAR
      )
      if not keep_prompting:
        self.reset()
      return self.active

    under_delivering = (
      monitoring_allowed
      and v_ego >= REGEN_DECEL_PROMPT_MIN_SPEED
      and pedal_di <= REGEN_COMMAND_TRIGGER_DI
      and limited_accel <= REGEN_DECEL_REQUEST_TRIGGER
      and decel_shortfall >= REGEN_DECEL_SHORTFALL_TRIGGER
    )
    if under_delivering:
      self.evidence_updates = min(self.evidence_updates + 1, REGEN_DECEL_PROMPT_DWELL_UPDATES)
    else:
      self.evidence_updates = max(self.evidence_updates - 1, 0)
    self.active = self.evidence_updates >= REGEN_DECEL_PROMPT_DWELL_UPDATES
    return self.active


def _finite(*values):
  try:
    return all(math.isfinite(float(value)) for value in values)
  except (TypeError, ValueError):
    return False


# The panda checks the first enabled frame against its own newest 0x552, on both channels (+5 / -2.5 DI). That frame
# can be one newer than the car's; on this interceptor channel 1 moves up to 67 counts (3.4 units) between frames at
# rest and channel 2 scatters around the fit on its own. Replayed over 14,503 logged rest frame pairs, an unramped
# first frame at +1.5 DI is rejected on 0.03 % of pairs at factor 1 (0.09 % at 1.25); +3.9 DI was rejected on 14 %.
# With no learned coast point above the pedal (low speed) the first frame is the measured pedal itself, as NAP.
FIRST_COMMAND_ABOVE_FEEDBACK = 1.5
FAILURE_HOLD_NS = 1_000_000_000          # the fault stays visible this long, whatever the engagement does
ACQUIRE_MAX_FRAMES = 10                  # 200 ms of disabled frames without the interceptor leaving its timeout state
ECHO_TIMEOUT_NS = 100_000_000
# The panda bounds every enabled frame by the speed table at the newest raw ESP_B speed it has (0x155, 50 Hz), not
# by the filtered vEgo. The car's raw sample can be up to two ESP_B frames older than the panda's when the frame goes
# out; 0.5 m/s covers 12 m/s^2 over those 40 ms, and costs at most 0.8 DI of ceiling.
SPEED_CAP_MARGIN = 0.5


def command_ceiling(v_ego_raw):
  """Highest DI the panda accepts at any raw speed the car could be behind by."""
  return pedal_max(max(0., v_ego_raw - SPEED_CAP_MARGIN))


class PreAPPedalControl:
  def __init__(self, calibration):
    self.codec = PedalCodec(calibration)
    self.zero_torque = PedalZeroTorque()
    self.vdas = VirtualDAS(zero_torque=self.zero_torque)
    self.previous_di = 0.
    self.was_requested = False
    self.was_active = False
    self.sending = False
    self.primed = False          # a disabled frame went out since pedal cruise was last wanted
    self.failed = False          # latched until the request goes away (NAP), and for at least FAILURE_HOLD_NS
    self.failed_ns = -FAILURE_HOLD_NS
    self.acquire_frames = 0
    self.enabled_since_ns = None
    self.engage_frame = -1_000_000
    # Snapshot of max-accel-at-engage-speed; the deterministic ceiling for the grace-period ramp (NAP).
    self.engage_a_max = 0.
    self.handoff_slew_active = False
    self.regen_decel_monitor = RegenDecelMonitor()

  def _observe(self, CC, CS, feedback):
    # NAP while not commanding: the aEgo filter and grade estimate stay warm (no authority), the zero-torque
    # learner sees an inactive update (its settle count and best-torque memory restart), the regen prompt clears.
    # Non-finite inputs are not fed to the filters: nothing in the engage path would ever clear them again.
    if _finite(CS.out.aEgo, *CC.orientationNED):
      self.vdas.observe(CS.out.aEgo, list(CC.orientationNED))
    self.zero_torque.update(float(getattr(feedback, 'torque_level', 0.)), self.previous_di, CS.out.vEgo,
                            control_active=False, accel_command=self.vdas.jerk_limiter.a_limited)
    self.handoff_slew_active = False
    self.regen_decel_monitor.reset()

  def _send(self, feedback, now_ns, di=0., *, enabled=False):
    command = self.codec.command(di, enabled=enabled)
    feedback.note_tx(enabled, now_ns)
    if not enabled:
      self.enabled_since_ns = None
    elif self.enabled_since_ns is None:
      self.enabled_since_ns = now_ns
    return [command]

  def _disabled(self, CC, CS, feedback, now_ns):
    self.previous_di = 0.
    self._observe(CC, CS, feedback)
    self.primed = True
    return self._send(feedback, now_ns)

  def _fail(self, CC, CS, feedback, now_ns):
    # NAP: a failed command or takeover latches until the request is dropped; it is not retried 20 ms later.
    # StarPilot treats pedal loss as a longitudinal fault of the whole engagement (carstate reads the flag).
    self.failed = True
    self.failed_ns = now_ns
    self.was_active = False
    CS.pedal_command_failed = True
    release = self.sending
    self.sending = self.primed = False
    self.previous_di = 0.
    try:
      self._observe(CC, CS, feedback)
    except Exception:
      pass
    return self._send(feedback, now_ns) if release else []

  def update(self, CC, CS, frame, now_ns):
    if frame % 2:
      # The monitor runs at 50 Hz; the prompt is a level (NAP publishes it every frame), CS.pccEvent is per frame.
      if self.regen_decel_monitor.active:
        CS.pccEvent = 'pedalMaxRegen'
      return []
    feedback = CS.pedal_feedback
    healthy = feedback is not None and feedback.ready(now_ns)
    requested = bool(healthy and CS.cruiseEnabled and CS.enableLongControl)
    if not (CS.cruiseEnabled and CS.enableLongControl) and now_ns - self.failed_ns > FAILURE_HOLD_NS:
      self.failed = False
      CS.pedal_command_failed = False
    try:
      return self._update(CC, CS, frame, now_ns, feedback, requested)
    except Exception:
      return self._fail(CC, CS, feedback, now_ns) if feedback is not None else []

  def _update(self, CC, CS, frame, now_ns, feedback, requested):
    requested = requested and not self.failed
    # Wanted and nothing in the way on the car side. While this holds, disabled frames bring the interceptor
    # out of its idle timeout and let the panda grant longitudinal; otherwise nothing is sent at all.
    wanted = bool(requested and CS.out.canValid and not CS.out.canTimeout and
                  not CS.out.gasPressed and not CS.out.brakePressed and not CS.out.accFaulted and
                  not CS.out.doorOpen and not CS.out.steerFaultPermanent and not CS.out.steeringDisengage)
    # An enabled frame only after a disabled one since pedal cruise was last wanted: the panda forgets its
    # accepted command state on gas, brake and faults and needs that frame again.
    active = bool(wanted and self.primed and CC.enabled and CC.longActive and feedback.commandable(now_ns))
    # NAP: the car's own cruise is cancelled when pedal cruise starts or stops and on every stalk press,
    # so a pull with DI cruise armed cannot leave stock cruise driving under the pedal.
    stalk_press = CS.cruise_buttons != CS.prev_cruise_buttons and CS.cruise_buttons != 0
    if requested != self.was_requested or stalk_press:
      CS.preap_cc_cancel_needed = True
    self.was_requested = requested
    finite = _finite(CC.actuators.accel, CS.out.vEgo, CS.out.vEgoRaw, CS.out.aEgo, *CC.orientationNED)
    first = bool(active and finite and not self.was_active)
    if first:
      # Seed and grace start when control really starts (NAP), not when it was first asked for.
      self.engage_frame = frame
      self.handoff_slew_active = True
      # NAP seeds at max(measured pedal, learned coast point). The first frame never goes above that, and never more
      # than FIRST_COMMAND_ABOVE_FEEDBACK above the measured pedal (the panda's window); the grace ramp climbs on.
      seed = min(max(feedback.value, self.zero_torque.get(CS.out.vEgo)), feedback.value + FIRST_COMMAND_ABOVE_FEEDBACK)
      self.previous_di = min(command_ceiling(CS.out.vEgoRaw), max(-5., seed))
      self.vdas.reset(measured_accel=CS.out.aEgo, commanded_accel=0., pedal_di_init=self.previous_di,
                      preserve_grade=True)
      self.engage_a_max = CS.preap_accel_max
    self.was_active = active and finite
    if not wanted:
      # Silent while idle, as logged on this interceptor: one disabled frame to release, then nothing.
      release = self.sending
      self.sending = self.primed = False
      self.acquire_frames = 0
      self.previous_di = 0.
      if feedback is not None:
        self._observe(CC, CS, feedback)
      return self._send(feedback, now_ns) if release else []
    self.sending = True
    if not finite:
      return self._fail(CC, CS, feedback, now_ns)
    if not active:
      # Disabled frames wake the interceptor within 45 ms on this car. One that stays in its timeout state
      # after ACQUIRE_MAX_FRAMES of them is not going to take commands.
      self.acquire_frames = self.acquire_frames + 1 if (self.primed and feedback.state != STATE_OK) else 0
      if self.acquire_frames > ACQUIRE_MAX_FRAMES:
        return self._fail(CC, CS, feedback, now_ns)
      return self._disabled(CC, CS, feedback, now_ns)
    self.acquire_frames = 0
    # BogPilot: enabled frames the panda does not send back are being blocked; stop instead of commanding blind.
    if (self.enabled_since_ns is not None and now_ns - self.enabled_since_ns > ECHO_TIMEOUT_NS and
        now_ns - feedback.echo_ns > ECHO_TIMEOUT_NS):
      return self._fail(CC, CS, feedback, now_ns)
    self.zero_torque.update(feedback.torque_level, self.previous_di, CS.out.vEgo,
                            control_active=True, accel_command=self.vdas.jerk_limiter.a_limited)
    elapsed = frame - self.engage_frame
    grace = elapsed < ENGAGE_GRACE_FRAMES
    demand = float(CC.actuators.accel)
    effort_limits = None
    # NAP "handoff slew": the gentle take-up rate holds after the grace until the ramp stops limiting.
    ramp_up = ENGAGE_GRACE_PEDAL_RAMP_RATE_UP if self.handoff_slew_active else PEDAL_RAMP_RATE_UP
    if grace:
      # Cap at grace progress * engage_a_max so the ceiling is the tuned accel-profile envelope at the
      # engage speed, not the live request. No regen and a frozen integrator during the grace.
      cap = elapsed / ENGAGE_GRACE_FRAMES * self.engage_a_max
      demand = max(0., min(demand, cap))
      effort_limits = (0., cap)
      ramp_up = ENGAGE_GRACE_PEDAL_RAMP_RATE_UP
    try:
      proposed = self.vdas.update(demand, CS.out.vEgo, self.previous_di, a_ego=CS.out.aEgo,
                                  freeze_integrator=grace, orientation_ned=list(CC.orientationNED),
                                  accel_effort_limits=effort_limits, pedal_ramp_rate_up=ramp_up)
      if self.handoff_slew_active and not grace and not self.vdas.pedal_ramp_limited_up:
        self.handoff_slew_active = False
      # Apply hard domain last: original rate clamp could otherwise escape it
      # when an initial driver observation lay above the speed envelope.
      proposed = max(-5., min(command_ceiling(CS.out.vEgoRaw), proposed))
      if first:
        proposed = self.previous_di
      self.previous_di = proposed
      command = self._send(feedback, now_ns, proposed, enabled=True)[0]
    except ValueError:
      # NAP: a command that cannot be computed or encoded ends pedal cruise, never an exception in card.
      return self._fail(CC, CS, feedback, now_ns)
    if self.regen_decel_monitor.update(pedal_control_active=True, in_engage_grace=grace, pedal_di=proposed,
                                       limited_accel=self.vdas.jerk_limiter.a_limited,
                                       actual_accel=CS.out.aEgo, v_ego=CS.out.vEgo):
      CS.pccEvent = 'pedalMaxRegen'
    return [command]
