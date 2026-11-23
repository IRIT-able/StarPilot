"""Dom VirtualDAS pedal control with frozen calibration and current axis authority."""
import math

import numpy as np

from opendbc.car.tesla.preap.pedal_contract import PedalCodec, pedal_max
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


class PreAPPedalControl:
  def __init__(self, calibration):
    self.codec = PedalCodec(calibration)
    self.zero_torque = PedalZeroTorque()
    self.vdas = VirtualDAS(zero_torque=self.zero_torque)
    self.previous_di = 0.
    self.was_requested = False
    self.was_active = False
    self.sending = False
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

  def _disabled(self, CC, CS, feedback):
    self.previous_di = 0.
    self._observe(CC, CS, feedback)
    return [self.codec.command(enabled=False)]

  def update(self, CC, CS, frame, now_ns):
    if frame % 2:
      # The monitor runs at 50 Hz; the prompt is a level (NAP publishes it every frame), CS.pccEvent is per frame.
      if self.regen_decel_monitor.active:
        CS.pccEvent = 'pedalMaxRegen'
      return []
    feedback = CS.pedal_feedback
    healthy = feedback is not None and feedback.ready(now_ns)
    requested = bool(healthy and CS.cruiseEnabled and CS.enableLongControl)
    # Wanted and nothing in the way on the car side. While this holds, disabled frames bring the interceptor
    # out of its idle timeout and let the panda grant longitudinal; otherwise nothing is sent at all.
    wanted = bool(requested and CS.out.canValid and not CS.out.canTimeout and
                  not CS.out.gasPressed and not CS.out.brakePressed and not CS.out.accFaulted and
                  not CS.out.doorOpen and not CS.out.steerFaultPermanent and not CS.out.steeringDisengage)
    active = bool(wanted and CC.enabled and CC.longActive and feedback.commandable(now_ns))
    # NAP: the car's own cruise is cancelled when pedal cruise starts or stops and on every stalk press,
    # so a pull with DI cruise armed cannot leave stock cruise driving under the pedal.
    stalk_press = CS.cruise_buttons != CS.prev_cruise_buttons and CS.cruise_buttons != 0
    if requested != self.was_requested or stalk_press:
      CS.preap_cc_cancel_needed = True
    self.was_requested = requested
    finite = _finite(CC.actuators.accel, CS.out.vEgo, CS.out.aEgo, *CC.orientationNED)
    if active and finite and not self.was_active:
      # Seed and grace start when control really starts (NAP), not when it was first asked for.
      # NAP never seeds below the learned coast position; the DI domain clamp is this port's.
      self.engage_frame = frame
      self.handoff_slew_active = True
      seed = max(feedback.value, self.zero_torque.get(CS.out.vEgo))
      self.previous_di = min(pedal_max(CS.out.vEgo), max(-5., seed))
      self.vdas.reset(measured_accel=CS.out.aEgo, commanded_accel=0., pedal_di_init=self.previous_di,
                      preserve_grade=True)
      self.engage_a_max = CS.preap_accel_max
    self.was_active = active and finite
    if not wanted:
      # Silent while idle, as logged on this interceptor: one disabled frame to release, then nothing.
      release = self.sending
      self.sending = False
      self.previous_di = 0.
      self._observe(CC, CS, feedback)
      return [self.codec.command(enabled=False)] if release else []
    self.sending = True
    if not active:
      return self._disabled(CC, CS, feedback)
    if not finite:
      return self._disabled(CC, CS, feedback)
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
      proposed = max(-5., min(pedal_max(CS.out.vEgo), proposed))
      self.previous_di = proposed
      command = self.codec.command(proposed, enabled=True)
    except ValueError:
      # NAP: a command that cannot be computed or encoded is a disabled frame, never an exception in card.
      self.was_active = False
      return self._disabled(CC, CS, feedback)
    if self.regen_decel_monitor.update(pedal_control_active=True, in_engage_grace=grace, pedal_di=proposed,
                                       limited_accel=self.vdas.jerk_limiter.a_limited,
                                       actual_accel=CS.out.aEgo, v_ego=CS.out.vEgo):
      CS.pccEvent = 'pedalMaxRegen'
    return [command]
