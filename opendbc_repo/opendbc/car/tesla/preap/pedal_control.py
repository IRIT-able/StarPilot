"""Dom VirtualDAS pedal control with frozen calibration and current axis authority."""
import math
from opendbc.car.tesla.preap.pedal_contract import PedalCodec, pedal_max
from opendbc.car.tesla.preap.pedal_dynamics import PedalZeroTorque
from opendbc.car.tesla.preap.virtual_das import VirtualDAS


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

  def _disabled(self):
    self.previous_di = 0.
    self.vdas.reset()
    return [self.codec.command(enabled=False)]

  def update(self, CC, CS, frame, now_ns):
    if frame % 2:
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
    if active and not self.was_active:
      # Seed and grace start when control really starts (NAP), not when it was first asked for.
      self.engage_frame = frame
      self.previous_di = min(pedal_max(CS.out.vEgo), max(-5., feedback.value))
      self.vdas.reset(a_init=0., pedal_di_init=self.previous_di)
    self.was_active = active
    if not wanted:
      # Silent while idle, as logged on this interceptor: one disabled frame to release, then nothing.
      release = self.sending
      self.sending = False
      self.previous_di = 0.
      self.vdas.reset()
      return [self.codec.command(enabled=False)] if release else []
    self.sending = True
    if not active:
      return self._disabled()
    values = (CC.actuators.accel, CS.out.vEgo, CS.out.aEgo, *CC.orientationNED)
    if not all(math.isfinite(float(value)) for value in values):
      self.was_active = False
      return self._disabled()
    self.zero_torque.update(feedback.torque_level, self.previous_di, CS.out.vEgo)
    elapsed = frame - self.engage_frame
    grace = elapsed < 50
    demand = float(CC.actuators.accel)
    if grace:
      # Original50-control-frame positive engagement grace; no old generic tune rollback.
      demand = max(0., min(demand, max(0., CS.preap_accel_max) * elapsed / 50.))
    proposed = self.vdas.update(demand, CS.out.vEgo, self.previous_di, a_ego=CS.out.aEgo,
                                freeze_integrator=grace, orientation_ned=list(CC.orientationNED))
    # Apply hard domain last: original rate clamp could otherwise escape it
    # when an initial driver observation lay above the speed envelope.
    proposed = max(-5., min(pedal_max(CS.out.vEgo), proposed))
    self.previous_di = proposed
    if proposed <= -4.75 and not grace:
      CS.pccEvent = 'pedalMaxRegen'
    try:
      return [self.codec.command(proposed, enabled=True)]
    except ValueError:
      # NAP: a command that cannot be encoded is a disabled frame, never an exception in card.
      self.was_active = False
      return self._disabled()
