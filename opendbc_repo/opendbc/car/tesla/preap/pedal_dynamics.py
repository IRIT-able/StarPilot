"""Original Dom preAP dynamics constants and per-drive torquezero owner."""
from math import isfinite

from numpy import clip

from opendbc.car.common.conversions import Conversions as CV

PEDAL_DI_MIN = -5.
PEDAL_DI_ZERO = 0.
ACCEL_MAX = 2.5
REGEN_MAX = -1.5
PEDAL_BP = [0., 5., 12., 20., 30., 40.]
PEDAL_MAX_VALUES = [50., 58., 66., 74., 82., 90.]
PEDAL_RAMP_RATE_UP = 5.
PEDAL_RAMP_RATE_DOWN = 2.5
# Zero-torque observations are only taken while the limited acceleration command is inside this band.
ACCEL_DEADBAND = 0.15  # m/s²
TORQUE_LEVEL_ACC = 0.
TORQUE_LEVEL_DECEL = -30.
ZERO_TORQUE_MIN_SPEED = 10. * CV.MPH_TO_MS
ZERO_TORQUE_SETTLE_UPDATES = 25  # 0.5s at 50 Hz, longer than the configured 0.4s actuator delay
ZERO_TORQUE_ADAPT_RATE = 0.1  # DI per update


class PedalZeroTorque:
  """Learns the pedal DI position that produces zero motor torque (NAP pedal/controller.py)."""

  def __init__(self):
    self.value = PEDAL_DI_ZERO  # start at coast, refine from real data
    self._target = PEDAL_DI_ZERO
    self._best_torque = TORQUE_LEVEL_DECEL
    self._settled_updates = 0

  def update(self, torque_level: float, current_pedal_di: float, v_ego: float, *,
             control_active: bool, accel_command: float):
    """Call every pedal frame with the current motor torque and pedal position."""
    observation_valid = (
      control_active
      and all(isfinite(value) for value in (torque_level, current_pedal_di, v_ego, accel_command))
      and v_ego >= ZERO_TORQUE_MIN_SPEED
      and abs(accel_command) < ACCEL_DEADBAND
    )
    if observation_valid:
      self._settled_updates += 1
    else:
      self._settled_updates = 0
      self._best_torque = TORQUE_LEVEL_DECEL

    # If torque is between decel and accel thresholds and closer to zero
    # than the best we've seen, this pedal position is near zero-torque
    if (self._settled_updates >= ZERO_TORQUE_SETTLE_UPDATES
        and TORQUE_LEVEL_DECEL < torque_level < TORQUE_LEVEL_ACC
        and abs(torque_level) < abs(self._best_torque)):
      self._target = current_pedal_di
      self._best_torque = torque_level

    # A previously accepted target cannot move the live feedforward anchor
    # across an authority or command boundary. Resume only after the new
    # coast observation has independently settled.
    if self._settled_updates >= ZERO_TORQUE_SETTLE_UPDATES:
      self.value = float(clip(
        self._target,
        self.value - ZERO_TORQUE_ADAPT_RATE,
        self.value + ZERO_TORQUE_ADAPT_RATE,
      ))

  def get(self, v_ego: float) -> float:
    """Returns the zero-torque DI value. Falls back to DI=0 at low speed."""
    if v_ego < 5.0 * CV.MPH_TO_MS:
      return PEDAL_DI_ZERO
    return self.value
