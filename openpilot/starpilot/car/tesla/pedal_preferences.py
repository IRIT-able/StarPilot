"""One complete frozen snapshot of the working Dom Pre-AP pedal settings."""
import math
from dataclasses import dataclass
from opendbc.car.tesla.preap.pedal_contract import D, M1, MAX_ENABLED_RAW, PedalCalibration, float32
from openpilot.starpilot.saved_source import read_saved

KEYS = ('NAPPedalEnabled', 'NAPPedalCalibDone', 'NAPPedalCanBus', 'NAPPedalCalibFactor', 'NAPPedalCalibZero')


@dataclass(frozen=True)
class PedalSettings:
  enabled: bool = False
  valid: bool = True
  calibration: PedalCalibration | None = None
  personality: int = 1


def read_pedal_settings(params):
  try:
    first = read_saved(params, 'NAPPedalEnabled', 8)
    if first in ((None, True), (b'0', True)):
      return PedalSettings()
    if first != (b'1', True):
      return PedalSettings(valid=False)
    # Detect concurrent multi-key edits; never read live settings in a controller.
    left = tuple(read_saved(params, name, 64) for name in KEYS)
    right = tuple(read_saved(params, name, 64) for name in KEYS)
    if left != right or not all(valid for _, valid in left) or left[0] != (b'1', True):
      return PedalSettings(enabled=True, valid=False)
    raw = {name: value for name, (value, _) in zip(KEYS, left, strict=True)}
    if raw['NAPPedalCalibDone'] != b'1':
      return PedalSettings(enabled=True, valid=False)
    bus_raw = raw['NAPPedalCanBus']
    if bus_raw not in (None, b'0', b'2'):
      return PedalSettings(enabled=True, valid=False)
    factor = float32(float(raw['NAPPedalCalibFactor'] if raw['NAPPedalCalibFactor'] is not None else b'1.0'))
    zero = float32(float(raw['NAPPedalCalibZero'] if raw['NAPPedalCalibZero'] is not None else b'0.0'))
    # Both references: the saved zero is the gas-onset point; 0 DI (coast) is one DI unit below it. They also
    # do not trust a calibration still at its default values (NAP).
    # BogPilot: a plausible factor (0.82-3.0) and a resting pedal below the command cap.
    if (factor == 1.0 and zero == 0.0) or not math.isfinite(factor) or not 0.82 <= factor <= 3.0:
      return PedalSettings(enabled=True, valid=False)
    calibration = PedalCalibration(factor, float32(zero - 1.0 / factor), 0 if bus_raw == b'0' else 2)
    if not calibration.valid() or (calibration.physical(0.) - D) / M1 >= MAX_ENABLED_RAW:
      return PedalSettings(enabled=True, valid=False)
    try:
      personality_raw, personality_valid = read_saved(params, 'LongitudinalPersonality', 64)
      personality = int(personality_raw) if personality_valid and personality_raw is not None else 1
    except (OSError, TypeError, ValueError, OverflowError):
      personality = 1
    return PedalSettings(True, True, calibration, personality if personality in (0, 1, 2) else 1)
  except (OSError, AttributeError, TypeError, ValueError, OverflowError):
    return PedalSettings(enabled=True, valid=False)
