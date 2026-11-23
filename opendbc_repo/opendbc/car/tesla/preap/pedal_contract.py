"""Frozen working Dom pedal calibration and0x551/0x552 wire contract."""
from dataclasses import dataclass
import math
import struct

M1 = 0.050796813
M2 = 0.101593626
D = -22.85856576
FEEDBACK_TIMEOUT_NS = 500_000_000
# 0x552 rules, the same as the panda's (safety/modes/tesla_preap_pedal.h): channel agreement around the
# measured relationship, checksum, counter advance of 1 or 2, interceptor STATE 0 or 5 (idle timeout).
AGREE_SLOPE = 0.00844
AGREE_OFFSET = -14.17
AGREE_TOLERANCE = 4.0 / M1
MIN_RAW_ONE = 300
MIN_RAW_TWO = 150
STATE_OK = 0
STATE_IDLE = 5
STATE_SETTLE_NS = 100_000_000
# Enabled commands stay below the panda's absolute cap (raw 2500) with BogPilot's margin.
MAX_ENABLED_RAW = 2450


def float32(value):
  return struct.unpack('<f', struct.pack('<f', value))[0]


@dataclass(frozen=True)
class PedalCalibration:
  factor: float
  zero: float
  bus: int

  def valid(self):
    try:
      if (type(self.bus) is not int or self.bus not in (0, 2) or
          not math.isfinite(self.factor) or abs(self.factor) <= 1e-6 or not math.isfinite(self.zero) or
          self.factor != float32(self.factor) or self.zero != float32(self.zero)):
        return False
      # Preserve Dom's clipping at the electrical floor. A saturated endpoint
      # may not create incompatible values for the two physical channels.
      for di in (-5., 90.):
        one = max(0, min(65534, int((self.physical(di) - D) / M1)))
        two = max(0, min(65534, int((self.physical(di) - D) / M2)))
        if not 0 <= one - 2 * two <= 1:
          return False
      return True
    except (ValueError, TypeError, OverflowError, struct.error):
      return False

  def physical(self, di):
    return self.zero + di / self.factor

  def di(self, physical):
    return (physical - self.zero) * self.factor


def checksum(address, first_five):
  return ((address & 255) + (address >> 8) + sum(first_five)) & 255


def pedal_max(speed):
  bp = (0., 5., 12., 20., 30., 40.)
  values = (50., 58., 66., 74., 82., 90.)
  if speed <= 0:
    return values[0]
  for index in range(1, len(bp)):
    if speed <= bp[index]:
      ratio = (speed - bp[index - 1]) / (bp[index] - bp[index - 1])
      return values[index - 1] + ratio * (values[index] - values[index - 1])
  return values[-1]


class PedalCodec:
  def __init__(self, calibration):
    if type(calibration) is not PedalCalibration or not calibration.valid():
      raise ValueError('invalid frozen Tesla pedal calibration')
    self.calibration = calibration
    self.counter = 0

  def command(self, di=0., *, enabled=False):
    if type(enabled) is not bool or not math.isfinite(di):
      raise ValueError('invalid Tesla pedal demand')
    one = two = 0
    if enabled:
      if not -5. <= di <= 90.:
        raise ValueError('Tesla pedal DI outside original envelope')
      physical = self.calibration.physical(di)
      one = max(0, min(65534, int((physical - D) / M1)))
      two = max(0, min(65534, int((physical - D) / M2)))
      if one > MAX_ENABLED_RAW:
        one, two = MAX_ENABLED_RAW, MAX_ENABLED_RAW // 2
      if not 0 <= one <= 65534 or not 0 <= two <= 65534 or abs(one - 2 * two) > 1:
        raise ValueError('invalid dual-channel Tesla pedal command')
    prefix = struct.pack('>HHB', one, two, (int(enabled) << 7) | self.counter)
    self.counter = (self.counter + 1) & 15
    return 0x551, prefix + bytes([checksum(0x551, prefix)]), self.calibration.bus


class PedalFeedback:
  def __init__(self, calibration):
    if type(calibration) is not PedalCalibration or not calibration.valid():
      raise ValueError('invalid frozen Tesla pedal calibration')
    self.calibration = calibration
    self.last_counter = None
    self.source_ns = 0
    self.last_observed_ns = 0
    self.healthy = False
    self.state = None
    self.value = self.value2 = 0.
    self.torque_level = 0.
    # What this side last sent, to judge STATE 5 exactly as the panda does, and the panda's echo of it.
    self.tx_enabled = False
    self.tx_last_ns = self.tx_stream_ns = None
    self.echo_ns = 0

  def note_tx(self, enabled, now_ns):
    if self.tx_last_ns is None or now_ns - self.tx_last_ns > STATE_SETTLE_NS:
      self.tx_stream_ns = now_ns
    self.tx_last_ns = now_ns
    self.tx_enabled = bool(enabled)

  def note_echo(self, now_ns):
    self.echo_ns = now_ns

  def _state_valid(self, state, now_ns):
    if state == STATE_OK:
      return True
    if state != STATE_IDLE:
      return False
    commanding = (self.tx_last_ns is not None and now_ns - self.tx_last_ns <= STATE_SETTLE_NS and
                  now_ns - self.tx_stream_ns > STATE_SETTLE_NS)
    return not self.tx_enabled and not commanding

  def clear(self):
    self.healthy = False
    self.state = None
    self.source_ns = 0
    self.last_counter = None

  def observe(self, payload, bus, source_ns):
    if (type(payload) is not bytes or len(payload) != 6 or bus != self.calibration.bus or
        type(source_ns) is not int or source_ns < self.last_observed_ns):
      self.clear()
      return False
    self.last_observed_ns = source_ns
    one, two, flags = struct.unpack('>HHB', payload[:5])
    state, counter = flags >> 4, flags & 15
    physical_one, physical_two = one * M1 + D, two * M2 + D
    # STATE 5 is the interceptor's idle state: valid only while nothing is being commanded, as in the panda.
    if (checksum(0x552, payload[:5]) != payload[5] or not self._state_valid(state, source_ns) or one == 65535 or two == 65535 or
        one < MIN_RAW_ONE or two < MIN_RAW_TWO or
        abs((one - 2 * two) - (AGREE_SLOPE * one + AGREE_OFFSET)) > AGREE_TOLERANCE):
      self.clear()
      return False
    previous = self.last_counter
    self.last_counter = counter
    self.healthy = previous is not None and ((counter - previous) & 15) in (1, 2)
    self.state = state if self.healthy else None
    self.source_ns = source_ns if self.healthy else 0
    self.value = self.calibration.di(physical_one)
    self.value2 = self.calibration.di(physical_two)
    return self.healthy

  def ready(self, now_ns):
    return bool(self.healthy and self.source_ns > 0 and self.source_ns <= now_ns and
                now_ns - self.source_ns <= FEEDBACK_TIMEOUT_NS)

  def commandable(self, now_ns):
    """The interceptor is out of its idle timeout: an enabled command may follow."""
    return self.ready(now_ns) and self.state == STATE_OK

  def gas_pressed(self):
    # Channel 1, as both references and the panda. Channel 2 is checked against it in observe().
    return self.value > 2.
