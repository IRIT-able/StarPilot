"""Pre-AP Bosch radar retrofit receiver.

Decoding, track lifecycle and error flags follow NotAutopilot naponsp-alpha @ e702d13397 (opendbc/car/tesla/radar_interface.py).
Point release is gated by BogPilot's RadarValidGate (fix/preap-all @ 9ed906a641), which NAP lacks: radar points are withheld
until the radar reports a clean alert matrix and live tracks, and again whenever it flags missing modules, a rejected
VIN/config, a hardware fault, or the panda stops feeding it. StarPilot's frozen startup context and frame-length filter stay.
"""
from collections import deque
from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.values import CANBUS, DBC
from opendbc.car.tesla.preap.radar_profile import qualified, valid_context
from opendbc.car.tesla.preap.radar_table_freeze import BoschTableFreezeWatch
from opendbc.car.tesla.preap.radar_valid import GTW_ECHO_ADDRS, GTW_ECHO_SRC, RadarValidGate

_BOSCH_RADAR_STATUS_MSG = 769
_BOSCH_RADAR_ALERT_MSG = 1281
_BOSCH_RADAR_POINT_A_BASE = 784
_BOSCH_RADAR_POINT_B_BASE = 785
_BOSCH_RADAR_POINT_STRIDE = 3
_BOSCH_RADAR_POINTS = 32
_BOSCH_TRIGGER_MSG = _BOSCH_RADAR_POINT_B_BASE + ((_BOSCH_RADAR_POINTS - 1) * _BOSCH_RADAR_POINT_STRIDE)
# TeslaRadarSguInfo carries 48 bits of signals. Retrofit units send it as 6 bytes; the generated DBC declares 8.
_BOSCH_RADAR_STATUS_LENGTHS = (6, 8)

BOSCH_TRACK_MAX_MISSED_CYCLES = 2
BOSCH_TRACK_MAX_DISTANCE_DELTA_M = 10.0
BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS = 10.0

# Radar silence (no cycle trigger), tiered. Measured on this car: the radar reboots itself now and then with
# the panda feed healthy (2 events in 10.1 h, 5.66 s and 5.68 s, init time identical to 0.3 ms); nothing else
# between 0.16 s and 1.8 s was ever seen mid-drive.
# Tracks and VALID dropped, empty error-free message: camera leads (BogPilot at 0.5 s). Slightly earlier here so the
# empty message is out before radard's own 0.5 s limits, and no radar data older than 0.5 s is ever in use.
SILENCE_DROP_S = 0.45
MAX_MEASUREMENT_AGE_S = 0.5     # a held (unmeasured) slot is never published past this age
SILENCE_WARN_S = 1.0            # driver warning, no disengage
SILENCE_FAULT_S = 8.0           # longer than one radar reboot: radarFault
SILENCE_REPEAT_WINDOW_S = 600.0  # a second silence inside this window: radarFault while both are inside it
SILENCE_REPEAT_COUNT = 2
MISSING_RADAR_PUBLISH_DIV = 5
_RADAR_FEED_FLAG = 2

STATUS_OK, STATUS_NOT_READY, STATUS_SILENT = 0, 1, 2


@dataclass(frozen=True)
class BoschTrackObservation:
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  measured: bool


# The radar itself reports the slot as not a target (untracked, out of range, low existence probability).
REJECTED = object()


@dataclass
class BoschTrack:
  point: structs.RadarData.RadarPoint
  missed_cycles: int = 0
  measured_at: float = 0.0


class BoschTrackLifecycle:
  """NAP track lifecycle: hold a slot through two missed cycles, new ID when a slot is reassigned."""
  def __init__(self):
    self._slots: dict[int, BoschTrack] = {}
    self._next_track_id = 0

  @property
  def points(self):
    return [self._slots[slot].point for slot in sorted(self._slots)]

  def observations(self):
    return {slot: (track.point.dRel, track.point.yRel, track.point.vRel)
            for slot, track in self._slots.items() if track.missed_cycles == 0}

  def clear(self):
    self._slots.clear()

  def update(self, slot: int, observation, now: float = 0.0):
    track = self._slots.get(slot)
    if observation is REJECTED:
      # BogPilot: a target the radar has withdrawn is dropped on the same cycle. The hold below is only for
      # a cycle whose frames did not all arrive.
      self._slots.pop(slot, None)
      return
    if observation is None:
      if track is None:
        return
      track.missed_cycles += 1
      if track.missed_cycles > BOSCH_TRACK_MAX_MISSED_CYCLES or now - track.measured_at > MAX_MEASUREMENT_AGE_S:
        del self._slots[slot]
      else:
        track.point.deprecated.measured = False
      return

    if track is None or self._is_discontinuous(track.point, observation):
      point = structs.RadarData.RadarPoint()
      point.trackId = self._next_track_id
      self._next_track_id += 1
      track = BoschTrack(point)
      self._slots[slot] = track

    track.missed_cycles = 0
    track.measured_at = now
    track.point.dRel = observation.d_rel
    track.point.yRel = observation.y_rel
    track.point.vRel = observation.v_rel
    track.point.deprecated.aRel = observation.a_rel
    track.point.deprecated.yvRel = observation.yv_rel
    track.point.deprecated.measured = observation.measured

  @staticmethod
  def _is_discontinuous(point, observation):
    return (abs(observation.d_rel - point.dRel) > BOSCH_TRACK_MAX_DISTANCE_DELTA_M or
            abs(observation.v_rel - point.vRel) > BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS)


class PreAPRadarInterface(RadarInterfaceBase):
  def __init__(self, CP, context=None, cp_bytes=None):
    super().__init__(CP)

    self.context_required = qualified(CP)
    self.radar_off_can = not valid_context(CP, context, cp_bytes) or Bus.radar not in DBC[CP.carFingerprint]
    self.updated_messages: set[int] = set()
    self.radar_offset = context.offset if not self.radar_off_can else 0.0
    self.tracks = BoschTrackLifecycle()
    self.table_freeze = BoschTableFreezeWatch()
    try:
      radar_feed = bool(int(CP.safetyConfigs[0].safetyParam) & _RADAR_FEED_FLAG)
    except (AttributeError, IndexError, TypeError, ValueError):
      radar_feed = True
    self.gate = RadarValidGate(radar_feed)
    self.fallback_now = 0.0
    self.start_time: float | None = None
    self.last_cycle_time: float | None = None
    self.silent = False
    self.silent_calls = 0
    self.silent_for = 0.0
    self.silence_starts: deque[float] = deque()

    if self.radar_off_can:
      self.rcp = None
    else:
      # Timeouts as BogPilot (10 periods): status 1.0 s, points 0.625 s. NAP lists both at 8 Hz (1.25 s).
      messages = [(_BOSCH_RADAR_STATUS_MSG, 10), (_BOSCH_RADAR_ALERT_MSG, float('nan'))]
      for i in range(_BOSCH_RADAR_POINTS):
        messages.append((_BOSCH_RADAR_POINT_A_BASE + (i * _BOSCH_RADAR_POINT_STRIDE), 16))
        messages.append((_BOSCH_RADAR_POINT_B_BASE + (i * _BOSCH_RADAR_POINT_STRIDE), 16))
      self.radar_message_addresses = frozenset(address for address, _ in messages)
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.vehicle)

    self.trigger_msg = _BOSCH_TRIGGER_MSG

  @staticmethod
  def _length_ok(address, data):
    if address == _BOSCH_RADAR_STATUS_MSG:
      return len(data) in _BOSCH_RADAR_STATUS_LENGTHS
    return len(data) == 8

  def _now(self, can_packets):
    self.fallback_now += 0.01
    stamps = [stamp for stamp, _ in can_packets if stamp]
    if stamps:
      self.fallback_now = max(stamps) * 1e-9
    return self.fallback_now

  def update(self, can_packets) -> structs.RadarDataT | None:
    if self.radar_off_can or self.rcp is None:
      result = super().update([])
      if result is not None and self.context_required:
        result.errors.canError = True
        result.errors.radarUnavailableTemporary = True
      return result

    now = self._now(can_packets)
    for _, frames in can_packets:
      for address, _, bus in frames:
        if bus == GTW_ECHO_SRC and address in GTW_ECHO_ADDRS:
          self.gate.on_echo(now)
          break

    # Only the configured Bosch receiver messages on the radar bus are length checked.
    # Preserve every unrelated bus/address packet unchanged.
    radar_packets = [(stamp, [(address, data, bus) for address, data, bus in frames
                             if bus != CANBUS.vehicle or address not in self.radar_message_addresses or self._length_ok(address, data)])
                     for stamp, frames in can_packets]
    vls = self.rcp.update(radar_packets)
    self.updated_messages.update(vls)

    if _BOSCH_RADAR_ALERT_MSG in vls:
      self.gate.on_alert_matrix(now, self.rcp.vl[_BOSCH_RADAR_ALERT_MSG])

    if self.start_time is None:
      self.start_time = now
    if self.trigger_msg not in self.updated_messages:
      # Evaluate the gate on every call, as BogPilot does: a lost-echo fault that starts and clears between
      # two radar cycles must still drop VALID.
      self.gate.update(now)
      self.silent_for = now - (self.start_time if self.last_cycle_time is None else self.last_cycle_time)
      if self.silent_for <= SILENCE_DROP_S:
        return None
      if not self.silent:
        # Radar went silent: never resume on stale tracks or a stale VALID. The frozen-table count is kept
        # (NAP), so intermittent silences cannot hide a frozen table.
        self.silent = True
        self.silent_calls = 0
        self.tracks.clear()
        self.updated_messages.clear()  # halves received before or during the silence are not a fresh cycle
        self.gate.valid = False
        self.gate.ok_since = None
        self.gate.last_move = None
        if self.last_cycle_time is not None:
          self.silence_starts.append(now)
      self.silent_calls += 1
      if (self.silent_calls - 1) % MISSING_RADAR_PUBLISH_DIV != 0:
        return None
      # BogPilot: an empty, error-free message, so radard drops its tracks and follows camera leads. A silence
      # longer than one radar reboot, or a repeated one, is a radar fault.
      silent = structs.RadarData()
      silent.errors.radarFault = self.silent_for > SILENCE_FAULT_S or self._repeated_silence(now)
      return silent
    self.silent = False
    self.silent_for = 0.0
    self.last_cycle_time = now

    ret = structs.RadarData()
    can_valid = self.rcp.can_valid  # read once: every read advances the parser's invalid-count debounce
    if not can_valid:
      ret.errors.canError = True

    radar_status = self.rcp.vl[_BOSCH_RADAR_STATUS_MSG]
    hw_fail = bool(radar_status["RADC_HWFail"])
    sgu_fail = bool(radar_status["RADC_SGUFail"])
    dirty = bool(radar_status["RADC_SensorDirty"])
    ret.errors.radarFault = hw_fail or self._repeated_silence(now)
    ret.errors.radarUnavailableTemporary = dirty
    if hw_fail:
      self.table_freeze.reset()

    for i in range(_BOSCH_RADAR_POINTS):
      msg_a_id = _BOSCH_RADAR_POINT_A_BASE + (i * _BOSCH_RADAR_POINT_STRIDE)
      msg_b_id = msg_a_id + 1
      observation = None
      # Matching one-bit indices can also match a cached previous cycle.
      # Require both halves since the last trigger before accepting an observation.
      if msg_a_id in self.updated_messages and msg_b_id in self.updated_messages:
        observation = self._parse_track(self.rcp.vl[msg_a_id], self.rcp.vl[msg_b_id])
      self.tracks.update(i, observation, now)

    points = self.tracks.points
    # NAP: a frozen table together with SGUFail is a radar fault.
    if self.table_freeze.update(points) and sgu_fail:
      ret.errors.radarFault = True

    # BogPilot gate: points reach radard only while the radar is VALID.
    if _BOSCH_RADAR_STATUS_MSG in self.updated_messages:
      self.gate.on_sgu(now, int(radar_status["RADC_SGU_ITC"]), sgu_fail, hw_fail, dirty)
    self.gate.on_points(now, self.tracks.observations())
    # No points while the parser is invalid, whatever the gate says (stricter than both references).
    ret.points = points if (self.gate.update(now) and can_valid) else []

    self.updated_messages.clear()
    return ret

  def _repeated_silence(self, now):
    while self.silence_starts and now - self.silence_starts[0] > SILENCE_REPEAT_WINDOW_S:
      self.silence_starts.popleft()
    return len(self.silence_starts) >= SILENCE_REPEAT_COUNT

  def status(self):
    """For the driver warning: 0 radar leads available, 1 points withheld (gate not VALID), 2 radar silent."""
    if self.radar_off_can:
      return STATUS_OK
    if self.silent and self.silent_for > SILENCE_WARN_S:
      return STATUS_SILENT
    return STATUS_OK if self.gate.valid else STATUS_NOT_READY

  def _parse_track(self, msg_a, msg_b):
    if msg_a["Index"] != msg_b["Index2"]:
      return None
    if not msg_a["Tracked"] or msg_a["LongDist"] <= 0.0 or msg_a["LongDist"] > 250.0 or msg_a["ProbExist"] < 50.0:
      return REJECTED
    return BoschTrackObservation(
      d_rel=msg_a["LongDist"],
      y_rel=msg_a["LatDist"] + self.radar_offset,
      v_rel=msg_a["LongSpeed"],
      a_rel=msg_a["LongAccel"],
      yv_rel=msg_b["LatSpeed"],
      measured=bool(msg_a["Meas"]),
    )
