"""Pre-AP Bosch radar retrofit receiver.

Decoding, track lifecycle and error flags follow NotAutopilot naponsp-alpha @ e702d13397 (opendbc/car/tesla/radar_interface.py).
Point release is gated by BogPilot's RadarValidGate (fix/preap-all @ 9ed906a641), which NAP lacks: radar points are withheld
until the radar reports a clean alert matrix and live tracks, and again whenever it flags missing modules, a rejected
VIN/config, a hardware fault, or the panda stops feeding it. StarPilot's frozen startup context and frame-length filter stay.
"""
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

# BogPilot: after this many update calls (100 Hz) without the cycle trigger, tracks and gate validity are dropped.
MISSING_RADAR_FRAMES = 50
_RADAR_FEED_FLAG = 2


@dataclass(frozen=True)
class BoschTrackObservation:
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  measured: bool


@dataclass
class BoschTrack:
  point: structs.RadarData.RadarPoint
  missed_cycles: int = 0


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

  def update(self, slot: int, observation: BoschTrackObservation | None):
    track = self._slots.get(slot)
    if observation is None:
      if track is None:
        return
      track.missed_cycles += 1
      if track.missed_cycles > BOSCH_TRACK_MAX_MISSED_CYCLES:
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
    self.missing_calls = 0
    self.fallback_now = 0.0

    if self.radar_off_can:
      self.rcp = None
    else:
      messages = [(_BOSCH_RADAR_STATUS_MSG, 8), (_BOSCH_RADAR_ALERT_MSG, float('nan'))]
      for i in range(_BOSCH_RADAR_POINTS):
        messages.append((_BOSCH_RADAR_POINT_A_BASE + (i * _BOSCH_RADAR_POINT_STRIDE), 8))
        messages.append((_BOSCH_RADAR_POINT_B_BASE + (i * _BOSCH_RADAR_POINT_STRIDE), 8))
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

    if self.trigger_msg not in self.updated_messages:
      self.missing_calls += 1
      if self.missing_calls == MISSING_RADAR_FRAMES:
        # Radar went silent: never resume on stale tracks or a stale VALID.
        self.tracks.clear()
        self.table_freeze.reset()
        self.gate.valid = False
        self.gate.ok_since = None
        self.gate.last_move = None
        self.gate.update(now)
      return None
    self.missing_calls = 0

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    radar_status = self.rcp.vl[_BOSCH_RADAR_STATUS_MSG]
    hw_fail = bool(radar_status["RADC_HWFail"])
    sgu_fail = bool(radar_status["RADC_SGUFail"])
    dirty = bool(radar_status["RADC_SensorDirty"])
    ret.errors.radarFault = hw_fail
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
      self.tracks.update(i, observation)

    points = self.tracks.points
    # NAP: a frozen table together with SGUFail is a radar fault.
    if self.table_freeze.update(points) and sgu_fail:
      ret.errors.radarFault = True

    # BogPilot gate: points reach radard only while the radar is VALID.
    if _BOSCH_RADAR_STATUS_MSG in self.updated_messages:
      self.gate.on_sgu(now, int(radar_status["RADC_SGU_ITC"]), sgu_fail, hw_fail, dirty)
    self.gate.on_points(now, self.tracks.observations())
    ret.points = points if self.gate.update(now) else []

    self.updated_messages.clear()
    return ret

  def _parse_track(self, msg_a, msg_b):
    if msg_a["Index"] != msg_b["Index2"] or not msg_a["Tracked"]:
      return None
    if msg_a["LongDist"] <= 0.0 or msg_a["LongDist"] > 250.0 or msg_a["ProbExist"] < 50.0:
      return None
    return BoschTrackObservation(
      d_rel=msg_a["LongDist"],
      y_rel=msg_a["LatDist"] + self.radar_offset,
      v_rel=msg_a["LongSpeed"],
      a_rel=msg_a["LongAccel"],
      yv_rel=msg_b["LatSpeed"],
      measured=bool(msg_a["Meas"]),
    )
