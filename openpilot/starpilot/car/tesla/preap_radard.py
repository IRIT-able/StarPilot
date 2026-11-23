"""radard policy for the pre-AP Tesla Bosch radar retrofit.

Track update and lead association follow NotAutopilot naponsp-alpha @ e702d13397 (openpilot/selfdrive/controls/radard.py,
`_update_preap`, `LeadTrackAssociation`): the Kalman filter runs at the observed radar interval, a point the receiver holds
as an estimate (measured=False) does not update the filter, and tracks are dropped after 0.5 s without a new radarTracks
message.

Kept from BogPilot fix/preap-all @ 9ed906a641 where it is stricter than NAP: the stuck-track filter, which withholds a
track whose range stays fixed while ego speed changes, and radarState validity that includes radarTracks.
"""
from __future__ import annotations

import math
from typing import Any

import capnp

from opendbc.car.tesla.preap.radar_filters import StuckTrackFilter
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.simple_kalman import KF1D

_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1
V_EGO_STATIONARY = 4.
RADAR_TO_CAMERA = 1.52

# Bosch radar nominal cycle; the observed interval is used when it is plausible.
RADAR_DT = 1.0 / 8
RADAR_MEASUREMENT_TIMEOUT = 0.5

ASSOCIATION_DISTANCE_GATE = 3.0
ASSOCIATION_MIN_DISTANCE_STD = 1.0
ASSOCIATION_LATERAL_GATE = 3.0
ASSOCIATION_MIN_LATERAL_STD = 0.5
ASSOCIATION_VELOCITY_GATE = 3.0
ASSOCIATION_MIN_VELOCITY_STD = 1.0
ASSOCIATION_MIN_SCORE = 0.001
ASSOCIATION_SWITCH_MARGIN = 1.5


def is_preap_radar(CP) -> bool:
  return bool(CP.brand == "tesla" and CP.carFingerprint == "TESLA_MODEL_S_PREAP" and not CP.radarUnavailable)


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A, self.K_C, self.K_K = kalman_params.A, kalman_params.C, kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    self.measured_at = -math.inf  # no measurement yet: a track never starts from a held estimate

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: bool = True, kalman_params=None,
             now: float = 0.0):
    self.dRel, self.yRel, self.vRel, self.vLead = d_rel, y_rel, v_rel, v_lead
    self.measured = measured
    if measured:
      self.measured_at = now
    if measured and kalman_params is not None:
      state = self.kf.x
      self.K_A, self.K_C, self.K_K = kalman_params.A, kalman_params.C, kalman_params.K
      self.kf = KF1D(state, self.K_A, self.K_C, self.K_K)
    if self.cnt > 0 and measured:
      self.kf.update(self.vLead)
    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)
    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel), "yRel": float(self.yRel), "vRel": float(self.vRel), "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK), "aLeadK": float(self.aLeadK), "aLeadTau": float(self.aLeadTau.x),
      "present": True, "modelProb": model_prob, "radar": True, "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)


def laplacian_pdf(x: float, mu: float, b: float):
  return math.exp(-abs(x - mu) / max(b, 1e-4))


def association_score(v_ego: float, vision_d_rel: float, lead: capnp._DynamicStructReader, track: Track) -> float:
  return (laplacian_pdf(track.dRel, vision_d_rel, lead.xStd[0]) * laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0]) *
          laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0]))


def is_association_candidate(v_ego: float, vision_d_rel: float, lead, track: Track, score: float) -> bool:
  distance_limit = ASSOCIATION_DISTANCE_GATE * max(lead.xStd[0], ASSOCIATION_MIN_DISTANCE_STD)
  lateral_limit = ASSOCIATION_LATERAL_GATE * max(lead.yStd[0], ASSOCIATION_MIN_LATERAL_STD)
  velocity_limit = ASSOCIATION_VELOCITY_GATE * max(lead.vStd[0], ASSOCIATION_MIN_VELOCITY_STD)
  return (abs(track.dRel - vision_d_rel) <= distance_limit and abs(track.yRel + lead.y[0]) <= lateral_limit and
          abs(track.vRel + v_ego - lead.v[0]) <= velocity_limit and score >= ASSOCIATION_MIN_SCORE)


def match_vision_to_track(v_ego: float, lead, tracks: dict[int, Track], incumbent_track_id: int | None = None) -> Track | None:
  vision_d_rel = lead.x[0] - RADAR_TO_CAMERA
  scores = {track_id: association_score(v_ego, vision_d_rel, lead, track) for track_id, track in tracks.items()}
  eligible = [track_id for track_id, track in tracks.items()
              if is_association_candidate(v_ego, vision_d_rel, lead, track, scores[track_id])]
  if not eligible:
    return None
  challenger = max(eligible, key=scores.__getitem__)
  if incumbent_track_id in eligible and challenger != incumbent_track_id:
    if not scores[challenger] > scores[incumbent_track_id] * ASSOCIATION_SWITCH_MARGIN:
      return tracks[incumbent_track_id]
  return tracks[challenger]


class LeadTrackAssociation:
  def __init__(self, low_speed_override: bool):
    self.low_speed_override = low_speed_override
    self.incumbent_track_id: int | None = None

  def update(self, v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg, model_v_ego: float,
             vision_lead) -> dict[str, Any]:
    track = None
    if tracks and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks, self.incumbent_track_id)
    lead_dict: dict[str, Any] = {'present': False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob)
    elif ready and lead_msg.prob > .5:
      lead_dict = vision_lead(lead_msg, v_ego, model_v_ego, lead_msg.prob)
    if self.low_speed_override:
      low_speed_tracks = [candidate for candidate in tracks.values() if candidate.potential_low_speed_lead(v_ego)]
      if low_speed_tracks:
        closest_track = min(low_speed_tracks, key=lambda candidate: candidate.dRel)
        if (not lead_dict.get('present')) or (closest_track.dRel < lead_dict['dRel']):
          lead_dict = closest_track.get_RadarState()
    self.incumbent_track_id = lead_dict['radarTrackId'] if lead_dict.get('radar', False) else None
    return lead_dict


class PreAPRadarPolicy:
  def __init__(self):
    self.last_radar_update_time: float | None = None
    self.lead_one = LeadTrackAssociation(low_speed_override=True)
    self.lead_two = LeadTrackAssociation(low_speed_override=False)
    self.stuck_filter = StuckTrackFilter()

  def update_tracks(self, sm, rr, radard) -> None:
    from openpilot.selfdrive.controls.radard import KalmanParams
    now = 1e-9 * max(sm.logMonoTime.values())
    if sm.updated['radarTracks']:
      radar_time = 1e-9 * sm.logMonoTime['radarTracks']
      radar_dt = RADAR_DT
      if self.last_radar_update_time is not None:
        observed = radar_time - self.last_radar_update_time
        if 0.01 < observed < 0.2:
          radar_dt = observed
      self.last_radar_update_time = radar_time
      kalman_params = KalmanParams(radar_dt)
      points = {pt.trackId: (pt.dRel, pt.yRel, pt.vRel, bool(pt.deprecated.measured)) for pt in rr.points}
      # BogPilot: a fixed-range track while ego speed changes is not a road object.
      self.stuck_filter.prune(points.keys())
      points = {k: p for k, p in points.items() if not self.stuck_filter.update(k, p[0], p[2], radard.v_ego, radar_time)}
      for ids in list(radard.tracks.keys()):
        if ids not in points:
          radard.tracks.pop(ids, None)
      for ids, (d_rel, y_rel, v_rel, measured) in points.items():
        v_lead = v_rel + radard.v_ego_hist[0]
        if ids not in radard.tracks:
          radard.tracks[ids] = Track(ids, v_lead, kalman_params)
        radard.tracks[ids].update(d_rel, y_rel, v_rel, v_lead, measured, kalman_params, radar_time)
    elif self.last_radar_update_time is None or now - self.last_radar_update_time > RADAR_MEASUREMENT_TIMEOUT:
      # Never follow radar tracks older than 0.5 s, whatever the receiver published last.
      radard.tracks.clear()
    # The same limit per track, counted from its last real measurement: a slot the receiver is only holding
    # as an estimate cannot outlive it.
    for ids in [ids for ids, track in radard.tracks.items() if now - track.measured_at > RADAR_MEASUREMENT_TIMEOUT]:
      del radard.tracks[ids]

  @staticmethod
  def valid(sm) -> bool:
    # BogPilot: radarTracks counts (alive, valid, at least the stock 8 Hz). NAP leaves it out.
    return sm.all_checks()

  def leads(self, sm, radard, model_v_ego, vision_lead):
    leads_v3 = sm['modelV2'].leadsV3
    return (self.lead_one.update(radard.v_ego, radard.ready, radard.tracks, leads_v3[0], model_v_ego, vision_lead),
            self.lead_two.update(radard.v_ego, radard.ready, radard.tracks, leads_v3[1], model_v_ego, vision_lead))
