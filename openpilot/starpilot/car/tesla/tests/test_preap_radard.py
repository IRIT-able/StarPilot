"""Pre-AP radard policy. Scenarios from NotAutopilot naponsp-alpha @ e702d13397
(openpilot/selfdrive/controls/tests/test_radard.py), adapted to StarPilot's RadarD constructor and the radarTracks service,
plus BogPilot's stuck-track filter reaching the lead and the non-pre-AP path staying untouched."""
from types import SimpleNamespace

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.radard import KalmanParams, RADAR_TO_CAMERA, RadarD
from openpilot.starpilot.car.tesla.preap_radard import RADAR_MEASUREMENT_TIMEOUT, Track, is_preap_radar

SERVICES = ["modelV2", "carState", "radarTracks"]


class RadarScenario:
  def __init__(self, v_ego: float = 20.0, *, preap: bool = True):
    self.sm = messaging.SubMaster(SERVICES, ignore_alive=SERVICES, ignore_avg_freq=SERVICES)
    self.radar = RadarD(0.0, radar_available=True, tesla_preap_radar=preap)
    self.v_ego = v_ego
    self.frame = 0

  def step(self, time_s, vision_d_rel, radar_points=None, vision_v=None, measured=True):
    messages = [self._model_message(time_s, vision_d_rel, vision_v), self._car_state_message(time_s)]
    if radar_points is not None:
      messages.append(self._radar_message(time_s, radar_points, measured))
    self.sm.update_msgs(time_s, [message.as_reader() for message in messages])
    self.radar.update(self.sm, self.sm["radarTracks"])
    self.frame += 1
    return self.radar.radar_state.leadOne

  def _model_message(self, time_s, vision_d_rel, vision_v):
    message = messaging.new_message("modelV2", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    message.modelV2.velocity.x = [self.v_ego]
    leads = message.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = 0.9
      lead.x = [vision_d_rel + RADAR_TO_CAMERA]
      lead.xStd = [3.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [self.v_ego if vision_v is None else vision_v]
      lead.vStd = [2.0]
      lead.a = [0.0]
    return message

  def _car_state_message(self, time_s):
    message = messaging.new_message("carState", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    message.carState.vEgo = self.v_ego
    return message

  @staticmethod
  def _radar_message(time_s, radar_points, measured):
    message = messaging.new_message("radarTracks", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    points = message.radarTracks.init("points", len(radar_points))
    for point, (track_id, d_rel, y_rel, v_rel) in zip(points, radar_points, strict=True):
      point.trackId = track_id
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.deprecated.measured = measured
    return message


def test_radar_silence_falls_back_to_current_vision_lead():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar

  model_dt = 0.05
  expiration_frame = int(RADAR_MEASUREMENT_TIMEOUT / model_dt) + 1
  for frame in range(1, expiration_frame):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)
  assert lead.radar

  lead = scenario.step(1.0 + expiration_frame * model_dt, vision_d_rel=36.0)
  assert not lead.radar
  assert not scenario.radar.tracks
  assert lead.dRel == pytest.approx(36.0)


def test_empty_radar_message_drops_tracks_at_once():
  scenario = RadarScenario()
  assert scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)]).radar
  lead = scenario.step(1.05, vision_d_rel=36.0, radar_points=[])
  assert not lead.radar and not scenario.radar.tracks
  assert lead.dRel == pytest.approx(36.0)


def test_held_estimate_is_dropped_half_a_second_after_its_last_measurement():
  # The receiver keeps publishing a slot as an unmeasured estimate; radard must not follow it past 0.5 s.
  scenario = RadarScenario()
  assert scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)]).radar
  lead = None
  for k in range(1, 12):
    lead = scenario.step(1.0 + k * 0.05, vision_d_rel=36.0, radar_points=[(7, 30.0, 0.0, 0.0)], measured=False)
    if k * 0.05 <= RADAR_MEASUREMENT_TIMEOUT:
      assert 7 in scenario.radar.tracks
  assert 7 not in scenario.radar.tracks and not lead.radar and lead.dRel == pytest.approx(36.0)


def test_a_track_never_starts_from_a_held_estimate():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)], measured=False)
  assert 7 not in scenario.radar.tracks and not lead.radar


def test_association_rejects_distance_outlier():
  lead = RadarScenario().step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])
  assert not lead.radar
  assert lead.dRel == pytest.approx(100.0)


def test_association_rejects_velocity_outlier():
  lead = RadarScenario().step(1.0, vision_d_rel=50.0, radar_points=[(7, 50.0, 0.0, 15.0)])
  assert not lead.radar
  assert lead.vLead == pytest.approx(20.0)


def test_association_requires_minimum_score():
  # Each residual is inside its own 3-sigma gate; the combined likelihood is below the score floor.
  assert not RadarScenario().step(1.0, vision_d_rel=50.0, radar_points=[(7, 58.0, 2.9, 5.5)]).radar


def test_association_retains_incumbent_until_challenger_wins():
  scenario = RadarScenario()
  two_tracks = [(81, 70.0, 0.0, 0.0), (82, 72.0, 0.0, 0.0)]
  vision_distances = [70.8, 71.2, 70.8, 71.2, 70.8, 71.2, 72.0, 70.8]
  selected_ids = [scenario.step(1.0 + frame * 0.1, vision_d_rel=d, radar_points=two_tracks).radarTrackId
                  for frame, d in enumerate(vision_distances)]
  selected_ids.append(scenario.step(1.8, vision_d_rel=70.0, radar_points=[two_tracks[0]]).radarTrackId)
  assert selected_ids == [81, 81, 81, 81, 81, 81, 82, 82, 81]


def test_unmeasured_track_does_not_update_kalman_state():
  track = Track(7, 20.0, KalmanParams(0.1))
  track.update(30.0, 0.0, 0.0, 20.0, True)
  track.update(30.0, 0.0, 1.0, 21.0, True)
  measured_state = (track.vLeadK, track.aLeadK)
  track.update(30.0, 0.0, 10.0, 30.0, False)
  assert (track.vLeadK, track.aLeadK) == pytest.approx(measured_state)


def test_kalman_uses_observed_radar_interval():
  scenario = RadarScenario()
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  scenario.step(1.08, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert scenario.radar.tracks[7].K_A[0][1] == pytest.approx(0.08)


def test_validity_includes_radar_tracks():
  # BogPilot: radarState is valid only with radarTracks alive, valid and at the stock minimum rate.
  services = ["modelV2", "carState", "radarTracks"]
  scenario = RadarScenario()
  scenario.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert scenario.radar.tesla_preap.valid(scenario.sm) and scenario.radar.radar_state_valid
  invalid = RadarScenario()
  invalid.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
  invalid.step(1.0, vision_d_rel=30.0)          # no radarTracks message yet: not valid
  assert not invalid.radar.radar_state_valid
  # As radard runs it (polling modelV2), the radarTracks floor is the stock 8 Hz; this radar sends 8.7-16.7 Hz.
  assert messaging.SubMaster(services, poll='modelV2').freq_tracker['radarTracks'].min_freq == 8.0


def test_stuck_track_is_withheld_from_the_lead():
  # BogPilot: range fixed at 17.25 m while ego goes 10 -> 18 m/s. The track is withheld after the 5 s window.
  scenario = RadarScenario(v_ego=10.0)
  lead = None
  seen_radar = False
  for k in range(0, 161):
    t = 1.0 + k * 0.05
    scenario.v_ego = 10.0 + 0.05 * k
    lead = scenario.step(t, vision_d_rel=17.25, radar_points=[(3, 17.25, 0.0, 0.0)], vision_v=scenario.v_ego)
    seen_radar = seen_radar or lead.radar
  assert seen_radar
  assert not lead.radar and 3 not in scenario.radar.tracks


def test_other_cars_keep_the_stock_radard_path():
  scenario = RadarScenario(preap=False)
  assert scenario.radar.tesla_preap is None
  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])
  # The stock association has no hard distance gate, so this is not rejected the pre-AP way.
  assert scenario.radar.tracks


def test_policy_only_selected_for_preap_with_radar():
  def cp(brand="tesla", fingerprint="TESLA_MODEL_S_PREAP", radar_unavailable=False):
    return SimpleNamespace(brand=brand, carFingerprint=fingerprint, radarUnavailable=radar_unavailable)
  assert is_preap_radar(cp())
  assert not is_preap_radar(cp(radar_unavailable=True))
  assert not is_preap_radar(cp(fingerprint="TESLA_MODEL_Y"))
  assert not is_preap_radar(cp(brand="honda"))


def test_low_speed_radar_only_lead_override_is_on_for_lead_one_only():
  # NAP keeps radard's low-speed override on pre-AP (BogPilot turns it off; the reviews leave the choice open): below
  # 4 m/s a close in-path radar track becomes leadOne even when the camera lead is farther away. Pinned as NAP.
  scenario = RadarScenario(v_ego=2.0)
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[(5, 10.0, 0.5, 0.0)], vision_v=2.0)
  assert lead.radar and lead.radarTrackId == 5 and lead.dRel == pytest.approx(10.0)
  assert not scenario.radar.radar_state.leadTwo.radar
  fast = RadarScenario(v_ego=6.0)
  lead = fast.step(1.0, vision_d_rel=40.0, radar_points=[(5, 10.0, 0.5, 0.0)], vision_v=6.0)
  assert not lead.radar and lead.dRel == pytest.approx(40.0)
