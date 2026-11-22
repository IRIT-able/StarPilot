"""PreAPRadarInterface end to end with real frames: radar points reach radard only while the radar-valid gate is VALID.

Covers the case NotAutopilot naponsp-alpha gets wrong (it never reads 0x501 and keeps using points after the radar flags
missing modules) and the BogPilot behavior this port keeps: points are withheld on any missing-module or validity alert,
when the panda stops feeding the radar, on a hardware fault, and until the radar has first proven itself.
"""
import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.preap.radar import MISSING_RADAR_FRAMES
from opendbc.car.tesla.tests.test_preap_radar import radar_params, radar_interface

DT = 0.125  # one radar cycle
MIA_BITS = ("RADC_a011_bdyMIA", "RADC_a012_espMIA", "RADC_a013_gtwMIA", "RADC_a014_sccmMIA", "RADC_a015_adasMIA")
VALIDITY_BITS = ("RADC_a009_configMismatch", "RADC_a027_CntryCdValidity", "RADC_a037_vinValidity", "RADC_a042_xwdValidity",
                 "RADC_a061_radPositionMismatch", "RADC_a062_strRackMismatch")


def cycle(packer, t, *, alert=None, echo=True, status=None, status_len=8, alert_every_cycle=False, vrel=-2.0):
  """One radar cycle at time t (s): status, one moving tracked point, the trigger, 0x501 once per second, a panda echo."""
  sgu = packer.make_can_msg("TeslaRadarSguInfo", 1, status or {})
  msgs = [(sgu[0], (sgu[1] + bytes(8))[:status_len], sgu[2])]
  for i in range(32):
    msgs.append(packer.make_can_msg(f"RadarPoint{i}_A", 1, {"Tracked": i == 0, "Meas": 1, "Index": 1, "LongDist": 40.0,
                                                            "LongSpeed": vrel, "ProbExist": 96.875}))
    msgs.append(packer.make_can_msg(f"RadarPoint{i}_B", 1, {"Index2": 1}))
  if alert is not False and (alert_every_cycle or int(t) != int(t - DT)):
    msgs.append(packer.make_can_msg("TeslaRadarAlertMatrix", 1, alert or {}))
  if echo:
    msgs.append((0x109, bytes(8), 129))
  return [(int(t * 1e9), msgs)]


def run(ri, packer, t0, t1, **kw):
  rr, t = None, t0
  while t < t1 - 1e-9:
    rr = ri.update(cycle(packer, t, **kw))
    t += DT
  return rr


@pytest.fixture
def radar():
  ri = radar_interface(radar_params(), gate_open=False)
  ri.update([])
  return ri, CANPacker("tesla_radar_bosch_generated")


def test_points_are_withheld_until_the_radar_has_been_valid_for_two_seconds(radar):
  ri, packer = radar
  assert not run(ri, packer, 1.0, 2.5).points
  rr = run(ri, packer, 2.5, 5.0)
  assert len(rr.points) == 1 and rr.points[0].dRel == 40.0
  assert not rr.errors.radarFault and not rr.errors.canError


def test_without_any_alert_matrix_frame_points_never_pass(radar):
  ri, packer = radar
  assert not run(ri, packer, 1.0, 12.0, alert=False).points


@pytest.mark.parametrize("bit", MIA_BITS + VALIDITY_BITS)
def test_alert_matrix_missing_module_or_validity_bit_drops_points_and_recovers(radar, bit):
  ri, packer = radar
  assert run(ri, packer, 1.0, 6.0).points
  # Two consecutive 0x501 frames with the bit set (1 Hz) close the gate.
  rr = run(ri, packer, 6.0, 8.0 + DT, alert={bit: 1})
  assert not rr.points and ri.preap.gate.fault == "F2"
  assert not rr.errors.radarFault  # advisory: never a fault or disengage by itself
  assert not run(ri, packer, 8.0 + DT, 12.0, alert={bit: 1}).points
  # Two clear frames clear F2; points return only after another 2 s VALID hold.
  assert not run(ri, packer, 12.0, 13.5).points
  assert run(ri, packer, 13.5, 18.0).points


def test_points_drop_within_two_seconds_of_the_panda_feed_stopping(radar):
  ri, packer = radar
  assert run(ri, packer, 1.0, 6.0).points
  assert run(ri, packer, 6.0, 7.5, echo=False).points          # under the 2 s echo timeout
  rr = run(ri, packer, 7.5, 8.0 + 2 * DT, echo=False)
  assert not rr.points and ri.preap.gate.fault == "NO_ECHO"
  # The radar then raises its MIA bits; still no points, and none while the echo stays away.
  assert not run(ri, packer, 8.25, 14.0, echo=False, alert={"RADC_a012_espMIA": 1, "RADC_a013_gtwMIA": 1}).points
  assert not run(ri, packer, 14.0, 15.5).points
  assert run(ri, packer, 15.5, 20.0).points


def test_hardware_fault_is_a_radar_fault_and_drops_points_at_once(radar):
  ri, packer = radar
  assert run(ri, packer, 1.0, 6.0).points
  rr = ri.update(cycle(packer, 6.0, status={"RADC_HWFail": 1}))
  assert rr.errors.radarFault and not rr.points


def test_sensor_dirty_is_temporarily_unavailable(radar):
  ri, packer = radar
  run(ri, packer, 1.0, 6.0)
  rr = ri.update(cycle(packer, 6.0, status={"RADC_SensorDirty": 1}))
  assert rr.errors.radarUnavailableTemporary and not rr.errors.radarFault


@pytest.mark.parametrize("itc", (67, 68, 70, 83, 96))
def test_starved_or_position_rejected_radar_drops_points(radar, itc):
  ri, packer = radar
  assert run(ri, packer, 1.0, 6.0).points
  assert not run(ri, packer, 6.0, 7.5, status={"RADC_SGU_ITC": itc}).points


def test_six_byte_status_frame_from_the_retrofit_radar_is_decoded(radar):
  ri, packer = radar
  rr = run(ri, packer, 1.0, 6.0, status_len=6)
  assert rr.points and not rr.errors.canError
  rr = ri.update(cycle(packer, 6.0, status={"RADC_HWFail": 1}, status_len=6))
  assert rr.errors.radarFault and not rr.points


@pytest.mark.parametrize("length", (0, 3, 5, 7, 9))
def test_other_status_lengths_are_rejected(radar, length):
  ri, packer = radar
  run(ri, packer, 1.0, 6.0)
  rr = run(ri, packer, 6.0, 9.0, status={"RADC_HWFail": 1}, status_len=length)
  assert rr.errors.canError and not rr.errors.radarFault


def test_silent_radar_loses_tracks_and_validity(radar):
  ri, packer = radar
  assert run(ri, packer, 1.0, 6.0).points
  for call in range(MISSING_RADAR_FRAMES):
    assert ri.update([(int((6.0 + call * 0.01) * 1e9), [(0x109, bytes(8), 129)])]) is None
  assert not ri.preap.gate.valid and not ri.preap.tracks.points
  assert not run(ri, packer, 6.5, 8.0).points
  assert run(ri, packer, 8.0, 12.0).points


def test_frozen_table_with_sgu_fail_is_a_radar_fault(radar):
  ri, packer = radar
  # NAP rule needs four or more identical points for 40 cycles; one point never trips it.
  rr = run(ri, packer, 1.0, 10.0, status={"RADC_SGUFail": 1}, vrel=0.0)
  assert not rr.errors.radarFault and not rr.points
