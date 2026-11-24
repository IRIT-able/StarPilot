"""Receiver paths the safety reviews listed as untested (radar reviews of 5e1ba5161, 1723915c6, 9debaab8f and later).

Time base without or with jumping CAN stamps, frame halves collected during a silence, a withdrawn target leaving no
internal slot, re-ID on a slot jump, held slots kept out of the gate's motion input, 0x501 length and loss, SensorDirty
and a frozen table after VALID, the frozen-table restart on HWFail, and the radar-feed flag decode.
"""
import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.preap.radar import SILENCE_DROP_S, PreAPRadarInterface
from opendbc.car.tesla.tests.test_preap_radar import frames, radar_interface, radar_params
from opendbc.car.tesla.tests.test_preap_radar_gate import DT, LAST_CYCLE, cycle, frozen_cycle, run, silent_calls

PACKER = CANPacker("tesla_radar_bosch_generated")
ALERT = 0x501
POINT0_A, POINT0_B = 784, 785


@pytest.fixture
def radar():
  ri = radar_interface(radar_params(), gate_open=False)
  ri.update([])
  assert run(ri, PACKER, 1.0, 6.0).points
  return ri


# *** U4: time base ***

def test_calls_without_can_stamps_advance_the_clock_ten_ms_each(radar):
  ri = radar
  quiet = int(SILENCE_DROP_S / 0.01) - 1          # 44 calls: 0.44 s since the last cycle
  out = [ri.update([(0, [(0x109, bytes(8), 129)])]) for _ in range(quiet)]
  assert all(rr is None for rr in out) and not ri.preap.silent and ri.preap.tracks.points
  out += [ri.update([(0, [(0x109, bytes(8), 129)])]) for _ in range(60 - quiet)]
  assert ri.preap.silent and not ri.preap.tracks.points and not ri.preap.gate.valid
  published = [rr for rr in out if rr is not None]
  assert published and all(not rr.points and not rr.errors.radarFault for rr in published)


def test_a_forward_jump_in_can_stamps_is_a_silence_at_once(radar):
  ri = radar
  rr = ri.update([(int((LAST_CYCLE + 0.6) * 1e9), [(0x109, bytes(8), 129)])])
  assert rr is not None and not rr.points and ri.preap.silent
  assert not ri.preap.tracks.points and not ri.preap.gate.valid and len(ri.preap.silence_starts) == 1
  assert not ri.update(cycle(PACKER, LAST_CYCLE + 0.7)).points, 'points must be re-earned after the jump'


def test_card_stall_then_burst_uses_the_newest_stamp(radar):
  ri = radar
  # card delivers nothing for 0.6 s, then two radar cycles in one call.
  burst = cycle(PACKER, LAST_CYCLE + 0.5) + cycle(PACKER, LAST_CYCLE + 0.6)
  rr = ri.update(burst)
  assert rr is not None and rr.points
  assert ri.preap.last_cycle_time == pytest.approx(LAST_CYCLE + 0.6)
  track = next(iter(ri.preap.tracks._slots.values()))
  assert track.measured_at == pytest.approx(LAST_CYCLE + 0.6), 'measurement age counts from the newest frame'
  # The receiver was not silent (cycles kept coming on the wire); radard's own 0.5 s limit covers the stall.
  assert not ri.preap.silence_starts


# *** U8 (N4): halves that trickle in during a silence ***

def test_frame_halves_received_during_a_silence_are_not_a_fresh_cycle(radar):
  ri = radar
  halves = [f for f in cycle(PACKER, 0.)[0][1] if f[0] in (POINT0_A, POINT0_B)]
  t = LAST_CYCLE + 0.125
  while t < LAST_CYCLE + 0.8:                    # past the drop threshold, halves keep arriving without the trigger
    ri.update([(int(t * 1e9), halves + [(0x109, bytes(8), 129)])])
    t += 0.01
  assert ri.preap.silent
  # Recovery: the trigger and status come back, slot 0's halves do not.
  recovery = [(stamp, [f for f in msgs if f[0] not in (POINT0_A, POINT0_B)]) for stamp, msgs in cycle(PACKER, t)]
  rr = ri.update(recovery)
  assert rr is not None and not rr.points
  assert not ri.preap.tracks.points and not ri.preap.tracks.observations()


# *** U9: a withdrawn target leaves no internal slot ***

@pytest.mark.parametrize("changes", ({"tracked": False}, {"probability": 46.875}, {"distance": 0.0}, {"distance": 251.0}))
def test_withdrawn_target_is_gone_from_the_receiver_tracks_too(changes):
  ri = radar_interface(radar_params())
  ri.update([])
  for tick in range(1, 5):
    ri.update([(tick * 125_000_000, frames(PACKER))])
  assert len(ri.preap.tracks.points) == 1
  result = ri.update([(625_000_000, frames(PACKER, **changes))])
  assert not result.points and not ri.preap.tracks.points and not ri.preap.tracks.observations()


# *** re-ID on a slot jump (NAP lifecycle; NAP test_preap_radar_e2e.py slot reuse) ***

def point_cycle(t, d_rel, v_rel):
  msgs = [PACKER.make_can_msg("TeslaRadarSguInfo", 1, {}), PACKER.make_can_msg("TeslaRadarAlertMatrix", 1, {}),
          (0x109, bytes(8), 129)]
  for i in range(32):
    msgs.append(PACKER.make_can_msg(f"RadarPoint{i}_A", 1, {"Tracked": i == 0, "Meas": 1, "Index": 1, "LongDist": d_rel,
                                                            "LongSpeed": v_rel, "ProbExist": 96.875}))
    msgs.append(PACKER.make_can_msg(f"RadarPoint{i}_B", 1, {"Index2": 1}))
  return [(int(t * 1e9), msgs)]


@pytest.mark.parametrize("d_rel,v_rel,same", [(39.5, -2.0, True), (40.5, -2.0, False), (30.0, 7.5, True), (30.0, 8.5, False),
                                              (20.5, -2.0, True), (19.5, -2.0, False)])
def test_slot_reassigned_to_another_object_gets_a_new_track_id(d_rel, v_rel, same):
  ri = radar_interface(radar_params())
  ri.update([])
  for tick in range(1, 4):
    first = ri.update(point_cycle(tick * DT, 30.0, -2.0))
  assert len(first.points) == 1
  first_id = first.points[0].trackId
  second = ri.update(point_cycle(4 * DT, d_rel, v_rel))
  assert len(second.points) == 1
  assert (second.points[0].trackId == first_id) == same
  assert second.points[0].dRel == d_rel and second.points[0].vRel == v_rel


# *** held slots are not motion evidence for the gate ***

def test_a_held_slot_is_not_fed_to_the_gate_as_a_moving_track(radar):
  ri = radar
  incomplete = [(int((LAST_CYCLE + DT) * 1e9), [f for f in cycle(PACKER, LAST_CYCLE + DT)[0][1] if f[0] != POINT0_A])]
  rr = ri.update(incomplete)
  assert len(ri.preap.tracks.points) == 1 and not ri.preap.tracks.points[0].deprecated.measured
  assert ri.preap.tracks.observations() == {}
  assert ri.preap.gate.slots == {}, 'the gate saw no observation this cycle'
  assert rr is not None


# *** 0x501 length and loss ***

def with_alert_length(packets, length):
  return [(stamp, [(a, d[:length], b) if a == ALERT else (a, d, b) for a, d, b in msgs]) for stamp, msgs in packets]


@pytest.mark.parametrize("length", (0, 5, 7))
def test_alert_matrix_of_the_wrong_length_never_opens_the_gate(length):
  ri = radar_interface(radar_params(), gate_open=False)
  ri.update([])
  t, rr = 1.0, None
  while t < 12.0:
    rr = ri.update(with_alert_length(cycle(PACKER, t, alert_every_cycle=True), length))
    t += DT
  assert not rr.points and not ri.preap.gate.valid and not ri.preap.gate.alert_seen


def test_alert_matrix_stopping_after_valid_keeps_the_last_state_and_a_later_bit_still_closes_the_gate(radar):
  # 0x501 has no timeout in BogPilot either (radar_interface.py lists it at nan); the radar sends it at 1 Hz.
  ri = radar
  assert run(ri, PACKER, 6.0, 12.0, alert=False).points
  assert not run(ri, PACKER, 12.0, 14.0 + DT, alert={"RADC_a012_espMIA": 1}).points
  assert ri.preap.gate.fault == "F2"


# *** SensorDirty and a frozen table after VALID ***

def test_sensor_dirty_after_valid_raises_the_error_on_every_cycle(radar):
  ri = radar
  t, seen = 6.0, []
  while t < 9.0:
    seen.append(ri.update(cycle(PACKER, t, status={"RADC_SensorDirty": 1})))
    t += DT
  # Same as both references: the gate stays VALID (it only latches off on a fault), the error goes to selfdrived.
  assert all(rr.errors.radarUnavailableTemporary and not rr.errors.radarFault for rr in seen)
  assert ri.preap.gate.valid


def test_frozen_table_with_sgu_fail_after_valid_is_still_a_radar_fault(radar):
  ri = radar
  faults = []
  for n in range(45):
    faults.append(ri.update(frozen_cycle(PACKER, 6.0 + n * DT)).errors.radarFault)
  assert not any(faults[:39]) and all(faults[39:]), faults


def test_hardware_fault_restarts_the_frozen_table_count():
  ri = radar_interface(radar_params(), gate_open=False)
  ri.update([])
  t = 1.0
  for _ in range(30):
    assert not ri.update(frozen_cycle(PACKER, t)).errors.radarFault
    t += DT
  hw = frozen_cycle(PACKER, t)
  hw[0][1][0] = PACKER.make_can_msg("TeslaRadarSguInfo", 1, {"RADC_SGUFail": 1, "RADC_HWFail": 1})
  assert ri.update(hw).errors.radarFault
  t += DT
  faults = []
  for _ in range(45):
    faults.append(ri.update(frozen_cycle(PACKER, t)).errors.radarFault)
    t += DT
  # NAP: the count restarts at the hardware fault, so a frozen-table fault needs 40 more frozen cycles.
  first = faults.index(True)
  assert 37 <= first <= 39 and all(faults[first:]), faults


# *** radar-feed flag ***

@pytest.mark.parametrize("param", (2, 3, 6, 7))
def test_every_radar_profile_expects_the_panda_feed_echo(param):
  cp = radar_params()
  cp.safetyConfigs[0].safetyParam = param
  assert PreAPRadarInterface(cp).gate.radar_emulation


def test_unreadable_safety_param_falls_back_to_expecting_the_echo():
  cp = radar_params()
  cp.init("safetyConfigs", 0)
  ri = PreAPRadarInterface(cp)
  assert ri.gate.radar_emulation and ri.radar_off_can
