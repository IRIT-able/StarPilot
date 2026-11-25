# Pre-AP Model S port notes (`preap-model-s`)

Car: 2014 Tesla Model S P85, pre-Autopilot, comma 3X. Pedal interceptor on bus 2 (0x551/0x552, sum checksum),
Bosch radar retrofit on bus 1 fed by the panda, newer SCCM retrofit.

## References

| Role | Repo and commit | Notes |
|---|---|---|
| Base | `StarPilotHQ/StarPilot` `testing` @ `0db704125` | Branch point. PRs target `testing`; it was 32 commits ahead of `stable`. |
| Primary | `NotAutopilot/openpilot` `naponsp-alpha` @ `e702d13397`, opendbc `81799e45e`, panda `649a6281f` | The commit named in `naponsp-alpha-prebuilt`'s `build.json`. The prebuilt's first commit (`5813362a4e`) has an opendbc tree identical to this pin. |
| Primary, device delta | `naponsp-alpha-prebuilt` @ `7eb09d2a7` | Five source-form commits on top of the build. Two are driving-relevant and neither is ported; see "NAP prebuilt-only changes". |
| Secondary | `IRIT-able/bogpilot-openpilot` `fix/preap-all` @ `9ed906a641` | Used to fill gaps and wherever it is stricter. The only branch with real drives on this car. |

## NAP prebuilt-only changes (`5813362a4..7eb09d2a7`)

| Commit | Change | Driving-relevant | In the port |
|---|---|---|---|
| `04c113384`, `757af40a3`, `c4739e370` | Upside-down radar option: `yRel` and `yvRel` negated when `/data/nap_params.json` has `radar_upside_down`; JSON re-read on change; reboot prompt in the UI | Yes, if the radar is mounted upside down | **No.** BogPilot `9ed906a641` has no such option and its logged drives on this car show radar leads matched to camera leads, so this radar is mounted the normal way up. |
| `2588137b8` | MADS waits up to 200 control samples (about 2 s) for the panda's first lateral grant before giving up; a loss after the grant is still immediate | Yes: first-pull engagement | **No direct port.** MADS does not exist here. StarPilot's equivalent wait is 200 ms (`car/tesla/preap/aol.py` `native_observation`), unchanged. If the first pull ever fails to engage on the car, this is the number to compare. |
| `04c113384` | Pedal calibration tool fix (`calibrate_pedal.py`: interceptor state only checked in one phase), panda heartbeat in the tool transport | No (offroad tool) | No. StarPilot has no calibration tool; it reads `NAPPedalCalibFactor` / `NAPPedalCalibZero`. |
| `04c113384`, merge `7eb09d2a7` | Radar VIN-learn tool (`vin_learn_radar.py`), settings UI | No (offroad tool) | No. |
| `04c113384` then `5700b9f16` | `qcomgpsd` added to selfdrived's ignored processes, then reverted | No (net zero) | No. |

So the reference for driving behavior is `e702d13397` with no prebuilt-only change carried over.

## Reference branch tips checked 2026-11-23

| Branch | Pinned | Tip now | What changed |
|---|---|---|---|
| BogPilot `fix/preap-all` | `9ed906a641` | `9ed906a641` | Nothing. The other pre-AP branches on that remote are ancestors of it or older lines. |
| NAP `naponsp-alpha` | `e702d13397` (opendbc `81799e45e`) | `78085a377` (opendbc `d1793c10`), three commits, 2026-09-08 to 09-10 | (1) Seat belt read from `SDM1` with a 1 s age limit instead of hard-coded latched: the port already does this. (2) Hands-on disengage level configurable through safety param bits 8-9, default still 2: the port uses 2. (3) "wait for initial lateral permission": the MADS change already listed under prebuilt-only. (4) Pedal calibration tool sessions reworked (when values are saved, output disabled before saving). The runtime pedal code, the calibration transform (`zero - 1/factor`), VirtualDAS and the panda's pedal rules are unchanged. |

Nothing in the port needs to change for these. Not ported: the configurable hands-on level.

## Evidence from logs recorded with faulty hardware

The owner reports that some logs were recorded with a radar that was not working and possibly a badly calibrated pedal.

- Radar: routes up to `0000001a` are the period in which the radar raised its VIN alert (not usable); from `0000001c` on it
  worked. Of the two mid-drive radar reboots, one (`00000009`) is from the faulty period, with the radar already reporting
  a hardware fault. With a working radar the evidence is **one** reboot in 4.8 radar-hours (`00000021`). The reboot
  signature and its 5.54 s init time are the same in both. The 8 s fault limit rests on that duration; the "2 silences in
  600 s" window rests on very little data.
- Pedal: everything the panda checks on `0x552` is raw sensor data and does not depend on the pedal calibration: the
  channel relationship (median residual within 0.11 units of the fitted line in each of the 43 routes, slopes 0.0074-0.0103),
  the sensor floors, checksum, counter and interceptor state. The calibration values in use during those drives are not
  recorded in the logs. Anything expressed in DI (gas pressed above 2 DI, the earlier "channel 2 reads as pressed on 1 % of
  frames") depends on a calibration that cannot be verified from the logs.
- The idle and engage wire sequence taken from the logs is also what BogPilot's current code does (silent while idle,
  disabled frames to wake the interceptor from its timeout or startup state, enabled frames only in state 0).

## Approach

StarPilot `testing` already had a pre-AP Model S stack (platform `TESLA_MODEL_S_PREAP`, safety mode 39). It is an older
adaptation of the NAP code to StarPilot's architecture (AOL instead of MADS, a USB startup capsule for pedal calibration).
Per the owner's decision (2026-11-21) there is one stack: StarPilot's existing pre-AP slot is updated to NAP's current
behavior and to this car, instead of adding a second platform. Nobody is known to drive StarPilot pre-AP today, so its
previous pre-AP behavior is not preserved where it conflicts with NAP. Non-pre-AP cars must be unaffected.

Safety rule used for every conflict: follow NAP's model; keep BogPilot's behavior where it is stricter; keep StarPilot's
own extra checks where this car's logs satisfy them. Anything removed is listed under "Checks removed".

## Findings on unmodified StarPilot (`0db704125`) against this car's logs

| Check | Result | Evidence |
|---|---|---|
| CAN fingerprint | No match on any route | 19 bus-0 addresses missing from the dict; GTW `0x227` is 5 bytes here, 4 in the dict |
| Pedal `0x552` two-channel agreement (limit 0.203) | 956 of 5,927 frames pass on `00000021` segs 0–1; median mismatch 0.71 | `safety/modes/tesla_preap_pedal.h` `tesla_preap_pedal_feedback`, `car/tesla/preap/pedal_contract.py` `PedalFeedback.observe` |
| Radar status `0x301` | Car sends 6 bytes; StarPilot DBC and parser filter expect 8 | `dbc/generator/tesla/tesla_radar_bosch.py` (`BO_ 769 ... 8`), `car/tesla/preap/radar.py` |
| Radar alert matrix `0x501` | Not parsed | `car/tesla/preap/radar.py` |
| Stalk `0x45` CRC and counter | CRC valid on 1,243/1,243 frames; counter continuous | StarPilot's checks can stay |
| Radar VIN feed | VIN fed on `0x2B9` equals the car's own `0x405` VIN on anchor drives | StarPilot's plain `0x405`→`0x2B9` copy works for the current radar pairing |

## Changes

### 1. Fingerprint (criterion 2)

- `opendbc_repo/opendbc/car/tesla/fingerprints.py`: second `TESLA_MODEL_S_PREAP` CAN fingerprint entry.
  Source: this car's logs (bus-0 union over all 44 routes, segment 0 plus a mid-drive segment), merged with the existing
  entry so addresses never seen in the sampled segments are still tolerated. `0x227` length 5.
- `opendbc_repo/opendbc/car/tesla/tests/test_preap_stock.py`: the uniqueness test now covers both entries.
- Reference handling: neither reference CAN-fingerprints this car. NAP forces the platform with the `NAPForcePreAP` param
  (`nap:openpilot/selfdrive/car/card.py:74`); its CAN dict is the same one StarPilot had. BogPilot detects it from a
  chassis signature plus "no Mobileye frames on bus 2" plus pedal present (`bog:opendbc_repo/opendbc/car/tesla_ap1/preap/detection.py`).
  StarPilot's pedal profile refuses fixed fingerprints (`car/tesla/preap/pedal_profile.py` `qualified`), so a real CAN
  match is required and a forced platform was not ported.
- Result: 44/44 routes fingerprint as `TESLA_MODEL_S_PREAP` from segment 0 and from a mid-drive start
  (`can_fingerprint` only; the FW-query half of `fingerprint()` needs a live car).

### 2. Radar retrofit receiver

Files: `opendbc_repo/opendbc/car/tesla/preap/radar.py` (rewritten), `preap/radar_valid.py` (new), `preap/radar_table_freeze.py` (new).

| Piece | Source | Notes |
|---|---|---|
| Track lifecycle: hold a slot through 2 missed cycles as an estimate, new track ID when a slot jumps > 10 m or > 10 m/s | NAP `opendbc/car/tesla/radar_interface.py` (`BoschTrackLifecycle`) | Replaces StarPilot's drop-on-first-miss. BogPilot also drops at once; NAP is the primary reference here. |
| Point filter (`Index == Index2`, `Tracked`, `0 < LongDist <= 250`, `ProbExist >= 50`, both halves fresh since the last trigger) | Same in all three | Unchanged |
| `RADC_HWFail` -> `radarFault` | All three | NAP's and BogPilot's "ignore HW fail" params are **not** ported (stricter). |
| `RADC_SensorDirty` -> `radarUnavailableTemporary` | NAP | StarPilot and BogPilot ignored it. |
| Frozen table (>= 4 points unchanged for 40 cycles) together with `RADC_SGUFail` -> `radarFault` | NAP `preap/radar_table_freeze.py`, copied unchanged | BogPilot only withholds points in this case. On the logged wrong-donor-VIN drives this raises `radarFault` from about 13-15 s after start, as NAP would. On healthy drives it never fires (0 cycles on `00000021`, `00000024`). |
| **Radar-valid gate**: points reach radard only while VALID | BogPilot `tesla_ap1/preap/radar_valid.py`, copied with behavior unchanged | **Gap fill where BogPilot is safer.** NAP (`e702d13397`) never reads `0x501`: `radar_interface.py` has no reference to it, and `preap/radar_donor_vin.py` only looks at the VIN bit to start a VIN read. So NAP keeps using points after the radar flags `espMIA`/`gtwMIA`/`sccmMIA`/`bdyMIA` or after the panda stops feeding it. That behavior is not carried over. |
| Radar silent (no cycle trigger), tiered | Owner decision 2026-11-23; BogPilot for the first tier | See "Radar silence" below. The gate is evaluated on every call, including calls without a radar cycle, as in BogPilot. The frozen-table count is not reset by a silence (NAP). |
| A target the radar itself withdraws (untracked, range outside 0-250 m, `ProbExist` < 50) is dropped on that cycle | BogPilot | NAP holds it two more cycles. The two-cycle hold is kept only for an incomplete cycle (a half missing, or halves whose index does not match). |
| No points while the parser is invalid | BogPilot | NAP publishes them with `canError`. |
| Status frame `0x301` accepted at 6 or 8 bytes | This car's logs (6 bytes); NAP's DBC says 6 | StarPilot's filter required 8, so status was never read. The shared generated DBC is left at 8 because the HW1 platform uses it too; all `0x301` signals sit in the first 48 bits. |
| Frozen startup context, per-address length filter, offset from the context | StarPilot | Kept. |

Not ported: NAP prebuilt's upside-down radar option. BogPilot `9ed906a641` has no such option and produced radar-matched leads on
this car, so the radar is mounted the normal way up.

Real-drive check (`work/probe/radar_replay.py`, logged CAN through the new receiver):

| Route | Condition | Result |
|---|---|---|
| `00000021` segs 3-6 | healthy | 2,436 of 2,537 cycles with points (BogPilot logged 2,472 of 2,641); no radar fault |
| `00000021`, `00000024` segs 0-1 | boot | PENDING, then NO_ECHO until the panda feed starts, then VALID; no radar fault |
| `00000012` | radar raising `vinValidity` | 0 points, gate F2 (`vinValidity`), same as BogPilot |
| `00000000` | radar never fed | 0 points, gate NO_ECHO then F1, `bdyMIA,espMIA,gtwMIA,sccmMIA` set |

Tests: `car/tesla/tests/test_preap_radar_valid.py` and `test_preap_radar_valid_bits.py` (BogPilot's gate tests, imports changed;
its tests of its own radar interface, card and pull FSM are replaced), `test_preap_radar_table_freeze.py` (NAP, unchanged),
`test_preap_radar_gate.py` (new, end to end). Existing StarPilot radar tests were changed where they asserted behavior this
port replaces: immediate track retirement (`test_preap_radar.py`, `test_preap_radar_receiver.py`) and immediate point
return after a hardware fault (`openpilot/starpilot/car/tesla/tests/test_radar_pipeline.py`).

Safety review of `5e1ba5161` found three blocking issues in the gate wiring and silence handling (gate only evaluated on
radar cycles; stale tracks left in radard during silence; silences resetting the frozen-table count). All three are fixed in
the following commit, with tests. Known remaining differences from that review: BogPilot's radard `StuckTrackFilter` is not
ported, and BogPilot drops a slot the radar marks invalid at once where the port (as NAP) holds it for two cycles.

Second review (of `1723915c6`) found two more blocking gaps in surrounding code; both are fixed in the next commit:

| Finding | Fix | Source |
|---|---|---|
| Parser timeouts were NAP's (status and points listed at 8 Hz = 1.25 s) where BogPilot's are shorter | Status listed at 10 Hz (1.0 s timeout), points at 16 Hz (0.625 s) | BogPilot `tesla_ap1/radar_interface.py` |
| BogPilot's `StuckTrackFilter` missing (a track whose range stays within 0.2 m with |vRel| <= 0.13 m/s for 5 s while ego speed changes >= 3 m/s is not a road object) | `car/tesla/preap/radar_filters.py`, copied; applied in radard | BogPilot `tesla_ap1/preap_radar_filters.py` |
| radard treated held (unmeasured) points as measurements and kept tracks as long as the receiver repeated them | `openpilot/starpilot/car/tesla/preap_radard.py`: NAP's pre-AP radard path. Kalman step from the observed radar interval, unmeasured points skip the filter update, tracks dropped 0.5 s after the last `radarTracks` message, vision-to-track association with distance/lateral/velocity gates and incumbent retention, `radarState` validity independent of the `radarTracks` rate | NAP `openpilot/selfdrive/controls/radard.py` (`_update_preap`, `LeadTrackAssociation`) |

`selfdrive/controls/radard.py` is shared code. The change is one constructor flag (`tesla_preap_radar`, default off) and three
`elif` branches behind it; `is_preap_radar(CP)` is true only for brand tesla, `TESLA_MODEL_S_PREAP`, radar available.
Tests: `openpilot/starpilot/car/tesla/tests/test_preap_radard.py` (NAP's eight radard scenarios adapted, plus empty-message
drop, stuck track withheld end to end, other cars unchanged), `car/tesla/tests/test_preap_radar_stuck_tracks.py` (BogPilot's).

Third review (of `7a8f3acf6`) found four blocking items, fixed in the commit after it:

| Finding | Fix |
|---|---|
| Withdrawn targets held for two cycles (NAP) where BogPilot drops at once | Dropped at once; see table above. The two tests that had been changed to expect the hold assert immediate retirement again (`test_preap_radar.py`); the incomplete-cycle hold keeps its own test (`test_preap_radar_receiver.py`). |
| `radarState` validity ignored `radarTracks` (NAP) | `sm.all_checks()` as BogPilot. As radard polls `modelV2`, the `radarTracks` rate floor is 8 Hz; this radar sends 8.7-16.7 Hz. |
| Radar `canError` only soft-disabled (StarPilot maps it to `radarFault`); both references disengage at once | `selfdrived.py`: for `TESLA_MODEL_S_PREAP` only, radar `canError` raises `canError` (immediate disable). Other cars unchanged. StarPilot's text for that event is "Unknown Vehicle Variant". |

### Radar silence

Cause, from the logs (`work/log-analyst/radar_gaps_all.csv`, all 45 radar routes, 10.1 h radar-active): the radar reboots
itself. Both mid-drive silences (`00000009` seg 19, 5.659 s; `00000021` seg 6, 5.681 s) show every radar frame stopping at
once, `0x631` (init sync) at 2 Hz for 5.54 s (the same to 0.3 ms in both), then the radar back with power-up state. The panda
feed was complete and byte-correct before and during both (no gap over 11 ms on a 100 Hz source, echoes equal to sources);
bus 0 had no errors; the radar raised no missing-module bit beforehand. It is not a panda feed gap. What makes the radar
reset (its firmware, its supply, interference) cannot be told from the logs. There is no other mid-drive gap between 0.16 s
and 1.8 s anywhere; boot gaps (1.8-2.2 s, before the feed starts) are the only other ones.

| Silence | Behavior | Value from |
|---|---|---|
| > 0.45 s | Tracks and VALID dropped, empty error-free `radarTracks` every 5th call: radard drops its tracks and follows camera leads. No error. | BogPilot |
| > 1.0 s | `starpilotCarState.preapRadarStatus = 2`: "Radar Silent / Using Camera Lead", warning only. Status 1 ("Radar Not Ready") while the gate is closed for any other reason, as BogPilot's `preapRadarNotReady`. | Owner |
| > 8.0 s (owner accepted 2026-11-23) | `radarFault` (soft disable, no entry) until the radar is back. Also applies to a radar that never speaks after startup. | One reboot is 5.66-5.68 s; 8 s is 40 % above it. |
| 2 silences within 600 s (owner accepted 2026-11-23) | `radarFault` while both are inside the window. | Observed: never more than one per route, 0.2 per radar hour. |

After a silence the points return only when the gate has been VALID-eligible for 2 s again.

No radar data older than 0.5 s is used anywhere (fourth review): the drop starts at 0.45 s so the empty message is out
before radard's own 0.5 s limit; a slot the receiver holds as an estimate is dropped once its last real measurement is
0.5 s old (receiver and radard both enforce it, neither reference does); frame halves received before a silence are not
counted as a fresh cycle after it.

Radar errors reach selfdrived for this car whenever the radar retrofit is configured, with or without pedal
longitudinal (NAP; BogPilot and StarPilot only with openpilot longitudinal): `canError` -> immediate disable, `radarFault`
-> soft disable. Test: `openpilot/starpilot/car/tesla/tests/test_preap_radar_events.py`.
`preapRadarStatus @40` is a new `StarPilotCarState` field, registered in `fixtures/custom_reviewed_schema_additions.json`.

### 3. Panda safety (mode 39): pedal interceptor checks and reference tightenings

Files: `opendbc_repo/opendbc/safety/modes/tesla_preap_pedal.h`, `tesla_preap.h`, `tesla_preap_radar.h`.
Evidence for every number: all 600 logged segments, 1,703,610 `0x552` frames (`work/probe/pedal_w.py`, `pedal_w_all.jsonl`).
Pedal travel in those logs runs from raw 416 to 2543 on channel 1 = 21.1 to 129.2 physical units, 108 units rest to floor.

Relaxed from StarPilot `0db704125` (owner approved 2026-11-23). Each is still stricter than NAP and BogPilot, which check
nothing on `0x552` beyond a timeout and read channel 1 only.

| # | StarPilot before | This car | Now | Still caught (test) |
|---|---|---|---|---|
| W1 | Channels must agree within 0.203 units (0.19 % of travel) | Difference -3.35 to +2.49 units. It depends weakly on position: median -0.51 at rest, about 0 at mid travel. Fitted: `one - 2*two = 0.00844*one - 14.17` raw counts; every frame within 2.98 units of the fit (sd 0.86) | Agreement checked against that fitted line, tolerance **4.0 units = 3.7 % of pedal travel** | Open (raw 0), shorted (4095, 65534, 65535) or stuck on **either** channel, engaged or idle: `test_open_or_shorted_channel_on_either_side_disengages`, `test_stuck_channel_on_either_side_disengages_within_the_tolerance` (trips after 4.0 units of pedal movement), `test_agreement_boundary_around_the_measured_relationship` |
| W2 | Gas pressed if either channel > 2.0 DI | Channel 2's offset read as pressed at rest on about 1 % of frames | Driver demand from channel 1 (both references); channel 2 only in the W1 check, which stays | `test_channel2_offset_is_not_driver_demand_but_channel1_is` |
| W3 | Counter must be previous + 1 | 135 breaks, every one a single missing frame (advance 2, 35-48 ms apart). None at the 557 segment joins: they are on the bus, not a logging artifact. 132 in the first 15 s after boot, 3 mid-drive, all while idle | Advance 1 **or 2** accepted. The owner approved 1-3; 3 was never seen, so it stays rejected | Repeated (0), 3, 4, 8, 15: `test_repeated_or_jumping_counter_disengages`; frozen frame: `test_frozen_frame_replayed_forever_disengages`; stream stopped (200 ms, StarPilot's existing rule): `test_stalled_interceptor_withdraws_longitudinal_and_lateral` |
| Idle | `STATE` must be 0 | The interceptor reports `STATE` 5 (command timeout) whenever nothing commands it; BogPilot sent nothing on `0x551` while idle | `STATE` 5 accepted only while not commanding: never after an enabled command, and not once a `0x551` stream is older than 100 ms. An enabled command needs the latest `STATE` to be 0. `STATE` 5 withdraws the longitudinal permission until a new disabled frame | `test_silent_idle_keeps_lateral_and_engages_with_the_logged_sequence`, `test_state5_while_a_command_stream_is_running_is_a_fault`, `test_state5_after_an_enabled_command_is_a_fault`, `test_any_other_interceptor_state_is_a_fault` |

Idle wire behavior chosen: **silent** (nothing on `0x551` while not engaged), which is what the logged drives did: 13
engagements, each `disabled-zero` frames 20 ms apart, `STATE` 5 -> 0 within 4-45 ms, then enabled frames, a disabled frame at
the end, and `STATE` back to 5 after 196-247 ms. StarPilot's alternative (disabled frames at 50 Hz all the time) appears in
no log from this interceptor, so it is not equally proven.

Known limit of W1 + W2, accepted by the owner on 2026-11-23 (found by the independent review): with channel 1 stuck, the
driver's press is seen only through the agreement check, after 4.0 units of pedal travel (3.7 %) from a centered pair and
up to 8.0 units (7.4 %) if the pair sat at the far edge of the band when it stuck
(`test_stuck_channel_from_anywhere_in_the_band_is_caught_within_twice_the_tolerance`). Before the change that was about
0.4 units, on a check this interceptor could never pass. Both references never detect it. With channel 2 stuck, channel 1
still reports gas at 2 DI as normal.

A failed check sets `gas_pressed`, clears `controls_allowed` and the physical arm, and needs a fresh stalk pull, as before.

Tightened to match or exceed both references:

| Change | Source | Test |
|---|---|---|
| Both sensors must read at least raw 300 (channel 1) and 150 (channel 2); the lowest logged readings are 416 and 203. A pair that drops toward zero together (lost or sagging sensor supply) still agrees with itself, so the agreement check alone accepted it, as StarPilot's did before | Review finding | `test_both_channels_low_together_is_a_fault` |
| `0x552` byte 5 checksum (`0x52 + 0x05 + bytes 0..4`) validated; it was unchecked. Valid on all 1.7 M logged frames | This car | `test_bad_feedback_checksum_disengages` |
| `DI_torque2.DI_brakePedal` (`0x118` byte 1 bit 7) is a second brake source, ORed with `0x20A` | Both references | `test_either_brake_source_withdraws_and_one_releasing_does_not_clear_the_other` |
| Hands-on level 2 disengages (was 3) | NAP | `test_hands_on_level_2_disengages` |
| Steering: BogPilot's per-frame table (8/4/2.5 deg up, 9/5/4.5 down at 2/7/17 m/s) and absolute 360 deg cap, on top of the vehicle-model limits | BogPilot | `test_low_speed_steering_rate_follows_the_table`, `test_absolute_angle_cap_is_360_deg` |
| Enabled pedal command capped at raw 2500 on channel 1, on top of the calibrated bounds | BogPilot | `test_enabled_command_above_raw_2500_is_rejected_whatever_the_calibration_allows` |
| `0x2A9` byte 7 left as the patched config byte (StarPilot overwrote it with a sum) | Both references; the logged `0x2A9` ends in `0x10` | `test_tesla_preap_radar.py::test_mount_and_counter_transforms_have_independent_literal_bytes` (expected bytes changed from `...f0`/`...00` to `...10`) |

Each test was checked by loosening its rule in a scratch copy and confirming the test fails.

Not ported on purpose (StarPilot is stricter without them): donor-VIN config `0x560`, radar UDS `0x641`, bus-0 `0x2B9`,
`0x3E9`, hands-on pause, the panda pedal-calibration mode, NAP's "brake does not drop controls".

### 4. Car side: pedal, engagement, longitudinal control

Files under `opendbc_repo/opendbc/car/tesla/preap/` unless noted.

| Change | Source | Notes |
|---|---|---|
| `pedal_contract.py` `PedalFeedback`: same `0x552` rules as the panda (section 3) | This car | `commandable()` is true only with `STATE` 0; `STATE` 5 is healthy idle. Gas from channel 1. |
| `pedal_control.py`: nothing on `0x551` while idle; disabled frames while pedal cruise is wanted but not yet active (interceptor leaves its timeout, panda can grant); enabled frames only with `STATE` 0; one disabled frame to release | Logged BogPilot drives (13 engagements) | StarPilot streamed disabled frames at 50 Hz all the time. The disabled preamble lasts until the panda grants (2-3 frames in the logs); it can last longer here, for example while selfdrived is not enabled. During gas or brake the wire is silent. |
| Calibration zero = saved value - 1/factor; a calibration still at the defaults (factor 1.0, zero 0.0) is refused | Both references | `openpilot/starpilot/car/tesla/pedal_preferences.py`. StarPilot used the saved value directly, which put every command 1 DI higher. |
| Stock cruise cancelled when pedal cruise starts or stops and on every stalk press | NAP | A single pull with DI cruise armed could leave the car's own cruise engaged. |
| Brake gate: a pull with the brake held gives lateral only; the brake level (not edge) drops pedal cruise; a new double pull is needed | NAP | `engagement.py` |
| Double-pull window 400 ms (was 500) | NAP | The panda's own window stays 500 ms. |
| Brake = `DI_brakePedal` or `driverBrakeStatus`; hands-on level 2 disengages; `vEgoCluster` from `DI_digitalSpeed` | NAP (brake: both) | `stock.py`. Published `brakePressed` also stays true for any `driverBrakeStatus` other than not-applied (StarPilot). |
| Steering command clamped to BogPilot's table and 360 deg before sending | BogPilot | Evaluated at the actual speed, the panda at speed - 1 m/s, so a clamped command always passes. |
| `virtual_das.py`, `ff_table_default.py`, `constants.py`, `pedal_dynamics.py`: NAP's VirtualDAS, feedforward table, constants and zero-torque learner | NAP, copied; only the plumbing differs (no `nap_conf` singleton, zero-torque object injected) | Outer `ki` is now zero; the 7-point accel tables apply. |
| Engage: grace timed from the real control start, 0.9 DI/update take-up, `observe()` while inactive, regen under-delivery monitor | NAP `carcontroller.py` | Kept from StarPilot: final clamp to [-5, max(v)], finite-input checks, the wanted/active gates. NAP's seed at `max(feedback, zero-torque)` was replaced; see the sixth-round table below. |
| Adaptive accel ratio uses `dRel + vLead^2/5` | NAP planner | `openpilot/starpilot/car/tesla/preap_adaptive.py`; StarPilot's freshness gates kept. |

Added after the car-side safety review (fifth review, 2026-11-23):

| Finding | Fix | Source |
|---|---|---|
| No failure latch: a command error or a failed takeover was retried every 20 ms without a fault | `pedal_control.py`: any exception, a non-finite input, an interceptor that does not leave its timeout state, or commands that do not come back from the panda latch `failed`; `CS.pedal_command_failed` makes carstate treat it as lost feedback (`accFaulted`, engagement reset, new pull needed) | NAP (latch), StarPilot (fault the engagement) |
| The car accepted `STATE` 5 frames its panda rejects | `PedalFeedback` records what the controller sent and applies the panda's rule (never after an enabled command, not on a stream older than 100 ms) | Panda rule |
| Commands the panda would reject, after which its counter check rejects everything | A disabled frame is sent before the first enabled one after every period in which pedal cruise was not wanted (gas, brake, faults); the engage seed is capped at measured pedal + 4 DI (panda: + 5; superseded in the sixth round below); the encoder clamps enabled commands at raw 2450 (panda: 2500) | Panda rules; BogPilot (2450) |
| Blocked `0x551` frames went unnoticed | The panda returns every frame it sent (bus + 128); enabled commands with no return for 100 ms latch the failure | BogPilot's intent, StarPilot's transport |
| Calibration accepted any factor | Factor must be 0.82-3.0 and the resting pedal below the command cap | BogPilot |
| Engagement dropped pedal cruise only on the narrow brake signal | The FSM gets the published `brakePressed` (DI bit, or brake status anything but "not applied") | BogPilot |

Added after the sixth review round (2026-11-23, the reviews' non-blocking items treated as required):

| Finding | Fix | Evidence |
|---|---|---|
| The command ceiling used the filtered `vEgo`; the panda bounds each enabled frame by the speed table at its newest raw `ESP_B` speed (0x155) with 1 count of slack | `pedal_control.command_ceiling(vEgoRaw)` = speed table at `vEgoRaw - 0.5 m/s`. The car's raw sample can be up to two `ESP_B` frames (40 ms) older than the panda's; 0.5 m/s covers 12 m/s² over that time and costs at most 0.8 DI of ceiling | `test_commands_at_the_ceiling_while_decelerating_hard_are_never_rejected` (5 m/s² at the ceiling, the car one frame behind, nothing rejected; it fails with `vEgo` or with no margin) |
| The first enabled frame (seed capped at feedback + 3.0, plus one 0.9 DI ramp step) is rejected by the panda under this interceptor's rest noise. The panda checks it against its own newest 0x552 on both channels (+5 / -2.5 DI), which can be one frame newer than the car's. | The first enabled frame goes out unramped at min(NAP's seed max(measured, coast point), measured + 1.5 DI); VirtualDAS takes over from the next frame. It is never above NAP's seed. At speed with a coast point well above the pedal it starts lower and the grace ramp climbs at 0.9 DI per update. With the pedal above the coast point it is the measured pedal itself, where NAP's ramped first frame can be up to 2.5 DI lower; the gap is at most 2 DI (an enabled frame needs the pedal at or below 2 DI) for one 20 ms frame, and equals the driver's own pedal at that moment | Logged rest noise, 14,503 consecutive 0x552 pairs (`00000021`, `00000024`, `0000002a`; all working-hardware routes): channel 1 moves up to 67 counts (3.4 units) between frames, and channel 2 scatters around the fit independently. Share of pairs on which a first frame at feedback + c is rejected, by calibration factor: |

| Factor | c = 0 | +1.0 | +1.5 | +2.0 | +3.0 | +3.9 (old) |
|---|---|---|---|---|---|---|
| 1.0 | 0.15 % | 0.03 % | 0.03 % | 0.06 % | 1.3 % | 13.9 % |
| 1.25 | 2.5 % | 0.09 % | 0.09 % | 0.21 % | 5.5 % | 20.2 % |
| 1.5 | 9.0 % | 0.52 % | 0.37 % | 1.3 % | 9.8 % | 25.0 % |
| 2.0 | 23.7 % | 8.6 % | 5.5 % | 7.1 % | 16.5 % | 30.3 % |

Replaying 1,790 consecutive logged rest frames (`opendbc/safety/tests/tesla_preap_pedal_rest_frames.json`, raw channel counts only) through the car controller and the panda hooks, with the panda one frame ahead, rejects nothing over about 35 engagements at factor 1.0 and 1.25. These rates count only the cases where the panda already has a newer frame than the car when the first enabled frame
arrives (likely 30-50 % of engagements). A rejected first frame applies no throttle, and the echo watchdog ends the
engagement within about 120 ms, so the cost is a nuisance fault, not a safety problem. Above a factor of about 2 the
panda's first-frame window (StarPilot's own check, in neither reference) is narrower than this interceptor's noise
whatever the car does. **The owner's calibration factor is not in the logs.** If it is above about 1.5, engagements will
sometimes fault at once; widening that panda window would be the fix, and that is the owner's decision.

`opendbc/safety/tests/test_tesla_preap_pedal_faults.py::test_everything_the_car_controller_sends_is_accepted_by_its_panda`
runs the real controller against the registered safety hooks, including a short driver press and a learned zero-torque
point far above the resting pedal. Changed expectation: NAP's `test_engage_reset_starts_estimator_from_measured_acceleration`
assumed the first controller update is the engage; with the disabled frame first the estimator is already warm, so the test
asserts continuity and keeps the cold-start seeding assertion on VirtualDAS itself.

Added after the closed-loop replay of card, radard, selfdrived and controlsd (seventh round, 2026-11-24, `c97447780`):

| Finding | Fix | Evidence |
|---|---|---|
| openpilot was never available on this car: the panda reports `interruptRateCan2` (FDCAN2, the radar bus), latched from boot, and StarPilot's selfdrived makes any panda fault not exempted for the platform `controlsMismatch`. NAP and BogPilot ignore panda faults except `relayMalfunction`. The CAN interrupt-rate limit (16000/s) is the same in StarPilot, BogPilot and NAP firmware | `starpilot/aol/panda_faults.py`: that bit alone is diagnostic for the admitted single-panda model-39 profiles (stock 0/2/6 and pedal 1/3/7), as StarPilot already does for named Honda, Hyundai and GM profiles. Every other fault, a mix of faults, and the interrupt rate of any other bus still block | All 600 logged segments report only this fault (99.8 % of frames). With the logged faults kept the port never engaged before the change and engages on all 13 logged engagements after it. `test_preap_panda_faults.py` (every fault type alone and mixed, profile mutations) |
| Releasing the gas while engaged was an immediate disable ("TAKE CONTROL IMMEDIATELY", 11 of 13 logged engagements). The panda drops its LONG bit on gas and keeps it off until a fresh disabled `0x551` after any interceptor STATE 5 frame; both edges reach the panda before card publishes them, and pandad's receipt is read at 10 Hz (`pandad.cc`), so selfdrived saw "LONG requested, not allowed, no override" for up to 100 ms after a release, or for one frame after the interceptor's wake frame | `selfdrived.ordinary_native_long_withdrawal`: a LONG-only denial on the admitted pedal profile (AE 32) is an ordinary withdrawal, as it already is for Ford, Honda, GM, Toyota and Hyundai; pedal cruise is a full engagement with no separate AOL latch, like ordinary Ford cruise. LONG output still stops on the first denied tick. Bounded by `starpilot/car/tesla/aol.long_denial_expired`: a LONG request the panda still denies after 1 s with no driver input is the lost-permission fault again (worst-case re-arm after a release is about 165 ms) | Replay: 0 `controlsMismatch`, every disengagement a driver action at the same time as NAP's (`work/replay2/REPORT.md`). `test_pedal_callers.py` `gas_wire` (release on pandad's read tick) and `gas_state5` (the interceptor's logged STATE 5 timeout and 40 ms wake): both fail without the change, the second already at the engagement from idle. `long_withheld`: the fault fires at 1.00 s, not before; fails without the bound. `single_pull`: lateral-only stays up (selfdrived is not "enabled", so the bound cannot apply). Every fault case now runs with AlwaysOnLateral off and on |

Safety review of the seventh round: NON-BLOCKING ONLY (two passes). Its remaining notes: a panda that grants and denies LONG
on alternate ticks with healthy feedback would restart the 1 s bound each time (no such panda path found); the bound needs
the panda's own LONG request bit, and a panda that clears its whole request mask is caught by the existing RX checks.

#### Owner decisions, 2026-11-24

- **Pedal calibration.** Inferred from manual driving in the logs by the harness session (BogPilot's definitions,
  `DI = (gas - Zero) * Factor`, Zero = gas at the first `DI_pedalPos >= 0.4 %`): factor median **0.973** (IQR 0.970-0.975),
  zero median **1.47** (IQR 1.34-1.83), stable across 24 drives and both remotes (`~/op-harness/DESIGN.md` section 5.5,
  read-only). At this factor the panda's first-frame window is wider than the interceptor's rest noise (the 1.0 row of the
  table above), and 0.973 is inside the accepted range (0.82-3.0). The panda check stays as it is. **To be confirmed against
  the device's saved values in `/data/params_bogpilot` once the owner has the device.**
- **Radar-bus interrupt-rate fault: exemption kept, threshold not changed.** The rates are logged per IRQ
  (`pandaStates[].canState1.irq0CallRate/irq1CallRate`, 600 segments, 698,000 samples). With the car's safety mode active
  (more than 2 s after leaving ELM327) bus 1 runs at a median of 1,315 interrupts/s per IRQ, 1,564 at most; the limit is
  16,000. The fault comes from 86 error storms, each 30,000-42,000 errors in 1.6-2.2 s, peaking at 18,800-21,551/s. In every one
  the panda has a frame queued on bus 1 and nothing acknowledges it (`rx` 0-350 during the storm), so each retry is an error
  interrupt. **All 86 happen in safety model 3 (ELM327, param 0 or 1), i.e. during fingerprinting and the firmware query,
  before openpilot or the panda knows which car this is**; none happens in the car's mode (its 28 small error bursts stay
  under 1,462/s). The bit is not permanent (`PERMANENT_FAULTS` is 0): the firmware only records it, and it stays set until
  the panda reboots, so 27 of the 43 routes started with it already set from an earlier drive. A threshold for this car
  would take effect only when safety mode 39 is set, after the bit has already latched; making it effective would mean
  raising the FDCAN2 limit for every car, or clearing a latched fault on a mode change. Neither is acceptable, so the
  exemption stays as it is.
- **Low-speed radar override:** stays on, as in NAP.

Kept from StarPilot although NAP differs (stricter): `steerFaultTemporary` while the EPAS reports `EAC_INHIBITED` (NAP: never).
On this car the EPAS idles in `EAC_AVAILABLE` (678,000 frames) and reports `EAC_INHIBITED` with error 3 or 4 on 56,000
frames while not steering, so NAP's "inhibited is the idle state" deadlock does not apply in general; whether lateral can
start from one of those inhibited periods is not known from the logs.

Not ported: NAP's pedal authority FSM and "keep lateral on pedal loss" (StarPilot faults the whole engagement), the
`/data/vdas_ff_table.json` override (no live file reads in controllers), the planner-side regen demand check, hands-on
pause, follow-distance dial, stalk-tap lane changes and `0x3E9`, donor VIN, `NAPForcePreAP`, the upside-down radar option.

Tests: `car/tesla/tests/test_preap_pedal_car.py`, `test_preap_virtual_das.py` (85 of NAP's 100; the 15 left out test the JSON
override), `test_preap_vdas_grade_control.py` (53), `test_preap_engage_grace.py` (7), `test_preap_accel_limits.py` (7),
`test_preap_pedal_authority.py` (15: zero-torque learner and regen prompt cases),
`openpilot/starpilot/car/tesla/tests/test_pedal_calibration_offset.py`, `test_preap_longitudinal_tuning.py`.
Changed expectation: `test_pedal_planner.py` outer integral gains `.05/.08/.1/.15` -> `0` (NAP).

### 5. Process replay against NAP (criterion 3)

`card` replayed on every logged segment by the port (`09dfac12d`) and by NAP (`e702d13397`), same pedal and radar settings,
`carState` compared frame by frame (`work/replay/`, results `out2/*.cmp`). The port fingerprints from the logged CAN; NAP is
forced with `NAPForcePreAP`. NAP's wall-clock stalk timing is paced at 10 ms per frame in the replay driver, otherwise a fast
replay turns every pair of pulls into a double pull.

600 segments: 578 compared (3,379,632 frames, identical frame counts and timestamps), 22 not replayable by either tree
(20 in routes with no `carParams` message in any segment, 2 with too few CAN frames to fingerprint).

Identical on every frame: `vEgo`, `vEgoRaw`, `aEgo`, steering angle, rate and torque, `steeringPressed`, `gearShifter`,
`doorOpen`, blinkers, `canValid`, `canTimeout`, `cruiseState.available`, `stockAeb`, `espDisabled`.

| Field | Frames differing | Why |
|---|---|---|
| `vEgoCluster` | 57.5 % | The port publishes the dash speed; NAP's `carState` carries 0 in replay. |
| `standstill` | 42.8 % | Port: speed below 0.1 m/s. NAP: only the DI's cruise STANDSTILL state, which a pre-AP car never reports. StarPilot's value kept. |
| `seatbeltUnlatched` | 21.0 % | Port reads the buckle (`0x201`); NAP hard-codes latched. All in parked or unbuckled segments. StarPilot's check kept. |
| `steerFaultTemporary` | 3.4 % | Port sets it while the EPAS reports `EAC_INHIBITED`; NAP never does. StarPilot's check kept. |
| `cruiseState.enabled`, `.speed` | 0.18 %, 0.23 %, in 11 and 20 segments | Always port disengaged where NAP stays engaged. Ten segments: a stalk CANCEL that NAP's 600 ms window ignores as an echo (`buttonEvents` `cancel` vs `unknown`, 23 frames). The logged CANCELs include BogPilot's own spoofed ones, which the port cannot recognize as its own in a replay. One segment (`00000021` seg 21): the segment starts with the stalk already pulled and the port's pedal feedback needs two frames before it allows an engagement. |
| `accFaulted` | 0.05 %, first 1-3 frames of each segment | The port starts fail-closed until two valid `0x552` frames have arrived. |
| `brakePressed`, `gasPressed`, `steerFaultPermanent` | under 0.01 % each | First frames of a segment (fail-closed start), EPAS status 4 at boot, and short runs around edges (longest: `gasPressed` for 17 frames in `00000008` seg 5, port pressed, NAP not). These were classified by where they occur, not examined one by one. |

No field differs in the direction of the port being engaged, or reporting no fault, where NAP is not.

card, radard, selfdrived and controlsd together, closed loop, against NAP on all 600 segments (`work/replay2/REPORT.md`).
Plain `process_replay` cannot run StarPilot's selfdrived on these logs (they have no AOL safety wire), so each tree's own
card, radard, selfdrived and controlsd are stepped in lockstep with its own libsafety as the panda, pandad's 10 Hz receipt
modelled, and the interceptor's STATE derived from that tree's own `0x551` frames. The first run (`7ee687018`) found the
two problems fixed in the seventh round above. On `c97447780`, keeping the logged panda faults: 0 `controlsMismatch`;
port disengagements are driver actions at the same moments as NAP's; differences are the intended stricter ones (radar
gating, seat-belt check, 360 deg steering cap, the panda's gas rule) or replay artifacts. Limits: open loop (BogPilot's
drive), pandad modelled not run, pedal calibration not in the logs (factor 1.0 used), only 13 logged engagements.

## Checks removed

| Check | Was | Now | Why |
|---|---|---|---|
| `0x552` exact two-channel agreement | 0.203 units | 4.0 units around the measured relationship | W1 above |
| Gas from either channel | max of both | channel 1 | W2 above |
| `0x552` counter + 1 | exact | + 1 or + 2 | W3 above |
| Interceptor `STATE` must be 0 | always | 0, or 5 while not commanding | Idle above |

Behavior replaced (not a safety check): StarPilot's immediate radar track retirement on an incomplete cycle, see section 2.

## Adaptations for StarPilot/FrogPilot infrastructure

None yet.
