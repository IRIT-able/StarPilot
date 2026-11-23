# Pre-AP Model S port notes (`preap-model-s`)

Car: 2014 Tesla Model S P85, pre-Autopilot, comma 3X. Pedal interceptor on bus 2 (0x551/0x552, sum checksum),
Bosch radar retrofit on bus 1 fed by the panda, newer SCCM retrofit.

## References

| Role | Repo and commit | Notes |
|---|---|---|
| Base | `StarPilotHQ/StarPilot` `testing` @ `0db704125` | Branch point. PRs target `testing`; it was 32 commits ahead of `stable`. |
| Primary | `NotAutopilot/openpilot` `naponsp-alpha` @ `e702d13397`, opendbc `81799e45e`, panda `649a6281f` | The commit named in `naponsp-alpha-prebuilt`'s `build.json`. The prebuilt's first commit (`5813362a4e`) has an opendbc tree identical to this pin. |
| Primary, device delta | `naponsp-alpha-prebuilt` @ `7eb09d2a7` | Six source-form commits on top of the build: upside-down radar option, MADS "wait for initial lateral permission", pedal calibration fixes, VIN-learn tool. Driving-relevant ones are ported and listed separately below. |
| Secondary | `IRIT-able/bogpilot-openpilot` `fix/preap-all` @ `9ed906a641` | Used to fill gaps and wherever it is stricter. The only branch with real drives on this car. |

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
| > 8.0 s (**proposed, awaiting owner**) | `radarFault` (soft disable, no entry) until the radar is back. Also applies to a radar that never speaks after startup. | One reboot is 5.66-5.68 s; 8 s is 40 % above it. |
| 2 silences within 600 s (**proposed, awaiting owner**) | `radarFault` while both are inside the window. | Observed: never more than one per route, 0.2 per radar hour. |

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

Known limit of W1 + W2, for the owner to accept or not (independent review, 2026-11-23): with channel 1 stuck, the
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
