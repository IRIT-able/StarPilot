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
| > 0.5 s | Tracks and VALID dropped, empty error-free `radarTracks` every 5th call: radard drops its tracks and follows camera leads. No error. | BogPilot |
| > 1.0 s | `starpilotCarState.preapRadarStatus = 2`: "Radar Silent / Using Camera Lead", warning only. Status 1 ("Radar Not Ready") while the gate is closed for any other reason, as BogPilot's `preapRadarNotReady`. | Owner |
| > 8.0 s (**proposed, awaiting owner**) | `radarFault` (soft disable, no entry) until the radar is back. Also applies to a radar that never speaks after startup. | One reboot is 5.66-5.68 s; 8 s is 40 % above it. |
| 2 silences within 600 s (**proposed, awaiting owner**) | `radarFault` while both are inside the window. | Observed: never more than one per route, 0.2 per radar hour. |

After a silence the points return only when the gate has been VALID-eligible for 2 s again.
`preapRadarStatus @40` is a new `StarPilotCarState` field, registered in `fixtures/custom_reviewed_schema_additions.json`.

## Checks removed

None yet. Behavior replaced (not a safety check): StarPilot's immediate radar track retirement, see section 2.

## Adaptations for StarPilot/FrogPilot infrastructure

None yet.
