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
| Radar silent for 50 update calls: tracks cleared, VALID dropped, then an empty message with `canError` every 5th call | BogPilot (empty message, so radard drops its last tracks) plus NAP's outcome (a radar error) | BogPilot's empty message is error-free and NAP publishes nothing; the port does the stricter of each. The gate is evaluated on every call, including calls without a radar cycle, as in BogPilot. The frozen-table count is not reset by a silence (NAP). |
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

Open: BogPilot shows a `preapRadarNotReady` warning while the gate is closed. The gate state is available on the interface
(`RI.preap.gate.status()`), but no StarPilot alert is wired to it yet.

## Checks removed

None yet. Behavior replaced (not a safety check): StarPilot's immediate radar track retirement, see section 2.

## Adaptations for StarPilot/FrogPilot infrastructure

None yet.
