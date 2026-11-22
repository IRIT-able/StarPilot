"""Pre-AP radar-valid gate: has the retrofit Bosch radar accepted its config and is it tracking, or only sending frames?

Ported unchanged in behavior from BogPilot fix/preap-all @ 9ed906a641
(opendbc_repo/opendbc/car/tesla_ap1/preap/radar_valid.py). NotAutopilot naponsp-alpha has no equivalent: it never reads
the 0x501 alert matrix, so it keeps using radar points after the radar flags missing modules. This gate is the stricter
behavior and is what preap/radar.py uses to decide whether points reach radard.

Advisory only: it withholds radar points, it is never a radarFault, a no-entry or a disengage. Only RADC_HWFail is a
radarFault, set by preap/radar.py itself. DBC: dbc/generator/tesla/tesla_radar_bosch.py (0x301 TeslaRadarSguInfo,
0x501 TeslaRadarAlertMatrix). Time 0 is the radar's first 0x301.

States:
- FAULT (points withheld), non-latching, any one of:
  F1 SGU_ITC in ITC_STARVED {67,68,70,83} or ITC_POS_REJECT {96} held F1_S (1.0 s);
  F2 any F2_BITS (0x501 configMismatch, CntryCdValidity, vinValidity, xwdValidity, radPositionMismatch, strRackMismatch,
     or the bdy/esp/gtw/sccm/adas MIA bits) set in 2 consecutive 0x501 frames; clears after 2 consecutive clear frames;
  F3 frozen table: FROZEN_MIN_POINTS (1) or more points whose rounded (dRel, yRel, vRel) set is unchanged for FROZEN_S
     (1.5 s), counted only while the ITC is in the F1 set;
  F4 HWFail, immediately;
  NO_ECHO: with the radar-feed safety flag, no returned gateway echo (GTW_ECHO_ADDRS, src 129) for ECHO_S: the panda has
     stopped feeding the radar vehicle data.
- VALID: ITC in ITC_OK {0, 107}, SGUFail, HWFail and SensorDirty all 0, a 0x501 frame seen and the last one clear of
  F2_BITS and the adjustment bits, and a moving track in the last MOVE_RECENT_S, all held VALID_S (2.0 s). VALID latches
  until a FAULT fires (a no-motion gap does not drop it).
- PENDING: neither. Points are withheld exactly as in FAULT; engage and speed control run on camera leads.
Moving track: a tracked point with |LongSpeed| >= MOVING_VREL or a LongDist change >= MOVING_DD against the same slot
MOVE_BACK_S earlier.
LATE (label only): not VALID DEADLINE_S after the later of the radar's first 0x301 and the car first moving.
Times come from the CAN packets' log time.
"""
from __future__ import annotations

from collections import deque

ITC_OK = frozenset({0, 107})
ITC_STARVED = frozenset({67, 68, 70, 83})
ITC_POS_REJECT = frozenset({96})
ITC_FAULT = ITC_STARVED | ITC_POS_REJECT
F1_S = 1.0
F2_FRAMES = 2
FROZEN_S = 1.5
FROZEN_MIN_POINTS = 1
FROZEN_GATED = True          # F3 only while the ITC is in ITC_FAULT
VALID_S = 2.0
MOVE_RECENT_S = 2.0
MOVING_VREL = 0.5            # m/s
MOVING_DD = 0.5              # m
MOVE_BACK_S = 2.0
SEEN_S = 0.3
ECHO_S = 2.0
DEADLINE_S = 20.0
EGO_MOVING = 0.3             # m/s
F2_BITS = ("RADC_a009_configMismatch", "RADC_a027_CntryCdValidity", "RADC_a037_vinValidity", "RADC_a042_xwdValidity",
           "RADC_a061_radPositionMismatch", "RADC_a062_strRackMismatch",
           "RADC_a011_bdyMIA", "RADC_a012_espMIA", "RADC_a013_gtwMIA", "RADC_a014_sccmMIA", "RADC_a015_adasMIA")
ADJUST_BITS = ("RADC_a004_adjustmentNotDone", "RADC_a005_adjustmentReq", "RADC_a006_adjustmentNotOk")
ALERT_BITS = F2_BITS + ADJUST_BITS
FAULT_ORDER = ("F4", "F1", "F2", "F3", "NO_ECHO")
# Bus-1 frames the panda sends to the radar; they come back to the host with src 129 (bus 1 + 128).
GTW_ECHO_SRC = 129
GTW_ECHO_ADDRS = frozenset({0x109, 0x119, 0x129, 0x149, 0x159, 0x169, 0x199, 0x1A9, 0x209, 0x219, 0x2A9, 0x2B9, 0x2D9})


def short_bit(name: str) -> str:
  """RADC_a037_vinValidity -> vinValidity, for the status line (the DBC names are too long to read in a log)."""
  parts = str(name).split("_", 2)
  return parts[-1] if len(parts) == 3 else str(name)


def itc_state(itc) -> str:
  """ok / fault / unknown for the status line. The ITC values are only partly characterised: ITC_OK and ITC_FAULT are
  the ones the CAN analysis covered, and anything else leaves the gate not VALID without being a FAULT (so PENDING),
  which read as a bare valid=0 in the log with no way to tell an unknown code from an aligning radar."""
  if itc is None:
    return "none"
  if itc in ITC_OK:
    return "ok"
  if itc in ITC_FAULT:
    return "fault"
  return "unknown"


class RadarValidGate:
  def __init__(self, radar_emulation: bool, frozen_s: float = FROZEN_S,
               frozen_min_points: int = FROZEN_MIN_POINTS, frozen_gated: bool = FROZEN_GATED):
    self.radar_emulation = bool(radar_emulation)
    self.frozen_s, self.frozen_min_points, self.frozen_gated = float(frozen_s), int(frozen_min_points), bool(frozen_gated)
    self.t0: float | None = None             # first 0x301
    self.ego_moved: float | None = None
    self.last_echo: float | None = None
    self.itc: int | None = None
    self.sgu_fail = self.hw_fail = self.dirty = False
    self.itc_fault_since: float | None = None
    self.alert_seen = False
    self.alert_clean = False                  # last 0x501 clear of ALERT_BITS
    self.f2_run = 0                           # consecutive 0x501 frames with F2 bits (>0) or clear (<0)
    self.f2 = False
    # Which bits are set, for the log-only status line. "F2" alone says a 0x501 alert bit latched but not which one,
    # and the remedies differ per bit: vinValidity wants a donor VIN, radPositionMismatch / configMismatch want
    # another Radar Donor Type, a *MIA bit means the radar is missing a vehicle-data frame.
    self.f2_bits: tuple = ()                  # F2 bits in the last 0x501 frame
    self.f2_bits_latched: tuple = ()          # F2 bits that latched F2, kept while F2 is on
    self.adjust_bits: tuple = ()              # ADJUST_BITS in the last 0x501 frame (these hold VALID off without being a FAULT)
    self.slots: dict[int, deque] = {}
    self.last_move: float | None = None
    self.sig = None
    self.sig_since: float | None = None
    self.ok_since: float | None = None
    self.echoed = not self.radar_emulation
    self.valid = False                        # radar points reach radard only once VALID (camera leads until then)
    self.fault: str | None = None
    self.active: frozenset = frozenset()
    self.late = False
    self.faults = 0                           # VALID -> FAULT edges

  # ---- inputs ----
  def on_echo(self, now: float) -> None:
    self.last_echo = now

  def on_ego(self, now: float, v_ego: float) -> None:
    if self.ego_moved is None and abs(float(v_ego)) >= EGO_MOVING:
      self.ego_moved = now

  def on_alert_matrix(self, now: float, vl) -> None:
    """One new 0x501 frame (parser values)."""
    self.f2_bits = tuple(b for b in F2_BITS if bool(vl[b]))
    self.adjust_bits = tuple(b for b in ADJUST_BITS if bool(vl[b]))
    bad = bool(self.f2_bits)
    self.alert_seen = True
    self.alert_clean = not bad and not self.adjust_bits
    self.f2_run = (max(self.f2_run, 0) + 1) if bad else (min(self.f2_run, 0) - 1)
    if self.f2_run >= F2_FRAMES:
      self.f2 = True
    elif self.f2_run <= -F2_FRAMES:
      self.f2 = False
    # Keep the bits that latched F2 while it is on: F2 needs 2 clear frames to clear, so the last frame can
    # already be clear while the fault still holds.
    self.f2_bits_latched = (self.f2_bits or self.f2_bits_latched) if self.f2 else ()

  def on_sgu(self, now: float, itc: int, sgu_fail: bool, hw_fail: bool, dirty: bool) -> None:
    """One 0x301 frame."""
    if self.t0 is None:
      self.t0 = now
    self.itc, self.sgu_fail, self.hw_fail, self.dirty = int(itc), bool(sgu_fail), bool(hw_fail), bool(dirty)
    if self.itc in ITC_FAULT:
      if self.itc_fault_since is None:
        self.itc_fault_since = now
    else:
      self.itc_fault_since = None

  def on_points(self, now: float, points: dict) -> None:
    """One radar cycle: {slot: (dRel, yRel, vRel)} of the tracked, in-range, ProbExist >= 50 % points before the gate drops them."""
    for slot in [s for s in self.slots if s not in points]:
      del self.slots[slot]
    moving = False
    for slot, (d, _y, v) in points.items():
      hist = self.slots.setdefault(slot, deque())
      hist.append((now, float(d)))
      while len(hist) > 1 and hist[1][0] <= now - MOVE_BACK_S:
        hist.popleft()
      t_then, d_then = hist[0]
      if abs(float(v)) >= MOVING_VREL or (now - t_then >= MOVE_BACK_S - 1e-6 and abs(float(d) - d_then) >= MOVING_DD):
        moving = True
    if moving:
      self.last_move = now
    rows = tuple(sorted((round(float(d), 2), round(float(y), 2), round(float(v), 2)) for d, y, v in points.values()))
    gate_on = (self.itc in ITC_FAULT) if self.frozen_gated else True
    if len(rows) < self.frozen_min_points or not gate_on:
      self.sig, self.sig_since = None, None
    elif rows != self.sig:
      self.sig, self.sig_since = rows, now

  # ---- state ----
  def _faults(self, now: float) -> frozenset:
    on = {
      "F4": self.hw_fail,
      "F1": self.itc_fault_since is not None and now - self.itc_fault_since >= F1_S - 1e-9,
      "F2": self.f2,
      "F3": self.sig_since is not None and now - self.sig_since >= self.frozen_s - 1e-9,
      "NO_ECHO": self.radar_emulation and now - (self.last_echo if self.last_echo is not None else
                                                 (self.t0 if self.t0 is not None else now)) >= ECHO_S,
    }
    return frozenset(k for k in FAULT_ORDER if on[k])

  def update(self, now: float) -> bool:
    """Returns whether radar points may reach radard (VALID). Advisory: never an engage or speed-control condition."""
    if self.radar_emulation:
      self.echoed = self.last_echo is not None and now - self.last_echo < ECHO_S
    self.active = self._faults(now)
    self.fault = next((k for k in FAULT_ORDER if k in self.active), None)     # the first, for the status line
    ok = (self.itc in ITC_OK and not (self.sgu_fail or self.hw_fail or self.dirty) and self.alert_seen and self.alert_clean
          and self.last_move is not None and now - self.last_move <= MOVE_RECENT_S)
    if self.fault is not None or not ok:
      self.ok_since = None
    elif self.ok_since is None:
      self.ok_since = now
    was = self.valid
    if self.fault is not None:
      self.valid = False
    elif not self.valid and self.ok_since is not None and now - self.ok_since >= VALID_S - 1e-9:
      self.valid = True
    if was and not self.valid:
      self.faults += 1
    start = self.t0 if self.ego_moved is None or self.t0 is None else max(self.t0, self.ego_moved)
    self.late = (not self.valid and self.t0 is not None and self.ego_moved is not None and now - start >= DEADLINE_S)
    return self.valid

  @property
  def pending(self) -> bool:
    return not self.valid and self.fault is None

  def bits_text(self) -> str:
    """The set 0x501 bits for the log-only status line: the F2 bits (latched ones while F2 is on) and, prefixed with
    "adj:", the adjustment bits, which keep VALID off without being a FAULT. "-" when none are set."""
    f2 = self.f2_bits_latched or self.f2_bits
    names = [short_bit(b) for b in f2] + ["adj:" + short_bit(b) for b in self.adjust_bits]
    return ",".join(names) if names else "-"

  def status(self) -> dict:
    """For the log-only status line."""
    return dict(echoed=self.echoed, pending=self.pending, valid=self.valid, fault=self.fault or "-", late=self.late,
                itc=self.itc, itc_state=itc_state(self.itc), bits=self.bits_text(), faults=self.faults)
