"""Pre-AP Model S: live interrupt-rate bound on the radar bus (panda bus 1, FDCAN2).

The panda's own interrupt-rate bit for this bus is exempt for this car (`starpilot/aol/panda_faults.py`): openpilot's
car identification sends an OBD query on bus 1 before the radar is up, nothing acknowledges it, and the retries latch
the bit (limit 16,000/s) on every drive, long before this car's safety mode is set. This check stands in for that bit
once the car's own safety mode is active, from the per-IRQ call rates pandad reports for bus 1
(`pandaStates[0].canState1.irq{0,1,2}CallRate`, each the count over the firmware's previous 1 s window).

Measured on all 600 logged segments with the car's safety mode active: median 1,315/s, at most 1,564/s per IRQ.
The bound is 4,000/s (2.5 times that maximum). A trip needs the rate above it for longer than one firmware window
is visible (two consecutive windows), so a burst that raises a single window does not trip; a strong burst of about
0.3 s or more split over a window boundary can raise both and trip. A window's count is reported only after the
window ends, so detection takes about 1.4-2.5 s after a storm-level runaway starts and up to about 3.5 s for a rate
just above the bound. selfdrived then raises `sensorDataInvalid` (no entry, soft disable on both axes) until the rate
has been back under the bound for the same time.
"""

RATE_LIMIT = 4000                    # interrupts per second, per IRQ, on bus 1
SUSTAIN_NS = 1_300_000_000           # one window is visible for at most 1.0 s in the logs (0.9 s typical), two for 1.8 s
SETTLE_NS = 2_100_000_000            # the first windows after the mode change may still count pre-mode traffic


def supported(cp):
  try:
    from opendbc.car.tesla.preap.aol import qualified
    return bool(qualified(cp))
  except (AttributeError, TypeError, ValueError):
    return False


def bus1_rate(panda_state):
  state = panda_state.canState1
  return max(int(state.irq0CallRate), int(state.irq1CallRate), int(state.irq2CallRate))


def _log_trip(rate):
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event('tesla_preap_bus1_interrupt_rate', rate=int(rate), limit=RATE_LIMIT)
  except Exception:
    pass


class RadarBusRateMonitor:
  def __init__(self, cp):
    self.enabled = supported(cp)
    self.expected = ((int(cp.safetyConfigs[0].safetyModel.raw if hasattr(cp.safetyConfigs[0].safetyModel, 'raw')
                          else cp.safetyConfigs[0].safetyModel), int(cp.safetyConfigs[0].safetyParam))
                     if self.enabled else None)
    self.reset()

  def reset(self):
    self.active_since = 0
    self.above_since = 0
    self.below_since = 0
    self.tripped = False

  def update(self, panda_states, stamp_ns):
    """Call once per new pandaStates message. Returns True while the bound is tripped."""
    try:
      if not self.enabled or len(panda_states) < 1:
        self.reset()
        return False
      ps = panda_states[0]
      model = ps.safetyModel.raw if hasattr(ps.safetyModel, 'raw') else ps.safetyModel
      if (int(model), int(ps.safetyParam)) != self.expected:
        self.reset()
        return False
      self.active_since = self.active_since or stamp_ns
      if stamp_ns - self.active_since < SETTLE_NS:
        return False
      if bus1_rate(ps) > RATE_LIMIT:
        self.below_since = 0
        self.above_since = self.above_since or stamp_ns
        if stamp_ns - self.above_since >= SUSTAIN_NS and not self.tripped:
          self.tripped = True
          _log_trip(bus1_rate(ps))
      else:
        self.above_since = 0
        if self.tripped:
          self.below_since = self.below_since or stamp_ns
          if stamp_ns - self.below_since >= SUSTAIN_NS:
            self.tripped, self.below_since = False, 0
      return self.tripped
    except (AttributeError, TypeError, ValueError, IndexError):
      return self.tripped
