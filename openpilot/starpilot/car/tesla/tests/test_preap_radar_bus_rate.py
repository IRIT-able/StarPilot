"""Pre-AP Model S: the live radar-bus (panda bus 1) interrupt-rate bound that stands in for the exempt panda bit.

The panda counts each IRQ's calls over a 1 s window; pandad reports the previous window's count in every pandaStates
(10 Hz). `reports()` models exactly that from an interrupt rate given as a function of time.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.starpilot.aol.transport_pause_conditions import AVAILABILITY_EVENTS, HOST_SOURCE_AVAILABILITY_EVENTS
from openpilot.starpilot.car.tesla.radar_bus_rate import RATE_LIMIT, SETTLE_NS, SUSTAIN_NS, RadarBusRateMonitor
from openpilot.starpilot.car.tesla.tests.test_preap_panda_faults import build, PROFILES
from openpilot.starpilot.nostalgia import LONGITUDINAL_ONLY_EVENTS

S = 1_000_000_000
NORMAL = 1_564          # highest bus-1 rate logged in the car's safety mode (600 segments)
STORM = 21_551          # highest rate of the logged identification storms


def state(rate, model, param, irq=0):
  rates = [0, 0, 0]
  rates[irq] = rate
  return SimpleNamespace(safetyModel=model, safetyParam=param,
                         canState1=SimpleNamespace(irq0CallRate=rates[0], irq1CallRate=rates[1], irq2CallRate=rates[2]))


def reports(rate_at, seconds, *, phase=0.03, mode_at=None, start=10 * S, irq=0):
  """(stamp, reported count) at 10 Hz. Window k spans [k, k+1) s; its count is reported from k+1 until k+2."""
  out = []
  for i in range(int(seconds * 10)):
    t = i / 10 + phase
    k = int(t) - 1
    count = 0 if k < 0 else int(sum(rate_at(k + j / 1000) for j in range(1000)) / 1000)
    out.append((start + int(t * S), count, mode_at(t) if mode_at else True))
  return out


def run(cp, rate_at, seconds, **kw):
  monitor = RadarBusRateMonitor(cp)
  model, param = int(cp.safetyConfigs[0].safetyModel.raw), int(cp.safetyConfigs[0].safetyParam)
  trace = []
  for stamp, count, car_mode in reports(rate_at, seconds, **kw):
    ps = state(count, model, param, kw.get('irq', 0)) if car_mode else state(count, 3, 0, kw.get('irq', 0))
    trace.append((stamp - kw.get('start', 10 * S), monitor.update([ps], stamp)))
  return trace


def first_trip(trace):
  return next((t for t, tripped in trace if tripped), None)


@pytest.fixture(params=PROFILES, ids=[f'{k}{w}' for k, w, _ in PROFILES])
def cp(request):
  return build(*request.param)


def test_normal_rates_never_trip(cp):
  assert first_trip(run(cp, lambda t: NORMAL, 120)) is None
  assert first_trip(run(cp, lambda t: RATE_LIMIT, 60)) is None          # the bound itself is not a trip


@pytest.mark.parametrize('irq', [0, 1, 2])
@pytest.mark.parametrize('rate', [RATE_LIMIT + 1, 8_000, STORM])
@pytest.mark.parametrize('onset', [5.0, 5.5, 5.97])
def test_sustained_runaway_trips_within_the_documented_delay_and_recovers(cp, irq, rate, onset):
  trace = run(cp, lambda t: rate if t >= onset else NORMAL, 20, irq=irq)
  trip = first_trip(trace) / S
  # Detection needs two full-or-partial windows above the bound; the first is reported at the next whole second.
  assert onset + 1.3 <= trip <= onset + 3.5                                 # the documented upper bound
  if rate == STORM:
    assert trip <= onset + 2.5                                               # the documented 1.4-2.5 s
  assert all(tripped for t, tripped in trace if t / S >= trip)              # stays tripped while it lasts
  recovered = run(cp, lambda t: rate if onset <= t < 12 else NORMAL, 20, irq=irq)
  ends = [t / S for t, tripped in recovered if tripped]
  assert ends and 13.0 <= max(ends) + 0.1 <= 14.5                           # clears once normal reports last 1.3 s


@pytest.mark.parametrize('spike_s', [0.1, 0.25, 0.9])
def test_a_spike_inside_one_window_does_not_trip(cp, spike_s):
  # Inside one firmware window, however strong: one window above the bound, visible for 0.9 s.
  assert first_trip(run(cp, lambda t: STORM if 5.05 <= t < 5.05 + spike_s else NORMAL, 20)) is None


def test_a_short_spike_across_a_window_boundary_does_not_trip(cp):
  # 0.24 s at storm rate split over two windows: each window stays under the bound.
  assert first_trip(run(cp, lambda t: STORM if 5.88 <= t < 6.12 else NORMAL, 20)) is None


def test_storm_while_identifying_the_car_and_right_after_the_mode_change_does_not_trip(cp):
  # Logged shape: storm in ELM327 mode, ending at the switch to the car's mode; the window that straddles the switch is
  # reported in car mode for up to 2 s.
  trace = run(cp, lambda t: STORM if 2.4 <= t < 4.6 else NORMAL, 30, mode_at=lambda t: t >= 4.6)
  assert first_trip(trace) is None
  # A storm that continues in the car's mode past the settling time is a runaway.
  trace = run(cp, lambda t: STORM if 2.4 <= t < 12 else NORMAL, 30, mode_at=lambda t: t >= 4.6)
  assert first_trip(trace) is not None and first_trip(trace) / S >= 4.6 + SETTLE_NS / S


def test_any_other_safety_mode_resets_and_never_trips(cp):
  monitor = RadarBusRateMonitor(cp)
  model, param = int(cp.safetyConfigs[0].safetyModel.raw), int(cp.safetyConfigs[0].safetyParam)
  for i in range(100):
    assert not monitor.update([state(STORM, 3, 1)], 10 * S + i * S // 10)
    assert not monitor.update([state(STORM, model, param ^ 4)], 30 * S + i * S // 10)
  # Leaving the car's mode mid-trip clears it (other checks own a safety-mode mismatch).
  for i in range(60):
    monitor.update([state(STORM, model, param)], 50 * S + i * S // 10)
  assert monitor.tripped
  assert not monitor.update([state(STORM, 0, 0)], 57 * S)


@pytest.mark.parametrize('mutation', ['other_tesla', 'other_brand', 'passive', 'fixed'])
def test_other_configurations_have_no_monitor(mutation):
  cp = build('pedal', 3, 32)
  if mutation == 'other_tesla':
    cp.carFingerprint = CAR.TESLA_MODEL_3
  elif mutation == 'other_brand':
    cp.brand = 'toyota'
  elif mutation == 'passive':
    cp.passive = True
  elif mutation == 'fixed':
    cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed
  monitor = RadarBusRateMonitor(cp)
  assert not monitor.enabled
  assert not any(monitor.update([state(STORM, 39, 3)], 10 * S + i * S // 10) for i in range(100))


def test_constants_match_the_measurement():
  assert RATE_LIMIT == 4000 and RATE_LIMIT >= 2.5 * NORMAL
  assert 1.0 * S + 0.25 * S <= SUSTAIN_NS <= 1.8 * S - 0.45 * S           # logged: one window <= 1.0 s, two >= 1.8 s
  assert SETTLE_NS >= 2.0 * S                                               # the straddling window is gone by then
  cp = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  assert cp.safetyConfigs[0].safetyModel.raw == 39


def test_the_event_is_a_soft_disable_on_both_axes():
  alerts = EVENTS[EventName.sensorDataInvalid]
  assert ET.SOFT_DISABLE in alerts and ET.NO_ENTRY in alerts and ET.IMMEDIATE_DISABLE not in alerts
  assert EventName.sensorDataInvalid not in LONGITUDINAL_ONLY_EVENTS          # AOL lateral does not keep steering
  assert EventName.sensorDataInvalid not in AVAILABILITY_EVENTS
  assert EventName.sensorDataInvalid not in HOST_SOURCE_AVAILABILITY_EVENTS


def test_real_selfdrived_stock_cruise_aol_runaway_is_a_takeover_warning_not_an_immediate_disable():
  """Stock cruise, radar configured, AOL lateral (selfdrived not engaged): StarPilot withdraws AOL steering on any
  soft-disable event with the takeover warning. A normal rate keeps lateral; a sustained runaway trips after 1.3 s."""
  from contextlib import ExitStack
  from unittest import mock
  from openpilot.starpilot.car.tesla.tests.test_preap_radar_end_to_end import aol_callers
  with ExitStack() as cleanup:
    sd, _, step = aol_callers(cleanup)
    for tick in (0, 1):
      axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
    assert axis.lateralActive and command.latActive and sd.preap_bus1_rate.enabled
    clock, original = [50 * S], sd.update_events
    onset = 50 * S + SETTLE_NS + 400_000_000

    def update(cs):
      clock[0] += S // 10                      # one pandaStates report per step
      states = []
      for panda in sd.sm['pandaStates']:
        builder = panda.as_builder()
        builder.canState1.irq1CallRate = STORM if clock[0] >= onset else NORMAL
        states.append(builder.as_reader())
      sd.sm.data['pandaStates'] = states
      sd.sm.updated['pandaStates'] = True
      sd.sm.logMonoTime['pandaStates'] = clock[0]
      original(cs)
    trip = None
    with mock.patch.object(sd, 'update_events', side_effect=update):
      for tick in range(2, 60):
        axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
        tripped = EventName.sensorDataInvalid in sd.events.names
        assert not sd.events.contains(ET.IMMEDIATE_DISABLE)
        if trip is None and tripped:
          trip = clock[0]
        if trip is None:
          assert axis.lateralActive and command.latActive, tick
        else:
          assert tripped and not command.latActive, tick
    assert trip is not None and SUSTAIN_NS <= trip - onset < SUSTAIN_NS + S // 10     # first report at or past 1.3 s
    assert any(entry.alert.alert_text_1 == 'TAKE CONTROL IMMEDIATELY' and entry.alert.alert_text_2 == 'Radar Bus Fault'
               for entry in sd.AM.alerts.values() if entry.active(sd.sm.frame))
    assert not any('Sensor Data Invalid' in (entry.alert.alert_text_1, entry.alert.alert_text_2)
                   for entry in sd.AM.alerts.values() if entry.active(sd.sm.frame))


@pytest.mark.parametrize('seed', range(20))
def test_report_jitter_and_missed_reports(cp, seed):
  """pandad's publish times jitter and reports go missing: one elevated window still never trips; two still do."""
  import random
  rng = random.Random(seed)
  model, param = int(cp.safetyConfigs[0].safetyModel.raw), int(cp.safetyConfigs[0].safetyParam)

  def jittered(rate_at):
    monitor, trips = RadarBusRateMonitor(cp), []
    for stamp, count, _ in reports(rate_at, 20):
      if rng.random() < 0.05:
        continue                                           # a missed report
      stamp += rng.randint(-20_000_000, 60_000_000)        # late more often than early (logged max interval 0.120 s)
      trips.append(monitor.update([state(count, model, param)], stamp))
    return any(trips)
  assert not jittered(lambda t: STORM if 6.0 <= t < 7.0 else NORMAL)          # exactly one window
  assert jittered(lambda t: STORM if 6.0 <= t < 8.0 else NORMAL)              # two windows


def test_a_strong_burst_of_a_third_of_a_second_across_a_boundary_can_trip(cp):
  # Documented behaviour (safe direction): about 4,350 counts in each of two windows.
  assert first_trip(run(cp, lambda t: STORM if 5.85 <= t < 6.15 else NORMAL, 20)) is not None


def test_a_mode_drop_mid_drive_settles_again_before_counting(cp):
  monitor = RadarBusRateMonitor(cp)
  model, param = int(cp.safetyConfigs[0].safetyModel.raw), int(cp.safetyConfigs[0].safetyParam)
  t = 10 * S
  for _ in range(100):
    monitor.update([state(NORMAL, model, param)], t); t += S // 10
  monitor.update([state(NORMAL, 0, 0)], t); t += S // 10            # e.g. the panda reset to silent
  back = t
  while t < back + SETTLE_NS:                                        # storm-level counts while settling: ignored
    assert not monitor.update([state(STORM, model, param)], t); t += S // 10
  while t < back + SETTLE_NS + SUSTAIN_NS - S // 10:
    assert not monitor.update([state(STORM, model, param)], t); t += S // 10
  t = back + SETTLE_NS + SUSTAIN_NS
  assert monitor.update([state(STORM, model, param)], t)


def test_relabel_copies_and_only_touches_sensor_data_invalid():
  from opendbc.car.structs import car
  from openpilot.starpilot.car.tesla.radar_bus_rate import relabel
  from openpilot.selfdrive.selfdrived.events import Events
  cp = car.CarParams.new_message()
  cs = car.CarState.new_message()
  events = Events()
  events.add(EventName.sensorDataInvalid)
  events.add(EventName.doorOpen)
  originals = {}
  for kinds in ([ET.SOFT_DISABLE], [ET.NO_ENTRY], [ET.PERMANENT]):
    alerts = events.create_alerts(kinds, [cp, cs, None, False, 300, 1])
    if kinds == [ET.PERMANENT]:                       # created only after its 1 s delay; take the shared object as the AM would
      import copy
      shown = copy.copy(EVENTS[EventName.sensorDataInvalid][ET.PERMANENT])
      shown.alert_type = 'sensorDataInvalid/permanent'
      alerts = [shown]
    before = [(a.alert_type, a.alert_text_1, a.alert_text_2) for a in alerts]
    out = relabel(alerts)
    assert [(a.alert_text_1, a.alert_text_2) for a in alerts] == [b[1:] for b in before]      # originals untouched
    for a, b in zip(out, before):
      source = next(x for x in alerts if x.alert_type == b[0])
      for field in ('alert_type', 'priority', 'audible_alert', 'visual_alert', 'alert_status', 'alert_size', 'duration',
                    'creation_delay', 'event_type'):
        assert getattr(a, field) == getattr(source, field), field           # presentation text only
      if b[0].startswith('sensorDataInvalid/'):
        assert 'Sensor Data Invalid' not in (a.alert_text_1, a.alert_text_2)
        assert 'Radar Bus Fault' in (a.alert_text_1, a.alert_text_2)
        originals[b[0]] = (a.alert_text_1, a.alert_text_2)
      else:
        assert (a.alert_text_1, a.alert_text_2) == b[1:]
  assert originals == {'sensorDataInvalid/softDisable': ('TAKE CONTROL IMMEDIATELY', 'Radar Bus Fault'),
                       'sensorDataInvalid/noEntry': ('openpilot Unavailable', 'Radar Bus Fault'),
                       'sensorDataInvalid/permanent': ('Radar Bus Fault', 'Radar CAN Interrupt Rate Too High')}
  assert EVENTS[EventName.sensorDataInvalid][ET.PERMANENT].alert_text_1 == 'Sensor Data Invalid'          # shared object


def test_real_selfdrived_keeps_the_generic_text_when_the_imu_check_also_fires():
  from contextlib import ExitStack
  from unittest import mock
  from openpilot.starpilot.car.tesla.tests.test_preap_radar_end_to_end import aol_callers
  with ExitStack() as cleanup:
    sd, _, step = aol_callers(cleanup)
    for tick in (0, 1):
      step(tick, native_request_long=False, panda_controls_allowed=False)
    clock, original, imu = [50 * S], sd.update_events, [False]

    def update(cs):
      clock[0] += S // 10
      states = []
      for panda in sd.sm['pandaStates']:
        builder = panda.as_builder()
        builder.canState1.irq0CallRate = STORM
        states.append(builder.as_reader())
      sd.sm.data['pandaStates'] = states
      sd.sm.updated['pandaStates'] = True
      sd.sm.logMonoTime['pandaStates'] = clock[0]
      # The IMU goes silent (20 s) on the very frame the bound trips, so both raise sensorDataInvalid first together.
      monitor = sd.preap_bus1_rate
      if monitor.tripped or (monitor.above_since and clock[0] - monitor.above_since >= SUSTAIN_NS):
        imu[0] = True                                # this report trips the bound
      if imu[0]:
        sd.sm.recv_frame['accelerometer'] = sd.sm.frame - 2000
      original(cs)
      assert sd.sensor_packets_stale == imu[0] and sd.preap_bus1_tripped == imu[0]
      assert (EventName.sensorDataInvalid in sd.events.names) == imu[0], clock[0]
    with mock.patch.object(sd, 'update_events', side_effect=update):
      for tick in range(2, 45):
        step(tick, native_request_long=False, panda_controls_allowed=False)
    assert sd.preap_bus1_tripped and EventName.sensorDataInvalid in sd.events.names
    texts = {(entry.alert.alert_text_1, entry.alert.alert_text_2) for entry in sd.AM.alerts.values() if entry.active(sd.sm.frame)}
    assert ('TAKE CONTROL IMMEDIATELY', 'Sensor Data Invalid') in texts and not any('Radar Bus Fault' in t for t in texts)
