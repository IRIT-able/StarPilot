"""Pre-AP stock cruise (no pedal) with the radar retrofit configured: the radar receiver's status reaches the driver as the
same warning as in pedal mode ("Radar Not Ready" / "Radar Silent", "Using Camera Lead"). Warning only: no event, no
disengagement. Configuration 0 (no radar) stays silent; the pedal profiles keep their own path.
"""
from contextlib import ExitStack
from unittest import mock

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.starpilot.car.tesla.pedal_events import stock_radar_companion, stock_radar_configured
from openpilot.starpilot.car.tesla.stock_events import StockCruiseConsumer
from openpilot.starpilot.car.tesla.tests.test_preap_panda_faults import pedal, stock
from openpilot.starpilot.car.tesla.tests.test_preap_radar_end_to_end import Chain, aol_callers

STAMP = 20_000_000_000


class Sources(dict):
  valid = {'carState': True, 'starpilotCarState': True}
  alive = {'carState': True, 'starpilotCarState': True}


def receive(cp, radar_interface, *, stamp=STAMP, pedal_fields=True):
  cs_send = messaging.new_message('carState', valid=True, logMonoTime=stamp)
  cs_send.carState.canValid = True
  cs_send.carState.cruiseState.enabled = True       # stock cruise engaged: the pedal chime logic would fire if it ran
  companion = stock_radar_companion(cp, radar_interface, cs_send)
  consumer = StockCruiseConsumer(cp)
  if companion is None:
    return None, consumer.poll(Sources(), now_ns=stamp, session='s', car_state=cs_send.carState.as_reader(),
                               source_ns=stamp, source_valid=True)
  if pedal_fields:  # pedal fields set anyway: stock cruise must not chime for them
    companion.starpilotCarState.pedalLongActive = companion.starpilotCarState.pedalMaxRegen = True
  sm = Sources(starpilotCarState=companion.starpilotCarState.as_reader())
  sm.logMonoTime = {'carState': stamp, 'starpilotCarState': companion.logMonoTime}
  return companion, consumer.poll(sm, now_ns=stamp, session='s', car_state=cs_send.carState.as_reader(),
                                  source_ns=stamp, source_valid=True)


@pytest.mark.parametrize('word', [2, 6])
def test_stock_cruise_radar_status_reaches_the_driver_as_a_warning(word):
  cp = stock(word)
  assert stock_radar_configured(cp)
  chain = Chain()
  for seconds, kwargs, status, text in ((1.0, {}, 1, 'Radar Not Ready'), (6.0, {}, 0, None),
                                        (1.2, {'radar': False}, 2, 'Radar Silent')):
    chain.run(seconds, **kwargs)
    assert chain.ri.preap.status() == status
    companion, alerts = receive(cp, chain.ri)
    assert companion.starpilotCarState.preapRadarStatus == status
    assert companion.starpilotCarState.sourceCarStateMonoTime == companion.logMonoTime == STAMP
    if text is None:
      assert alerts == []
    else:
      assert [(a.alert_text_1, a.alert_text_2, a.event_type) for a in alerts] == [(text, 'Using Camera Lead', ET.WARNING)]
      assert alerts[0].alert_type.startswith('teslaPreapRadar/')


def test_stock_cruise_without_radar_stays_silent():
  cp = stock(0)
  assert not stock_radar_configured(cp)
  chain = Chain()
  chain.run(1.2, radar=False)
  assert chain.ri.preap.status() != 0                # a status the car would never publish in configuration 0
  companion, alerts = receive(cp, chain.ri)
  assert companion is None and alerts == []
  assert not StockCruiseConsumer(cp).radar.enabled


@pytest.mark.parametrize('word', [1, 3, 7])
def test_pedal_profiles_keep_their_own_path(word):
  cp = pedal(word)
  assert not stock_radar_configured(cp)
  cs_send = messaging.new_message('carState', valid=True, logMonoTime=STAMP)
  assert stock_radar_companion(cp, Chain().ri, cs_send) is None
  consumer = StockCruiseConsumer(cp)
  assert consumer.pedal.enabled and not consumer.radar.enabled


@pytest.mark.parametrize('mutation', ['stale', 'other_stamp', 'invalid'])
def test_stale_or_unbound_status_is_not_shown(mutation):
  cp, chain = stock(2), Chain()
  chain.run(1.2, radar=False)
  cs_send = messaging.new_message('carState', valid=True, logMonoTime=STAMP)
  cs_send.carState.canValid = True
  companion = stock_radar_companion(cp, chain.ri, cs_send)
  sm = Sources(starpilotCarState=companion.starpilotCarState.as_reader())
  sm.logMonoTime = {'carState': STAMP, 'starpilotCarState': STAMP + (1 if mutation == 'other_stamp' else 0)}
  if mutation == 'invalid':
    sm.valid = {'carState': True, 'starpilotCarState': False}
  now = STAMP + (200_000_000 if mutation == 'stale' else 0)
  assert StockCruiseConsumer(cp).poll(sm, now_ns=now, session='s', car_state=cs_send.carState.as_reader(),
                                      source_ns=STAMP, source_valid=True) == []


def test_real_selfdrived_shows_the_warning_and_keeps_aol_lateral():
  """Stock cruise, radar configured, AOL lateral: a silent radar is a warning on the real selfdrived, never a fault."""
  with ExitStack() as cleanup:
    sd, _, step = aol_callers(cleanup)
    for tick in (0, 1):
      axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
    assert axis.lateralActive and command.latActive and sd.tesla_stock_consumer.radar.enabled
    original = sd.update_events

    def update(cs):
      stamp = sd.aol_car_state_log_ns
      message = messaging.new_message('starpilotCarState', valid=True, logMonoTime=stamp)
      message.starpilotCarState.sourceCarStateMonoTime = stamp
      message.starpilotCarState.preapRadarStatus = 2
      sd.sm.data['starpilotCarState'] = message.starpilotCarState.as_reader()
      sd.sm.logMonoTime['starpilotCarState'] = stamp
      sd.sm.valid['starpilotCarState'] = sd.sm.alive['starpilotCarState'] = True
      original(cs)
    seen = set()
    with mock.patch.object(sd, 'update_events', side_effect=update):
      for tick in range(2, 40):
        axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
        seen.update(alert.alert_type for alert in sd.tesla_stock_alerts)
        assert axis.lateralActive and command.latActive
        assert not sd.events.contains(ET.SOFT_DISABLE) and not sd.events.contains(ET.IMMEDIATE_DISABLE)
        assert not sd.events.contains(ET.NO_ENTRY)
    assert seen == {'teslaPreapRadar/silent/warning'}, seen
