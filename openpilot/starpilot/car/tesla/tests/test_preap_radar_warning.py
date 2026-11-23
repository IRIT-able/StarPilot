"""Radar status reaches the driver as a warning only (BogPilot preapRadarNotReady): no disengaging event type."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.selfdrived.events import ET
from openpilot.starpilot.car.tesla.pedal_events import PedalStatusConsumer


def consumer_alerts(radar_status, stamp=1_000):
  consumer = PedalStatusConsumer.__new__(PedalStatusConsumer)
  consumer.enabled, consumer.session, consumer.previous, consumer.stamp = True, None, False, 0
  cs = SimpleNamespace(canValid=True, canTimeout=False, cruiseState=SimpleNamespace(enabled=False))
  status = SimpleNamespace(pedalLongActive=False, pedalMaxRegen=False, preapRadarStatus=radar_status,
                           sourceCarStateMonoTime=stamp)
  data = {'carState': cs, 'starpilotCarState': status}
  sm = SimpleNamespace(__getitem__=None, valid={'carState': True, 'starpilotCarState': True},
                       alive={'carState': True, 'starpilotCarState': True},
                       logMonoTime={'carState': stamp, 'starpilotCarState': stamp})

  class SM(dict):
    valid, alive, logMonoTime = sm.valid, sm.alive, sm.logMonoTime
  return consumer.update(SM(data), now_ns=stamp + 1, session='s')


@pytest.mark.parametrize('status,text', [(0, None), (1, 'Radar Not Ready'), (2, 'Radar Silent')])
def test_radar_status_is_a_warning_only(status, text):
  alerts = consumer_alerts(status)
  if text is None:
    assert alerts == []
    return
  assert [a.alert_text_1 for a in alerts] == [text]
  assert alerts[0].event_type == ET.WARNING and alerts[0].alert_text_2 == 'Using Camera Lead'
