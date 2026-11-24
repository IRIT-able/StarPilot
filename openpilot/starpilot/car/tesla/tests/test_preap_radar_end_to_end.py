"""Pre-AP radar retrofit end to end, receiver -> card -> radard -> selfdrived, with radard's real SubMaster checks
(no ignore_alive / ignore_avg_freq), plus card's preapRadarStatus wiring and AOL lateral retention in stock-cruise mode.

Paths the radar safety reviews listed as untested: U2 (radar fault end to end), U3 (card status wiring), U5 (radarTracks
alive / rate / valid in radarState validity), N8 (stock-cruise pre-AP with the radar configured).
"""
from contextlib import ExitStack
import os
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.interface import CarInterface as TeslaInterface
from opendbc.car.tesla.tests.test_preap_radar import radar_interface, radar_params
from opendbc.car.tesla.tests.test_preap_radar_gate import cycle
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, RadarD
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, EventName
from openpilot.selfdrive.selfdrived.tests.test_jetlink_event_faults import events_drive
from openpilot.starpilot.nostalgia import LONGITUDINAL_ONLY_EVENTS

PACKER = CANPacker("tesla_radar_bosch_generated")
RADAR_DT = 0.1          # 10 Hz radar cycle, inside the 8.7-16.7 Hz this radar sends


class Capture:
  def __init__(self):
    self.sent = {}

  def send(self, service, msg):
    self.sent[service] = msg


class Chain:
  """CAN at 100 Hz into the receiver, card's own radar_publish, radard as it runs (SubMaster polling modelV2, no
  ignored checks), and radarState into selfdrived's event mapping."""
  def __init__(self):
    self.ri = radar_interface(radar_params(), gate_open=False)
    self.ri.update([])
    self.card = SimpleNamespace(pm=Capture())
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
    self.rd = RadarD(0.0, radar_available=True, tesla_preap_radar=True)
    self.pm = Capture()
    self.t = 1.0
    self.call = 0
    self.next_cycle = 1.0
    self.radar_msg = None

  def run(self, seconds, *, radar=True, period=RADAR_DT, card_alive=True, **cycle_kw):
    end = self.t + seconds
    while self.t < end - 1e-9:
      if card_alive:
        if radar and self.t >= self.next_cycle - 1e-9:
          packets = cycle(PACKER, self.t, **cycle_kw)
          self.next_cycle = self.t + period
        else:
          packets = [(int(self.t * 1e9), [(0x109, bytes(8), 129)])]
          if not radar:
            self.next_cycle = self.t
        Car.radar_publish(self.card, self.ri.update(packets))
      self.call += 1
      if self.call % 5 == 0:
        msgs = [self._model(), self._car_state()]
        tracks = self.card.pm.sent.pop('radarTracks', None)
        if tracks is not None:
          tracks.logMonoTime = int(self.t * 1e9)
          msgs.append(tracks.as_reader())
        self.sm.update_msgs(self.t, msgs)
        self.rd.update(self.sm, self.sm['radarTracks'])
        self.rd.publish(self.pm)
        self.radar_msg = self.pm.sent['radarState']
      self.t = round(self.t + 0.01, 6)
    return self.radar_msg

  def _model(self):
    message = messaging.new_message('modelV2', valid=True)
    message.logMonoTime = int(self.t * 1e9)
    message.modelV2.velocity.x = [20.0]
    for lead in message.modelV2.init('leadsV3', 2):
      lead.prob, lead.x, lead.xStd, lead.y, lead.yStd = 0.9, [40.0 + RADAR_TO_CAMERA], [3.0], [0.0], [1.0]
      lead.v, lead.vStd, lead.a = [18.0], [2.0], [0.0]
    return message.as_reader()

  def _car_state(self):
    message = messaging.new_message('carState', valid=True)
    message.logMonoTime = int(self.t * 1e9)
    message.carState.vEgo = 20.0
    return message.as_reader()

  def events(self, op_long):
    cp = TeslaInterface.get_non_essential_params('TESLA_MODEL_S_PREAP')
    cp.radarUnavailable = False
    cp.openpilotLongitudinalControl = op_long
    drive, cs = events_drive()
    drive.CP = cp
    drive.sm.services = list(drive.sm.data)   # the stub has no service list; selfdrived's radarState-invalid path reads it
    drive.sm.data['radarState'] = self.radar_msg.radarState
    drive.sm.valid['radarState'] = self.radar_msg.valid
    with mock.patch('openpilot.selfdrive.selfdrived.helpers.time.monotonic', return_value=100.):
      drive.update_events(cs)
      drive.update_events(cs)
    return set(drive.events.names)


RADAR_EVENTS = {EventName.canError, EventName.radarFault, EventName.radarTempUnavailable}


@pytest.fixture
def chain():
  chain = Chain()
  msg = chain.run(6.0)
  assert msg.valid and chain.rd.tracks, 'healthy radar points reach radard'
  return chain


@pytest.mark.parametrize('op_long', [False, True])
def test_healthy_radar_and_a_radar_reboot_raise_no_radar_event(chain, op_long):
  assert not RADAR_EVENTS & chain.events(op_long)
  for _ in range(11):                               # one logged radar reboot: 5.66 s
    msg = chain.run(0.5, radar=False)
    assert msg.valid and not any(msg.radarState.radarErrors.to_dict().values())
    assert not RADAR_EVENTS & chain.events(op_long)
  assert not chain.rd.tracks and not msg.radarState.leadOne.radar and msg.radarState.leadOne.present, 'camera lead'


@pytest.mark.parametrize('op_long', [False, True])
def test_silence_past_the_limit_is_a_soft_disable_end_to_end(chain, op_long):
  msg = chain.run(7.9, radar=False)
  assert not RADAR_EVENTS & chain.events(op_long)
  msg = chain.run(0.5, radar=False)
  assert msg.valid and msg.radarState.radarErrors.radarFault
  names = chain.events(op_long)
  assert EventName.radarFault in names and EventName.canError not in names
  assert ET.SOFT_DISABLE in EVENTS[EventName.radarFault]


@pytest.mark.parametrize('op_long', [False, True])
def test_second_silence_inside_the_window_is_a_soft_disable_end_to_end(chain, op_long):
  chain.run(1.0, radar=False)
  chain.run(20.0)
  assert not RADAR_EVENTS & chain.events(op_long)
  chain.run(1.0, radar=False)
  names = chain.events(op_long)
  assert EventName.radarFault in names and EventName.canError not in names


@pytest.mark.parametrize('op_long', [False, True])
def test_invalid_radar_data_is_an_immediate_disable_end_to_end(chain, op_long):
  msg = chain.run(3.0, status_len=7)                 # 0x301 filtered: the radar parser goes invalid
  assert msg.radarState.radarErrors.canError and not msg.valid, 'radarTracks.valid=False makes radarState invalid'
  assert not chain.rd.tracks
  names = chain.events(op_long)
  assert EventName.canError in names and ET.IMMEDIATE_DISABLE in EVENTS[EventName.canError]


# *** U5: radarTracks alive and rate count in radarState validity (BogPilot test_ap1_radartracks_min_freq.py) ***

@pytest.mark.parametrize('period', [1 / 8.7, 1 / 12.5, 1 / 16.7])
def test_logged_radar_rates_keep_radar_state_valid(period):
  chain = Chain()
  assert chain.run(8.0, period=period).valid


def test_radar_slower_than_8_hz_makes_radar_state_invalid_and_is_a_radar_fault():
  chain = Chain()
  chain.run(6.0)
  msg = chain.run(6.0, period=0.2)                   # 5 Hz, never a 0.45 s silence
  assert not msg.valid and not any(msg.radarState.radarErrors.to_dict().values())
  assert EventName.radarFault in chain.events(True)


def test_radar_tracks_stopping_altogether_makes_radar_state_invalid():
  chain = Chain()
  chain.run(6.0)
  msg = chain.run(0.4, card_alive=False)
  assert msg.valid, 'radarTracks still alive inside its 0.5 s window'
  msg = chain.run(0.3, card_alive=False)
  assert not msg.valid and not chain.rd.tracks
  assert EventName.radarFault in chain.events(False)


# *** U3: card publishes the receiver status for the driver warning ***

def test_card_publishes_the_receiver_status_in_the_pedal_companion():
  import inspect
  source = inspect.getsource(Car.state_publish)
  line = next(l.strip() for l in source.splitlines() if 'preapRadarStatus' in l)
  assert line == 'companion.starpilotCarState.preapRadarStatus = int(radar.status()) if radar is not None else 0'
  chain = Chain()
  chain.run(1.0)
  assert chain.ri.preap.status() == 1
  chain.run(6.0)
  assert chain.ri.preap.status() == 0
  chain.run(1.2, radar=False)
  assert chain.ri.preap.status() == 2
  msg = messaging.new_message('starpilotCarState')
  msg.starpilotCarState.preapRadarStatus = int(chain.ri.preap.status())
  from openpilot.starpilot.car.tesla.tests.test_preap_radar_warning import consumer_alerts
  alerts = consumer_alerts(msg.starpilotCarState.preapRadarStatus)
  assert [a.alert_text_1 for a in alerts] == ['Radar Silent'] and alerts[0].event_type == ET.WARNING


# *** N8: stock-cruise pre-AP (no pedal) with the radar configured, under AOL ***

def aol_callers(cleanup):
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
  from openpilot.starpilot.aol.tests.test_vehicle_policy_cache import profile
  cp = profile('tesla')
  cp.radarUnavailable = False
  cp.safetyConfigs[0].safetyParam = 2           # stock cruise with the front radar profile
  assert not cp.openpilotLongitudinalControl
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  cleanup.enter_context(OpenpilotPrefix())
  cleanup.enter_context(mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}))
  return case._authority_callers(False, cp=cp, genuine_events=True)


def radar_error(sd, error):
  original = sd.update_events
  def update(cs):
    radar = messaging.new_message('radarState', valid=True)
    setattr(radar.radarState.radarErrors, error, True)
    sd.sm.data['radarState'] = radar.radarState.as_reader()
    original(cs)
  return mock.patch.object(sd, 'update_events', side_effect=update)


@pytest.mark.parametrize('error,event,keeps_lateral', [('radarFault', EventName.radarFault, True),
                                                       ('radarUnavailableTemporary', EventName.radarTempUnavailable, True),
                                                       ('canError', EventName.canError, False)])
def test_stock_cruise_preap_with_radar_keeps_lateral_on_radar_faults_but_not_on_invalid_radar_data(error, event, keeps_lateral):
  assert (event in LONGITUDINAL_ONLY_EVENTS) == keeps_lateral
  with ExitStack() as cleanup:
    sd, _, step = aol_callers(cleanup)
    for tick in (0, 1):
      axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
    assert axis.lateralActive and command.latActive
    with radar_error(sd, error):
      for tick in range(2, 12):
        axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
        assert event in sd.events.names
    assert (axis.lateralActive and command.latActive) == keeps_lateral
