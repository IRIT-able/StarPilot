"""selfdrived mapping of radar errors for the pre-AP radar retrofit: invalid radar data disengages at once (both
references), radar faults count with or without pedal longitudinal (NAP), other cars keep StarPilot's mapping."""
from unittest.mock import patch

import pytest

from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.tesla.interface import CarInterface as TeslaInterface
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, EventName
from openpilot.selfdrive.selfdrived.tests.test_jetlink_event_faults import events_drive


def events_for(cp, error):
  drive, cs = events_drive()
  drive.CP = cp
  with patch('openpilot.selfdrive.selfdrived.helpers.time.monotonic', return_value=100.):
    drive.update_events(cs)
    setattr(drive.sm['radarState'].radarErrors, error, True)
    drive.update_events(cs)
  return set(drive.events.names)


def preap(op_long, radar=True):
  cp = TeslaInterface.get_non_essential_params('TESLA_MODEL_S_PREAP')
  cp.radarUnavailable = not radar
  cp.openpilotLongitudinalControl = op_long
  return cp


@pytest.mark.parametrize('op_long', [False, True])
def test_preap_radar_can_error_is_an_immediate_disable(op_long):
  names = events_for(preap(op_long), 'canError')
  assert EventName.canError in names and ET.IMMEDIATE_DISABLE in EVENTS[EventName.canError]


@pytest.mark.parametrize('op_long', [False, True])
def test_preap_radar_fault_is_a_soft_disable_with_or_without_pedal_longitudinal(op_long):
  names = events_for(preap(op_long), 'radarFault')
  assert EventName.radarFault in names and EventName.canError not in names
  assert ET.SOFT_DISABLE in EVENTS[EventName.radarFault] and ET.IMMEDIATE_DISABLE not in EVENTS[EventName.radarFault]


def test_preap_without_the_radar_retrofit_and_without_pedal_ignores_radar_errors():
  assert not {EventName.canError, EventName.radarFault} & events_for(preap(False, radar=False), 'canError')


@pytest.mark.parametrize('op_long,expected', [(False, False), (True, True)])
def test_other_cars_keep_their_mapping(op_long, expected):
  cp = HondaInterface.get_non_essential_params(HONDA.HONDA_CIVIC)
  cp.openpilotLongitudinalControl = op_long
  names = events_for(cp, 'canError')
  assert EventName.canError not in names and (EventName.radarFault in names) == expected
