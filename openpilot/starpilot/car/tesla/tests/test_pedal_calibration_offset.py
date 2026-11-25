"""Saved calibration -> frozen PedalCalibration: both references place 0 DI one unit below the saved onset point."""
import pytest

from openpilot.starpilot.car.tesla import pedal_preferences
from openpilot.starpilot.car.tesla.pedal_preferences import read_pedal_settings


def settings(monkeypatch, factor, zero):
  saved = {'NAPPedalEnabled': b'1', 'NAPPedalCalibDone': b'1', 'NAPPedalCanBus': b'2',
           'NAPPedalCalibFactor': factor, 'NAPPedalCalibZero': zero, 'LongitudinalPersonality': b'1'}
  monkeypatch.setattr(pedal_preferences, 'read_saved', lambda params, name, size: (saved.get(name), True))
  return read_pedal_settings(None)


@pytest.mark.parametrize('factor,zero', [(b'1.25', b'24.5'), (b'0.9', b'30.0'), (b'2.0', b'1.0')])
def test_zero_is_onset_minus_one_di(monkeypatch, factor, zero):
  result = settings(monkeypatch, factor, zero)
  assert result.valid and result.enabled
  cal = result.calibration
  assert cal.zero == pytest.approx(float(zero) - 1.0 / float(factor), abs=1e-5)
  assert cal.di(float(zero)) == pytest.approx(1.0, abs=1e-4), 'the saved point reads as 1 DI, as in NAP and BogPilot'
  assert cal.physical(0.) == pytest.approx(cal.zero)


def test_default_calibration_values_are_not_trusted(monkeypatch):
  result = settings(monkeypatch, b'1.0', b'0.0')
  assert result.enabled and not result.valid


@pytest.mark.parametrize('factor,zero', [(b'0.8', b'30.0'), (b'3.1', b'1.0'), (b'-1.0', b'100.0'), (b'nan', b'1.0'), (b'0', b'1.0'),
                                         (b'1.0', b'120.0')])
def test_implausible_calibration_is_refused(monkeypatch, factor, zero):
  # BogPilot: factor 0.82-3.0, and a resting pedal below the command cap.
  result = settings(monkeypatch, factor, zero)
  assert result.enabled and not result.valid


def test_saved_onset_at_the_command_limit_is_refused(monkeypatch):
  # raw 2450 is 101.6 physical units: the saved onset itself must be below it (BogPilot), not only the coast point.
  assert settings(monkeypatch, b'1.0', b'101.0').valid
  assert not settings(monkeypatch, b'1.0', b'101.7').valid
  assert not settings(monkeypatch, b'1.0', b'102.3').valid


OWNER_FACTOR, OWNER_ZERO = b'0.973', b'6.062'   # harness session, models/pedal_calib.json (factor, zero_gp); to be
                                                 # confirmed against the device's /data/params_bogpilot


def test_owner_calibration_coast_zero_and_logged_rest_pedal(monkeypatch):
  import json, os
  from opendbc.car.tesla.preap.pedal_contract import D, M1
  result = settings(monkeypatch, OWNER_FACTOR, OWNER_ZERO)
  assert result.valid and result.enabled
  cal = result.calibration
  assert cal.zero == pytest.approx(6.062 - 1.0 / 0.973, abs=1e-4) == pytest.approx(5.0343, abs=1e-4)   # BogPilot calib.py
  assert round((cal.zero - D) / M1) == 549                                         # coast point, raw channel 1
  path = os.path.join(os.path.dirname(pedal_preferences.__file__), '..', '..', '..', '..', 'opendbc_repo', 'opendbc',
                      'safety', 'tests', 'tesla_preap_pedal_rest_frames.json')
  rest = [cal.di(one * M1 + D) for one, _ in json.load(open(os.path.normpath(path)))]
  # Logged resting pedal (1,790 frames): well below the panda's 2 DI gas threshold and inside its -5 DI floor margin.
  assert max(rest) < 2.0 and -6.2 < min(rest) and -4.7 < sorted(rest)[len(rest) // 2] < -4.4
