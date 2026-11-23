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
