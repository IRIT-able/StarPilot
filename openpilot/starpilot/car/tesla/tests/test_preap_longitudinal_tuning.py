"""Pre-AP longitudinal feedback ownership contracts (NAP preap/tests/test_longitudinal_tuning.py).

NAP asserts kpV and kiV all zero and kf == 1.0 on the params from get_preap_params. StarPilot's CarParams has no
kpV or kf: its LongControl has no proportional term unless an extension supplies one and feeds the target forward
1:1. The same contract is asserted on the params and the LongControl that the actual pedal startup builds.
"""
import pytest
from opendbc.car import structs
from openpilot.starpilot.car.tesla.tests.pedal_fixture import actual_pedal_startup


def test_pedal_params_leave_generic_outer_feedback_disabled(monkeypatch):
  with actual_pedal_startup(2, 'off', False, monkeypatch) as (card, controls, calibration, cp_bytes):
    configured_params = card.CP

    assert list(configured_params.longitudinalTuning.kiV) == [0.0] * 4
    loc = controls.LoC
    assert loc.extension is None
    assert loc.pid.k_p == 0.0
    # kf == 1.0: the acceleration target is the output, with no feedback contribution despite a tracking error.
    state = structs.CarState(vEgo=20., aEgo=-0.5, gearShifter='drive', canValid=True)
    for _ in range(50):
      output = loc.update(True, state, .2, False, (-1.5, 1.))
    assert output == pytest.approx(.2)
    assert loc.pid.i == 0.0
