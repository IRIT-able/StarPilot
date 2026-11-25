"""Actual selected Card startup and guarded native pedal configuration helpers."""
import hashlib
import struct
from contextlib import contextmanager
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.fingerprints import FINGERPRINTS
from opendbc.car.tesla.values import CAR
from opendbc.car.tesla.preap.pedal_contract import M1, M2, D
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.controlsd import Controls


OWNER_FACTOR, OWNER_ZERO = 0.973, 6.062
REST_DI = -4.55        # the released pedal at the owner's calibration (median of the logged rest frames)
# name -> (saved factor, saved zero, where the released pedal reads in DI). 'unit' is the earlier fixture calibration.
CALIBRATIONS = {'owner': (OWNER_FACTOR, OWNER_ZERO, REST_DI), 'unit': (1.0, 1.0, 0.0)}


def native_words(calibration, word, cp_bytes):
  prefix = struct.pack('<HHHHff', 1, calibration.bus, word, 0xD051, calibration.factor, calibration.zero)
  return struct.unpack('<24H', prefix + hashlib.sha256(cp_bytes).digest())


def configure_native(safety, calibration, word, cp_bytes):
  safety.init_tests()
  assert safety.set_safety_hooks(0, 0) == 0
  safety.set_aol_test_heartbeat(False)
  safety.set_alternative_experience(32)
  words = native_words(calibration, word, cp_bytes)
  for i, value in enumerate(words):
    assert safety.tesla_preap_test_config_word(i, value)
  assert safety.tesla_preap_test_config_word(0xFFFF, 0xC051)
  assert safety.set_safety_hooks(39, word) == 0
  assert not safety.tesla_preap_test_config_word(0, 1), 'car-mode calibration mutation must fail'


def feedback_frame(calibration, counter, *, di=0., state=0, wrong_bus=False, bad_channel=False,
                   raw_one=None, raw_two=None, bad_checksum=False):
  physical = calibration.physical(di)
  one = int((physical - D) / M1) if raw_one is None else raw_one
  two = (int((physical - D) / M2) + (200 if bad_channel else 0)) if raw_two is None else raw_two
  prefix = struct.pack('>HHB', one, two, ((state & 15) << 4) | (counter & 15))
  # Byte 5 is the interceptor's sum checksum (0x52 + 0x05 + bytes 0..4), valid on every logged frame.
  checksum = (0x52 + 0x05 + sum(prefix) + (1 if bad_checksum else 0)) & 255
  return 0x552, prefix + bytes([checksum]), 2 - calibration.bus if wrong_bus else calibration.bus


@contextmanager
def actual_pedal_startup(bus, radar_mode, release, monkeypatch, always_on_lateral=False, calibration='owner'):
  from opendbc.car import car_helpers
  from openpilot.selfdrive.car import card as card_module
  from openpilot.starpilot.car.tesla.pedal_startup import KEY, decode
  for name, value in (('SIMULATION', '1'), ('REPLAY', '1'), ('AOL_REPLAY_RUNTIME', '1')):
    monkeypatch.setenv(name, value)
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = dict(FINGERPRINTS[CAR.TESLA_MODEL_S_PREAP][0])
  with OpenpilotPrefix():
    params = Params()
    for key, value in (('OpenpilotEnabledToggle', True), ('AlwaysOnLateral', always_on_lateral),
                       ('NAPPedalEnabled', True), ('NAPPedalCalibDone', True), ('IsReleaseBranch', release)):
      params.put_bool(key, value, block=True)
    params.put('NAPPedalCanBus', bus, block=True)
    # The owner's calibration (harness session, models/pedal_calib.json: factor, zero_gp), saved as BogPilot/NAP store it;
    # 0 DI (coast) is 1/factor below the saved zero. To be confirmed against the device's /data/params_bogpilot.
    params.put('NAPPedalCalibFactor', CALIBRATIONS[calibration][0], block=True)
    params.put('NAPPedalCalibZero', CALIBRATIONS[calibration][1], block=True)
    params.put('TeslaPreAPRadarConfig', {'version': 1, 'mode': radar_mode, 'offset': .25}, block=True)
    observed = (CAR.TESLA_MODEL_S_PREAP, fingerprint, '0' * 17, [], structs.CarParams.FingerprintSource.can, True)
    with patch.object(car_helpers, 'fingerprint', return_value=observed), \
         patch.object(card_module, 'can_comm_callbacks', return_value=(lambda *args: [], lambda frames: None)), \
         patch.object(card_module.messaging, 'recv_one_retry', return_value=messaging.new_message('can', 1)):
      card = Car()
    word = {'off': 1, 'front': 3, 'nosecone': 7}[radar_mode]
    assert card.CP.openpilotLongitudinalControl and not card.CP.pcmCruise
    assert card.CP.safetyConfigs[0].safetyParam == word and card.CP.alternativeExperience == 32
    assert card.CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap
    assert card.CP.radarUnavailable == (radar_mode == 'off')
    cp_bytes = params.get('CarParams')
    calibration = decode(card.CP, cp_bytes, params.get(KEY))
    assert calibration is not None and calibration.bus == bus
    assert card.CI.CS.preap.pedal_feedback.calibration == calibration
    assert card.CI.CC.preap.pedal_control.codec.calibration == calibration
    controls = Controls()
    assert controls.CP.to_dict() == card.CP.to_dict()
    # Later saved edits cannot change the per-drive parser/controller contract.
    params.put('NAPPedalCalibFactor', 2.0, block=True)
    params.put('NAPPedalCanBus', 2 - bus, block=True)
    assert card.CI.CS.preap.pedal_feedback.calibration == calibration
    yield card, controls, calibration, cp_bytes
