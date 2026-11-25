"""Pre-AP Model S: only the CAN2 interrupt-rate diagnostic the logging panda reported on every drive is exempt,
and the native LONG drop on a pedal edge is classified as ordinary only for the exact admitted pedal profile."""
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.aol import qualified
from opendbc.car.tesla.preap.pedal_profile import qualified as pedal_qualified
from opendbc.car.tesla.values import CAR
from openpilot.cereal import log
from openpilot.starpilot.aol.panda_faults import blocking_fault_mask, blocking_fault_list
from openpilot.starpilot.car.tesla.aol import early_native_long_withdrawal_supported

CAN2 = log.PandaState.FaultType.interruptRateCan2


def stock(word=0, alternative=32):
  cp = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  cp.fingerprintSource = structs.CarParams.FingerprintSource.can
  if word in (2, 6):
    cp.radarUnavailable = False
  cp.safetyConfigs[0].safetyParam = word
  cp.alternativeExperience = alternative
  return cp


def pedal(word=3, alternative=32):
  cp = stock(0, alternative)
  cp.openpilotLongitudinalControl, cp.pcmCruise = True, False
  cp.radarUnavailable = word == 1
  cp.safetyConfigs[0].safetyParam = word
  assert pedal_qualified(cp)
  return cp


PROFILES = [('stock', 0, 32), ('stock', 2, 32), ('stock', 6, 0), ('pedal', 1, 32), ('pedal', 3, 32), ('pedal', 7, 0)]


def build(kind, word, alternative):
  cp = (stock if kind == 'stock' else pedal)(word, alternative)
  assert qualified(cp)
  return cp


@pytest.mark.parametrize('kind,word,alternative', PROFILES)
def test_logged_can2_interrupt_rate_alone_is_diagnostic(kind, word, alternative):
  cp = build(kind, word, alternative)
  assert not blocking_fault_mask(1 << 3, cp)
  assert not blocking_fault_list([CAN2], cp)
  assert not blocking_fault_list([], cp) and not blocking_fault_mask(0, cp)


@pytest.mark.parametrize('kind,word,alternative', PROFILES)
@pytest.mark.parametrize('fault,value', tuple(log.PandaState.FaultType.schema.enumerants.items()))
def test_every_other_and_mixed_fault_still_blocks(kind, word, alternative, fault, value):
  cp = build(kind, word, alternative)
  blocks = fault != 'interruptRateCan2'
  assert blocking_fault_mask(1 << value, cp) == blocks
  assert blocking_fault_mask((1 << value) | (1 << 3), cp) == blocks
  assert blocking_fault_list([value], cp) == blocks
  assert blocking_fault_list([CAN2, value], cp) == blocks


@pytest.mark.parametrize('mutation', ['passive', 'dashcam', 'notcar', 'fixed', 'flags', 'extra_panda', 'wrong_model',
                                      'wrong_word', 'other_tesla', 'other_brand'])
def test_other_configurations_cannot_borrow_the_exemption(mutation):
  for cp in (stock(), pedal()):
    if mutation == 'passive':
      cp.passive = True
    elif mutation == 'dashcam':
      cp.dashcamOnly = True
    elif mutation == 'notcar':
      cp.notCar = True
    elif mutation == 'fixed':
      cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed
    elif mutation == 'flags':
      cp.flags = 1
    elif mutation == 'extra_panda':
      cp.safetyConfigs = list(cp.safetyConfigs) + [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.noOutput)]
    elif mutation == 'wrong_model':
      cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.tesla
    elif mutation == 'wrong_word':
      cp.safetyConfigs[0].safetyParam = 4
    elif mutation == 'other_tesla':
      cp.carFingerprint = CAR.TESLA_MODEL_3
    elif mutation == 'other_brand':
      cp.brand = 'toyota'
    assert not qualified(cp)
    assert blocking_fault_mask(1 << 3, cp)
    assert blocking_fault_list([CAN2], cp)


@pytest.mark.parametrize('word', [1, 3, 7])
def test_long_withdrawal_classification_is_the_marked_pedal_profile_only(word):
  assert early_native_long_withdrawal_supported(pedal(word, 32))
  assert not early_native_long_withdrawal_supported(pedal(word, 0))      # not marked for AOL
  for word_stock in (0, 2, 6):
    assert not early_native_long_withdrawal_supported(stock(word_stock))  # stock cruise: no openpilot LONG
  cp = pedal(word, 32)
  cp.passive = True
  assert not early_native_long_withdrawal_supported(cp)


class _Native:
  def __init__(self, allowed, requested=True):
    self.longitudinalAllowed, self.requestedLongitudinal = allowed, requested


class _Decision:
  def __init__(self, desired=True):
    self.desired_longitudinal = desired


def test_long_denial_is_bounded_to_one_second_and_resets_on_any_grant_or_driver_input():
  from types import SimpleNamespace
  from openpilot.starpilot.car.tesla.aol import long_denial_expired, LONG_DENIAL_LIMIT_NS
  cp, owner, t0 = pedal(), SimpleNamespace(), 10_000_000_000

  def step(dt, allowed=False, override=False, enabled=True, cp=cp, requested=True, desired=True):
    return long_denial_expired(owner, cp, _Native(allowed, requested), _Decision(desired), enabled=enabled,
                               driver_override=override, now_ns=t0 + dt)
  assert not step(0) and not step(LONG_DENIAL_LIMIT_NS - 1)
  assert step(LONG_DENIAL_LIMIT_NS)
  for reset in ({'allowed': True}, {'override': True}, {'enabled': False}, {'requested': False}, {'desired': False}):
    owner = SimpleNamespace()
    assert not step(0) and not step(LONG_DENIAL_LIMIT_NS // 2, **reset)
    assert not step(LONG_DENIAL_LIMIT_NS + 1)                     # the timer restarts at the next denied frame
    assert not step(2 * LONG_DENIAL_LIMIT_NS)
    assert step(2 * LONG_DENIAL_LIMIT_NS + 1)
  for other in (stock(), pedal(3, 0)):
    owner = SimpleNamespace()
    assert not step(0, cp=other) and not step(10 * LONG_DENIAL_LIMIT_NS, cp=other)
