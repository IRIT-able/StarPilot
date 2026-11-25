from opendbc.car import structs
from opendbc.car.tesla.preap.aol import qualified, temporary_restriction as preap_temporary_restriction
from openpilot.starpilot.aol.policy import AolVehiclePolicy
from openpilot.starpilot.car.tesla.stock_intent import TeslaPreapCardIntent
from opendbc.car.tesla.screen_button import modern_aol_supported, modern_aol_qualified, AOL_WORDS
from openpilot.starpilot.car.tesla.screen_intent import TeslaScreenCardIntent


def policy_for(cp):
  if modern_aol_supported(cp):
    admitted = modern_aol_qualified(cp)
    return AolVehiclePolicy(intent_supported=admitted, settings_supported=True,
                            runtime_supported=admitted, normal_runtime_supported=admitted,
                            ordinary_axis_ack_required=modern_aol_qualified(cp, marked_only=True),
                            full_axis_runtime_required=admitted,
                            cruise_main_required=not modern_aol_qualified(cp, marked_only=True),
                            explicit_latch=admitted, alternative_experience_addition=32 if admitted else 0)
  admitted = qualified(cp)
  return AolVehiclePolicy(intent_supported=admitted, settings_supported=admitted,
                          runtime_supported=admitted, normal_runtime_supported=admitted,
                          ordinary_axis_ack_required=qualified(cp, marked_only=True),
                          explicit_latch=admitted, physical_stalk_owner=admitted, fixed_cruise_buttons=admitted,
                          full_axis_runtime_required=admitted and cp.openpilotLongitudinalControl,
                          alternative_experience_addition=32 if admitted else 0)


def native_profile_supported(model, param):
  return (model == 39 and param in (0, 1, 2, 3, 6, 7)) or (model == 10 and param in AOL_WORDS)


def native_accepts_cp(cp, model, param):
  return bool((qualified(cp, marked_only=True) or modern_aol_qualified(cp, marked_only=True)) and
              native_profile_supported(model, param) and len(cp.safetyConfigs) == 1 and
              int(cp.safetyConfigs[0].safetyModel.raw) == model and int(cp.safetyConfigs[0].safetyParam) == param)


def native_latch_rejected(cp, native):
  return (qualified(cp, marked_only=True) or modern_aol_qualified(cp, marked_only=True)) and native.requestedLateral and not native.lateralAllowed


def ordinary_axis_request_allowed(cp, state):
  return bool((qualified(cp, marked_only=True) or modern_aol_qualified(cp, marked_only=True)) and
              state.canValid and not state.canTimeout and not state.steerFaultPermanent)


def retain_on_native_denial(cp, native, state):
  return preap_temporary_restriction(cp, state) or bool(
    modern_aol_qualified(cp, marked_only=True) and state is not None and state.canValid and not state.canTimeout and
    not state.steerFaultPermanent and not state.steeringDisengage and not state.doorOpen and not state.accFaulted and
    (state.gearShifter in (structs.CarState.GearShifter.reverse, structs.CarState.GearShifter.neutral) or
     state.gearShifter == structs.CarState.GearShifter.drive and state.steerFaultTemporary))


def early_native_long_withdrawal_supported(cp):
  """Pre-AP pedal: the panda's LONG bit also requires no gas (0x552 above 2 DI) and a fresh disabled
  0x551 after any interceptor STATE 5 frame. Both edges reach the panda before card publishes them and
  the pandad receipt is up to 100 ms old, so a LONG-only denial after a gas release is ordinary."""
  from opendbc.car.tesla.preap.pedal_profile import qualified as pedal_qualified
  return bool(pedal_qualified(cp) and qualified(cp, marked_only=True))


LONG_DENIAL_LIMIT_NS = 1_000_000_000


def long_denial_expired(owner, cp, native, requested, *, enabled, driver_override, now_ns):
  """Bound on the ordinary classification above. Re-arming after a gas release takes one disabled 0x551 (20 ms), the
  interceptor's wake (4-45 ms logged) and one pandad receipt (100 ms). A LONG request the panda keeps denying for
  1 s with no driver input is not that, and becomes the lost-permission fault it was before the classification."""
  denied = bool(early_native_long_withdrawal_supported(cp) and enabled and native is not None and
                requested.desired_longitudinal and native.requestedLongitudinal and not native.longitudinalAllowed and
                not driver_override)
  if not denied:
    owner.preap_long_denied_since_ns = 0
    return False
  since = getattr(owner, 'preap_long_denied_since_ns', 0) or now_ns
  owner.preap_long_denied_since_ns = since
  return now_ns - since >= LONG_DENIAL_LIMIT_NS


def create_intent(cp, settings):
  return TeslaScreenCardIntent(cp, settings) if modern_aol_qualified(cp) else TeslaPreapCardIntent(cp, settings)
