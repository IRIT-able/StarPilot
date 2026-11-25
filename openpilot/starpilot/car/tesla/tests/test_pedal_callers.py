"""Actual selected startup -> physical CAN -> Card/SD/Controls/Card.apply -> registered native."""
import gc
from unittest.mock import patch
import pytest

from opendbc.car import structs
from opendbc.car.tesla.tests.preap_stream import PreAPStream
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import log, messaging
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD, EventName, ET
from openpilot.starpilot.aol.wire import SafetyState, PandaSlot, encode_safety
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed
from openpilot.starpilot.tests.test_tesla_screen_integration import selfdrive_step
from openpilot.starpilot.car.tesla.tests.pedal_fixture import CALIBRATIONS, actual_pedal_startup, configure_native, feedback_frame


# gas_wire: released on the tick pandad reads the panda, before the car's next 0x551 re-arms it, so the receipt says
# "LONG not allowed" for 100 ms with the gas already up (replay C2a). gas_state5: the car is silent for 240 ms of
# override, the interceptor drops to STATE 5 and still reports it once after the commands resume (replay C2b). The
# same wake frame follows the first command from idle, so without the selfdrived classification this case already fails
# at the initial engagement (controlsMismatch at tick 78).
# long_withheld: from tick 130 the panda's receipt keeps LONG denied with healthy feedback and no driver input (a panda
# that never re-grants); the bounded classification must turn that into the lost-permission fault after 1 s.
BAD_WINDOWS = {'gas_wire': (126, 130), 'gas_state5': (124, 150), 'long_withheld': (130, 310)}


FAULTS = ['gas', 'brake', 'dropout', 'reverse', 'feedback_state', 'counter', 'channels', 'wrong_bus',
                                   'eps', 'door', 'belt', 'eps_temporary',
                                   # carstate plumbing: broad brake and the DI brake bit into the engagement logic, hands-on
                                   # level 2, a missing chassis message, and a blocked command stream (latch, then a new pull)
                                   'brake_status_0', 'brake_status_3', 'di_brake', 'hands2', 'can_missing', 'echo_loss',
                                   # gas release seen through pandad's 10 Hz receipt (still "LONG not allowed"), and a long
                                   # press after which the interceptor still reports its STATE 5 timeout once re-acquired
                                   'gas_wire', 'gas_state5', 'long_withheld',
                                   # one stalk pull only: lateral without pedal LONG must stay up, with no fault
                                   'single_pull',
                                   # radar-bus interrupt runaway while engaged: the live bus-1 bound soft-disables
                                   'bus1_runaway']


@pytest.mark.parametrize('bus', [0, 2])
@pytest.mark.parametrize('radar_mode', ['off', 'front', 'nosecone'])
@pytest.mark.parametrize('fault,cal', [(fault, 'owner') for fault in FAULTS] +
                         [(fault, 'unit') for fault in ('gas', 'brake', 'gas_state5', 'single_pull', 'bus1_runaway')])
@pytest.mark.parametrize('aol', [False, True])
def test_real_selected_pedal_fault_withdrawal_and_physical_reengagement(bus, radar_mode, fault, aol, cal, monkeypatch):
  safety = libsafety_py.libsafety
  release = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) != 0
  modelled = fault in ('gas_wire', 'gas_state5')
  with actual_pedal_startup(bus, radar_mode, release, monkeypatch, always_on_lateral=aol or modelled,
                            calibration=cal) as (card, controls, calibration, cp_bytes):
    ci, word = card.CI, int(card.CP.safetyConfigs[0].safetyParam)
    configure_native(safety, calibration, word, cp_bytes)
    with patch('openpilot.selfdrive.selfdrived.selfdrived.REPLAY', False):
      sd = SelfdriveD(CP=card.CP)
    stream = PreAPStream()
    previous = structs.CarControl()
    receipt = None
    stale_wire = fault == 'gas_wire'
    gas = fault in ('gas', 'gas_wire', 'gas_state5')
    mismatch_ticks, enabled_ticks, bus1_ticks, soft_ticks = [], [], [], []
    selfdriven, observed, diagnostics = {}, {}, {}
    alerts_seen = set()
    radar_statuses = set()
    echoes = []
    # The owner's interceptor (logged): STATE 5 from about 200 ms after the last 0x551, and still 5 for 4-45 ms after
    # the commands resume (40 ms here). Used by the two cases modelled on the replayed drives.
    last_command, stream_start = None, None
    try:
      for tick in range(800 if fault == 'bus1_runaway' else 310):
        now = 2_000_000_000 + tick * 10_000_000
        # Fault window; the two cases modelled on the replayed drives are described at BAD_WINDOWS.
        start, end = BAD_WINDOWS.get(fault, (96 if fault == 'echo_loss' else 120,
                                             180 if fault in ('dropout', 'reverse', 'wrong_bus', 'can_missing') else 140))
        bad = start <= tick < end
        brake = 2 if bad and fault == 'brake' else 0 if bad and fault == 'brake_status_0' else 3 if bad and fault == 'brake_status_3' else 1
        lever = (2 if tick == 65 else 0) if fault == 'single_pull' else \
                2 if tick in (65, 75) or tick in (220, 230) and fault not in ('long_withheld', 'bus1_runaway') else \
                1 if tick == 275 and fault != 'bus1_runaway' else 2 if fault == 'bus1_runaway' and tick in (770, 780) else 0
        incoming = stream.frames(tick, lever=lever, cruise=1, brake=brake, di_brake=int(bad and fault == 'di_brake'),
          hands=2 if bad and fault == 'hands2' else 0, omit='ESP_B' if bad and fault == 'can_missing' else None,
          gear=2 if bad and fault == 'reverse' else 4,
          doors=1 if bad and fault == 'door' else 0, belt=0 if bad and fault == 'belt' else 1,
          eps_status=3 if bad and fault == 'eps' else 0 if bad and fault == 'eps_temporary' else 1,
          eps_error=15 if bad and fault == 'eps' else 0)
        if tick % 2 == 0 and not (bad and fault == 'dropout'):
          counter = 59 if bad and fault == 'counter' else tick // 2
          incoming.append(feedback_frame(calibration, counter, di=8. if bad and gas else CALIBRATIONS[cal][2],
            state=1 if bad and fault == 'feedback_state' else
                  (0 if last_command is not None and tick - last_command <= 20 and tick - stream_start >= 4 else 5) if modelled else 0,
            bad_channel=bad and fault == 'channels', wrong_bus=bad and fault == 'wrong_bus'))
        safety.set_timer(now // 1000)
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        for address, data, source in incoming:
          safety.safety_rx_hook(libsafety_py.make_CANPacket(address, source, data))
        # The panda returns every 0x551 frame it accepted and sent (source bus + 128); the car watches for that.
        if not (bad and fault == 'echo_loss'):
          incoming.extend(echoes)
        echoes = []
        safety.safety_tick()
        peers = []
        cc = messaging.new_message('carControl', valid=True, logMonoTime=now)
        cc.carControl = previous
        peers.append(cc.as_reader())
        peers.append(selfdriven['onroadEvents'] if selfdriven else
                     messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=now).as_reader())
        panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
        panda.pandaStates[0].safetyModel = card.CP.safetyConfigs[0].safetyModel
        panda.pandaStates[0].safetyParam = word
        panda.pandaStates[0].alternativeExperience = 32
        panda.pandaStates[0].controlsAllowed = bool(safety.get_controls_allowed())
        panda.pandaStates[0].safetyRxChecksInvalid = not bool(safety.safety_config_valid())
        # Bus-1 interrupt rate as pandad reports it: the logged normal level, or a storm-level runaway from tick 300.
        panda.pandaStates[0].canState1.irq0CallRate = 21_551 if fault == 'bus1_runaway' and tick >= 300 else 1_315
        peers.append(panda.as_reader())
        if receipt is not None:
          peers.append(receipt.as_reader())
        if selfdriven:
          peers.extend(selfdriven[key] for key in ('aolAxisState', 'selfdriveState'))
        card.sm.update_msgs(now / 1e9, peers)
        card.CC_prev = previous
        with patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[b'can']), \
             patch('openpilot.selfdrive.car.card.can_capnp_to_list', return_value=[(now, incoming)]), \
             patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now), \
             patch('openpilot.selfdrive.car.card.time.clock_gettime_ns', return_value=now), \
             patch('openpilot.selfdrive.car.card.REPLAY', False), patch.object(card.sm, 'update'):
          cs, radar = card.state_update()
        published = {}
        factory = messaging.new_message

        def stamped(service, size=None, factory=factory, now=now, **kwargs):
          return factory(service, size, **({'logMonoTime': now} | kwargs))

        with patch('openpilot.selfdrive.car.card.messaging.new_message', side_effect=stamped), \
             patch.object(card.pm, 'send', side_effect=lambda service, msg, published=published: published.update({service: msg.as_reader()})):
          card.state_publish(cs, radar)
        published['pandaStates'] = panda.as_reader()
        assert published['starpilotCarState'].logMonoTime == published['carState'].logMonoTime
        status = published['starpilotCarState'].starpilotCarState
        assert status.sourceCarStateMonoTime == published['carState'].logMonoTime
        assert status.pedalLongActive == ci.CS.preap.enableLongControl
        assert status.pedalMaxRegen == (ci.CS.preap.pccEvent == 'pedalMaxRegen')
        # The receiver's status reaches the companion through the real state_publish; this radar never speaks.
        receiver = getattr(card.RI, 'preap', None)
        assert status.preapRadarStatus == (int(receiver.status()) if receiver is not None else 0)
        radar_statuses.add(int(status.preapRadarStatus))
        mask = safety.aol_get_permission_mask() & (1 if bad and fault == 'long_withheld' else 3)
        refresh = not stale_wire or tick % 10 == 0 or receipt is None
        validity = 200_000_000 if stale_wire else 30_000_000
        if refresh:
          native = SafetyState(1, True, now, now + validity, 39, word, bool(mask & 1), bool(mask & 2),
                               bool(safety.aol_get_request_mask() & 1), bool(safety.aol_get_request_mask() & 2),
                               'tesla-pedal-native', sd.aol_session_id, now,
                               (PandaSlot(0, 'tesla-pedal-native', 39, word, 32, bool(safety.get_controls_allowed()),
                                          not bool(safety.safety_config_valid())),))
          receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
          receipt.aolSafetyWire = encode_safety(native)

        def with_actual_companion(service, size=None, published=published, factory=factory, **kwargs):
          if service == 'starpilotCarState' and service in published:
            return published[service].as_builder()
          return factory(service, size, **kwargs)

        with patch('openpilot.selfdrive.selfdrived.selfdrived.messaging.new_message', side_effect=with_actual_companion):
          selfdriven.update(selfdrive_step(sd, published, receipt, previous, card.CP, safety, now))
        alerts_seen.update(alert.alert_type for alert in sd.tesla_stock_alerts)
        if EventName.controlsMismatch in sd.events.names or sd.events.contains(ET.IMMEDIATE_DISABLE):
          mismatch_ticks.append(tick)
        wanted = sd.aol_axis_decision
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        safety.aol_set_host_request(int(wanted.desired_lateral) | (int(wanted.desired_longitudinal) << 1))
        mask = safety.aol_get_permission_mask() & (1 if bad and fault == 'long_withheld' else 3)
        if refresh:
          native = SafetyState(1, True, now, now + validity, 39, word, bool(mask & 1), bool(mask & 2),
                               wanted.desired_lateral, wanted.desired_longitudinal, 'tesla-pedal-native', sd.aol_session_id, now,
                               (PandaSlot(0, 'tesla-pedal-native', 39, word, 32, bool(safety.get_controls_allowed()),
                                          not bool(safety.safety_config_valid())),))
          receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
          receipt.aolSafetyWire = encode_safety(native)
        feed(controls, now, tick, active=sd.active, enabled=sd.enabled)
        controls.sm.update_msgs((now + 1000) / 1e9, [published['carState'],
          selfdriven['aolAxisState'], selfdriven['selfdriveState'], selfdriven['onroadEvents'], receipt.as_reader()])
        command, lateral = controls.state_control()
        controls.publish(command, lateral)
        current = messaging.new_message('carControl', valid=True, logMonoTime=now)
        current.carControl = command
        card.sm.update_msgs((now + 1000) / 1e9, [current.as_reader(), selfdriven['aolAxisState'], selfdriven['selfdriveState'], receipt.as_reader()])
        sent = {}
        with patch('openpilot.selfdrive.car.card.time.monotonic', return_value=(now + 1000) / 1e9), \
             patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now + 1000), \
             patch.object(card.pm, 'send', side_effect=lambda service, msg, sent=sent: sent.update({service:
               messaging.log_from_bytes(msg) if isinstance(msg, bytes) else msg.as_reader()})):
          card.controls_update(cs, command.as_reader())
        outgoing = [(item.address, bytes(item.dat), item.src) for item in sent['sendcan'].sendcan] if 'sendcan' in sent else []
        enabled_pedal = False
        for address, data, source in outgoing:
          assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, source, data)), (fault, tick, hex(address), data.hex())
          if address == 0x551:
            if last_command is None or tick - last_command > 20:
              stream_start = tick
            last_command = tick
            assert source == bus and len(data) == 6
            echoes.append((address, data, source + 128))
            enabled_pedal |= bool(data[4] & 128)
        assert not command.longActive or bool(mask & 2)
        assert not enabled_pedal or command.longActive
        if bad and fault in ('gas', 'gas_wire', 'gas_state5', 'brake', 'reverse', 'feedback_state', 'counter', 'channels', 'eps', 'door', 'belt',
                             'brake_status_0', 'brake_status_3', 'di_brake', 'hands2', 'can_missing', 'echo_loss') and \
           tick >= (145 if fault == 'can_missing' else 122):   # a missing message: canValid falls after the parser's 10 periods
          assert not enabled_pedal
        if bad and fault == 'eps_temporary' and tick >= 122:
          assert not command.latActive, 'temporary EPS inhibition must suppress lateral commands'
        observed[tick] = (bool(command.longActive), enabled_pedal, bool(cs.canValid), bool(mask & 2), bool(command.latActive))
        enabled_ticks.append((tick, sd.enabled))
        if EventName.sensorDataInvalid in sd.events.names:
          bus1_ticks.append(tick)
          alerts_seen.update(f'shown:{entry.alert.alert_text_2}' for entry in sd.AM.alerts.values()
                             if entry.active(sd.sm.frame) and entry.alert.alert_type.startswith('sensorDataInvalid/'))
        if sd.state_machine.state == log.SelfdriveState.OpenpilotState.softDisabling:
          soft_ticks.append(tick)
        if tick in (64, 65, 66, 74, 75, 76, 95, 100, 110, 140, 145, 150, 155, 160, 250):
          diagnostics[tick] = (cs.cruiseState.enabled, ci.CS.preap.enableLongControl, card.aol_card_intent.allowed_latch, sd.enabled,
                               str(wanted), sd.events.names, mask, observed[tick], command.actuators.accel)
        previous = command
      if fault == 'single_pull':
        lateral = [t for t in range(100, 310) if observed[t][4]]
        assert not any(observed[t][0] or observed[t][1] for t in range(310)), diagnostics
        assert not [t for t in mismatch_ticks if t >= 60], (mismatch_ticks, diagnostics)
        # Lateral through the AOL latch for the whole drive; selfdrived is never "enabled", so the 1 s LONG-denial bound
        # (enabled only) cannot end it.
        assert lateral == list(range(100, 310)), (lateral[:5], diagnostics)
        assert not any(enabled for t, enabled in enabled_ticks), diagnostics
        return
      assert any(observed[t][0] and observed[t][1] for t in range(95, 115)), ('actual initial pedal engagement missing', diagnostics)
      if fault == 'bus1_runaway':
        # Reported above the bound from tick 300; the 2.1 s settling after the mode change is long over. Trip after
        # 1.3 s, then the ordinary 3 s soft disable (no immediate disable); a new double pull while it lasts does not
        # engage (no entry), and nothing is commanded after the disable.
        assert bus1_ticks and 430 <= bus1_ticks[0] <= 432 and bus1_ticks == list(range(bus1_ticks[0], 800)), bus1_ticks[:3]
        assert not [t for t in mismatch_ticks if t >= 60], (mismatch_ticks, diagnostics)
        assert all(enabled for t, enabled in enabled_ticks if 100 <= t < bus1_ticks[0]), diagnostics
        assert soft_ticks and soft_ticks[0] == bus1_ticks[0] and len(soft_ticks) >= 290, (soft_ticks[:2], len(soft_ticks))
        assert all(enabled for t, enabled in enabled_ticks if bus1_ticks[0] <= t < soft_ticks[-1])
        assert not any(enabled for t, enabled in enabled_ticks if t > soft_ticks[-1] + 1), soft_ticks[-1]
        assert 720 <= soft_ticks[-1] < 760
        assert 'shown:Radar Bus Fault' in alerts_seen and 'shown:Sensor Data Invalid' not in alerts_seen, alerts_seen
        assert not any(observed[t][0] or observed[t][1] or observed[t][4] for t in range(soft_ticks[-1] + 2, 800)), diagnostics
        return
      if fault == 'long_withheld':
        # Withdrawn at once (no LONG output), tolerated as ordinary for 1 s, then the immediate-disable fault.
        assert not any(observed[t][0] or observed[t][1] for t in range(132, 230)), diagnostics
        fired = [t for t in mismatch_ticks if t >= 100]
        assert fired and 229 <= fired[0] <= 232, (mismatch_ticks, diagnostics)
        assert all(enabled for t, enabled in enabled_ticks if 100 <= t < 229)
        assert not any(enabled for t, enabled in enabled_ticks if fired[0] < t < 300), (
          [t for t, enabled in enabled_ticks if enabled and t > fired[0]], mismatch_ticks)
        return
      # Off: nothing to warn about. Configured but never heard from: not ready (1), then silent (2) after 1 s.
      assert radar_statuses == ({0} if radar_mode == 'off' else {1, 2}), radar_statuses
      assert 'teslaPreapPedal/engaged/warning' in alerts_seen
      assert 'teslaPreapPedal/disengaged/warning' in alerts_seen
      assert not any(name.startswith('teslaStockCruise/') for name in alerts_seen)
      if gas:
        # A gas override and its release are the driver's, not a fault: the engagement stays, no immediate disable.
        assert not [t for t in mismatch_ticks if 100 <= t < 220], (mismatch_ticks, diagnostics)
        assert all(sd_enabled for t, sd_enabled in enabled_ticks if 100 <= t < 220), (enabled_ticks, diagnostics)
      if not gas and fault != 'eps_temporary':
        assert not any(observed[t][0] or observed[t][1] for t in range(200, 219)), 'fault/gear/brake recovery must not auto-resume long'
      else:
        assert any(observed[t][0] and observed[t][1] for t in range(160, 175)), (
          'healthy override/temporary-pause recovery did not preserve existing active long')
      assert any(observed[t][0] and observed[t][1] for t in range(250, 270)), 'fresh physical reengagement missing'
      assert not any(observed[t][0] or observed[t][1] for t in range(278, 310)), 'physical cancel must withdraw pedal long'
    finally:
      safety.set_aol_test_heartbeat(False)
      safety.set_safety_hooks(0, 0)
      del sd
      gc.collect()
