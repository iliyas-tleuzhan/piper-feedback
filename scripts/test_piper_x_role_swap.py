"""Role switching tests use fake sockets and no ROS or physical hardware."""
import unittest
from unittest.mock import Mock, patch

from canframe_swap import role_indices, swap_roles
from piper_x_role_swap import ContactGate, RoleSwap, validate
from test_piper_x_native import frames
from test_piper_x_haptics import CFG as NATIVE_CFG

CFG = dict(teleop_hz=250, feedback_hz=50, contact_on=2., contact_off=1.,
           contact_on_seconds=.08, contact_off_seconds=.3, next_timeout=.3,
           warmup_timeout=15., role_timeout=3.)


class RoleTests(unittest.TestCase):
    def test_roles_and_wire_format_both_directions(self):
        for mode, order in [('teleop', (0, 1)), ('feedback', (1, 0))]:
            sockets = [Mock(), Mock()]
            self.assertEqual(role_indices(mode), order)
            swap_roles(sockets, mode)
            for index, sock in enumerate(sockets):
                packets = frames(sock)
                self.assertEqual([i for i, _, _ in packets[:6]], [0x471]*6)
                self.assertTrue(all(d[1] == 1 for _, _, d in packets[:6]))
                self.assertEqual(packets[-1], (0x470, 8, bytes([
                    0xfa if index == order[0] else 0xfc, 0, 0, 0, 0, 0, 0, 0])))

    def test_invalid_mode_sends_nothing(self):
        sockets = [Mock(), Mock()]
        with self.assertRaises(ValueError):
            swap_roles(sockets, 'bad')
        for s in sockets:
            s.send.assert_not_called()

    def test_contact_dwell_hysteresis_and_repeated_cycles(self):
        gate = ContactGate(CFG)
        for start in (0, 2, 4):
            gate.reset()
            self.assertIsNone(gate.update([2, 0, 0, 0, 0, 0], start, 'teleop'))
            self.assertEqual(gate.update([-2, 0, 0, 0, 0, 0], start+.09, 'teleop'), 'feedback')
            gate.reset()
            self.assertIsNone(gate.update([1.5, 0, 0, 0, 0, 0], start+.1, 'feedback'))
            self.assertIsNone(gate.update([0]*6, start+.2, 'feedback'))
            self.assertIsNone(gate.update([0]*6, start+.4, 'feedback'))
            self.assertEqual(gate.update([0]*6, start+.51, 'feedback'), 'teleop')

    def test_gaps_and_spikes_cannot_complete_dwell(self):
        gate = ContactGate(CFG)
        gate.update([3]*6, 0, 'teleop')
        self.assertIsNone(gate.update([0]*6, .04, 'teleop'))
        self.assertIsNone(gate.update([3]*6, .09, 'teleop'))
        self.assertIsNone(gate.update([3]*6, 1, 'teleop'))
        for bad in ([float('nan')]*6, [float('inf')]*6, [0]*5):
            with self.assertRaises(RuntimeError):
                gate.update(bad, 2, 'teleop')

    def make(self):
        c = RoleSwap.__new__(RoleSwap)
        c.cfg, c.native_cfg = CFG, NATIVE_CFG
        c.sockets = [Mock(), Mock()]
        c.node = Mock()
        c.node.get_clock().now().nanoseconds = 10_000_000_000
        c.gate = ContactGate(CFG)
        c.controller = None
        c.transition('teleop')
        return c

    def test_transition_discards_old_state_and_reverses_sockets(self):
        c = self.make()
        c.controller.target = [1]*6
        c.capture.frames[0x155] = (0, bytes(8))
        c.transition('feedback')
        self.assertIs(c.controller.sockets[0], c.sockets[1])
        self.assertIs(c.controller.sockets[1], c.sockets[0])
        self.assertIsNone(c.controller.target)
        self.assertFalse(c.capture.frames)
        self.assertIsNone(c.gate.last)
        c.transition('teleop')
        self.assertIs(c.controller.sockets[0], c.sockets[0])

    def test_can0_is_always_captured_and_pretransition_packets_ignored(self):
        import struct
        c = self.make()
        for mode in ('teleop', 'feedback'):
            c.transition(mode)
            packet = struct.pack('=IB3x8s', 0x155, 8, bytes(8))
            c.receive(1, packet, c.started-1)
            self.assertFalse(c.capture.frames)
            c.receive(0, packet, c.started+.01)
            self.assertFalse(c.capture.frames)
            c.receive(1, packet, c.started+.01)
            self.assertIn(0x155, c.capture.frames)

    def test_missing_master_times_out(self):
        c = self.make()
        with self.assertRaisesRegex(RuntimeError, 'firmware may require restart'):
            c.tick(c.started+3.1)

    def test_stale_next_faults_instead_of_reverting_to_teleop(self):
        c = self.make()
        c.controller = Mock(target=[0]*6, phase='TELEOP')
        c.controller.last_sent_payloads = {}
        c.controller.states = [Mock(), Mock()]
        c.controller.states[1].frames = {0x2a1: (c.started, bytes([1,0,1,0,0,0,0,0]))}
        c.publish = Mock()
        c.gate.last = c.started
        with self.assertRaisesRegex(RuntimeError, 'NEXT estimates stale'):
            c.tick(c.started+.31)

    def test_old_duplicate_and_future_predictions_are_ignored(self):
        c = self.make()
        c.controller.phase = 'TELEOP'
        msg = Mock(position=[0]*6)
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        c.estimate(msg)
        self.assertIsNone(c.gate.last)
        c.node.get_clock().now().nanoseconds = 10_200_000_000
        msg.header.stamp.nanosec = 100_000_000
        c.estimate(msg)
        last = c.gate.last
        c.estimate(msg)
        self.assertEqual(c.gate.last, last)
        msg.header.stamp.sec = 11
        c.estimate(msg)
        self.assertEqual(c.gate.last, last)

    def test_tick_performs_both_transitions_and_resets_detector(self):
        c = self.make()
        for mode in ('feedback', 'teleop'):
            old = c.controller
            old.tick = Mock()
            old.phase = 'TELEOP'
            old.target = [0]*6
            old.states[1].frames[0x2a1] = (c.started, bytes([1, 0, 1, 0, 0, 0, 0, 0]))
            c.publish = Mock()
            c.gate.last = c.started
            c.pending = mode
            self.assertTrue(c.tick(c.started+.01))
            self.assertEqual(c.mode, mode)
            self.assertIsNot(c.controller, old)
            self.assertIsNone(c.gate.last)
            self.assertEqual(c.controller.phase, 'WAIT_ROLE')

    def test_unconfirmed_follower_mode_faults(self):
        c = self.make()
        c.controller.tick = Mock()
        c.controller.phase = 'TELEOP'
        c.controller.target = [0]*6
        c.controller.states[1].frames[0x2a1] = (c.started, bytes([2, 0, 1, 0, 0, 0, 0, 0]))
        with self.assertRaisesRegex(RuntimeError, 'did not confirm CAN control mode'):
            c.tick(c.started+.01)

    def test_manual_alignment_does_not_consume_next_warmup(self):
        c = self.make()
        c.controller = Mock(target=[0]*6, phase='WAIT_ALIGN')
        c.controller.last_sent_payloads = {}
        c.publish = Mock()
        self.assertFalse(c.tick(c.started+60))
        self.assertIsNone(c.next_warmup_started)
        c.controller.phase = 'TELEOP'
        c.controller.last_sent_payloads = {}
        c.controller.states = [Mock(), Mock()]
        c.controller.states[1].frames = {0x2a1: (c.started+60, bytes([1,0,1,0,0,0,0,0]))}
        self.assertFalse(c.tick(c.started+61))
        self.assertEqual(c.next_warmup_started,c.started+61)
        with self.assertRaisesRegex(RuntimeError, 'warmup timed out'):
            c.tick(c.started+77)

    def test_own_transmitted_commands_complete_next_sample_without_echo(self):
        from types import SimpleNamespace
        from piper_x_haptic_core import joint_frames
        import struct
        c = self.make()
        c.controller.phase = 'TELEOP'
        c.JointState = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None))
        c.pubs = [Mock() for _ in range(4)]
        c.node.get_clock().now().to_msg.return_value = 'stamp'
        for step in (1, 2, 3):
            now = c.started+step*.02
            # Only real telemetry arrives on RX, never the controller's own TX.
            for identifier, data in joint_frames([.1]*6):
                c.capture.receive(identifier+0x150, data, now)
            for identifier in range(0x251,0x257):
                c.capture.receive(identifier, struct.pack('>hhi',0,100,0), now)
            with patch('piper_x_native_teleop.time.monotonic', return_value=now):
                c.controller.send_position([.2]*6)
            c.record_sent_commands()
            self.assertFalse(c.capture.missing(now))
            c.publish(now)
        self.assertEqual(c.published_samples, 2)
        command = c.pubs[2].publish.call_args.args[0].position
        self.assertAlmostEqual(command[0],.2,places=4)
        self.assertNotEqual(command,c.pubs[0].publish.call_args.args[0].position)

    def test_failed_tx_is_not_recorded_as_model_input(self):
        c = self.make()
        c.controller.sockets[1].send.side_effect = OSError('disconnected')
        with self.assertRaises(OSError):
            c.controller.send_position([.2]*6)
        self.assertFalse(c.controller.last_sent_payloads)

    def test_automatic_role_swap_never_disables(self):
        c = self.make()
        for sock in c.sockets:
            self.assertTrue(all(i == 0x470 for i,_,_ in frames(sock)))

    def test_hold_repeats_frozen_target_without_enabling_or_disabling(self):
        c = self.make()
        c.controller.command = [0.1]*6
        c.hold_target = c.hold_once()
        for sock in c.sockets:sock.reset_mock()
        c.controller.command = [0.5]*6
        c.hold_tick();c.hold_tick()
        self.assertEqual([i for i,_,_ in frames(c.sockets[1])], [0x155,0x156,0x157]*2)
        self.assertEqual(frames(c.sockets[1])[:3],frames(c.sockets[1])[3:])
        c.sockets[0].send.assert_not_called()

    def test_watchdog_parent_loss_requests_hold_without_disable(self):
        from piper_x_role_swap import hold_watchdog
        pipe = Mock()
        pipe.poll.return_value = True
        pipe.recv.side_effect = [(1,[.1]*6), EOFError()]
        sockets = [Mock(),Mock()]
        with patch('piper_x_role_swap.bus', side_effect=sockets):
            hold_watchdog(pipe,['can1','can0'])
        sockets[0].send.assert_not_called()
        self.assertEqual([i for i,_,_ in frames(sockets[1])], [0x155,0x156,0x157])
        for sock in sockets:sock.close.assert_called_once()

    def test_zero_release_dwell_returns_on_first_low_sample(self):
        cfg = dict(CFG, contact_off_seconds=0.)
        validate(cfg)
        gate = ContactGate(cfg)
        self.assertEqual(gate.update([0]*6,10.,'feedback'),'teleop')

    def test_feedback_deadline_returns_even_before_reverse_arm_ready(self):
        c = self.make();c.transition('feedback')
        c.controller.tick = Mock(side_effect=AssertionError('must return before tick'))
        self.assertTrue(c.tick(c.started+.501))
        self.assertEqual(c.mode,'teleop')
        self.assertEqual(c.controller.phase,'WAIT_ROLE')
        self.assertFalse(c.feedback_armed)

    def test_release_during_handover_returns_without_aligning(self):
        c = self.make();c.transition('feedback')
        c.pending='teleop'
        c.controller.tick=Mock(side_effect=AssertionError('must not move on release'))
        self.assertTrue(c.tick(c.started+.01))
        self.assertEqual(c.mode,'teleop')
        self.assertTrue(c.controller.skip_alignment)

    def test_feedback_fault_returns_to_default_roles(self):
        c=self.make();c.transition('feedback')
        c.controller.tick=Mock(side_effect=RuntimeError('stale'))
        self.assertTrue(c.tick(c.started+.01))
        self.assertEqual(c.mode,'teleop')

    def test_return_requires_release_before_feedback_can_rearm(self):
        c=self.make();c.transition('feedback');c.transition('teleop')
        c.controller.phase='TELEOP'
        from types import SimpleNamespace
        for seq, torque in enumerate(([3]*6,[0]*6),1):
            c.node.get_clock().now().nanoseconds=10_000_000_000+seq*20_000_000
            msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                sec=10,nanosec=seq*20_000_000)),position=torque)
            c.estimate(msg)
            self.assertEqual(c.feedback_armed,seq==2)
            self.assertIsNone(c.pending)

    def test_threshold_validation(self):
        validate(CFG)
        for change in ({'contact_off': 3}, {'teleop_hz': 0}, {'feedback_hz': float('nan')}):
            with self.assertRaises(ValueError):
                validate(dict(CFG, **change))


if __name__ == '__main__':
    unittest.main()
