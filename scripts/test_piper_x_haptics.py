import math
from pathlib import Path
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from piper_x_haptic_core import (Feedback, Gravity, TorqueResponseGuard, PIPER_X_TORQUE_C,
                                joint_frames, limited_target, mit_frame)
from piper_x_haptics import Controller, State, disable_joints, receive_stamped, SO_TIMESTAMPNS
import socket

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT/'config/piper_x_haptics.yaml').read_text())


class HapticTest(unittest.TestCase):
    def test_shutdown_with_invalid_ros_disables_and_closes(self):
        for mode in ('MONITOR', 'TELEOP'):
            with self.subTest(mode=mode):
                c = Controller.__new__(Controller)
                c.mode = mode;c.control = mode != 'MONITOR';c.owned = c.control;c.mask = np.ones(6);c.feedback = Feedback(CFG['feedback'])
                c.guard = None;c.qm = c.qs = np.zeros(6)
                c.sockets = [Mock(), Mock()];c.node = Mock();c.rclpy = Mock()
                c.rclpy.ok.return_value = False
                c.publish_feedback = Mock(side_effect=RuntimeError('ROS context is invalid'))
                c.log = Mock()
                c.close()
                c.publish_feedback.assert_not_called()
                for sock in c.sockets:
                    sock.close.assert_called_once()
                    self.assertEqual(sock.send.call_count, 18 if mode == 'TELEOP' else 0)
                c.node.destroy_node.assert_called_once()
                c.rclpy.try_shutdown.assert_called_once()

    def test_feedback_publish_skips_closed_ros_context(self):
        c = Controller.__new__(Controller)
        c.rclpy = Mock();c.rclpy.ok.return_value = False
        c.fb_pub = Mock();c.fb_gate = Mock()
        c.publish_feedback(np.zeros(6), False)
        c.fb_pub.publish.assert_not_called();c.fb_gate.publish.assert_not_called()

    def test_gravity_matches_potential_gradient(self):
        g = Gravity(CFG['gravity_urdf'], CFG['gripper_xacro'])
        rng = np.random.default_rng(0)
        for _ in range(5):
            q = rng.uniform(g.limits[:, 0], g.limits[:, 1])
            torque, _ = g.evaluate(q)
            for i in range(6):
                a, b = q.copy(), q.copy();a[i] += 1e-6;b[i] -= 1e-6
                derivative = (g.evaluate(a)[1]-g.evaluate(b)[1])/2e-6
                self.assertAlmostEqual(derivative, torque[i], places=6)
            self.assertAlmostEqual(torque[0], 0, places=8)

    def test_v189_encoding_roundtrip(self):
        for tau in [-15, 0, .2, 10]:
            b = mit_frame(.7, .5, tau)
            self.assertEqual(len(b), 8)
            self.assertEqual(((b[3]&15)<<8)|b[4], 0)  # kp = zero
            t = ((b[6]&15)<<8)|b[7]
            self.assertLess(abs(t*32/4095-16-tau), .008)
            self.assertLess(abs(int.from_bytes(b[:2], 'big')*25/65535-12.5-.7), .0004)
        with self.assertRaises(ValueError):mit_frame(0, .5, float('nan'))
        with self.assertRaises(ValueError):mit_frame(0, .5, 20)

    def test_feedback_deadband_limits_slew_stale(self):
        fb = Feedback(CFG['feedback'])
        np.testing.assert_array_equal(fb.update(np.zeros(6), .01, True, True, True, np.ones(6)), np.zeros(6))
        v = fb.update(np.ones(6)*100, .01, True, True, True, np.ones(6))
        self.assertTrue(np.all(v <= 0));self.assertTrue(np.all(abs(v) <= .010001))
        for _ in range(100):v = fb.update(np.ones(6)*100, .01, True, True, True, np.ones(6))
        np.testing.assert_allclose(v, -fb.limits)
        np.testing.assert_array_equal(fb.update(np.ones(6), .01, False, True, True, np.ones(6)), np.zeros(6))
        np.testing.assert_array_equal(fb.update(np.ones(6), .01, True, False, True, np.ones(6)), np.zeros(6))
        with self.assertRaises(ValueError):fb.update(np.ones(6)*np.nan, .01, True, True, True, np.ones(6))

    def test_motion_rate_bound(self):
        limits = np.array([[-2, 2]]*6)
        q = limited_target(np.zeros(6), np.ones(6)*4, .25, .01, limits)
        np.testing.assert_allclose(q, .0025)
        with self.assertRaises(ValueError):limited_target(q, q, .25, .2, limits)

    def test_deselect_joint_clears_previous_feedback_immediately(self):
        fb = Feedback(CFG['feedback'])
        fb.value[:] = -.2
        mask = np.array([1, 0, 0, 0, 0, 0])
        value = fb.update(np.ones(6)*100, .01, True, True, True, mask)
        np.testing.assert_array_equal(value[1:], np.zeros(5))

    def test_can_backlog_preserves_actual_age(self):
        sock = Mock()
        sock.recvmsg.return_value = (bytes(16), [(socket.SOL_SOCKET, SO_TIMESTAMPNS,
                                                struct.pack('@ll', 100, 0))], 0, None)
        with patch('piper_x_haptics.time.time', return_value=100.5), \
             patch('piper_x_haptics.time.monotonic', return_value=20):
            _, arrival = receive_stamped(sock)
        self.assertEqual(arrival, 19.5)
        state = State();state.frames[0x2a5] = (arrival, bytes(8))
        self.assertFalse(state.fresh([0x2a5], 20, .1))
        sock.recvmsg.return_value = (bytes(16), [], 0, None)
        with self.assertRaises(RuntimeError):receive_stamped(sock)

    def test_next_to_master_packets_and_stale_removal_offline(self):
        c = Controller.__new__(Controller)
        c.cfg = CFG;c.mode = 'TELEOP';c.tick_time = 9.99;c.transition = 9
        c.last_pub = c.last_log = 10;c.last_next = c.last_contact = 10
        c.next_tau = np.ones(6)*100;c.contact = True;c.mask = np.ones(6)
        c.feedback = Feedback(CFG['feedback'])
        c.torque_response = [TorqueResponseGuard(), TorqueResponseGuard()]
        c.gravity = Mock();c.gravity.limits = np.array([[-3, 3]]*6)
        c.gravity.evaluate.return_value = (np.zeros(6), 0)
        c.command = c.offset = np.zeros(6)
        c.sockets = [Mock(), Mock()];c.heartbeat = Mock()
        c.states = [State(), State()]
        for state in c.states:
            for i in range(0x2a5, 0x2a8):state.frames[i] = (10, bytes(8))
            for i in range(0x251, 0x257):state.frames[i] = (10, bytes(8))
            for i in range(0x261, 0x267):state.frames[i] = (10, bytes([0,0,0,0,0,0x40,0,0]))
            state.frames[0x2a1] = (10, bytes([1,0,6,0,0,0,0,0]))
        c.states[1].frames[0x2a1] = (10, bytes([1,0,1,0,0,0,0,0]))
        c.tick(10)
        self.assertTrue(np.all(c.feedback.value < 0))
        master_packets = [struct.unpack('=IB3x8s', call.args[0]) for call in c.sockets[0].send.call_args_list]
        self.assertEqual([p[0] for p in master_packets], list(range(0x15a, 0x160)))
        self.assertEqual(c.sockets[1].send.call_count, 3)
        for (_, _, packet), tau, scale in zip(master_packets, c.feedback.value, PIPER_X_TORQUE_C):
            decoded = (((packet[6]&15)<<8)|packet[7])*32/4095-16
            self.assertLess(abs(decoded*scale-tau), .009)
        c.last_next = 0
        c.tick(10.01)
        np.testing.assert_array_equal(c.feedback.value, np.zeros(6))

    def test_state_decode_and_disable_never_holds(self):
        st = State()
        for i, d in joint_frames([.1]*6):
            st.receive(struct.pack('=IB3x8s', i+0x150, 8, d), 1)
        np.testing.assert_allclose(st.q(), [.1]*6, atol=1e-5)
        self.assertTrue(st.fresh(range(0x2a5, 0x2a8), 1.01, .1))
        self.assertFalse(st.fresh(range(0x2a5, 0x2a8), 1.2, .1))
        for i in range(0x261,0x267):st.frames[i] = (1, bytes([0,0,0,0,0,0x40,0,0]))
        self.assertTrue(st.enabled())
        st.frames[0x261] = (1, bytes([0,0,0,0,0,0x41,0,0]))
        self.assertFalse(st.enabled())
        sock = Mock();disable_joints(sock)
        ids = [struct.unpack('=IB3x8s', call.args[0])[0] for call in sock.send.call_args_list]
        self.assertEqual(ids, [0x471]*6)
        for joint, call in enumerate(sock.send.call_args_list, 1):
            self.assertEqual(struct.unpack('=IB3x8s', call.args[0])[2][:2], bytes([joint, 1]))


if __name__ == '__main__':unittest.main()
