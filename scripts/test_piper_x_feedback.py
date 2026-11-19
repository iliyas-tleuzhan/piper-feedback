"""Force-feedback regression checks; no CAN sockets or ROS initialization."""
import json
import unittest
from unittest.mock import Mock, patch
import numpy as np
from piper_x_haptic_core import TorqueResponseGuard, PIPER_X_TORQUE_C
from test_piper_x_auto import fixture, packets, update
from run_piper_x_auto import check_can_queues


class FeedbackPathTests(unittest.TestCase):
    def test_logged_missing_joint_three_torque_stops(self):
        guard = TorqueResponseGuard()
        wanted = np.array([0, 1.977, -5.96, -1.257, .044, 0])
        measured = np.array([-.288, .017, -.043, .267, .14, -.084])
        guard.check(0., wanted, measured, np.zeros(6))
        guard.check(.99, wanted, measured, np.zeros(6))
        with self.assertRaisesRegex(RuntimeError, 'joint\\(s\\) 3'):
            guard.check(1., wanted, measured, np.zeros(6))

    def test_response_recovery_resets_timer(self):
        guard = TorqueResponseGuard()
        requested = np.ones(6)*3
        guard.check(0., requested, np.zeros(6), np.zeros(6))
        guard.check(.9, requested, requested, np.zeros(6))
        guard.check(1., requested, np.zeros(6), np.zeros(6))
        guard.check(1.99, requested, np.zeros(6), np.zeros(6))

    def test_command_wrist_scaling_and_zero_stiffness(self):
        c = fixture()
        torque = np.ones(6)
        c.gravity.evaluate.return_value = torque, 0.
        c.compliant(0, np.zeros(6), np.zeros(6))
        for i, (_, _, data) in enumerate(packets(c.sockets[0])):
            self.assertEqual(((data[3]&15)<<8)|data[4], 0)
            encoded = (((data[6]&15)<<8)|data[7])*32/4095-16
            self.assertLess(abs(encoded*PIPER_X_TORQUE_C[i]-1), .009)

    def test_enable_retry_targets_only_missing_motor(self):
        c = fixture('ENABLING')
        update(c, 10.)
        c.states[0].frames[0x263] = (10., bytes(8))
        c.tick(10.)
        enables = [d for i, _, d in packets(c.sockets[0]) if i == 0x471]
        self.assertEqual(enables, [bytes([3,2,0,0,0,0,0,0])])
        self.assertFalse(any(i == 0x471 for i, _, d in packets(c.sockets[1])))

    def test_queue_preflight_rejects_small_queue(self):
        link = dict(flags=['UP'], txqlen=10,
                    linkinfo=dict(info_data=dict(state='ERROR-ACTIVE', bittiming=dict(bitrate=1000000))))
        with patch('run_piper_x_auto.subprocess.run', return_value=Mock(stdout=json.dumps([link]))):
            with self.assertRaisesRegex(RuntimeError, 'TX queue'):
                check_can_queues()
            link['txqlen'] = 64
        with patch('run_piper_x_auto.subprocess.run', return_value=Mock(stdout=json.dumps([link]))):
            check_can_queues()


if __name__ == '__main__':
    unittest.main()
