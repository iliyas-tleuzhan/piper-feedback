"""Automatic controller transition tests. Fake CAN sockets only."""
import math
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np

from piper_x_haptics import Controller, State, watchdog
from piper_x_haptic_core import Feedback, TorqueResponseGuard, joint_frames
from test_piper_x_haptics import CFG


def fixture(mode='WAIT_ALIGN'):
    c = Controller.__new__(Controller)
    c.cfg = CFG;c.control = True;c.owned = True;c.mode = mode
    c.tick_time = 9.99;c.transition = 9.;c.last_pub = c.last_log = 10.
    c.mask = np.zeros(6);c.feedback = Feedback(CFG['feedback'])
    c.torque_response = [TorqueResponseGuard(), TorqueResponseGuard()]
    c.last_enable_retry = -math.inf
    c.last_disable_retry = -math.inf
    c.gravity = Mock();c.gravity.limits = np.array([[-3, 3]]*6)
    c.gravity.evaluate.return_value = (np.zeros(6), 0.)
    c.states = [State(), State()];c.sockets = [Mock(), Mock()]
    c.guard = None;c.heartbeat = Mock();c.start_guard = Mock();c.log = Mock()
    c.publish_feedback = Mock();c.prev_sample = None;c.aligned_since = None
    c.qm = c.qs = c.command = np.zeros(6)
    c.last_next = c.last_contact = -math.inf;c.next_tau = np.zeros(6);c.contact = False
    update(c, 10.)
    return c


def update(c, now, master=0., slave=0., enabled=True, slave_mode=None):
    if slave_mode is None:
        slave_mode = 1 if c.mode in ('ALIGNING', 'TELEOP') else 6
    for index, state in enumerate(c.states):
        state.firmware = bytearray(b'S-V1.8-9')
        for identifier, data in joint_frames([master if index == 0 else slave]*6):
            state.frames[identifier+0x150] = (now, data)
        for identifier in range(0x251, 0x257):state.frames[identifier] = (now, bytes(8))
        for identifier in range(0x261, 0x267):
            state.frames[identifier] = (now, bytes([0,0,0,0,0,0x40 if enabled else 0,0,0]))
        state.frames[0x2a1] = (now, bytes([1,0,6 if index == 0 else slave_mode,0,0,0,0,0]))
    c.tick_time = now-.01
    c.last_pub = now # Separate publisher/history tests exercise ROS message construction.


def packets(sock):
    return [struct.unpack('=IB3x8s', call.args[0]) for call in sock.send.call_args_list]


class AutoTests(unittest.TestCase):
    def test_mode_refresh_reasserts_mit_on_master_only_position_on_aligned_slave(self):
        for mode, slave_mode in (('WAIT_ALIGN', 6), ('ALIGNING', 1), ('TELEOP', 1)):
            c = fixture(mode);c.last_mode_refresh = 9.
            c.refresh_modes(10.)
            self.assertEqual(packets(c.sockets[0])[0][2], bytes([1,6,10,0xad,0,1,0,0]))
            self.assertEqual(packets(c.sockets[1])[0][2][1], slave_mode)
            for sock in c.sockets:sock.reset_mock()
            c.refresh_modes(10.05)
            for sock in c.sockets:sock.send.assert_not_called()
            c.refresh_modes(10.11)
            for sock in c.sockets:self.assertEqual(sock.send.call_count, 1)

    def test_watchdog_parent_loss_disables_instead_of_hold(self):
        pipe = Mock();pipe.poll.return_value = True
        pipe.recv.side_effect = [True, EOFError()]
        sockets = [Mock(), Mock()]
        with patch('piper_x_haptics.bus', side_effect=sockets), patch('piper_x_haptics.time.sleep'):
            watchdog(pipe, ['can1', 'can0'])
        for sock in sockets:
            p = packets(sock)
            self.assertEqual(len(p), 18)
            self.assertTrue(all(i == 0x471 and d[1] == 1 for i, _, d in p))
            sock.close.assert_called_once()

    def test_startup_releases_old_hold_without_native_targets(self):
        c = fixture('STARTUP');c.owned = False
        c.tick(10)
        self.assertEqual(c.mode, 'PREPARE')
        for sock in c.sockets:
            p = packets(sock)
            self.assertEqual([item[0] for item in p[:6]], [0x471]*6)
            self.assertTrue(all(item[2][1] == 1 for item in p[:6]))
            self.assertFalse(any(item[0] in (0x155,0x156,0x157) for item in p))
            self.assertEqual(p[-1][2][1], 6)
        update(c, 10.01, enabled=False)
        c.tick(10.01)
        self.assertEqual(c.mode, 'ENABLING')
        update(c, 10.02)
        c.tick(10.02)
        self.assertEqual(c.mode, 'WAIT_ALIGN')

    def test_above_or_exactly_ten_degrees_sends_only_zero_stiffness(self):
        for degrees in (10, 15):
            c = fixture();update(c, 10., master=math.radians(degrees))
            c.tick(10.)
            self.assertEqual(c.mode, 'WAIT_ALIGN')
            for sock in c.sockets:
                for identifier, _, data in packets(sock):
                    self.assertIn(identifier, range(0x15a, 0x160))
                    self.assertEqual(((data[3]&15)<<8)|data[4], 0)

    def test_wait_alignment_ignores_even_large_fresh_next_feedback(self):
        c = fixture();update(c, 10., master=math.radians(15))
        c.next_tau = np.ones(6)*100;c.contact = True
        c.last_next = c.last_contact = 10.;c.mask[:] = 1
        c.last_log = 0
        c.tick(10.)
        np.testing.assert_array_equal(c.master_expected_torque, np.zeros(6))
        self.assertTrue(any('NEXT feedback OFF' in call.args[0] for call in c.log.call_args_list))
        for _, _, data in packets(c.sockets[0]):
            torque = (((data[6]&15)<<8)|data[7])*32/4095-16
            self.assertLess(abs(torque), .008)

    def test_below_threshold_automatically_aligns_slowly(self):
        c = fixture();update(c, 10., master=math.radians(9))
        c.tick(10.)
        self.assertEqual(c.mode, 'ALIGNING')
        update(c, 10.01, master=math.radians(9))
        c.tick(10.01)
        np.testing.assert_allclose(c.command, np.ones(6)*.0005)
        self.assertFalse(c.mask.any())

    def test_alignment_escape_returns_to_compliant_wait(self):
        c = fixture('ALIGNING');update(c, 10., master=math.radians(11))
        c.tick(10.)
        self.assertEqual(c.mode, 'WAIT_ALIGN')
        self.assertTrue(any(i == 0x151 and d[1] == 6 for i, _, d in packets(c.sockets[1])))
        self.assertFalse(any(i in (0x155,0x156,0x157) for i, _, _ in packets(c.sockets[1])))

    def test_complete_alignment_requires_dwell_then_automatic_feedback(self):
        c = fixture('ALIGNING')
        for now in (10., 10.1, 10.2):
            update(c, now, master=.005, slave=.004)
            c.tick(now)
            self.assertEqual(c.mode, 'ALIGNING')
        update(c, 10.31, master=.005, slave=.004)
        c.tick(10.31)
        self.assertEqual(c.mode, 'TELEOP')
        self.assertTrue(c.mask.all())
        self.assertEqual(c.last_next, -math.inf)
        update(c, 10.32, master=.02, slave=.004)
        before = c.command.copy();c.tick(10.32)
        self.assertTrue(np.all(c.command-before <= .00250001))

    def test_stale_fault_cleanup_disables_never_holds(self):
        c = fixture('TELEOP');c.tick_time = 10.49
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            c.tick(10.5)
        c.stop('test stale')
        for sock in c.sockets:
            p = packets(sock)
            self.assertEqual(len(p), 18)
            self.assertTrue(all(i == 0x471 and d[1] == 1 for i, _, d in p))
        self.assertEqual(c.mode, 'STOPPED')
        for sock in c.sockets:sock.reset_mock()
        c.tick(10.51)
        for sock in c.sockets:sock.send.assert_not_called()

    def test_missing_firmware_cannot_release_or_enable(self):
        c = fixture('STARTUP');c.owned = False;c.states[0].firmware.clear()
        c.tick(10.)
        self.assertEqual(c.mode, 'STARTUP')
        for sock in c.sockets:sock.send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
