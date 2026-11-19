"""Offline tests: no CAN interfaces are opened."""
import struct
import unittest
from unittest.mock import Mock, patch

import piper_x_can_teleop as relay


class RelayTests(unittest.TestCase):
    def test_packet_allowlist(self):
        allowed = relay.JOINT_IDS + (0x159,)
        payload = bytes(range(8))
        for identifier in allowed:
            self.assertEqual(relay.decode(relay.FRAME.pack(identifier, 8, payload), allowed),
                             (identifier, payload))
        for identifier in (0x151, 0x471, 0x2A5, 0x155 | 0x80000000,
                           0x155 | 0x40000000, 0x155 | 0x20000000):
            self.assertIsNone(relay.decode(relay.FRAME.pack(identifier, 8, payload), allowed))
        self.assertIsNone(relay.decode(relay.FRAME.pack(0x155, 4, payload), allowed))
        self.assertIsNone(relay.decode(b'bad', allowed))
        self.assertIsNone(relay.decode(relay.FRAME.pack(0x159, 8, payload), relay.JOINT_IDS))

    def test_payload_preserved(self):
        bus = Mock()
        payload = struct.pack('>ii', -123456, 98765)
        relay.send(bus, 0x155, payload)
        bus.sendall.assert_called_once_with(relay.FRAME.pack(0x155, 8, payload))

    def test_alignment_requires_all_joints_recent_and_close(self):
        payload = struct.pack('>ii', 10000, -20000)
        master = {i: (10.0, payload) for i in relay.JOINT_IDS}
        slave = {i: (10.0, payload) for i in relay.FEEDBACK_IDS}
        self.assertTrue(relay.aligned(master, slave, 10.1))
        self.assertFalse(relay.aligned(master, slave, 10.3))
        slave[0x2A7] = (10.0, struct.pack('>ii', 18000, -20000))
        self.assertFalse(relay.aligned(master, slave, 10.1))
        del slave[0x2A7]
        self.assertFalse(relay.aligned(master, slave, 10.1))

    def test_default_loop_only_forwards_master_commands(self):
        master_bus, slave_bus = Mock(), Mock()
        master_bus.__enter__ = Mock(return_value=master_bus)
        slave_bus.__enter__ = Mock(return_value=slave_bus)
        master_bus.__exit__ = Mock(return_value=False)
        slave_bus.__exit__ = Mock(return_value=False)
        command = relay.FRAME.pack(0x155, 8, struct.pack('>ii', 123, -456))
        master_bus.recv.return_value = command
        slave_bus.recv.return_value = relay.FRAME.pack(0x2A5, 8, bytes(8))
        handlers = {}

        def select_once_then_stop(*_):
            if not master_bus.recv.called:
                return [master_bus, slave_bus], [], []
            handlers[relay.signal.SIGINT]()
            return [], [], []

        with patch.object(relay, 'open_bus', side_effect=[master_bus, slave_bus]), \
             patch.object(relay.signal, 'signal', side_effect=lambda sig, fn: handlers.update({sig: fn})), \
             patch.object(relay.select, 'select', side_effect=select_once_then_stop), \
             patch('sys.argv', ['piper_x_can_teleop.py']):
            relay.main()
        slave_bus.sendall.assert_called_once_with(command)
        master_bus.sendall.assert_not_called()


if __name__ == '__main__':
    unittest.main()
