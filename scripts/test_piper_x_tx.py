"""CAN backpressure handling without opening hardware sockets."""
import errno
import unittest
from unittest.mock import Mock, patch

from piper_x_haptics import send, disable_joints


class TransmitTests(unittest.TestCase):
    def test_full_queue_retries_same_packet_then_succeeds(self):
        sock = Mock();sock.send.side_effect = [OSError(errno.ENOBUFS, 'full'), 16]
        with patch('piper_x_haptics.time.monotonic', side_effect=[0., .001]), \
             patch('piper_x_haptics.time.sleep') as sleep:
            send(sock, 0x15a, bytes(8))
        self.assertEqual(sock.send.call_count, 2)
        self.assertEqual(sock.send.call_args_list[0], sock.send.call_args_list[1])
        sleep.assert_called_once_with(.0005)

    def test_full_queue_deadline_is_bounded(self):
        sock = Mock();sock.send.side_effect = OSError(errno.ENOBUFS, 'full')
        with patch('piper_x_haptics.time.monotonic', side_effect=[0., .003]):
            with self.assertRaisesRegex(OSError, 'TX queue stalled'):
                send(sock, 0x15a, bytes(8))
        self.assertEqual(sock.send.call_count, 1)

    def test_link_failure_is_not_retried(self):
        sock = Mock();sock.send.side_effect = OSError(errno.ENETDOWN, 'down')
        with self.assertRaises(OSError):send(sock, 0x15a, bytes(8))
        self.assertEqual(sock.send.call_count, 1)

    def test_disable_attempts_every_joint_even_after_failure(self):
        sock = Mock()
        with patch('piper_x_haptics.send', side_effect=[OSError(errno.ENOBUFS, 'full'), None, None, None, None, None]) as tx, \
             patch('piper_x_haptics.time.sleep'):
            with self.assertRaises(OSError):disable_joints(sock)
        self.assertEqual([c.args[2][0] for c in tx.call_args_list], list(range(1, 7)))
        self.assertTrue(all(c.kwargs['timeout'] == .02 for c in tx.call_args_list))


if __name__ == '__main__':
    unittest.main()
