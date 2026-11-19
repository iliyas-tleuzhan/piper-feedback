"""Receive backlog and startup regressions. No hardware access."""
from collections import deque
import struct
import unittest
from unittest.mock import Mock

from piper_x_haptics import drain_feedback, State
from test_piper_x_auto import fixture, update, packets


class ReceiveTests(unittest.TestCase):
    def test_backlog_drains_both_arms_preserving_old_arrival(self):
        sockets = [object(), object()]
        queues = {sock: deque((struct.pack('=IB3x8s', 0x2a5, 8, bytes(8)), n*.001)
                             for n in range(100)) for sock in sockets}
        states = [State(), State()]
        order = []
        def receive(sock):
            if not queues[sock]:
                raise BlockingIOError
            return queues[sock].popleft()
        def consume(i, frame, arrival):
            order.append(i)
            states[i].receive(frame, arrival)
        # An overdue command deadline no longer limits this to one frame per arm.
        drain_feedback(sockets, sockets, consume, clock=lambda: 10., receiver=receive)
        self.assertEqual(order, [0, 1]*100)
        for state in states:
            self.assertEqual(state.frames[0x2a5][0], .099)
            self.assertFalse(state.fresh([0x2a5], 10., .1))

    def test_flood_service_is_bounded_and_fair(self):
        sockets = [object(), object()]
        consume = Mock()
        drain_feedback(sockets, sockets, consume, clock=lambda: 0.,
                       receiver=lambda _: (bytes(16), 0.))
        self.assertEqual(consume.call_count, 256)
        consume.reset_mock()
        times = iter([0., .003])
        drain_feedback(sockets, sockets, consume, clock=lambda: next(times),
                       receiver=lambda _: (bytes(16), 0.))
        self.assertEqual(consume.call_count, 2)

    def test_ros_initialization_not_charged_to_first_control_tick(self):
        c = fixture('STARTUP');c.owned = False
        c.reset_loop_clock(100.)
        c.tick(100.001)
        self.assertEqual(c.mode, 'PREPARE')
        # Real deadline misses still fault after startup.
        with self.assertRaisesRegex(RuntimeError, '100 ms deadline'):
            c.tick(100.2)

    def test_prepare_retries_remaining_enabled_joint_without_enabling(self):
        c = fixture('PREPARE')
        update(c, 10., enabled=False)
        c.states[0].frames[0x262] = (10., bytes([0,0,0,0,0,0x40,0,0]))
        c.tick(10.)
        self.assertEqual(c.mode, 'PREPARE')
        self.assertEqual([(i, d[:2]) for i, _, d in packets(c.sockets[0])],
                         [(0x471, bytes([2,1]))])
        c.sockets[0].reset_mock()
        c.tick(10.01)
        c.sockets[0].send.assert_not_called()

    def test_slave_startup_speed_guard_kept_with_specific_diagnostic(self):
        c = fixture('ENABLING')
        c.states[1].frames[0x253] = (10., struct.pack('>hhi', 1600, 0, 0))
        with self.assertRaisesRegex(RuntimeError, 'slave joint 3 speed 1.600 rad/s'):
            c.tick(10.)
        for sock in c.sockets:
            sock.send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
