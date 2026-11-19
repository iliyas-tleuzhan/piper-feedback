import math
import struct
import unittest

from piper_can_record_bridge import Capture


class CaptureTest(unittest.TestCase):
    def test_signed_feedback_and_piper_x_torque(self):
        capture = Capture()
        for i in range(3):
            capture.receive(0x2A5 + i, struct.pack('>ii', 90000, -90000), 1)
            capture.receive(0x155 + i, struct.pack('>ii', 0, 180000), 1)
        for i in range(6):
            capture.receive(0x251 + i, struct.pack('>hhi', 0, -1000, 0), 1)
        self.assertEqual(capture.missing(1.01), [])
        pos, cmd, torque = capture.values()
        self.assertAlmostEqual(pos[0], math.pi / 2)
        self.assertAlmostEqual(pos[1], -math.pi / 2)
        self.assertAlmostEqual(cmd[1], math.pi)
        for actual, expected in zip(torque, [-4.725, -4.134375, -4.725,
                                             -1.150128, -0.766752, -0.95844]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(len(capture.missing(1.1)), 12)

    def test_feedback_cannot_replace_missing_command(self):
        capture = Capture()
        for can_id in capture.required:
            if can_id >= 0x251:
                capture.receive(can_id, bytes(8), 1)
        self.assertEqual(capture.missing(1.01), [0x155, 0x156, 0x157])
        capture.receive(0x155, bytes(7), 1)
        self.assertIn(0x155, capture.missing(1.01))


if __name__ == '__main__':
    unittest.main()
