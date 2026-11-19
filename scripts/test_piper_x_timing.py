import unittest

from piper_x_timing import Periodic, LoopRate


class TimingTests(unittest.TestCase):
    def test_250hz_stays_on_phase_despite_work_time(self):
        clock = Periodic(250, 0.)
        for i in range(1000):
            self.assertAlmostEqual(clock.deadline, i*.004, places=10)
            self.assertEqual(clock.advance(clock.deadline+.001), 0)

    def test_overrun_skips_slots_without_burst(self):
        clock = Periodic(250, 0.)
        self.assertEqual(clock.advance(.013), 3)
        self.assertAlmostEqual(clock.deadline, .016)
        self.assertGreater(clock.deadline, .013)

    def test_rate_reports_actual_cycles_and_delays(self):
        rate = LoopRate(0.)
        for i in range(500):rate.sample((i+1)*.004, (i+1)*.004)
        report = rate.report(2., 250)
        self.assertIn('250.0 Hz', report)
        self.assertIn('max interval 4.0 ms', report)
        self.assertIn('skipped slots 0', report)

    def test_50hz_schedule_does_not_inherit_tick_jitter(self):
        last_pub = 0.
        publications = []
        import math
        for i in range(1, 2501):
            now = i*.004 + .0001*(i % 3)
            if now-last_pub >= .02:
                last_pub += max(1, math.floor((now-last_pub)/.02))*.02
                publications.append(now)
        self.assertLessEqual(abs(len(publications)-500), 1)


if __name__ == '__main__':
    unittest.main()
