"""Monotonic periodic scheduling, without catch-up command bursts."""
import math


class Periodic:
    def __init__(self, hz, start):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError('Frequency must be finite and positive')
        self.period = 1.0/hz
        self.deadline = start

    def advance(self, now):
        # Keep the original phase, skip expired slots, and schedule strictly ahead.
        slots = max(1, math.floor((now-self.deadline)/self.period)+1)
        self.deadline += slots*self.period
        if self.deadline <= now:  # floating-point rounding at a period boundary
            self.deadline += self.period
            slots += 1
        return slots-1


class LoopRate:
    def __init__(self, start):
        self.start = start
        self.previous = None
        self.count = self.skipped = 0
        self.max_gap = self.max_late = 0.0

    def sample(self, now, deadline):
        if self.previous is not None:
            self.max_gap = max(self.max_gap, now-self.previous)
        self.previous = now
        self.count += 1
        self.max_late = max(self.max_late, max(0., now-deadline))

    def report(self, now, target):
        elapsed = now-self.start
        if elapsed < 2:
            return None
        message = (f'CONTROL RATE: {self.count/elapsed:.1f} Hz / target {target:g} Hz; '
                   f'max interval {self.max_gap*1000:.1f} ms; '
                   f'max lateness {self.max_late*1000:.1f} ms; skipped slots {self.skipped}')
        self.start = now
        self.count = self.skipped = 0
        self.max_gap = self.max_late = 0.
        return message
