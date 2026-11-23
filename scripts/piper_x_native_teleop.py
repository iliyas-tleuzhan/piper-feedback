#!/usr/bin/env python3
"""Automatic native-leader Piper X teleop; no master MIT/follower commands.

can1 stays native leader (0xFA), can0 follower (0xFC). NEXT is observation only.
"""
import argparse
import fcntl
import math
import multiprocessing as mp
from pathlib import Path
import select
import signal
import time

import numpy as np
import yaml

from piper_x_haptics import State, bus, send, position, disable_joints, receive_stamped, watchdog, drain_feedback
from piper_x_haptic_core import limited_target
from piper_x_timing import Periodic, LoopRate

ROOT = Path(__file__).resolve().parents[1]


class NativeTeleop:
    def __init__(self, sockets, cfg):
        self.sockets, self.cfg = sockets, cfg
        self.feedback_timeout = float(cfg.get('native_feedback_timeout', .5))
        if not math.isfinite(self.feedback_timeout) or self.feedback_timeout < .2:
            raise ValueError('native_feedback_timeout must be finite and at least 0.2 seconds')
        self.states = [State(), State()]
        self.phase = 'WAIT_ALIGN'
        self.target = self.command = None
        self.target_payloads = None
        self.skip_alignment = False
        self.recover_handover = False
        self.gripper_payload = None
        self.last_sent_payloads = {}
        self.sample_since = None
        self.align_target = None
        self.align_next = 0.
        self.complete_since = None
        self.started = self.last_tick = self.last_log = time.monotonic()
        self.transition = self.started
        self.last_mode = self.started
        self.last_enable = -math.inf
        self.owned = False
        # Match the existing Piper X model's limits; no gravity model/torque commands.
        self.limits = np.array([[-2.617994,2.617994], [0,3.141593], [-2.967060,0],
                                [-1.553344,1.553344], [-1.553344,1.553344], [-2.967060,2.967060]])

    def setup(self):
        self.owned = True
        self.hold_before_alignment = True
        self.phase = 'HOLD_STARTUP'
        send(self.sockets[0], 0x470, [0xfa,0,0,0,0,0,0,0])
        send(self.sockets[1], 0x470, [0xfc,0,0,0,0,0,0,0])
        print('NATIVE: can1=MASTER (0xFA), can0=SLAVE (0xFC). NEXT estimates only; no reflected torque.', flush=True)
        print('HOLD_STARTUP: retaining slave enables; waiting for measured pose to establish a position hold.', flush=True)

    def receive(self, index, frame, now):
        self.states[index].receive(frame, now)
        if index == 0:
            import struct
            identifier, size, payload = struct.unpack('=IB3x8s', frame)
            if identifier == 0x159 and size == 8:
                self.gripper_payload = bytes(payload)
        m = self.states[0]
        # A native leader may stop sending while stationary. Only adopt a new
        # target when all three packet pairs were recently received together.
        if index == 0 and m.fresh(range(0x155,0x158), now, .1):
            self.target = m.q(native=True)
            self.target_payloads = {i: m.frames[i][1] for i in range(0x155,0x158)}

    def tick(self, now):
        dt = now-self.last_tick;self.last_tick = now
        if not 0 < dt <= .1:
            raise RuntimeError('Control loop missed its deadline')
        m, s = self.states
        required = [*range(0x2a5,0x2a8),*range(0x261,0x267),0x2a1]
        timeout = self.feedback_timeout
        if not s.fresh(required, now, timeout) or not s.normal():
            if self.phase in ('HOLD_STARTUP', 'WAIT_ALIGN', 'WAIT_ROLE') and now-self.started < 3:
                return
            raise RuntimeError(f'Slave feedback missing/stale/faulted during {self.phase}: '
                               +s.diagnostic(now, required=required, max_age=timeout))
        if not s.fresh(required, now, .2):
            # Allow a bounded feedback gap without advancing targets or retrying
            # enable using old state. The last commanded target remains in effect.
            self.complete_since = None
            return
        if now-self.started > .5 and now-s.last_external_command < .1:
            raise RuntimeError('Another controller is sending slave commands')
        qs = s.q()
        if self.phase == 'HOLD_STARTUP':
            if self.command is None:
                if np.any(qs < self.limits[:,0]-.05) or np.any(qs > self.limits[:,1]+.05):
                    raise RuntimeError('Slave startup pose outside Piper X limits; hold not commanded')
                self.command = qs.copy()
                self.transition = now
                from piper_x_simulate_torque import request_hold
                request_hold(self.sockets[1], self.command)
                return
            self.send_position(self.command)
            stamp, status = s.frames[0x2a1]
            if now-self.transition > 2:
                raise RuntimeError('Slave startup position hold acknowledgement timed out: '+s.diagnostic(now))
            if not (stamp > self.transition and status[0] == 1 and status[2] == 1):
                return
            if not s.enabled():
                if now-self.last_enable >= .1:
                    for joint in range(1,7):
                        if not s.frames[0x260+joint][1][5] & 0x40:
                            send(self.sockets[1],0x471,[joint,2,0,0,0,0,0,0])
                    self.last_enable = now
                return
            self.phase = 'WAIT_ALIGN'
            self.sample_since = None
            print('WAIT_ALIGN: slave holds its startup pose; move the master within 15 degrees to begin alignment.', flush=True)
            return
        if self.phase == 'WAIT_ALIGN' and getattr(self, 'hold_before_alignment', False):
            if not s.enabled():
                raise RuntimeError('Slave driver enable lost while holding for alignment')
            self.send_position(self.command)
        if self.target is None:
            if now-self.last_log > 2:
                self.last_log = now
                print('WAIT_ALIGN: waiting for native master 0x155/156/157; gently move master.', flush=True)
            return
        for source, q in ((f'{self.cfg["slave_can"]} measured', qs), ('leader target', self.target)):
            outside = np.flatnonzero((q < self.limits[:,0]-.05) | (q > self.limits[:,1]+.05))
            if outside.size:
                details = []
                for j in outside:
                    low, high = np.degrees(self.limits[j])
                    details.append(f'joint {j+1}: angle={math.degrees(q[j]):.3f} deg, '
                                   f'limits=[{low:.3f}, {high:.3f}] deg')
                raise RuntimeError('Joint pose outside Piper X limits: '
                                   f'{source}; '+ '; '.join(details)+
                                   f'; tolerance={math.degrees(.05):.3f} deg; phase={self.phase}')
        # Port of piper-wireless-teleop startup: sampled pose, <=15 degrees,
        # fixed target, 0.3 degree steps at 50 Hz, 10 second alignment timeout.
        tolerance = math.radians(15)
        error = float(np.max(abs(self.target-qs)))
        if self.phase == 'WAIT_ROLE':
            # No sampled alignment/ramp during a live handover. Still require
            # fresh targets and reject a discontinuous pose jump.
            if not m.fresh(range(0x155,0x158), now, .5):
                return
            if error > tolerance:
                differences = np.degrees(self.target-qs).round(1).tolist()
                details = f'joint differences (master target minus slave pose), degrees={differences}'
                if self.recover_handover:
                    # Returning control must not turn a geometric mismatch into
                    # an unrecoverable fault. Keep the slave at its current pose
                    # while the operator brings the master back within range.
                    self.command = qs.copy()
                    self.send_position(self.command)
                    self.skip_alignment = False
                    self.phase = 'WAIT_ALIGN'
                    self.sample_since = None
                    print('RECOVER TELEOP: '+details+'. Slave hold target sent; move can1 '
                          'within 15 degrees of can0 to resume via the startup ramp.', flush=True)
                    return
                raise RuntimeError('Role handover pose mismatch exceeds 15 degrees; '+details)
            self.command = qs.copy()
            self.align_target = np.clip(self.target, self.limits[:,0], self.limits[:,1])
            self.send_position(self.command)
            self.phase = 'ENABLING';self.transition = now
            self.last_enable = -math.inf
            return
        if self.phase == 'WAIT_ALIGN':
            if not m.fresh(range(0x155,0x158), now, .5):
                self.sample_since = None
                return
            if self.sample_since is None:
                self.sample_since = now
            if now-self.sample_since < .5:
                return
            if error <= tolerance:
                self.align_target = np.clip(self.target, self.limits[:,0], self.limits[:,1])
                self.command = qs.copy()
                self.send_position(self.command)
                self.phase = 'ENABLING';self.transition = now
                self.last_enable = -math.inf
                print('ENABLING slave; sampled poses within 15 degrees.', flush=True)
            elif now-self.last_log > 2:
                self.last_log = now
                print(f'WAIT_ALIGN: max joint difference {math.degrees(error):.1f} degrees (need <=15).', flush=True)
            return
        if self.phase == 'ENABLING':
            if now-self.transition > 2:
                raise RuntimeError('Slave enable not confirmed: '+s.diagnostic(now))
            if s.enabled():
                self.command = qs.copy()
                if np.max(abs(self.align_target-qs)) > tolerance:
                    self.phase = 'WAIT_ALIGN';self.sample_since = None
                    return
                self.send_position(self.command)
                # Same four MotionCtrl_2 arguments as piper-wireless-teleop.
                send(self.sockets[1],0x151,[1,1,100,0xad,0,0,0,0])
                self.phase='CONFIRM_ROLE' if self.skip_alignment else 'ALIGNING'
                self.transition=now;self.align_next=now
                print('CONFIRM_ROLE: waiting for CAN control acknowledgement; no alignment.' if self.skip_alignment else
                      'ALIGNING: fixed sampled master pose, 0.3 degrees every 20 ms.', flush=True)
            elif now-self.last_enable >= .1:
                send(self.sockets[1],0x471,[7,2,0,0,0,0,0,0])
                self.last_enable = now
            return
        if not s.enabled():
            raise RuntimeError('Slave driver enable lost/faulted')
        if self.phase == 'CONFIRM_ROLE':
            stamp, status = s.frames[0x2a1]
            if stamp > self.transition and status[0] == 1 and status[2] == 1:
                self.phase = 'TELEOP'
                print('TELEOP: live handover confirmed; no alignment.', flush=True)
            elif now-self.transition > .5:
                raise RuntimeError('New slave CAN control acknowledgement timed out')
            return
        if now-self.transition>.5 and s.frames[0x2a1][1][2]!=1:
            raise RuntimeError('Slave joint position mode not confirmed')
        if self.phase == 'ALIGNING':
            if now-self.transition > 10:
                raise RuntimeError(
                    'Alignment exceeded 10 seconds; commanded-to-target error degrees='
                    +str(np.degrees(self.align_target-self.command).round(2).tolist())
                    +'; measured-to-target error degrees='
                    +str(np.degrees(self.align_target-qs).round(2).tolist())
                    +'; '+s.diagnostic(now, required=required, max_age=timeout))
            if now < self.align_next:
                return
            self.align_next = now+.02  # No catch-up bursts after delays.
            if np.max(abs(self.command-self.align_target)) <= math.radians(.5):
                self.command = self.align_target.copy()
                self.send_position(self.command)
                # Re-sample if the operator moved far during the fixed-pose ramp.
                if np.max(abs(self.target-qs)) > tolerance:
                    self.phase='WAIT_ALIGN';self.sample_since=None
                    print('WAIT_ALIGN: master moved away during alignment; re-sampling.', flush=True)
                    return
                self.phase='TELEOP'
                print('TELEOP: forwarding native joint payloads unchanged; no software slew limit.', flush=True)
            else:
                self.command = limited_target(self.command, self.align_target,
                                              math.radians(15), .02, self.limits)
                self.send_position(self.command)
            return
        # The native master may be quiet when stationary. Retain the last complete
        # target, exactly as the wireless receiver holds its last valid command.
        # Position payloads only: never relay master mode/enable/config messages.
        for identifier in range(0x155, 0x158):
            self.send_target(identifier, self.target_payloads[identifier])
        if self.gripper_payload is not None:
            self.send_target(0x159, self.gripper_payload)
        self.command = self.target.copy()

    def send_target(self, identifier, payload):
        send(self.sockets[1], identifier, payload)
        self.last_sent_payloads[identifier] = (time.monotonic(), bytes(payload))

    def send_position(self, q):
        from piper_x_haptic_core import joint_frames
        for identifier, payload in joint_frames(q):
            self.send_target(identifier, payload)

    def hold_poses(self):
        state = self.states[1]
        q = state.q() if state.fresh(range(0x2a5, 0x2a8), time.monotonic(), self.feedback_timeout) else self.command
        return [None, None if q is None else list(q)]

    def close(self):
        if self.owned:
            from piper_x_simulate_torque import request_hold
            q = self.hold_poses()[1]
            if q is not None:
                try:request_hold(self.sockets[1], q)
                except OSError as exc:print('HOLD FAILED:',exc,flush=True)
            print('STOP: slave hold requested where a pose is available; no disable commands. Hold unconfirmed.',flush=True)
        for sock in self.sockets:sock.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/piper_x_haptics.yaml')
    args=parser.parse_args()
    cfg=yaml.safe_load(args.config.read_text())
    if cfg['master_can']==cfg['slave_can']:parser.error('Separate interfaces required')
    with open('/tmp/factr2_piper_x_control.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        sockets=[]
        try:
            sockets=[bus(cfg['master_can'])]
            sockets.append(bus(cfg['slave_can']))
        except BaseException:
            for sock in sockets:sock.close()
            raise
        controller=NativeTeleop(sockets,cfg)
        def interrupted(*_):raise KeyboardInterrupt
        signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
        guard = pipe = None
        try:
            ctx=mp.get_context('spawn')
            receiver,pipe=ctx.Pipe(duplex=False)
            from piper_x_simulate_torque import torque_hold_watchdog
            guard=ctx.Process(target=torque_hold_watchdog,args=(receiver,[cfg['master_can'],cfg['slave_can']]))
            guard.start();receiver.close();pipe.send(controller.hold_poses())
            controller.setup()
            start=time.monotonic();schedule=Periodic(cfg['control_hz'],start);rate=LoopRate(start)
            controller.last_tick=start
            while True:
                ready = select.select(sockets, [], [], max(0, schedule.deadline-time.monotonic()))[0]
                drain_feedback(sockets, ready, controller.receive)
                now=time.monotonic()
                if now>=schedule.deadline:
                    if not guard.is_alive():raise RuntimeError('Independent watchdog exited')
                    pipe.send(controller.hold_poses())
                    rate.sample(now,schedule.deadline);controller.tick(now)
                    completed=time.monotonic();rate.skipped+=schedule.advance(completed)
                    report=rate.report(completed,cfg['control_hz'])
                    if report:print(report,flush=True)
        except KeyboardInterrupt:pass
        except (OSError,RuntimeError,ValueError) as exc:
            print('FAULT:',exc,flush=True);raise SystemExit(1)
        finally:
            try:controller.close()
            finally:
                if pipe is not None:
                    try:pipe.send(None)
                    except (BrokenPipeError,OSError):pass
                    pipe.close()
                if guard is not None:guard.join(timeout=1)


if __name__=='__main__':main()
