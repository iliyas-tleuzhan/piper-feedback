#!/usr/bin/env python3
"""Continuous NEXT torque reflection with fixed software master/slave roles.

Master can_master runs MIT gravity compensation plus bounded reflected torque.
Slave can_slave follows measured can_master angles. No runtime role swaps.
"""
import math
import multiprocessing as mp
import time

import numpy as np

from piper_x_haptics import Controller, bus, position, send


def request_hold(sock, q):
    """Seed a pose before switching from torque to position mode. No enable/disable."""
    position(sock, q)
    send(sock, 0x151, [1, 1, 100, 0xad, 0, 0, 0, 0])
    position(sock, q)


def torque_hold_watchdog(pipe, channels, release_gripper=False):
    sockets = []
    poses = None
    try:
        sockets = [bus(name) for name in channels]
        while pipe.poll(.5):
            try:
                message = pipe.recv()
            except EOFError:
                break
            if message is None:
                return
            poses = message
        if poses is not None:
            if release_gripper:
                try:send(sockets[0],0x159,bytes(8))
                except OSError as exc:print('WATCHDOG GRIPPER RELEASE FAILED:',exc,flush=True)
            for sock, q in zip(sockets, poses):
                if q is not None:
                    try:
                        request_hold(sock, q)
                    except OSError as exc:
                        print('WATCHDOG HOLD FAILED:', exc, flush=True)
    finally:
        for sock in sockets:
            sock.close()
        pipe.close()


class TorqueTeleop(Controller):
    """Reuse verified encoders and NEXT input publication, without contact switching."""
    check_master_pose_limits = False
    software_damping = True

    def update_gripper_feedback(self, now):
        cfg = self.cfg.get('gripper_feedback', {})
        was_active = getattr(self, 'gripper_feedback_effort', 0) > 0
        if not cfg.get('enabled') and not was_active:
            self.gripper_feedback_reason = 'disabled'
            return
        command = bytes(8)
        reason = 'disabled' if not cfg.get('enabled') else 'outside TELEOP'
        if cfg.get('enabled') and self.mode == 'TELEOP':
            reason = 'missing or stale telemetry'
            if all(st.fresh([0x2a8],now,cfg['timeout']) for st in self.states):
                from piper_x_gripper_feedback import evaluate
                command, reason = evaluate(self.states[0].frames[0x2a8][1],
                                           self.states[1].frames[0x2a8][1],cfg,was_active)
        self.gripper_feedback_reason = reason
        # Check release every control tick, even between 50 Hz command updates.
        if now-getattr(self,'last_gripper_feedback',-math.inf) < .02 and not (was_active and command == bytes(8)):
            return
        self.last_gripper_feedback = now
        send(self.sockets[0],0x159,command)
        self.gripper_feedback_effort = int.from_bytes(command[4:6],'big')/1000

    def gripper_diagnostic(self, now):
        readings = []
        for label, state in zip(('can_master', 'can_slave'), self.states):
            frame = state.frames.get(0x2a8)
            if frame is None:
                readings.append(f'{label}=missing')
                continue
            stamp, data = frame
            opening = int.from_bytes(data[:4], 'big', signed=True)/1000
            effort = int.from_bytes(data[4:6], 'big', signed=True)
            readings.append(f'{label} opening_mm={opening:.3f} effort_raw={effort} '
                            f'status=0x{data[6]:02x} age_ms={(now-stamp)*1000:.1f}')
        return ('; gripper: '+ '; '.join(readings)+
                f'; gate={getattr(self,"gripper_feedback_reason","inactive")}'
                f'; command_effort_raw={round(getattr(self,"gripper_feedback_effort",0)*1000)}')

    def feedback_diagnostic(self, now, torque):
        def values(v):
            return str(np.round(v,4).tolist())
        def age(stamp):
            return 'missing' if not math.isfinite(stamp) else f'{(now-stamp)*1000:.1f}'
        velocity, measured = self.states[0].motor()
        return (
            f'TORQUE DIAG: phase={self.mode}; NEXT age_ms={age(self.last_next)}; '
            f'contact={self.contact}; contact age_ms={age(self.last_contact)}; '
            f'NEXT filtered residual Nm={values(self.next_tau)}; '
            f'can_slave measured Nm={values(self.states[1].motor()[1])}; '
            f'can_master gravity Nm={values(self.master_gravity)}; '
            f'can_master modeled damping Nm={values(-np.asarray(self.cfg["master_damping"])*velocity)}; '
            f'reflected Nm={values(torque)}; '
            f'reflection blocked joints={np.flatnonzero(self.reflection_motion_gate.blocked).tolist() if hasattr(self,"reflection_motion_gate") else []} (zero-based); '
            f'can_master modeled total Nm={values(self.master_expected_torque)}; '
            f'can_master measured Nm={values(measured)}'+self.gripper_diagnostic(now))

    def startup(self, now):
        """Retain enabled joints; support can_master and hold can_slave before following."""
        m, s = self.states
        required = [*range(0x2a5,0x2a8), *range(0x251,0x257), *range(0x261,0x267),0x2a1]
        if self.mode == 'STARTUP':
            if now-self.transition > 5:
                raise RuntimeError('Startup firmware/fresh telemetry timed out; no joints disabled')
            if not all(self.cfg['firmware'].encode() in st.firmware and
                       st.fresh(required,now,self.cfg['state_timeout']) and st.normal()
                       for st in self.states):
                return
            if any(now-st.last_external_command < .5 for st in self.states):
                raise RuntimeError('Another controller is commanding the arms')
            self.qm, self.qs = m.q(), s.q()
            self.command = self.qs.copy()
            self.start_guard()
            self.owned = True
            self.mode = 'PREPARE'
            # Seed supported targets before changing modes, then repeat them.
            self.compliant(0,self.qm,m.motor()[0])
            position(self.sockets[1],self.command)
            for sock in self.sockets:
                send(sock,0x470,[0xfc,0,0,0,0,0,0,0])
            send(self.sockets[0],0x151,[1,6,10,0xad,0,0,0,0])
            send(self.sockets[1],0x151,[1,1,100,0xad,0,0,0,0])
            self.compliant(0,self.qm,m.motor()[0])
            position(self.sockets[1],self.command)
            self.transition = now
            self.log('PREPARE: retaining joint enables; can_master gravity support, can_slave pose hold. No disable commands.')
            return
        if not all(st.fresh(required,now,self.cfg['state_timeout']) and st.normal()
                   for st in self.states):
            raise RuntimeError('Startup feedback stale/faulted; requesting holds')
        self.heartbeat()
        self.qm, self.qs = m.q(), s.q()
        self.compliant(0,self.qm,m.motor()[0])
        position(self.sockets[1],self.command)
        confirmed = all(st.frames[0x2a1][0] > self.transition and
                        st.frames[0x2a1][1][0] == 1 and st.frames[0x2a1][1][2] == mode
                        for st,mode in zip(self.states,(6,1)))
        if now-self.transition > 2:
            raise RuntimeError('Supported startup mode/enable acknowledgement timed out')
        if not confirmed:
            return
        if not all(st.enabled() for st in self.states):
            if now-self.last_enable_retry >= .1:
                self.last_enable_retry = now
                for sock,st in zip(self.sockets,self.states):
                    for joint in range(1,7):
                        if not st.frames[0x260+joint][1][5] & 0x40:
                            send(sock,0x471,[joint,2,0,0,0,0,0,0])
            return
        self.mode = 'WAIT_ALIGN'
        self.log('WAIT_ALIGN: can_slave holds its pose; alignment starts within 15 degrees. The master interface remains compliant.')

    def refresh_modes(self, now, force=False):
        # Startup MIT mode only. Once following begins the native follower owns
        # its mode transition; avoid repeated installation-position writes.
        if not force and now-getattr(self,'last_mode_refresh',now) < .1:
            return
        for sock in self.sockets:
            send(sock,0x151,[1,6,10,0xad,0,0,0,0])
        self.last_mode_refresh=now

    def poses_for_hold(self):
        result = []
        for index, state in enumerate(self.states):
            if state.fresh(range(0x2a5, 0x2a8), time.monotonic(), .5):
                q = state.q()
            else:
                q = self.qm if index == 0 else self.command
            result.append(None if q is None else list(q))
        return result

    def start_guard(self):
        ctx = mp.get_context('spawn')
        rx, self.guard_pipe = ctx.Pipe(duplex=False)
        self.guard = ctx.Process(target=torque_hold_watchdog,
                                 args=(rx, [self.cfg['master_can'], self.cfg['slave_can']], True))
        self.guard.start()
        rx.close()
        self.heartbeat()

    def heartbeat(self):
        if self.guard is not None:
            if not self.guard.is_alive():
                raise RuntimeError('Torque hold watchdog exited')
            self.guard_pipe.send(self.poses_for_hold())

    def stop(self, reason, publish=True):
        self.mode = 'STOPPED' if self.control else 'MONITOR'
        self.mask[:] = 0
        self.feedback.value[:] = 0
        self.prev_sample = None
        if self.owned:
            try:send(self.sockets[0],0x159,bytes(8))
            except OSError as exc:self.log('GRIPPER RELEASE FAILED: '+str(exc))
            for sock, q in zip(self.sockets, self.poses_for_hold()):
                if q is None:
                    self.log('HOLD unavailable: no measured or commanded pose')
                    continue
                try:
                    request_hold(sock, q)
                except OSError as exc:
                    self.log('HOLD FAILED: '+str(exc))
            self.log('STOP: '+reason+'; position holds requested, no disable commands. '
                     'Motors remain enabled; holding is not confirmed.')
            self.owned = False
        if self.guard is not None:
            try:
                self.guard_pipe.send(None)
            except (BrokenPipeError, OSError):
                pass
            self.guard_pipe.close()
            self.guard.join(timeout=1)
            self.guard = self.guard_pipe = None
        if publish:
            self.publish_feedback(np.zeros(6), False)

    def tick(self, now):
        if not hasattr(self, 'following') and self.mode not in ('WAIT_ALIGN', 'ALIGNING', 'TELEOP'):
            if self.mode in ('STARTUP','PREPARE','ENABLING'):
                if not 0 < now-self.tick_time <= .1:
                    raise RuntimeError('Startup loop missed 100 ms deadline')
                self.tick_time = now
                return self.startup(now)
            return super().tick(now)
        from piper_x_native_teleop import NativeTeleop
        from piper_x_haptic_core import joint_frames
        if not hasattr(self, 'following'):
            # Release only the operator gripper motor; arm joints stay supported.
            # SDK 0x159 byte 6 = 0 disables, byte 7 = 0 preserves calibration.
            send(self.sockets[0], 0x159, bytes(8))
            self.following = NativeTeleop(self.sockets, self.cfg)
            self.following.started = now
            self.following.last_tick = now-.004
            self.following.states[1] = self.states[1]
        dt = now-self.tick_time
        self.tick_time = now
        if not 0 < dt <= .1:
            raise RuntimeError('Torque loop missed 100 ms deadline')
        self.heartbeat()
        m, s = self.states
        required = [*range(0x2a5,0x2a8), *range(0x251,0x257), *range(0x261,0x267),0x2a1]
        if not m.fresh(required, now, self.cfg['state_timeout']) or not m.normal() or not m.enabled():
            raise RuntimeError('Master torque-mode telemetry stale/faulted: '+m.diagnostic(now))
        if m.frames[0x2a1][1][0] != 1 or m.frames[0x2a1][1][2] != 6:
            raise RuntimeError('Master MIT mode lost')
        if now-m.last_external_command < .1:
            raise RuntimeError('Competing CAN command source on master')
        self.qm = m.q()
        vm, _ = m.motor()
        f = self.following
        # MIT master supplies measured positions, not native leader 0x155 frames.
        # Keep the same native follower alignment/forwarding engine.
        stamp = min(m.frames[i][0] for i in range(0x2a5,0x2a8))
        # Match alignment's command limits for small accepted encoder zero
        # offsets. Keep raw measured qm for gravity and torque computation.
        f.target = np.clip(self.qm, f.limits[:,0], f.limits[:,1])
        f.target_payloads = dict(joint_frames(f.target))
        for identifier, payload in f.target_payloads.items():
            f.states[0].frames[identifier] = (stamp,payload)
        fresh = 0 <= now-self.last_next < self.cfg['next_timeout']
        contact = self.contact and 0 <= now-self.last_contact < self.cfg['next_timeout']
        torque = self.feedback.update(self.next_tau, dt, fresh and contact,
                                      f.phase == 'TELEOP', contact, np.ones(6))
        if 'reflection_motion_speed' in self.cfg:
            if not hasattr(self,'reflection_motion_gate'):
                from piper_x_haptic_core import ReflectionMotionGate
                self.reflection_motion_gate = ReflectionMotionGate(self.cfg['reflection_motion_speed'])
            # Only a fresh, explicit no-contact reading rearms latched joints.
            gate_contact = not (fresh and not self.contact and
                               0 <= now-self.last_contact < self.cfg['next_timeout'])
            torque = self.reflection_motion_gate.update(torque,vm,gate_contact)
            self.feedback.value[:] = torque
        self.compliant(0, self.qm, vm, torque)
        # Retain the startup position hold until alignment takes ownership.
        if f.phase in ('WAIT_ALIGN','ENABLING'):
            if not s.fresh(required,now,self.cfg['state_timeout']) or not s.normal():
                raise RuntimeError('Slave startup telemetry stale/faulted')
            position(self.sockets[1],self.command)
        f.tick(now)
        self.mode = f.phase
        if f.command is not None:
            self.command = f.command
        self.qs = s.q()
        self.update_gripper_feedback(now)
        if self.mode != 'TELEOP':
            self.prev_sample = None
            return
        if m.fresh([0x2a8],now,.5):
            # Follow observed opening only, never a retained command that could
            # keep the follower closed while the operator opens the gripper.
            opening = m.frames[0x2a8][1][:4]
            effort = int(self.cfg.get('gripper_follow_effort',200))
            send(self.sockets[1],0x159,opening+effort.to_bytes(2,'big')+bytes([1,0]))
        period = 1/self.cfg['publish_hz']
        if now-self.last_pub+1e-6 >= period:
            self.last_pub += max(1,math.floor((now-self.last_pub+1e-6)/period))*period
            self.publish_feedback(torque,fresh and contact)
            sample = min(s.frames[i][0] for i in range(0x2a5,0x2a8))
            if self.prev_sample is not None:
                old_time, old_q = self.prev_sample
                elapsed = sample-old_time
                if 0 < elapsed <= .04:
                    stamp_msg = self.node.get_clock().now().to_msg()
                    for pub, values in zip(self.inputs,(self.qs,(self.qs-old_q)/elapsed,
                                                       self.command,s.motor()[1])):
                        msg=self.JointState();msg.header.stamp=stamp_msg
                        msg.position=[float(x) for x in values];pub.publish(msg)
            self.prev_sample=(sample,self.qs.copy())
        if now-self.last_log > 2:
            self.last_log=now
            self.log(self.feedback_diagnostic(now,torque))
