#!/usr/bin/env python3
"""NEXT-triggered native role reversal; physical can0 is always observed."""
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

from canframe_swap import LOCK, role_indices, swap_roles
from piper_can_record_bridge import Capture
from piper_x_haptics import bus, drain_feedback, position
from piper_x_native_teleop import NativeTeleop
from piper_x_timing import Periodic, LoopRate

ROOT = Path(__file__).resolve().parents[1]


def hold_watchdog(pipe, channels):
    """On parent loss, request the last supplied hold pose; never disable motors."""
    sockets = []
    snapshot = None
    try:
        sockets = [bus(channel) for channel in channels]
        while pipe.poll(.5):
            try:
                message = pipe.recv()
            except EOFError:
                break
            if message is None:
                return
            if message == "clear":
                snapshot = None
            if isinstance(message, tuple):
                snapshot = message
        if snapshot is not None:
            index, q = snapshot
            position(sockets[index], q)
            print('WATCHDOG: requested last slave hold target; motors left enabled.', flush=True)
    finally:
        for sock in sockets:
            sock.close()
        pipe.close()


class ContactGate:
    """Hysteresis and continuous dwell; only fresh predictions can change mode."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.last = None
        self.since = None

    def update(self, torque, now, mode):
        values = np.asarray(torque, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise RuntimeError('NEXT returned an invalid six-joint torque estimate')
        if self.last is not None and now - self.last > self.cfg['next_timeout']:
            self.since = None
        self.last = now
        magnitude = float(np.abs(values).sum())
        trigger = (magnitude >= self.cfg['contact_on'] if mode == 'teleop'
                   else magnitude <= self.cfg['contact_off'])
        if not trigger:
            self.since = None
            return None
        if self.since is None:
            self.since = now
        dwell = self.cfg['contact_on_seconds' if mode == 'teleop' else 'contact_off_seconds']
        if now - self.since >= dwell:
            return 'feedback' if mode == 'teleop' else 'teleop'
        return None


def validate(cfg):
    required = ('teleop_hz', 'feedback_hz', 'contact_on', 'contact_off',
                'contact_on_seconds', 'contact_off_seconds', 'next_timeout',
                'warmup_timeout', 'role_timeout')
    for key in required:
        value = cfg[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (value == 0 and key != 'contact_off_seconds'):
            raise ValueError(f'{key} must be positive (contact_off_seconds may be zero)')
    if not math.isfinite(cfg.get('max_feedback_seconds', .5)) or cfg.get('max_feedback_seconds', .5) <= 0:
        raise ValueError('max_feedback_seconds must be finite and positive')
    if cfg['contact_off'] >= cfg['contact_on']:
        raise ValueError('contact_off must be below contact_on')


class RoleSwap:
    def __init__(self, sockets, native_cfg, cfg, node, joint_state):
        from rclpy.qos import qos_profile_sensor_data
        self.sockets, self.native_cfg, self.cfg = sockets, native_cfg, cfg
        self.node, self.JointState = node, joint_state
        self.gate = ContactGate(cfg)
        root = native_cfg['robot_topic_root'].rstrip('/')
        self.pubs = [node.create_publisher(joint_state, root+'/'+name, qos_profile_sensor_data)
                     for name in ('joint_pos_obs', 'joint_vel_obs', 'joint_pos_cmd', 'joint_effort_obs')]
        self.controller = None
        self.mode = 'teleop'
        self.pending = None
        self.transition('teleop')

    def transition(self, mode):
        # Invoked synchronously by the sole writer, between control ticks.
        self.pending = None
        live_handover = self.controller is not None
        if live_handover:
            self.hold_once()
        self.mode = mode
        master, slave = role_indices(mode)
        self.controller = NativeTeleop([self.sockets[master], self.sockets[slave]], self.native_cfg)
        self.controller.owned = True
        if live_handover:
            self.controller.skip_alignment = True
            self.controller.recover_handover = mode == 'teleop'
            self.controller.phase = 'WAIT_ROLE'
        swap_roles(self.sockets, mode, disable=False)
        self.started = time.monotonic()
        self.last_estimate_stamp = -math.inf
        self.epoch = self.node.get_clock().now().nanoseconds / 1e9
        self.controller.started = self.controller.last_tick = self.started
        self.capture = Capture()  # Discard packets and commands from the previous roles.
        self.previous = None
        self.last_publish = self.started
        self.published_samples = 0
        self.hold_target = None
        self.gate.reset()
        self.feedback_armed = not live_handover
        self.next_warmup_started = None
        print(f'{mode.upper()}: requesting can{1-master} master -> can{1-slave} slave; '
              'waiting for fresh native targets and telemetry.', flush=True)

    def snapshot(self):
        """Prefer fresh measured position; fall back to the last sent target."""
        c = self.controller
        _, slave = role_indices(self.mode)
        if c.states[1].fresh(range(0x2a5,0x2a8), time.monotonic(), .5):
            q = c.states[1].q()
        else:
            q = c.command
        return None if q is None else (slave, list(q))

    def hold_once(self):
        snapshot = self.snapshot()
        if snapshot is not None:
            index, q = snapshot
            position(self.sockets[index], q)
        return snapshot

    def hold_tick(self):
        if self.hold_target is not None:
            index, q = self.hold_target
            position(self.sockets[index], q)

    def record_sent_commands(self):
        # SocketCAN does not echo a socket's own TX by default. These bytes are
        # committed only after send() succeeded, not inferred from measured pose.
        if self.mode == 'teleop':
            for identifier, (stamp, payload) in self.controller.last_sent_payloads.items():
                self.capture.receive(identifier, payload, stamp)

    def receive(self, physical, frame, arrival):
        if arrival < self.started:
            return
        master, _ = role_indices(self.mode)
        self.controller.receive(0 if physical == master else 1, frame, arrival)
        if physical == 1:  # Physical can0, regardless of role.
            import struct
            identifier, size, data = struct.unpack('=IB3x8s', frame)
            self.capture.receive(identifier, data[:size], arrival)

    def estimate(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if stamp <= max(self.epoch, self.last_estimate_stamp) or not 0 <= age < self.cfg['next_timeout']:
            return
        if self.controller.phase != 'TELEOP' and self.mode != 'feedback':
            return
        if self.gate.last is None:
            print(f'NEXT ONLINE: can0 predictions received after {self.published_samples} input samples.', flush=True)
        self.last_estimate_stamp = stamp
        proposed = self.gate.update(msg.position, now, self.mode)
        if self.mode == 'teleop' and not self.feedback_armed:
            # Require release before another pulse, including after timeout return.
            if float(np.abs(msg.position).sum()) <= self.cfg['contact_off']:
                self.feedback_armed = True
            self.pending = None
        else:
            self.pending = proposed

    def publish(self, now):
        # Pause during role changes/alignment, forcing NEXT's input-gap history reset.
        if (self.controller.phase != 'TELEOP' and self.mode != 'feedback') or now + 1e-6 < self.last_publish + .02:
            return
        slots = max(1, math.floor((now-self.last_publish+1e-6)/.02))
        self.last_publish += slots*.02
        if self.capture.missing(now):
            self.previous = None
            return
        pos, cmd, torque = self.capture.values()
        sample_time = min(self.capture.frames[i][0] for i in range(0x2a5, 0x2a8))
        previous = self.previous
        self.previous = (sample_time, pos)
        if previous is None or not 0 < sample_time - previous[0] <= .04:
            return
        vel = [(q-old)/(sample_time-previous[0]) for q, old in zip(pos, previous[1])]
        stamp = self.node.get_clock().now().to_msg()
        for pub, values in zip(self.pubs, (pos, vel, cmd, torque)):
            msg = self.JointState()
            msg.header.stamp = stamp
            msg.position = list(values)
            pub.publish(msg)
        self.published_samples += 1

    def tick(self, now):
        c = self.controller
        # Release is handled before any further reverse command or enable step.
        if self.mode == 'feedback' and (
            self.pending == 'teleop' or now-self.started >= self.cfg.get('max_feedback_seconds', .5)
        ):
            print('RETURN TELEOP: contact released or feedback pulse deadline reached.', flush=True)
            self.transition('teleop')
            return True
        if c.target is None and now - self.started > self.cfg['role_timeout']:
            raise RuntimeError('New master produced no fresh native targets; firmware may require restart')
        try:
            c.tick(now)
        except RuntimeError as exc:
            if self.mode == 'feedback':
                print(f'RETURN TELEOP: reverse following unavailable: {exc}', flush=True)
                self.transition('teleop')
                return True
            master, slave = role_indices(self.mode)
            raise RuntimeError(f'{self.mode}: can{1-master} master, can{1-slave} slave: {exc}') from exc
        if c.phase == 'TELEOP':
            # Native controller already checks status, enable, pose and tracking.
            if c.states[1].frames[0x2a1][1][0] != 1:
                raise RuntimeError('New slave did not confirm CAN control mode; firmware may require restart')
        self.record_sent_commands()
        self.publish(now)
        if c.phase != 'TELEOP':
            self.next_warmup_started = None
            return False
        if self.next_warmup_started is None:
            self.next_warmup_started = now
        if self.gate.last is None:
            if now - self.next_warmup_started > self.cfg['warmup_timeout']:
                raise RuntimeError(f'NEXT warmup timed out: published={self.published_samples}; '
                                   f'missing CAN inputs={[hex(i) for i in self.capture.missing(now)]}')
        elif now - self.gate.last > self.cfg['next_timeout']:
            raise RuntimeError('NEXT estimates stale; stopping both arms')
        if self.pending is not None:
            self.transition(self.pending)
            return True
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/piper_x_role_swap.yaml')
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    validate(cfg)
    native_cfg = yaml.safe_load((ROOT/'config/piper_x_haptics.yaml').read_text())
    if (native_cfg['master_can'], native_cfg['slave_can']) != ('can1', 'can0'):
        raise ValueError('Role switching requires physical can1 operator and can0 environment arms')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('piper_x_role_swap')
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    sockets = []
    controller = pipe = guard = None
    try:
        with open(LOCK, 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                for channel in ('can1', 'can0'):
                    sockets.append(bus(channel))
                ctx = mp.get_context('spawn')
                receiver, pipe = ctx.Pipe(duplex=False)
                guard = ctx.Process(target=hold_watchdog, args=(receiver, ['can1', 'can0']))
                guard.start()
                receiver.close()
                pipe.send(True)
                controller = RoleSwap(sockets, native_cfg, cfg, node, JointState)
                node.create_subscription(JointState, native_cfg['next_topic_root']+'/external_joint_torque',
                                         controller.estimate, qos_profile_sensor_data)
                hz = cfg['teleop_hz']
                schedule = Periodic(hz, time.monotonic())
                rate = LoopRate(time.monotonic())
                holding = False
                while rclpy.ok():
                    ready = select.select(sockets, [], [], max(0, schedule.deadline-time.monotonic()))[0]
                    drain_feedback(sockets, ready, controller.receive)
                    rclpy.spin_once(node, timeout_sec=0)
                    now = time.monotonic()
                    if now >= schedule.deadline:
                        if not guard.is_alive():
                            raise RuntimeError('Independent watchdog exited')
                        if not holding and controller.pending is not None:
                            pipe.send("clear")
                        snapshot = controller.hold_target if holding else controller.snapshot()
                        pipe.send(snapshot if snapshot is not None and controller.pending is None else "clear")
                        rate.sample(now, schedule.deadline)
                        changed = False
                        if holding:
                            try:
                                controller.hold_tick()
                            except OSError as exc:
                                print(f'HOLD TX failed: {exc}; hold cannot be confirmed.', flush=True)
                                raise
                        else:
                            try:
                                changed = controller.tick(now)
                            except (RuntimeError, OSError, ValueError) as exc:
                                controller.hold_target = controller.hold_once()
                                holding = True
                                print(f'HOLD: {exc}. Following stopped; motors left enabled. Ctrl-C exits.', flush=True)
                                hz = 50
                                schedule = Periodic(hz, time.monotonic())
                        if changed:
                            hz = cfg[controller.mode+'_hz']
                            schedule = Periodic(hz, time.monotonic())
                            rate = LoopRate(time.monotonic())
                        completed = time.monotonic()
                        rate.skipped += schedule.advance(completed)
                        report = rate.report(completed, hz)
                        if report:
                            print(report, flush=True)
            finally:
                # Also handles partial construction/role write failures. Keep lock until stopped.
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                try:
                    if controller is not None:
                        if controller.hold_target is not None:
                            try:
                                controller.hold_tick()
                            except OSError as exc:
                                print(f'HOLD TX failed: {exc}; hold cannot be confirmed.', flush=True)
                                raise
                        else:
                            controller.hold_once()
                        print('STOP: slave hold requested; motors left enabled. Hold is not confirmed.', flush=True)
                    if pipe is not None:
                        pipe.send(None)
                finally:
                    for sock in sockets:
                        sock.close()
                    if pipe is not None:
                        pipe.close()
                if guard is not None:
                    guard.join(timeout=1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
