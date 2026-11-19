#!/usr/bin/env python3
"""Dual Piper X S-V1.8-9 teleop. Passive by default; --control runs automatic alignment and feedback."""
import argparse
import errno
import fcntl
import math
import multiprocessing as mp
from pathlib import Path
import select
import signal
import socket
import struct
import time

import numpy as np
import yaml

from piper_x_haptic_core import (Feedback, Gravity, TorqueResponseGuard, PIPER_X_TORQUE_C,
                                 joint_frames, limited_target, mit_frame, vector)
from piper_can_record_bridge import TORQUE_K, TORQUE_B
from piper_x_timing import Periodic, LoopRate

ROOT = Path(__file__).resolve().parents[1]
# Linux SO_TIMESTAMPNS, using the native timespec layout on this ThinkPad.
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)


def check_setup(cfg):
    """Validate local configuration/artifacts without ROS or opening CAN sockets."""
    if cfg.get('gripper_feedback', {}).get('enabled'):
        from piper_x_gripper_feedback import validate
        validate(cfg['gripper_feedback'])
    if 'reflection_motion_speed' in cfg:
        from piper_x_haptic_core import ReflectionMotionGate
        ReflectionMotionGate(cfg['reflection_motion_speed'])
    if cfg['master_can'] == cfg['slave_can']:
        raise ValueError('Master and slave must use separate CAN interfaces')
    if cfg['firmware'] != 'S-V1.8-9':
        raise ValueError('MIT encoder is pinned to S-V1.8-9 firmware')
    for key in ('master_damping', 'master_total_torque_limit'):
        values = vector(cfg[key])
        if np.any(values < 0) or (key == 'master_total_torque_limit' and np.any(values == 0)):
            raise ValueError(key+' must be nonnegative (torque limits strictly positive)')
    if np.any(vector(cfg['master_damping']) > 5):
        raise ValueError('MIT damping exceeds the encoder range')
    if np.any(vector(cfg['master_total_torque_limit']) >= 16):
        raise ValueError('Torque limits must be below the MIT encoder limit of 16 Nm')
    for key in ('control_hz', 'publish_hz', 'state_timeout', 'next_timeout',
                'alignment_tolerance', 'alignment_complete_tolerance', 'alignment_velocity',
                'alignment_settle_seconds', 'max_velocity', 'max_tracking_error', 'master_max_velocity'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(key+' must be finite and positive')
    if not cfg['alignment_complete_tolerance'] < cfg['alignment_tolerance'] <= math.radians(10):
        raise ValueError('Alignment completion must be tighter than the entry threshold (at most 10 degrees)')
    if cfg['alignment_velocity'] > cfg['max_velocity']:
        raise ValueError('Alignment speed cannot exceed normal teleop speed')
    for key in ('deadband', 'limits'):
        if np.any(vector(cfg['feedback'][key]) < 0):
            raise ValueError('Feedback '+key+' must be nonnegative')
    vector(cfg['feedback']['gains'])
    slew = cfg['feedback']['slew_nm_per_second']
    if not math.isfinite(slew) or slew <= 0:
        raise ValueError('Feedback slew must be finite and positive')
    Gravity(cfg['gravity_urdf'], cfg['gripper_xacro'])
    inference = yaml.safe_load((ROOT/'src/factr2_next/factr2_next/config/inference.yaml').read_text())
    for key in ('robot_topic_root', 'next_topic_root'):
        if cfg[key] != inference[key]:
            raise ValueError(key+' does not match NEXT inference configuration')
    if cfg['publish_hz'] != 50:
        raise ValueError('This trained model requires 50 Hz inputs')
    checkpoint = Path(inference['checkpoint_dir'])
    for name in ('model.pt', 'normalization.npz', 'config.yaml'):
        if not (checkpoint/name).is_file():
            raise ValueError('Missing checkpoint artifact: '+str(checkpoint/name))
    print(f'Setup OK: {cfg["master_can"]} master, {cfg["slave_can"]} slave; '
          f'NEXT checkpoint {checkpoint}. No hardware accessed.', flush=True)


def bus(channel):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    s.bind((channel,))
    s.setblocking(False)
    return s


def receive_stamped(s):
    """Use kernel arrival time so queued CAN packets cannot look freshly received."""
    frame, ancillary, flags, _ = s.recvmsg(16, socket.CMSG_SPACE(struct.calcsize('@ll')),
                                         socket.MSG_DONTWAIT)
    if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
        raise RuntimeError('Truncated CAN frame/timestamp')
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
            seconds, nanos = struct.unpack('@ll', data[:struct.calcsize('@ll')])
            age = time.time() - (seconds+nanos/1e9)
            if age < -.01:
                raise RuntimeError('CAN timestamp clock changed')
            return frame, time.monotonic()-max(0, age)
    raise RuntimeError('CAN kernel timestamp missing')


def send(s, identifier, data, timeout=.002):
    frame = struct.pack('=IB3x8s', identifier, len(data), bytes(data).ljust(8, b'\0'))
    deadline = time.monotonic()+timeout
    while True:
        try:
            s.send(frame)
            return
        except OSError as exc:
            if exc.errno not in (errno.ENOBUFS, errno.EAGAIN, errno.EWOULDBLOCK):
                raise
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise OSError(exc.errno, f'CAN TX queue stalled sending {identifier:#x}; '
                              'check queue length, bus errors and other controllers') from exc
            # Polling writability alone can busy-loop on SocketCAN ENOBUFS.
            time.sleep(min(.0005, remaining))


def position(s, q):
    for i, d in joint_frames(q):
        send(s, i, d)


def disable_joints(s):
    # Individual indices: no ambiguous "all joints" firmware alias, no gripper.
    errors = []
    for joint in range(1, 7):
        try:
            send(s, 0x471, [joint, 1, 0, 0, 0, 0, 0, 0], timeout=.02)
        except OSError as exc:
            errors.append(exc)
        time.sleep(.0005)
    if errors:
        raise errors[0]


def watchdog(pipe, channels):
    """Independent best-effort motor disable on heartbeat loss/parent exit."""
    sockets = [bus(c) for c in channels]
    deadline = None
    active = False
    try:
        while True:
            if pipe.poll(0.025):
                try:
                    message = pipe.recv()
                except EOFError:
                    break
                if message is None:
                    active = False
                    break
                active = True
                deadline = time.monotonic() + 0.2
            if deadline is not None and time.monotonic() > deadline:
                break
    finally:
        if active:
            print('WATCHDOG: heartbeat lost; disabling six arm joints on both arms.', flush=True)
            for _ in range(3):
                for s in sockets:
                    try:
                        disable_joints(s)
                    except OSError as exc:
                        print('WATCHDOG disable failed:', exc, flush=True)
                time.sleep(0.02)
        for s in sockets:
            s.close()
        pipe.close()


def drain_feedback(sockets, ready, consume, *, clock=time.monotonic, receiver=receive_stamped):
    """Fair RX service independent of expired command deadlines; retain arrival times."""
    active = [i for i, sock in enumerate(sockets) if sock in ready]
    end = clock()+.002
    for _ in range(128):
        for i in active[:]:
            try:
                frame, arrival = receiver(sockets[i])
            except BlockingIOError:
                active.remove(i)
                continue
            consume(i, frame, arrival)
        if not active or clock() >= end:
            break


class State:
    def __init__(self):
        self.frames = {}
        self.firmware = bytearray()
        self.last_external_command = -math.inf

    def receive(self, frame, now):
        i, n, d = struct.unpack('=IB3x8s', frame)
        if i & 0xE0000000:
            return
        d = d[:n]
        if i == 0x4af:
            self.firmware.extend(d)
        if n == 8:
            self.frames[i] = (now, d)
        if i in [0x150, 0x151, 0x470, 0x471] or 0x155 <= i <= 0x15f:
            self.last_external_command = now

    def fresh(self, ids, now, age):
        return all(i in self.frames and 0 <= now-self.frames[i][0] < age for i in ids)

    def q(self, native=False):
        base = 0x155 if native else 0x2a5
        return np.array([v*math.pi/180000 for i in range(3)
                         for v in struct.unpack('>ii', self.frames[base+i][1])])

    def motor(self):
        vals = [struct.unpack('>hhi', self.frames[i][1]) for i in range(0x251, 0x257)]
        velocity = np.array([v[0]/1000 for v in vals])
        torque = np.array([v[1]/1000 for v in vals])*np.array(TORQUE_K)*np.array(TORQUE_B)
        return velocity, torque

    def normal(self):
        d = self.frames[0x2a1][1]
        return d[1] == 0 and d[6:8] == b'\0\0'

    def enabled(self):
        # Enable bit alone must be set: reject undervoltage, collision, stall,
        # overcurrent, driver faults, and temperature faults as well.
        return all(self.frames[i][1][5] == 0x40 for i in range(0x261, 0x267))

    def diagnostic(self, now, required=None, max_age=.1):
        if required is None:
            required = list(range(0x2a5, 0x2a8))+list(range(0x251, 0x257))+[0x2a1]
        stale = [hex(i) for i in required if not self.fresh([i], now, max_age)]
        ages = ','.join(f'{i:#x}:' + (
            f'{(now-self.frames[i][0])*1000:.1f}' if i in self.frames else 'missing')
            for i in required)
        status = self.frames.get(0x2a1)
        codes = [f'{self.frames[i][1][5]:02x}' if i in self.frames else 'missing'
                 for i in range(0x261, 0x267)]
        return ('stale='+str(stale)+f' limit_ms={max_age*1000:g} ages_ms=[{ages}] status='+(status[1].hex() if status else 'missing')+
                ' motor-status='+','.join(codes))


class Controller:
    def __init__(self, cfg, control):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float32
        self.rclpy, self.JointState, self.Float32 = rclpy, JointState, Float32
        self.cfg, self.control = cfg, control
        if cfg['master_can'] == cfg['slave_can']:
            raise ValueError('Master and slave must use separate CAN interfaces')
        self.gravity = Gravity(cfg['gravity_urdf'], cfg['gripper_xacro'])
        self.feedback = Feedback(cfg['feedback'])
        self.torque_response = [TorqueResponseGuard(), TorqueResponseGuard()]
        self.last_enable_retry = -math.inf
        self.last_disable_retry = -math.inf
        self.sockets = [bus(cfg['master_can']), bus(cfg['slave_can'])]
        self.states = [State(), State()]
        self.guard = self.guard_pipe = None
        self.mode = 'STARTUP' if control else 'MONITOR'
        self.owned = False
        self.aligned_since = None
        self.mask = np.zeros(6)
        self.qm = self.qs = self.command = None
        self.last_next = self.last_contact = -math.inf
        self.next_tau = np.zeros(6)
        self.contact = False
        self.prev_sample = None
        self.tick_time = time.monotonic()
        self.last_pub = self.last_log = self.tick_time
        self.transition = self.tick_time
        self.last_mode_refresh = self.tick_time
        # Own SIGINT/SIGTERM in main: disable arm joints before ROS is shut down.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('piper_x_haptic_teleop')
        root = cfg['robot_topic_root']
        names = ['joint_pos_obs', 'joint_vel_obs', 'joint_pos_cmd', 'joint_effort_obs']
        self.inputs = [self.node.create_publisher(JointState, root+'/'+n, qos_profile_sensor_data) for n in names]
        fb = cfg['feedback_topic_root']
        self.fb_pub = self.node.create_publisher(JointState, fb+'/torque', 10)
        self.fb_gate = self.node.create_publisher(Float32, fb+'/gate', 10)
        self.subs = [self.node.create_subscription(JointState, cfg['next_topic_root']+'/external_joint_torque', self.next_callback, qos_profile_sensor_data),
                     self.node.create_subscription(Bool, cfg['next_topic_root']+'/contact_state', self.contact_callback, 10)]
        # Firmware request only; unlike pyAgxArm's one-byte request, these arms need DLC=8.
        for s in self.sockets:
            send(s, 0x4af, [1, 0, 0, 0, 0, 0, 0, 0])

    def reset_loop_clock(self, now):
        # ROS/DDS initialization is not an active motor-control cycle.
        self.tick_time = self.transition = self.last_pub = self.last_log = now
        self.last_mode_refresh = now

    def next_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        age = self.node.get_clock().now().nanoseconds/1e9 - stamp
        try:
            value = vector(msg.position)
            if not 0 <= age < self.cfg['next_timeout']:
                raise ValueError('stale timestamp')
            self.next_tau = value
            self.last_next = time.monotonic()
        except ValueError:
            self.last_next = -math.inf

    def contact_callback(self, msg):
        self.contact = msg.data
        self.last_contact = time.monotonic()

    def log(self, text):
        print(text, flush=True)

    def heartbeat(self):
        if self.guard is not None:
            if not self.guard.is_alive():
                raise RuntimeError('Watchdog exited; control is latched off')
            self.guard_pipe.send(True)

    def start_guard(self):
        ctx = mp.get_context('spawn')
        rx, tx = ctx.Pipe(duplex=False)
        self.guard_pipe = tx
        self.guard = ctx.Process(target=watchdog, args=(rx, [self.cfg['master_can'], self.cfg['slave_can']]))
        self.guard.start()
        rx.close()
        self.heartbeat()

    def compliant(self, index, q, velocity, feedback=None):
        damping = vector(self.cfg['master_damping'])
        gravity = self.gravity.evaluate(q)[0]
        total = gravity
        if feedback is not None:
            total = total+feedback
        expected = total-damping*velocity
        limits = vector(self.cfg['master_total_torque_limit'])
        if np.any(np.maximum(abs(total), abs(expected)) > limits):
            raise RuntimeError(('Master' if index == 0 else 'Slave')+' gravity/feedback torque exceeds limit')
        if self.mode not in ('PREPARE', 'ENABLING'):
            _, measured = self.states[index].motor()
            self.torque_response[index].check(self.tick_time, expected, measured, velocity)
        # Continuous teleop uses explicit damping torque instead of firmware kd:
        # the observed velocity-loop resistance did not match modeled -D*v.
        software_damping = getattr(self, 'software_damping', False)
        command = expected if software_damping else total
        packets = [mit_frame(q[i], 0. if software_damping else damping[i],
                             command[i]/PIPER_X_TORQUE_C[i]) for i in range(6)]
        for i, data in enumerate(packets):
            send(self.sockets[index], 0x15a+i, data)
        if index == 0:
            self.master_gravity = gravity.copy()
            self.master_expected_torque = expected.copy()

    def refresh_modes(self, now, force=False):
        # Match the SDK's default auto-set-mode path, throttled to 10 Hz.
        # In particular, reassert MIT after enabling rather than relying on a
        # mode selection made while the motor drivers were disabled.
        if not force and now-getattr(self, 'last_mode_refresh', now) < .1:
            return
        slave_mode = 1 if self.mode in ('ALIGNING', 'TELEOP') else 6
        for sock, mode in zip(self.sockets, (6, slave_mode)):
            send(sock, 0x151, [1, mode, 10, 0xad if mode == 6 else 0, 0, 1, 0, 0])
        self.last_mode_refresh = now

    def master_diagnostic(self):
        m = self.states[0]
        velocity, measured = m.motor()
        return ('MASTER: mode='+str(m.frames[0x2a1][1][2])+', enabled='+str(m.enabled())+
                ', kp=0, kd='+str(self.cfg['master_damping'])+
                '; gravity Nm='+str(np.round(self.master_gravity, 3))+
                '; commanded incl damping Nm='+str(np.round(self.master_expected_torque, 3))+
                '; measured Nm='+str(np.round(measured, 3))+
                '; speed rad/s='+str(np.round(velocity, 3)))

    def switch_slave_compliant(self, now):
        self.compliant(1, self.qs, self.states[1].motor()[0])
        send(self.sockets[1], 0x151, [1, 6, 10, 0xad, 0, 1, 0, 0])
        self.mode = 'WAIT_ALIGN'
        self.transition = now
        self.aligned_since = None
        self.command = self.qs.copy()
        self.mask[:] = 0
        self.feedback.value[:] = 0
        self.prev_sample = None
        self.log('WAIT_ALIGN: both arms compliant; bring every joint within 10 degrees.')

    def stop(self, reason, publish=True):
        # Faults latch off; restarting the command is required to enable again.
        self.mode = 'STOPPED' if self.control else 'MONITOR'
        self.mask[:] = 0
        self.prev_sample = None
        self.feedback.value[:] = 0
        if self.owned:
            released = True
            for _ in range(3):
                for sock in self.sockets:
                    try:
                        disable_joints(sock)
                    except OSError as exc:
                        released = False
                        self.log('DISABLE FAILED: '+str(exc))
                time.sleep(.01)
            self.log('STOP: '+reason+'; disable commands '+('sent' if released else 'partly failed')+
                     ' for both arms; support them. No position hold. Release is not confirmed.')
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

    def publish_feedback(self, torque, gate):
        if not self.rclpy.ok():
            return
        msg = self.JointState();msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.position = [float(x) for x in torque]
        self.fb_pub.publish(msg);self.fb_gate.publish(self.Float32(data=float(gate)))

    def tick(self, now):
        dt = now-self.tick_time
        self.tick_time = now
        m, s = self.states
        if self.mode in ('MONITOR', 'STOPPED'):
            if now-self.last_log > 2:
                self.last_log = now
                self.log(self.mode+': master '+m.diagnostic(now)+'; slave '+s.diagnostic(now))
            return
        if dt > .1:
            raise RuntimeError('Control loop missed 100 ms deadline')
        if self.mode == 'STARTUP':
            if now-self.transition > 5:
                raise RuntimeError('Firmware verification timed out; no motors enabled')
            if not all(self.cfg['firmware'].encode() in st.firmware for st in self.states):
                return
            if now-s.last_external_command < .5:
                raise RuntimeError('Another controller is commanding the slave; stop it first')
            self.start_guard()
            self.owned = True
            # Release retained hold first, then request normal CAN feedback.
            # No pose is needed to disable, so stationary native targets cannot block startup.
            for sock in self.sockets:
                disable_joints(sock)
                send(sock, 0x470, [0xfc, 0, 0, 0, 0, 0, 0, 0])
                send(sock, 0x151, [1, 6, 10, 0xad, 0, 1, 0, 0])
            self.mode = 'PREPARE'
            self.transition = now
            self.log('PREPARE: released old hold; waiting for measured joint feedback. Support both arms.')
            return
        self.heartbeat()
        required = list(range(0x2a5, 0x2a8))+list(range(0x251, 0x257))+[0x2a1]
        good = all(st.fresh(required, now, self.cfg['state_timeout']) and st.normal()
                   and st.fresh(range(0x261, 0x267), now, .2) for st in self.states)
        if not good:
            if self.mode in ('PREPARE', 'ENABLING') and now-self.transition < 2:
                return
            raise RuntimeError('Arm feedback stale/faulted; master: '+m.diagnostic(now)+'; slave: '+s.diagnostic(now))
        self.qm, self.qs = m.q(), s.q()
        vm, _ = m.motor()
        vs, _ = s.motor()
        poses_to_check = (self.qm, self.qs) if getattr(self, 'check_master_pose_limits', True) else (self.qs,)
        for q in poses_to_check:
            if np.any(q < self.gravity.limits[:, 0]-.05) or np.any(q > self.gravity.limits[:, 1]+.05):
                raise RuntimeError('Arm joint limit reached')
        for name, velocity in (('slave', vs),):
            if np.max(abs(velocity)) > self.cfg['master_max_velocity']:
                joint = int(np.argmax(abs(velocity)))
                raise RuntimeError(f'{name} joint {joint+1} speed {velocity[joint]:.3f} rad/s '
                                   f'exceeded {self.cfg["master_max_velocity"]:.3f} rad/s in {self.mode}')
        if now-self.transition > .3:
            if now-s.last_external_command < .1 or now-m.last_external_command < .1:
                raise RuntimeError('Competing CAN command source detected')
        self.refresh_modes(now)
        if self.mode == 'PREPARE':
            # Require feedback newer than mode change and confirmed disabled before enabling.
            fresh_transition = all(st.frames[i][0] > self.transition for st in self.states
                                   for i in required+list(range(0x261, 0x267)))
            disabled = all(st.frames[i][1][5] == 0 for st in self.states for i in range(0x261, 0x267))
            mit_mode = all(st.frames[0x2a1][1][0] == 1 and st.frames[0x2a1][1][2] == 6
                           for st in self.states)
            if not (fresh_transition and disabled and mit_mode):
                if now-self.transition > 2:
                    raise RuntimeError('Could not confirm released joints and MIT mode; master: '+m.diagnostic(now)+'; slave: '+s.diagnostic(now))
                if now-self.last_disable_retry >= .1:
                    self.last_disable_retry = now
                    for sock, st in zip(self.sockets, self.states):
                        for joint in range(1, 7):
                            if st.frames[0x260+joint][1][5] & 0x40:
                                send(sock, 0x471, [joint, 1, 0, 0, 0, 0, 0, 0])
                                time.sleep(.0005)
                return
            self.compliant(0, self.qm, vm)
            self.compliant(1, self.qs, vs)
            for sock in self.sockets:
                for joint in range(1, 7):
                    send(sock, 0x471, [joint, 2, 0, 0, 0, 0, 0, 0])
                    time.sleep(.0005)
            self.mode = 'ENABLING'
            self.transition = now
            self.refresh_modes(now, force=True)
            self.log('ENABLING: gravity compensation, zero position stiffness on both arms.')
            return
        if self.mode == 'ENABLING':
            self.compliant(0, self.qm, vm)
            self.compliant(1, self.qs, vs)
            if not all(st.enabled() for st in self.states):
                if now-self.transition > 2:
                    raise RuntimeError('Compliant motor enable not confirmed; master: '+
                                       m.diagnostic(now)+'; slave: '+s.diagnostic(now))
                if now-self.last_enable_retry >= .1:
                    self.last_enable_retry = now
                    for sock, st in zip(self.sockets, self.states):
                        for joint in range(1, 7):
                            if not st.frames[0x260+joint][1][5] & 0x40:
                                send(sock, 0x471, [joint, 2, 0, 0, 0, 0, 0, 0])
                                time.sleep(.0005)
                return
            self.mode = 'WAIT_ALIGN'
            self.transition = now
            self.refresh_modes(now, force=True)
            self.command = self.qs.copy()
            self.log('WAIT_ALIGN: both arms compliant; automatic alignment starts below 10 degrees on every joint.')
            return
        if not all(st.enabled() for st in self.states):
            raise RuntimeError('Joint enable state lost')
        if now-self.transition > .3:
            slave_mode = 6 if self.mode == 'WAIT_ALIGN' else 1
            if (m.frames[0x2a1][1][2] != 6 or s.frames[0x2a1][1][2] != slave_mode
                    or any(st.frames[0x2a1][1][0] != 1 for st in self.states)):
                raise RuntimeError('Requested CAN control mode not confirmed')
        error = float(np.max(abs(self.qm-self.qs)))
        torque = np.zeros(6)
        fresh = False
        if self.mode == 'WAIT_ALIGN':
            self.compliant(0, self.qm, vm)
            self.compliant(1, self.qs, vs)
            if error < self.cfg['alignment_tolerance'] and now-self.transition >= .3:
                self.command = self.qs.copy()
                position(self.sockets[1], self.command)
                send(self.sockets[1], 0x151, [1, 1, 10, 0, 0, 1, 0, 0])
                self.mode = 'ALIGNING'
                self.transition = now
                self.aligned_since = None
                self.log('ALIGNING: slave follows slowly at '+str(self.cfg['alignment_velocity'])+' rad/s.')
            elif now-self.last_log > 2:
                self.last_log = now
                self.log('WAIT_ALIGN: max joint difference '+str(round(math.degrees(error), 1))+' degrees; both arms compliant.')
                self.log('NEXT feedback OFF during WAIT_ALIGN. '+self.master_diagnostic())
            return
        if self.mode == 'ALIGNING':
            self.compliant(0, self.qm, vm)
            if error >= self.cfg['alignment_tolerance']:
                self.switch_slave_compliant(now)
                return
            if error <= self.cfg['alignment_complete_tolerance']:
                if self.aligned_since is None:
                    self.aligned_since = now
                if now-self.aligned_since >= self.cfg['alignment_settle_seconds']:
                    self.mode = 'TELEOP'
                    self.mask[:] = 1
                    self.last_next = self.last_contact = -math.inf
                    self.prev_sample = None
                    self.log('TELEOP: aligned; normal following and automatic NEXT feedback (when fresh/contact).')
            else:
                self.aligned_since = None
            speed = self.cfg['alignment_velocity']
        else:
            speed = self.cfg['max_velocity']
            fresh = (0 <= now-self.last_next < self.cfg['next_timeout'] and
                     0 <= now-self.last_contact < self.cfg['next_timeout'])
            torque = self.feedback.update(self.next_tau, dt, fresh, True, self.contact, self.mask)
            self.compliant(0, self.qm, vm, torque)
        if np.max(abs(self.command-self.qs)) > self.cfg['max_tracking_error']:
            raise RuntimeError('Slave tracking error exceeded limit')
        # Absolute joint mapping: remove startup offset only through slow alignment.
        self.command = limited_target(self.command, self.qm, speed, dt, self.gravity.limits)
        position(self.sockets[1], self.command)
        if self.mode != 'TELEOP':
            return
        if now-self.last_pub+1e-6 >= 1/self.cfg['publish_hz']:
            # Maintain a 50 Hz phase even with small scheduling jitter at 250 Hz.
            period = 1/self.cfg['publish_hz']
            self.last_pub += max(1, math.floor((now-self.last_pub+1e-6)/period))*period
            self.publish_feedback(torque, fresh and bool(self.mask.any()) and self.contact)
            _, measured = s.motor()
            sample_stamp = min(s.frames[i][0] for i in range(0x2a5, 0x2a8))
            if self.prev_sample is not None:
                old_time, old_q = self.prev_sample
                elapsed = sample_stamp-old_time
                if 0 < elapsed <= .04:
                    vel = (self.qs-old_q)/elapsed
                    stamp = self.node.get_clock().now().to_msg()
                    for pub, values in zip(self.inputs, [self.qs, vel, self.command, measured]):
                        msg = self.JointState();msg.header.stamp = stamp;msg.position = [float(v) for v in values]
                        pub.publish(msg)
            self.prev_sample = (sample_stamp, self.qs.copy())
        if now-self.last_log > 2:
            self.last_log = now
            self.log('TELEOP: NEXT fresh='+str(fresh)+'; feedback Nm='+str(np.round(torque, 3)))
            self.log(self.master_diagnostic())

    def close(self):
        try:
            # Joint disable must work even when the ROS context is already gone.
            # Do not publish diagnostics through potentially destroyed publishers.
            self.stop('controller exit', publish=False)
        finally:
            for s in self.sockets:
                s.close()
            try:
                self.node.destroy_node()
            finally:
                self.rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/piper_x_haptics.yaml')
    parser.add_argument('--continuous-torque', action='store_true', help='Continuous NEXT reflection; fixed software roles and hold-on-stop')
    parser.add_argument('--control', action='store_true', help='Automatically release old hold, align and run teleop/NEXT; no keyboard arming')
    parser.add_argument('--experimental-torque-master', action='store_true',
                        help='Explicitly allow replacing native leader mode with follower/MIT control (unverified)')
    parser.add_argument('--seconds', type=float, default=0, help='Bounded passive check only')
    parser.add_argument('--check', action='store_true', help='Validate setup offline, without opening CAN or ROS')
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    if args.check:
        check_setup(cfg)
        return
    if args.control and not (args.experimental_torque_master or args.continuous_torque):
        parser.error('This experimental torque controller changes the master role. '
                     'Use bash scripts/run_piper_x_haptics.sh control for native-master teleop instead.')
    if args.control and args.seconds:
        parser.error('--seconds is only for passive monitoring')
    if args.control:
        check_setup(cfg)
    lock = open('/tmp/factr2_piper_x_control.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    controller_type = Controller
    if args.continuous_torque:
        from piper_x_simulate_torque import TorqueTeleop
        controller_type = TorqueTeleop
    controller = controller_type(cfg, args.control)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        print(('AUTO: continuous NEXT torque feedback; Ctrl-C requests position holds.' if args.continuous_torque else
               'AUTO: support both arms during startup/release. No keyboard input required. Ctrl-C disables arm joints.')
              if args.control else 'MONITOR: passive inspection; no motor control.', flush=True)
        start = time.monotonic()
        controller.reset_loop_clock(start)
        schedule = Periodic(cfg['control_hz'], start)
        ros_schedule = Periodic(100, start)
        rate = LoopRate(start)
        while not args.seconds or time.monotonic()-start < args.seconds:
            timeout = max(0., schedule.deadline-time.monotonic())
            ready = select.select(controller.sockets, [], [], timeout)[0]
            drain_feedback(controller.sockets, ready,
                           lambda i, frame, arrival: controller.states[i].receive(frame, arrival))
            now = time.monotonic()
            if now >= schedule.deadline:
                if now >= ros_schedule.deadline:
                    for _ in range(4):
                        controller.rclpy.spin_once(controller.node, timeout_sec=0)
                    ros_schedule.advance(time.monotonic())
                tick_start = time.monotonic()
                rate.sample(tick_start, schedule.deadline)
                controller.tick(tick_start)
                completed = time.monotonic()
                rate.skipped += schedule.advance(completed)
                report = rate.report(completed, cfg['control_hz'])
                if report:
                    controller.log(report)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as exc:
        controller.log('FAULT: '+str(exc))
        raise SystemExit(1)
    finally:
        try:
            controller.close()
        finally:
            lock.close()


if __name__ == '__main__':
    main()
