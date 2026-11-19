"""Pure math and S-V1.8-9 encoding; no hardware access."""
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

# Current pyAgxArm Piper X joint_torque_c, used on the command side only.
# NEXT input effort keeps the original training-time K*B convention.
PIPER_X_TORQUE_C = np.array([1., 1., 1., 1.0723, 1.016565, .813252])


class TorqueResponseGuard:
    """Detect grossly missing torque response; this does not certify compliance."""
    def __init__(self):
        self.since = np.full(6, np.nan)

    def check(self, now, requested, measured, velocity):
        # Only inspect sustained, substantial demands at low speed. Allow
        # tracking error, quantization, transients and current calibration error.
        missing = ((abs(vector(requested)) >= 2.) &
                   (abs(vector(measured)) < .2*abs(vector(requested))) & (abs(vector(velocity)) < .1))
        self.since[~missing] = np.nan
        self.since[missing & np.isnan(self.since)] = now
        failed = np.flatnonzero(missing & (now-self.since >= 1.))
        if failed.size:
            raise RuntimeError('MIT torque response missing on joint(s) '+
                               ','.join(str(i+1) for i in failed)+
                               '; stopping instead of waiting stiff. Check torque-mode/firmware support.')


def vector(value, size=6):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'Expected {size} finite values')
    return a


def uint(value, low, high, bits):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{value} outside [{low}, {high}]')
    return int((value-low) * ((1 << bits)-1) / (high-low))


def mit_frame(q, damping, torque):
    # pyAgxArm v188/v189: 12-bit t_ff, +/-16 Nm, NO legacy CRC or b division.
    p = uint(q, -12.5, 12.5, 16)
    v = uint(0., -45., 45., 12)
    k = 0
    d = uint(damping, -5., 5., 12)
    t = uint(torque, -16., 16., 12)
    return bytes([p >> 8, p & 255, v >> 4, (v & 15) << 4 | k >> 8,
                  k & 255, d >> 4, (d & 15) << 4 | t >> 8, t & 255])


def joint_frames(q):
    q = vector(q)
    raw = np.rint(q * 180000 / math.pi).astype(np.int64)
    return [(0x155+i, struct.pack('>ii', *raw[i*2:i*2+2])) for i in range(3)]


class Gravity:
    """Static generalized gravity for the vendor URDF plus standard gripper.

    Gripper prismatic joints are fixed closed for the gravity calculation;
    their paired masses are symmetric. Base is upright, gravity along -Z.
    No mesh processing or motor SDK is needed.
    """
    def __init__(self, urdf, gripper):
        roots = [ET.parse(urdf).getroot(), ET.parse(gripper).getroot()]
        self.links = {}
        self.children = {}
        self.limits = np.zeros((6, 2))
        for root in roots:
            for link in root.findall('link'):
                inertial = link.find('inertial')
                if inertial is not None:
                    origin = inertial.find('origin')
                    xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                    self.links[link.get('name')] = (float(inertial.find('mass').get('value')), xyz)
            for j in root.findall('joint'):
                self.children.setdefault(j.find('parent').get('link'), []).append(j)
                if j.get('name') in [f'joint{i}' for i in range(1, 7)]:
                    limit = j.find('limit');i = int(j.get('name')[5:])-1
                    self.limits[i] = [float(limit.get('lower')), float(limit.get('upper'))]

    def evaluate(self, q):
        q = vector(q);tau = np.zeros(6);potential = 0.
        def walk(link, R, p, axes):
            nonlocal potential
            if link in self.links:
                mass, com = self.links[link];c = p + R @ com
                force = np.array([0., 0., mass*9.81]);potential += mass*9.81*c[2]
                for i, axis, origin in axes:
                    tau[i] += np.dot(axis, np.cross(c-origin, force))
            for j in self.children.get(link, []):
                origin = j.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                jr = R @ Rotation.from_euler('xyz', rpy).as_matrix();jp = p + R @ xyz
                next_axes = list(axes)
                name = j.get('name')
                if name in [f'joint{i}' for i in range(1, 7)]:
                    i = int(name[5:])-1
                    axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                    next_axes.append((i, jr @ axis, jp.copy()))
                    jr = jr @ Rotation.from_rotvec(axis*q[i]).as_matrix()
                walk(j.find('child').get('link'), jr, jp, next_axes)
        walk('world', np.eye(3), np.zeros(3), [])
        return tau, potential


class Feedback:
    def __init__(self, cfg):
        self.gains = vector(cfg['gains']);self.deadband = vector(cfg['deadband'])
        self.limits = vector(cfg['limits']);self.slew = float(cfg['slew_nm_per_second'])
        if np.any(self.deadband < 0) or np.any(self.limits < 0) or self.slew <= 0:
            raise ValueError('Invalid feedback bounds')
        self.value = np.zeros(6)

    def update(self, torque, dt, fresh, enabled, contact, joint_mask):
        # Stale/invalid messages remove feedback immediately; ramp only valid input.
        if not fresh or not enabled or not contact:
            self.value[:] = 0
            return self.value.copy()
        torque = vector(torque)
        target = self.gains * np.sign(torque) * np.maximum(abs(torque)-self.deadband, 0)
        target = np.clip(target, -self.limits, self.limits) * vector(joint_mask)
        # Do not carry old force through a release or direction reversal.
        reversal = self.value * target < 0
        self.value[reversal] = 0
        falling = abs(target) < abs(self.value)
        self.value[falling] = target[falling]
        self.value += np.clip(target-self.value, -self.slew*dt, self.slew*dt)
        # Deselected joints must stop immediately, even during a previous ramp.
        self.value *= vector(joint_mask)
        return self.value.copy()


class ReflectionMotionGate:
    """Suppress driving torque; permit renewed resisting motion, not rest rearming."""
    def __init__(self, speed):
        self.speed = float(speed)
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('Reflection motion threshold must be positive')
        self.blocked = np.zeros(6, dtype=bool)

    def update(self, torque, velocity, contact):
        torque, velocity = vector(torque), vector(velocity)
        if not contact:
            self.blocked[:] = False
            return np.zeros(6)
        resisting = (torque*velocity < 0) & (abs(velocity) >= self.speed)
        self.blocked[resisting] = False
        self.blocked |= (torque*velocity > 0) & (abs(velocity) >= self.speed)
        return np.where(self.blocked, 0., torque)


def limited_target(previous, desired, max_velocity, dt, limits):
    previous, desired = vector(previous), vector(desired)
    if not 0 < dt <= 0.1:
        raise ValueError('Control loop timing fault')
    # Small zero calibration offsets near mechanical limits are allowed at startup.
    desired = np.clip(desired, limits[:, 0], limits[:, 1])
    return previous + np.clip(desired-previous, -max_velocity*dt, max_velocity*dt)
