#!/usr/bin/env python3
"""Relay native Piper X leader commands from can1 to can0 (Linux, stdlib only).

The master must already be in native teaching/leader mode. Only standard
8-byte joint targets (0x155..0x157) and gripper commands (0x159) are relayed.
No NEXT, ROS, torque control, homing, or commands to the master are involved.
"""
import argparse
import fcntl
from contextlib import ExitStack
import select
import signal
import socket
import struct
import time

FRAME = struct.Struct('=IB3x8s')
JOINT_IDS = (0x155, 0x156, 0x157)
FEEDBACK_IDS = (0x2A5, 0x2A6, 0x2A7)


def open_bus(channel, identifiers):
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        # Match standard data frames only (exclude extended, RTR, error frames).
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       b''.join(struct.pack('=II', i, 0xE00007FF)
                                for i in identifiers))
        bus.bind((channel,))
        bus.settimeout(0.1)
        return bus
    except BaseException:
        bus.close()
        raise


def decode(frame, allowed):
    if len(frame) != FRAME.size:
        return None
    identifier, length, data = FRAME.unpack(frame)
    if identifier not in allowed or length != 8:
        return None
    return identifier, data


def send(bus, identifier, data):
    bus.sendall(FRAME.pack(identifier, 8, bytes(data)))


def aligned(master, slave, now, tolerance=7.0):
    """Require a complete, recent pose on both buses before optional enabling."""
    if not all(i in master and now-master[i][0] < 0.2 for i in JOINT_IDS):
        return False
    if not all(i in slave and now-slave[i][0] < 0.2 for i in FEEDBACK_IDS):
        return False
    for command, feedback in zip(JOINT_IDS, FEEDBACK_IDS):
        q_master = struct.unpack('>ii', master[command][1])
        q_slave = struct.unpack('>ii', slave[feedback][1])
        if any(abs(a-b) > tolerance*1000 for a, b in zip(q_master, q_slave)):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master', default='can1')
    parser.add_argument('--slave', default='can0')
    parser.add_argument('--no-gripper', action='store_true')
    parser.add_argument('--enable-slave', action='store_true', help=
                        'Configure slave as CAN MOVE J and enable its six joints '
                        'after fresh poses match within 7 degrees. May move the arm.')
    args = parser.parse_args()
    if args.master == args.slave:
        parser.error('Master and slave must be separate CAN interfaces.')
    allowed = JOINT_IDS + (() if args.no_gripper else (0x159,))
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with ExitStack() as stack:
        lock = stack.enter_context(open('/tmp/factr2_piper_x_control.lock', 'w'))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Another Piper X relay/haptics controller is running; stop it first.')
        master_bus = stack.enter_context(open_bus(args.master, allowed))
        slave_bus = stack.enter_context(open_bus(args.slave, FEEDBACK_IDS))
        master, slave = {}, {}
        ready = not args.enable_slave
        count = 0
        next_report = time.monotonic() + 3
        print(f'{args.master} -> {args.slave}: '
              f'{", ".join(hex(i) for i in allowed)}. Ctrl-C stops forwarding.', flush=True)
        if not ready:
            print('Waiting for fresh, aligned poses (within 7 degrees). '
                  'Match the arms and gently move the master. No commands sent yet.', flush=True)
        try:
            while running:
                readable, _, _ = select.select([master_bus, slave_bus], [], [], 0.1)
                for source in readable:
                    if not running:
                        break
                    packet = source.recv(FRAME.size)
                    decoded = decode(packet, allowed if source is master_bus else FEEDBACK_IDS)
                    if decoded is None:
                        continue
                    identifier, data = decoded
                    now = time.monotonic()
                    (master if source is master_bus else slave)[identifier] = (now, data)
                    if ready and source is master_bus:
                        send(slave_bus, identifier, data)
                        count += 1
                    elif not ready and aligned(master, slave, now):
                        # Seed the slave's current pose before changing mode/enabling.
                        send(slave_bus, 0x470, [0xFC, 0, 0, 0, 0, 0, 0, 0])
                        for target, feedback in zip(JOINT_IDS, FEEDBACK_IDS):
                            send(slave_bus, target, slave[feedback][1])
                        send(slave_bus, 0x151, [1, 1, 10, 0, 0, 1, 0, 0])
                        for joint in range(1, 7):
                            send(slave_bus, 0x471, [joint, 2, 0, 0, 0, 0, 0, 0])
                        ready = True
                        print('Slave setup/enable commands sent; forwarding new master packets.', flush=True)
                if time.monotonic() >= next_report:
                    print(f'Forwarded {count} packets.' if ready else
                          'Still waiting for fresh, aligned master/slave poses.', flush=True)
                    next_report = time.monotonic() + 3
        finally:
            print(f'Stopped forwarding ({count} packets). '
                  'No disable/home command sent; slave retains its last target.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except OSError as exc:
        raise SystemExit(f'CAN error: {exc}')
