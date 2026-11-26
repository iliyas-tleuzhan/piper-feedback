#!/usr/bin/env python3
"""Receive-only Piper X CAN to NEXT bridge. Never enables or commands an arm.

Wire formats and torque coefficients match the installed pyAgxArm Piper X
driver (api/constants.py and drivers/piper/default/parser.py).
The chosen feedback IDs MUST belong to the follower. Commands must be visible
on this bus; no measured-position fallback is used for missing commands.
"""
import argparse
import math
import socket
import struct
import time

# pyAgxArm computes torque = current_A * joint_torque_k * joint_torque_b.
TORQUE_K = (1.18125, 1.65375, 1.18125, 0.95844, 0.95844, 0.95844)
TORQUE_B = (4.0, 2.5, 4.0, 1.2, 0.8, 1.0)

class Capture:
    def __init__(self, feedback_base=0x2A5, command_base=0x155):
        self.feedback_base = feedback_base
        self.command_base = command_base
        self.frames = {}
        self.required = [feedback_base + i for i in range(3)]
        self.required += [command_base + i for i in range(3)]
        self.required += list(range(0x251, 0x257))

    def receive(self, can_id, payload, now):
        if can_id in self.required and len(payload) == 8:
            self.frames[can_id] = (now, payload)

    def missing(self, now, max_age=0.03):
        return [i for i in self.required
                if i not in self.frames or now - self.frames[i][0] > max_age]

    def values(self):
        def joints(base):
            raw = [v for i in range(3)
                   for v in struct.unpack('>ii', self.frames[base + i][1])]
            return [v * math.pi / 180000 for v in raw]
        # SDK decoder treats current as signed int16, despite an outdated
        # uint16 label in the SDK message documentation.
        currents = [struct.unpack('>h', self.frames[i][1][2:4])[0] / 1000
                    for i in range(0x251, 0x257)]
        torque = [v * TORQUE_K[i] * TORQUE_B[i]
                  for i, v in enumerate(currents)]
        return joints(self.feedback_base), joints(self.command_base), torque


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can_slave')
    parser.add_argument('--namespace', default='/next_capture/piper_x_follower')
    parser.add_argument('--feedback-base', type=lambda s: int(s, 0),
                        choices=[0x2A5, 0x2B5, 0x2C5], default=0x2A5)
    parser.add_argument('--command-base', type=lambda s: int(s, 0),
                        choices=[0x155, 0x165, 0x175], default=0x155)
    parser.add_argument('--seconds', type=float, default=0,
                        help='Stop after this duration; zero runs until Ctrl-C.')
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    capture = Capture(args.feedback_base, args.command_base)
    # Raw SocketCAN receive only: no SDK initialization or CAN send calls.
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                   b''.join(struct.pack('=II', i, 0xC00007FF)
                            for i in capture.required))
    bus.bind((args.can,))
    bus.settimeout(0.005)
    rclpy.init()
    node = rclpy.create_node('next_passive_can_capture')
    names = ['joint_pos_obs', 'joint_vel_obs', 'joint_pos_cmd', 'joint_effort_obs']
    pubs = [node.create_publisher(JointState,
            args.namespace.rstrip('/') + '/' + name, qos_profile_sensor_data)
            for name in names]
    start = time.monotonic()
    next_publish = start
    next_warning = start + 2
    previous = None
    count = 0
    node.get_logger().info(f'Receive-only {args.can} -> {args.namespace}, 50 Hz')
    try:
        while rclpy.ok() and (not args.seconds or time.monotonic() - start < args.seconds):
            try:
                frame = bus.recv(16)
                can_id, length, payload = struct.unpack('=IB3x8s', frame)
                capture.receive(can_id, payload[:length], time.monotonic())
            except socket.timeout:
                pass
            now = time.monotonic()
            if now < next_publish:
                continue
            next_publish = now + 0.02
            missing = capture.missing(now)
            if missing:
                previous = None
                if now >= next_warning:
                    node.get_logger().warning('No complete sample; missing/stale CAN IDs: ' +
                                              ', '.join(hex(i) for i in missing))
                    next_warning = now + 2
                continue
            pos, cmd, torque = capture.values()
            sample_time = min(capture.frames[i][0]
                              for i in range(args.feedback_base, args.feedback_base + 3))
            if previous is None:
                previous = (sample_time, pos)
                continue
            dt = sample_time - previous[0]
            if dt <= 0:
                continue
            vel = [(q - old) / dt for q, old in zip(pos, previous[1])]
            previous = (sample_time, pos)
            stamp = node.get_clock().now().to_msg()
            for pub, values in zip(pubs, [pos, vel, cmd, torque]):
                msg = JointState()
                msg.header.stamp = stamp
                msg.name = [f'joint{i}' for i in range(1, 7)]
                msg.position = values
                pub.publish(msg)
            count += 1
    except KeyboardInterrupt:
        pass
    finally:
        print(f'Published {count} complete samples.')
        bus.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
