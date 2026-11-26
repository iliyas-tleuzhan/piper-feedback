#!/usr/bin/env python3
"""Request native Piper roles using the SDK demos' 0x470 wire format.

This does not acknowledge a firmware role change or enable either arm.
The controller imports swap_roles while holding the shared controller lock.
"""
import argparse
import fcntl
from contextlib import ExitStack

from piper_x_haptics import bus, disable_joints, send

LOCK = '/tmp/factr2_piper_x_control.lock'


def role_indices(mode):
    if mode not in ('teleop', 'feedback'):
        raise ValueError('mode must be teleop or feedback')
    return (0, 1) if mode == 'teleop' else (1, 0)


def swap_roles(sockets, mode, *, disable=True):
    """Sockets stay ordered [can_master, can_slave]; caller must stop all forwarding first."""
    master, slave = role_indices(mode)
    if disable:
        for sock in sockets:
            disable_joints(sock)
    # Same MasterSlaveConfig(0xFC/0xFA, 0, 0, 0) as the installed SDK demos.
    send(sockets[slave], 0x470, [0xfc, 0, 0, 0, 0, 0, 0, 0])
    send(sockets[master], 0x470, [0xfa, 0, 0, 0, 0, 0, 0, 0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('feedback', 'teleop'), default='feedback')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    master, slave = role_indices(args.mode)
    channels = ['can_master', 'can_slave']
    print(f'{args.mode}: {channels[master]} master, {channels[slave]} slave')
    if args.dry_run:
        return
    with ExitStack() as stack:
        lock = stack.enter_context(open(LOCK, 'w'))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sockets = [stack.enter_context(bus(name)) for name in channels]
        try:
            swap_roles(sockets, args.mode)
        except BaseException:
            for sock in sockets:
                try:
                    disable_joints(sock)
                except OSError:
                    pass
            raise
    print('Role requests sent; joints disabled. Firmware may require a restart. '
          'No role acknowledgement or forwarding performed.')


if __name__ == '__main__':
    main()
