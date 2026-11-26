#!/usr/bin/env python3
"""Print udev rules that name two USB SocketCAN adapters by serial number."""

import argparse
from pathlib import Path
import re

NET_DEVICES = Path('/sys/class/net')
ROLES = ('can_master', 'can_slave')


def usb_identity(interface):
    """Return the identity of the USB device above a SocketCAN interface."""
    try:
        if (interface / 'type').read_text().strip() != '280':  # ARPHRD_CAN
            return None
        device = (interface / 'device').resolve(strict=True)
    except FileNotFoundError:
        return None

    for parent in (device, *device.parents):
        vendor = parent / 'idVendor'
        product = parent / 'idProduct'
        if vendor.is_file() and product.is_file():
            serial = parent / 'serial'
            if serial.is_file():
                return tuple(path.read_text().strip() for path in (vendor, product, serial))
            return None
    return None


def rules_for(master, slave, devices=NET_DEVICES):
    identities = {entry.name: usb_identity(entry) for entry in devices.iterdir()}
    rules = []
    for name, role in zip((master, slave), ROLES):
        identity = identities.get(name)
        if identity is None:
            raise ValueError(f'{name}: expected a USB SocketCAN adapter exposing a serial number')
        vendor, product, serial = identity
        if (not re.fullmatch(r'[0-9a-fA-F]{4}', vendor)
                or not re.fullmatch(r'[0-9a-fA-F]{4}', product)
                or not re.fullmatch(r'[A-Za-z0-9._:+ -]+', serial)):
            raise ValueError(f'{name}: USB identity contains unsupported characters or an empty serial')
        if sum(other == identity for other in identities.values()) != 1:
            raise ValueError(f'{name}: USB identity matches multiple CAN interfaces')
        if role in identities and role != name:
            raise ValueError(f'{role} is already assigned to another interface')
        rules.append(
            f'ACTION=="add", SUBSYSTEM=="net", ATTR{{type}}=="280", '
            f'ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", '
            f'ATTRS{{serial}}=="{serial}", NAME:="{role}"'
        )
    if identities[master][2] == identities[slave][2]:
        raise ValueError('The master and slave adapters must expose different USB serial numbers')
    return rules


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('master', help='Current interface attached to the operator arm')
    parser.add_argument('slave', help='Current interface attached to the follower arm')
    args = parser.parse_args()
    try:
        rules = rules_for(args.master, args.slave)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print('\n'.join(rules))


if __name__ == '__main__':
    main()
