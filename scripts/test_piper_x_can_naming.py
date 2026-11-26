"""Offline coverage for persistent USB-to-CAN role assignment."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from configure_piper_x_can import rules_for


class CanNamingTests(unittest.TestCase):
    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.devices = Path(temp.name)

    def adapter(self, name, serial):
        interface = self.devices / name
        usb_device = interface / 'device'
        usb_device.mkdir(parents=True)
        (interface / 'type').write_text('280\n')
        (usb_device / 'idVendor').write_text('1d50\n')
        (usb_device / 'idProduct').write_text('606f\n')
        if serial is not None:
            (usb_device / 'serial').write_text(serial + '\n')

    def test_rules_follow_serial_when_kernel_names_swap(self):
        self.adapter('can0', 'Follower456')
        self.adapter('can1', 'Leader123')
        rules = rules_for('can1', 'can0', self.devices)
        self.assertIn('ATTRS{serial}=="Leader123", NAME:="can_master"', rules[0])
        self.assertIn('ATTRS{serial}=="Follower456", NAME:="can_slave"', rules[1])

        (self.devices / 'can0').rename(self.devices / 'temporary')
        (self.devices / 'can1').rename(self.devices / 'can0')
        (self.devices / 'temporary').rename(self.devices / 'can1')
        self.assertEqual(rules_for('can0', 'can1', self.devices), rules)

    def test_rejects_duplicate_usb_identity(self):
        self.adapter('can0', 'Duplicate')
        self.adapter('can1', 'Duplicate')
        with self.assertRaisesRegex(ValueError, 'multiple CAN interfaces'):
            rules_for('can1', 'can0', self.devices)

    def test_rejects_adapter_without_serial(self):
        self.adapter('can0', None)
        self.adapter('can1', 'Leader123')
        with self.assertRaisesRegex(ValueError, 'exposing a serial number'):
            rules_for('can1', 'can0', self.devices)


if __name__ == '__main__':
    unittest.main()
