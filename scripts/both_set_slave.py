#!/usr/bin/env python3
"""Set can_slave and can_master to firmware slave mode.

Adapted from piper_sdk/demo/V2/piper_set_slave.py.
Requires the Piper SDK. Stop teleop before running this script.
The SDK example requires restarting an arm after switching master to slave.
"""
from piper_sdk import C_PiperInterface_V2


if __name__ == "__main__":
    can_slave = C_PiperInterface_V2("can_slave")
    can_master = C_PiperInterface_V2("can_master")
    can_slave.ConnectPort()
    can_master.ConnectPort()
    can_slave.MasterSlaveConfig(0xFC, 0, 0, 0)
    can_master.MasterSlaveConfig(0xFC, 0, 0, 0)
    print("Slave mode requested on can_slave and can_master. Restart any arm switched from master to slave.")
