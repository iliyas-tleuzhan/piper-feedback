#!/usr/bin/env python3
"""Set can0 and can1 to firmware slave mode.

Adapted from piper_sdk/demo/V2/piper_set_slave.py.
Requires the Piper SDK. Stop teleop before running this script.
The SDK example requires restarting an arm after switching master to slave.
"""
from piper_sdk import C_PiperInterface_V2


if __name__ == "__main__":
    can0 = C_PiperInterface_V2("can0")
    can1 = C_PiperInterface_V2("can1")
    can0.ConnectPort()
    can1.ConnectPort()
    can0.MasterSlaveConfig(0xFC, 0, 0, 0)
    can1.MasterSlaveConfig(0xFC, 0, 0, 0)
    print("Slave mode requested on can0 and can1. Restart any arm switched from master to slave.")
