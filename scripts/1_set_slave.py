#!/usr/bin/env python3
"""Set can_master to firmware slave mode.

Adapted from piper_sdk/demo/V2/piper_set_slave.py.
Requires the Piper SDK. Stop teleop before running this script.
The SDK example requires restarting an arm after switching master to slave.
"""
from piper_sdk import C_PiperInterface_V2


if __name__ == "__main__":
    piper = C_PiperInterface_V2("can_master")
    piper.ConnectPort()
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)
    print("Slave mode requested on can_master. Restart the arm if switching from master to slave.")
