#!/usr/bin/env python3
"""Set can1 to firmware master mode.

Adapted from piper_sdk/demo/V2/piper_set_slave.py; uses master role 0xFA.
Requires the Piper SDK. Stop teleop before running this script.
"""
from piper_sdk import C_PiperInterface_V2


if __name__ == "__main__":
    piper = C_PiperInterface_V2("can1")
    piper.ConnectPort()
    piper.MasterSlaveConfig(0xFA, 0, 0, 0)
    print("Master mode requested on can1.")
