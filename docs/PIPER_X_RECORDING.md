# Recording Piper X data for NEXT

Follow the [recording and training steps in the README](../README.md#3-record-and-train-a-next-model).

The receive-only CAN bridge reads commanded joint angles, measured follower
angles, and motor current on the follower bus. By default it listens on
`can_slave` and publishes under `/next_capture/piper_x_follower` at 50 Hz.
It estimates joint velocity from position changes and converts signed motor
current to joint torque using the Piper X coefficients from
[pyAgxArm](https://github.com/agilexrobotics/pyAgxArm).

The recorder writes joint position, velocity, commanded position, and measured
torque in six-element vectors. It opens a new episode each time you start
recording. Stop an episode before pauses or disconnections to avoid recording
broken motion sequences.

If the bridge reports missing frames, first confirm that the follower is receiving
native teleop commands and that measured-state frames are present. The default
position feedback base is `0x2A5` and the default command base is `0x155`.
Run `python scripts/piper_can_record_bridge.py --help` for the supported offsets.
Keep recorder and bridge namespaces aligned if you change either one.
