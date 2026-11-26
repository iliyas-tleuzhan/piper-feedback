# Piper X CAN teleoperation

Standalone Python 3 relay for the two independently connected arms:
master **can_master**, slave **can_slave**. No ROS environment or Python packages needed.
The existing `teacher_arm/piper_teleop.py` uses a Gello teacher, so this script
instead uses the native Piper X teaching packets already observed on your master.

Both CAN interfaces must be up at 1 Mbit/s. Keep the master in its existing native
leader/teaching mode, with standard command IDs. Stop other controllers writing
to the slave, including the haptics controller. The relay never configures the master.

From the repository directory, with both arms physically matched in pose:

```bash
python3 scripts/piper_x_can_teleop.py --enable-slave
```

This waits for recent master targets and slave feedback within 7 degrees on every
joint, then sends follower configuration, CAN joint-position mode (10% speed,
upright mounting), and six individual joint-enable commands to the slave. Gently
move the master to supply targets: it may publish only while moving. Enabling and
forwarding can move the slave. Setup commands are sent once; successful motor
enable is not verified by this minimal relay.

If the slave is already enabled and configured to accept CAN joint-position
commands, use the plain relay (no setup or alignment check):

```bash
python3 scripts/piper_x_can_teleop.py
```

The relay copies `0x155`, `0x156`, `0x157` (six joint targets) and `0x159`
(gripper command, including the master's gripper enable/effort fields) unchanged.
Use `--no-gripper` to exclude the gripper. Interface names can be overridden with
`--master can_master --slave can_slave`.

It does not relay feedback, mode changes, calibration, MIT torque packets or
arbitrary CAN traffic. There is no force feedback or NEXT inference. It forwards
only newly received packets, so no data while stationary is normal.

**Ctrl-C stops forwarding.** It does not disable motors or cancel an in-progress
move: the slave can finish moving to its last target and remain enabled. CAN loss
has the same limitation; use the hardware stop to stop physical motion.

Validated offline only; this script has not been run against the arms.
