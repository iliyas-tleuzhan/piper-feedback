# Piper X hardware teleop recording

`record.yaml` now uses `/next_capture/piper_x_follower`, a namespace supplied
by the receive-only bridge below. CAN hardware itself has no ROS namespace.
Record the follower, with the leader's joint targets as commands. This is a
single-arm dataset even though two arms are used for teleoperation.

## Current verification status

Live capture on `can0` (1 Mbps) verified feedback IDs `0x2A5–0x2A7` and
`0x251–0x256` at approximately 200 Hz, and leader command IDs `0x155–0x157`
while teleop was active. The bridge published 236 complete samples in a
five-second check. An end-to-end recorder test saved 133 rows of six joints
for each of the four training fields. That temporary test file was removed.

Leader commands stopped during part of the recording test. The bridge stops
publishing when any input is more than 30 ms old; it does not substitute
observed positions or indefinitely hold old commands. If warnings report
missing command IDs, resume teleop and start a fresh episode. Stop an episode
before pauses/disconnections so training windows do not span gaps.

## Run

From this workspace, prepare each terminal:

```bash
cd /home/dase-hw101/Iliyas/factr2_next
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Terminal 1:

```bash
python scripts/piper_can_record_bridge.py --can can0
```

The bridge only receives CAN frames; it never enables, reconfigures, or commands
an arm. It publishes four synchronized six-joint topics at up to 50 Hz.
Velocity is derived from joint position. Torque is current times the Piper X
`joint_torque_k * joint_torque_b` constants from the installed pyAgxArm SDK
(`/home/dase-hw101/pyAgxArm/pyAgxArm/api/constants.py`). It is a current-derived
torque estimate, not a dedicated torque sensor measurement.
Only change `--feedback-base` or `--command-base` when the corresponding IDs
have been verified on the bus. Multiple arms using the same feedback IDs
cannot be separated by this bridge.

Terminal 2, after the bridge receives complete samples:

```bash
ros2 run factr2_next next_record
```

Press `r` to start/stop each episode, then Ctrl-C to close the H5 file. Move
the leader while keeping the follower in free space without external contact.
Files are saved under `data/`. The installed config is symlinked to the edited
source config, so this config change needs no rebuild.

For training, set `data.train_h5_paths` and `data.val_h5_paths` in
`src/factr2_next/factr2_next/config/train.yaml` to the collected training and
held-out validation files. Its existing single-arm keys already match.
Then run `ros2 run factr2_next next_train`. Training paths are still placeholders;
no training-ready dataset was created during these checks.

Avoid launching this repository's standard Piper control demo merely to expose
topics: it can enable/home the arm and is not a passive teleop recording bridge.
