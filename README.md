# piper-feedback

Piper X bilateral teleoperation over Linux SocketCAN. The operator moves a
leader arm and a follower arm mirrors it. In feedback mode, a trained NEXT
model estimates external loads on the follower's six joints, and the controller
reflects those loads to the leader. A separate gripper controller adds resistance
based on follower effort and the difference in gripper opening.

The hardware configuration targets two upright Piper X arms with standard
grippers and S-V1.8-9 firmware. Plain teleop uses the arms' native master and
follower modes. Feedback mode puts both arms in firmware slave mode so the PC
can command the leader in MIT torque mode and the follower in position mode.
Keep a hardware stop available and support both arms during startup and
transitions. Stopping the software can leave the motors enabled in position hold.

## 1. Install

1. On Ubuntu 24.04, install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
   With the ROS apt repository configured, install the host dependencies.

   ```bash
   sudo apt update
   sudo apt install git iproute2 python3.12-venv ros-jazzy-ros-base ros-jazzy-message-filters
   ```

   The optional upstream GELLO/teacher-arm nodes also require
   `ros-jazzy-pinocchio`.

2. Clone the repository and install its Python dependencies in a virtual
   environment using Ubuntu's Python 3.12.

   ```bash
   git clone https://github.com/iliyas-tleuzhan/piper-feedback.git
   cd piper-feedback
   /usr/bin/python3.12 -m venv --system-site-packages .venv
   source .venv/bin/activate
   python -m pip install -r requirements.txt
   ```

3. Build the ROS packages. Run project commands from the repository root.

   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

   In each new terminal used for recording or training, activate `.venv` and
   source both ROS setup files again.

## 2. Assign CAN adapters by USB serial number

1. Connect each USB-to-CAN adapter one at a time and run `ip -brief link` to
   identify its current interface. With both connected, verify which current name
   goes to the leader and which goes to the follower. Label the adapters and
   keep each adapter paired with its arm.
2. Generate the Linux udev rules with the **current leader interface first**.
   For example, if the leader currently appears as `can1` and the follower as
   `can0`, run:

   ```bash
   rules_file=$(mktemp) &&
     python3 scripts/configure_piper_x_can.py can1 can0 > "$rules_file" &&
     sudo install -m 0644 "$rules_file" /etc/udev/rules.d/99-piper-x-can.rules &&
     sudo udevadm control --reload-rules
   rm -f -- "$rules_file"
   ```

   Use USB SocketCAN adapters with distinct USB serial numbers and one CAN
   interface per adapter. Repeat this step if you replace an adapter.
3. Reconnect both USB adapters and verify that the leader appears as
   `can_master` and the follower as `can_slave`. Those names follow the
   adapters across reboots, USB ports, and plug-in order.

   ```bash
   ip -brief link show dev can_master
   ip -brief link show dev can_slave
   sudo bash scripts/bringup_piper_x_can.sh
   ```

   The bring-up script configures both links at 1 Mbit/s with transmit queues
   of 64. Stop controllers before reconfiguring or reconnecting either link.

## 3. Record and train a NEXT model

1. Record the follower moving freely without external contact. Open three
   terminals from the repository root. Run one command per terminal and keep
   plain teleop and the receive-only bridge running during recording.

   ```bash
   # Terminal 1
   bash scripts/run_piper_x_haptics.sh teleop
   ```
   ```bash
   # Terminal 2, with the environment from step 1 active
   python scripts/piper_can_record_bridge.py
   ```
   ```bash
   # Terminal 3, with the environment from step 1 active
   ros2 run factr2_next next_record
   ```

   In the recorder terminal, press **R** to start an episode and **R** again to
   finish it. Record multiple representative free-motion episodes. Press Ctrl-C
   to finish the session and find the HDF5 path printed by the recorder.
   See the [recording notes](docs/PIPER_X_RECORDING.md) for capture details.

2. Prepare training and validation files. Replace the example input with your
   recording path. The output directory must be new. The script splits timing
   gaps and holds out complete episodes, so the input must contain at least two
   usable episodes with continuous 50-sample segments.

   ```bash
   python scripts/prepare_next_data.py data/YOUR_RECORDING.h5 data/prepared
   ```

3. Train the model. The default training config reads
   `data/prepared/train.h5` and `data/prepared/val.h5`. To use different files
   or training settings, edit the YAML supplied to this command.

   ```bash
   ros2 run factr2_next next_train --config src/factr2_next/factr2_next/config/train.yaml
   ```

   The trainer prints the run directory under `runs/`. Set `checkpoint_dir`
   in `src/factr2_next/factr2_next/config/inference.yaml` to that directory,
   which must contain `model.pt`, `normalization.npz`, and `config.yaml`.

## 4. Run feedback

1. From the [AgileX arm descriptions](https://github.com/agilexrobotics/agx_arm_urdf/tree/main/piper_x/urdf),
   put the matching Piper X `piper_x_description.urdf` and
   `piper_x_with_gripper_description.xacro` in `assets/piper_x/`. You can also
   set `gravity_urdf` and `gripper_xacro` in the controller YAMLs under `config/`.
   Review the gains and limits for your hardware.
2. Stop the recording processes and plain teleop. Follow your firmware's
   restart procedure when changing an arm from native master to slave.
   Validate the configuration, then launch the controller.

   ```bash
   bash scripts/run_piper_x_haptics.sh check
   bash scripts/run_piper_x_haptics.sh feedback
   ```

   Use `teleop` for plain following or `native` for plain following with NEXT
   observation. NEXT writes its log to `logs/piper_x_next.log`. Run one
   controller at a time.

## Repository guide

- [scripts/](scripts/) contains the Piper X controllers, CAN tools, launcher,
  data preparation, and offline tests.
- [config/](config/) contains controller and hardware settings.
- [src/factr2_next/](src/factr2_next/) contains the ROS recorder, trainer, and
  inference nodes. The other ROS packages retain upstream teleoperation support.
- [docs/](docs/) contains [feedback controller details](docs/PIPER_X_SIMULATE_TORQUE.md)
  and historical hardware notes.

Run the offline controller tests with:

```bash
python -m unittest discover -s scripts -p 'test_piper_x*.py'
```

## Upstream and license

The data collection, NEXT training, inference, and ROS infrastructure are
adapted from [FACTR2 / NEXT](https://github.com/philiphan0109/factr2_next).
See also [FACTR Teleop](https://github.com/JasonJZLiu/FACTR_Teleop), the
[AgileX Piper SDK](https://github.com/agilexrobotics/piper_sdk),
[AgileX pyAgxArm](https://github.com/agilexrobotics/pyAgxArm), and the
[Piper X gravity compensation implementation](https://github.com/kehuanjack/agilex-arm-gravity-compensation/tree/imp/piper_x).
The repository retains its upstream attribution and [Apache License 2.0](LICENSE).
