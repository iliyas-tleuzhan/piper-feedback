# piper-feedback

Piper X bilateral teleoperation and experimental force feedback over SocketCAN.
Two arms, one operator: **can1 is the leader in software, can0 is the follower**.
The feedback controller reflects estimated follower joint loads to the leader,
while a separate native teleop mode provides ordinary following without feedback.

This project focuses on the Piper control path: CAN commands, startup holds and
alignment, gravity compensation, continuous joint torque reflection, gripper
resistance, fault diagnostics, and offline controller tests. It preserves the
`simulate-torque` development history and retains NEXT inference and training
components from FACTR2. See [References and acknowledgments](#references-and-acknowledgments)
for its origins and licensing.

## Operating modes

| Command | can1 firmware role | can0 firmware role | Behavior |
| --- | --- | --- | --- |
| `feedback` (default), `control`, `torque-feedback` | Slave / CAN MIT control | Slave / position control | Software leader/follower with NEXT joint feedback and gripper resistance |
| `teleop` | Native master | Native slave | Plain arm and gripper following; no NEXT process or force feedback |
| `native` | Native master | Native slave | Native teleop with NEXT observation only |
| `monitor` | Unchanged | Unchanged | Passive controller inspection |
| `check` | Unchanged | Unchanged | Offline configuration and artifact validation |

“Leader” describes the operator's arm. In feedback mode **both firmware roles
must be slave** so the PC can command them. The name `simulate-torque` does not
mean a simulator: this controller sends commands to real motors.

## Hardware and environment

The current configuration targets two upright Piper X arms with standard grippers
and **S-V1.8-9 firmware**, Ubuntu, Python 3.12, and ROS 2 Jazzy. The torque encoder
is pinned to that firmware. Other hardware or firmware needs separate validation.

- Two SocketCAN interfaces: `can1` for the operator arm and `can0` for the follower.
- CAN links at **1,000,000 bit/s**, with transmit queues of at least **64** frames.
- Piper X URDF and gripper xacro files for the gravity model.
- A compatible trained NEXT checkpoint for feedback mode. Plain `teleop` does not
  need a checkpoint or a running inference node.

Checkpoints, recorded datasets, Python environments, and ROS build products are
not distributed here. Configurations contain paths from the development machine;
**update them before running on another machine**.

## Installation

```bash
git clone https://github.com/iliyas-tleuzhan/piper-feedback.git
cd piper-feedback
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install pip wheel setuptools==79.0.1
uv pip install colcon-core colcon-common-extensions
uv pip install 'numpy<2.0' pyyaml termcolor h5py torch cffi scipy
uv pip install python-can piper-sdk==0.6.1 dynamixel-sdk==4.0.5 pyserial
source /opt/ros/jazzy/setup.bash
python -m colcon build --symlink-install
source install/setup.bash
```

Choose the appropriate PyTorch build for your CPU or CUDA environment. The
bundled ROS package and Python import names remain `factr2_next` for compatibility;
the GitHub repository name is `piper-feedback`.

Before feedback operation, edit:

- `config/piper_x_simulate_torque.yaml`: CAN interfaces, gravity model paths,
  feedback gains and limits, gripper settings, and topic roots.
- `config/piper_x_haptics.yaml`: native teleop configuration and offline setup inputs.
- `src/factr2_next/factr2_next/config/inference.yaml`: checkpoint directory and topic roots.

The checkpoint directory must contain `model.pt`, `normalization.npz`, and
`config.yaml`. Controller and inference topic roots must agree. The current NEXT
input pipeline publishes at 50 Hz. Rebuild after changing installed ROS resources
as needed; do not assume a copied `install/` tree is portable.

Configure the CAN adapters for 1 Mbit/s first. This helper only adjusts queue
lengths; it does not configure bitrate, bring interfaces up, or move motors:

```bash
sudo bash scripts/setup_piper_x_can.sh
bash scripts/run_piper_x_haptics.sh check
```

## Run

Run only one controller at a time. Stop an existing controller before changing modes.

### Plain teleop without feedback

```bash
bash scripts/run_piper_x_haptics.sh teleop
```

Can1 is the native master and can0 the native follower. Startup retains follower
enables, captures a fresh measured pose, requests position hold, and confirms
position mode before enabling any disabled joints. Can0 holds while the operator
brings can1 within 15 degrees on every joint. A sampled alignment ramp then leads
into native command forwarding. There is no initial slave-disable command.
An arm that starts disabled needs support until its hold is established.

### Continuous feedback

```bash
bash scripts/run_piper_x_haptics.sh feedback
```

The launcher starts NEXT inference and the torque controller. Can1 receives
gravity compensation, software damping, and gated joint feedback in MIT mode;
can0 follows can1's measured joint angles. Both arms are requested as firmware
slaves. The installed Piper SDK's role-switch example notes that switching from
native master to slave requires an arm restart; follow the applicable firmware
procedure rather than assuming the role write took effect immediately.

Joint reflection uses the six NEXT external-torque estimates with contact and
freshness gates, signed gains, caps, a buildup slew limit, and suppression of
reflection that drives measured motion. A combined torque guard checks gravity,
reflection, and damping before sending commands. See
[PIPER_X_SIMULATE_TORQUE.md](PIPER_X_SIMULATE_TORQUE.md) for the controller details.

### Gripper feedback

Gripper resistance uses the follower's measured effort and the difference in
opening between the arms. It commands an effort-limited position target on the
leader; **it is not a seventh NEXT estimate or direct signed MIT torque control**.
It operates independently of the six-joint NEXT contact flag.

Current defaults engage above 0.5 mm mismatch, release at 0.2 mm, subtract a
20-raw-unit effort deadband, and use unity gain with a 5,000-raw-unit ceiling.
The follower gripper command is still 200 raw units. Raising the feedback ceiling
does not force maximum output: measured load still determines the command.
Legacy SDK effort units are not calibrated fingertip force.

Missing or stale gripper telemetry, faults, lost load, and leaving TELEOP release
leader resistance. Negative opening readings currently fail the opening-range
check, including small encoder offsets near closure. Logs expose these gates.

## Diagnostics and testing

`TORQUE DIAG` reports NEXT freshness, joint estimates, reflected torque, modeled
and measured leader effort, and gripper opening, status, raw effort, age, gate,
and last commanded effort. A native follower joint-limit fault reports the arm,
one-based joint number, angle, limits, and controller phase.

NEXT process output is written to `logs/piper_x_next.log`. Runtime logs are
ignored in new commits; historical logs remain in the preserved Git history.

```bash
.venv/bin/python -m unittest discover -s scripts -p 'test_piper_x*.py'
bash -n scripts/run_piper_x_haptics.sh
bash scripts/run_piper_x_haptics.sh check
```

The controller suite uses fake CAN sockets. The `check` command additionally
requires local model files and the trained checkpoint. Neither validates physical
force, gravity accuracy, stability, or firmware role switching on the real arms.

On Ctrl-C or a fault, the current native and continuous-torque controllers request
position holds; motors may remain enabled. Watchdog holds and CAN commands are
best effort. Support the arms during setup and transitions and keep a hardware
stop available. This is experimental research software, not a validated force
controller.

## Code map

- `scripts/piper_x_simulate_torque.py`: continuous feedback controller and hold watchdog.
- `scripts/piper_x_native_teleop.py`: native teleop, startup hold, alignment, and forwarding.
- `scripts/piper_x_haptics.py`: shared CAN state, ROS inputs, command encoding integration, and validation.
- `scripts/piper_x_haptic_core.py`: gravity model and feedback filtering/guards.
- `scripts/piper_x_gripper_feedback.py`: gripper resistance and gate reasons.
- `scripts/run_piper_x_haptics.sh`: mode selection.
- `scripts/test_piper_x*.py`: offline regression coverage.
- `src/factr2_next`: retained NEXT data collection, training, inference, and visualization.

Older `PIPER_X_*.md` documents retain development notes, including superseded
role-switching modes and paths from the original checkout. Use this README and
the current launcher as the entry point.

## References and acknowledgments

This is a separately maintained repository created from the `simulate-torque`
branch, not a GitHub fork. Its Git ancestry, original authorship, and applicable
license notices are preserved. It includes adapted FACTR2/NEXT code as well as
Piper-specific controller work; the relationship is more than conceptual inspiration.

- [FACTR2 / NEXT — philiphan0109/factr2_next](https://github.com/philiphan0109/factr2_next):
  source of the retained NEXT estimation, training, and ROS infrastructure.
- [FACTR Teleop](https://github.com/JasonJZLiu/FACTR_Teleop) and the
  [FACTR paper](https://arxiv.org/abs/2502.17432): background on low-cost bilateral
  teleoperation and force feedback, also credited by the upstream project.
- [AgileX Piper SDK](https://github.com/agilexrobotics/piper_sdk): CAN protocol,
  firmware-role configuration, and gripper command conventions.
- [AgileX pyAgxArm](https://github.com/agilexrobotics/pyAgxArm): additional protocol
  and firmware reference material.
- [Piper X gravity-compensation implementation](https://github.com/kehuanjack/agilex-arm-gravity-compensation/tree/imp/piper_x):
  reference for gravity torque with zero MIT stiffness/damping.

Distributed under the retained [Apache License 2.0](LICENSE). Original attribution
and package metadata remain with the upstream components.
