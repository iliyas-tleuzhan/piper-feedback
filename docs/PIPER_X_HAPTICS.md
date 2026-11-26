> Historical development notes. See [the README](../README.md) for the current launcher and installation workflow.

The default launcher mode is feedback, with **both arms as firmware slaves**.
`control` also selects feedback. For plain teleop with can_master firmware master
and can_slave slave, explicitly use:

```bash
cd /path/to/piper-feedback
bash scripts/run_piper_x_haptics.sh teleop
```

The explicit `teleop` mode forwards arm
and gripper commands without NEXT inference or force feedback. It uses the
existing native alignment sequence. The follower retains its enables and holds
its measured startup pose while the leader is brought within 15 degrees.
Use `feedback` explicitly for continuous torque reflection.

> The mode descriptions below are historical. See [PIPER_X_SIMULATE_TORQUE.md](PIPER_X_SIMULATE_TORQUE.md) for the explicit feedback mode.

> `control` and `feedback` now launch NEXT-triggered native role reversal. See [PIPER_X_ROLE_SWAP.md](PIPER_X_ROLE_SWAP.md). The fixed-role and legacy torque modes below remain available under `native` and `torque-feedback`.

# Native Piper X master/slave teleop and NEXT estimation

For fixed-role operation, the `native` command uses `scripts/piper_x_native_teleop.py`:
**can_master remains native master (0xFA); can_slave is follower (0xFC)**. It never sends MIT
or position-mode commands to the master. This replaces the earlier controller
that changed both arms into follower/CAN torque mode.

Stop old controllers and support both arms before starting or stopping.

```bash
cd /path/to/piper-feedback
# Once after adapter reconnect/reboot, if queues have returned to 10:
sudo bash scripts/setup_piper_x_can.sh
# Start native teleop, passive slave data bridge, and NEXT inference:
bash scripts/run_piper_x_haptics.sh native
```

No arming keys are required. Startup selects native master on can_master and follower
on can_slave without disabling the slave. Once fresh measured state is available,
it seeds a fixed position hold, confirms position mode, and enables any disabled
slave joints. It holds that pose while waiting for the master's native targets
(0x155–0x157) to be within 15 degrees, then runs the alignment ramp and teleop.
There is no deliberate release between holding, alignment, and following.
An initially disabled arm needs physical support until the hold is established.

Native alignment now uses the wireless bridge's sampled-pose ramp: a 15-degree
entry threshold and 0.3-degree steps every 20 ms toward a fixed sampled master
pose. Once commanded alignment finishes, the native joint-frame payloads are
forwarded unchanged at 250 Hz, without the old 0.25 rad/s limiter. See
[PIPER_X_ROLE_SWAP.md](PIPER_X_ROLE_SWAP.md) for the complete behavior and limits.

A native master can stop publishing while stationary. The last complete native
target remains the target; the program does not mistake it for missing slave
feedback. This also means a disconnected master cannot reliably be distinguished
from a stationary master: the slave may finish moving to its last target.
Fresh slave feedback and the tracking-error limit are still required.

**NEXT is estimation-only in this mode. No external torque is applied to the
master.** The passive bridge supplies the same trained 50 Hz input format, and
the inference checkpoint remains unchanged. Logs: `logs/piper_x_next.log`.
Do not start duplicate inference or bridge processes.

Optional dashboard:

```bash
bash scripts/run_piper_x_haptics.sh dashboard
```

Open http://127.0.0.1:8080. NEXT estimates remain useful, but old force-reflection
fields are not evidence of torque actuation in this native-mode controller.

Ctrl-C and faults request disable of the six arm joints on both arms. Support
them first. The independent watchdog also requests disable if the controller
stops heartbeating. CAN failures can prevent release; software motor disable
is not a substitute for mechanical support or the hardware stop.

## Experimental force reflection

There is now a separate, explicit command for the software-master torque path:

```bash
cd /path/to/piper-feedback
sudo bash scripts/setup_piper_x_can.sh
bash scripts/run_piper_x_haptics.sh torque-feedback
```

Stop other controllers first and support both arms during startup and shutdown.
This command **changes can_master to firmware follower/CAN MIT mode**. The master interface remains
the operator's master in software: its measured angles drive can_slave, and its
motors receive gravity compensation plus bounded NEXT feedback. Both arms
publish normal measured feedback frames; these are intentional in this path.
Native master mode and this software-master mode are different operating modes.

The launcher starts inference and the torque controller, with no separate data
bridge: the controller supplies NEXT inputs itself. It rejects down interfaces,
CAN error states, wrong bitrates, and TX queues smaller than 64 before starting.
It uses zero commanded position stiffness and zero commanded damping, requests
250 Hz, and follows the same automatic 10-degree entry / slow alignment /
1-degree completion sequence. No terminal key presses. NEXT torque stays off
until TELEOP and fresh contact estimates. Ctrl-C/faults request disable, not hold.

The revised command encoding includes the current SDK's Piper X wrist torque
coefficients. Missing enable acknowledgments are retried for disabled joints
within the existing two-second timeout. A gross torque-response check stops if
a stationary joint requests at least 2 Nm but measures less than 20% of that
for one second. This detects the earlier joint-3 log pattern; it does **not**
prove compliance or detect every torque fault. Old K*B effort units are retained
for NEXT inputs because the checkpoint was trained with them.

This is a candidate implementation, **not a physically verified fix for the
stiff master**. Zero damping removes commanded viscous resistance but does not
remove mechanical friction or fix ignored torque commands. If the master is
still stiff, stop; do not force it through the alignment threshold. The wrist
scaling correction cannot explain the old joint-3 torque mismatch.

Protocol reference: [AgileX firmware reference](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/firmware_reference.md).
Implementation reference: [Piper X gravity-compensation example](https://github.com/kehuanjack/agilex-arm-gravity-compensation/blob/imp/piper_x/main_gc.py)
uses zero stiffness/damping and MIT gravity torque. That example is independent
evidence of an implementation approach, not verification on these particular arms.

Offline checks (no CAN commands):

```bash
bash scripts/run_piper_x_haptics.sh check
.venv/bin/python -m unittest discover -s scripts -p 'test_piper_x*.py'
```

The revised native workflow has been tested offline only, not run on the arms.

Startup/receive corrections after the failed feedback runs:
- Start the control deadline clock after ROS initialization.
- Drain CAN fairly across both interfaces independently of an expired control
  deadline, bounded to 128 frames per arm and a 2 ms receive budget. Preserve
  kernel arrival timestamps so a genuine outage still faults.
- Retry disable for joints still reporting enabled during PREPARE, at most
  10 Hz within the existing two-second timeout; never enable until release
  and mode feedback are confirmed.
- Report arm, joint, speed and phase on overspeed without increasing the limit.

46 offline tests pass. The reported speed fault during ENABLING remains
unexplained; these changes do not verify physical compliance or gravity accuracy.
