> On branch `simulate-torque`, `control` and `feedback` now use continuous torque reflection. See [PIPER_X_SIMULATE_TORQUE.md](PIPER_X_SIMULATE_TORQUE.md). The role-switching behavior below is historical.

# NEXT-triggered Piper X role switching

The default launcher implements the requested state cycle:

| Mode | Master | Slave | Command rate |
| --- | --- | --- | --- |
| teleop | can1 | can0 | 250 Hz |
| feedback | can0 | can1 | 50 Hz (from the whiteboard) |

NEXT always observes physical **can0**, with the existing checkpoint and 50 Hz
input format. Contact enters feedback; sustained release returns to teleop.
The cycle continues until Ctrl-C or a fault. The six arm joints are followed;
native gripper commands (0x159) are forwarded after following starts.

## Firmware and model limitations

The installed `~/piper_sdk/piper_sdk/demo/V2/piper_set_slave.py` explicitly
says that changing a master into a slave requires an arm restart. The role
requests alone therefore do **not** establish that live switching works on
these arms. This implementation is hardware-unverified. It requires new master
native targets and new slave CAN position-control telemetry, and stops if those
do not become operational. It does not reboot the arms automatically.

Native role reversal mirrors joint motion caused by contact. It does not command
the measured external torque at the operator arm, and a force that causes no
motion may produce no mirrored motion. The NEXT checkpoint was trained with
follower command/current data; its contact accuracy in native master mode has
not been validated. Native master can0 must still emit complete position,
current, and native command frames. Missing inputs/predictions stop operation;
there is no fabricated command or position fallback. A stationary native master
that stops publishing commands can therefore cause a NEXT timeout.

## Run

Support both arms during startup, transitions and shutdown: role switching
requests joint disable before reconfiguration, and this program supplies no
gravity compensation to a disabled arm.

```bash
cd /home/dase-hw101/Iliyas/factr2_next
bash scripts/run_piper_x_haptics.sh control
```

`feedback` selects the same new behavior. `native` selects fixed roles with
observation-only NEXT; `torque-feedback` selects the previous experimental MIT
controller. Stop other controllers, inference instances and bridges first.
The launcher checks CAN queues and starts one inference process and one switching
controller. The switching controller publishes the can0 NEXT inputs itself.
Inference logs go to `logs/piper_x_next.log`.

Edit `config/piper_x_role_swap.yaml` for rates, thresholds and dwell times.
The detector uses the sum of absolute values of the six **filtered external
torque** estimates: at least 2.2 Nm for 0.08 s enters feedback; at most 1.0 Nm
with no added dwell exits. These are initial configurable thresholds, not calibrated values.
They are independent of the inference node's Bool contact topic. Timestamped
vectors are used so stale, duplicate, nonfinite or pre-transition estimates
cannot trigger a swap. Teleop and feedback rates are scheduling targets, with
measured rate reports; Linux/Python scheduling is not hard real time.

Every transition pauses forwarding, requests the outgoing slave hold its pose, writes the new
roles, clears cached targets and waits for fresh native targets and telemetry.
Initial startup alignment follows `piper-wireless-teleop/scripts/slave_receiver.py`: sample
fresh poses over 0.5 seconds, allow at most 15 degrees difference per joint,
freeze the sampled master target, then ramp by at most 0.3 degrees per joint
every 20 ms. Within 0.5 degrees of the commanded target, send the exact target
and enter TELEOP. This completion criterion describes commanded position, not
proof that measured position has settled. If the master has moved more than
15 degrees from the slave at handover, re-sample instead of jumping to it.
The alignment ramp has a 10-second timeout and does not run catch-up bursts.

The slave mode matches the wireless configuration: MotionCtrl_2(1, 1, 100, 0xAD),
with remaining bytes zero. It is selected after enable confirmation and is not
repeated every 100 ms. Native joint payloads 0x155–0x157 are cached and sent
unchanged each control tick (250 Hz teleop, 50 Hz feedback), with no software
slew limiter or offset. Gripper command 0x159 is forwarded unchanged as well. Mode/enable/config
frames are not forwarded. Telemetry/fault checks remain active.

The NEXT input stream pauses during alignment so inference drops the previous
history. Its 15-second warmup timer starts only after entry to TELEOP; manual
pose matching no longer consumes that timer. Missing initial master targets
still time out after 3 seconds. After valid predictions start, a 0.3-second
NEXT gap faults. CAN feedback fault timeout is 0.5 seconds in all phases.

The automatic role-switching controller no longer sends joint-disable commands
on transitions or shutdown. Runtime faults enter persistent HOLD: stop following
the master, freeze the fresh measured slave pose (or last command if feedback is
stale), and repeat that target at 50 Hz. No automatic resume occurs. Ctrl-C exits
after requesting the hold once more, leaving motors enabled and firmware retaining
the target. The watchdog requests the last supplied slave hold target on heartbeat
loss, instead of disabling. With no known pose, it cannot construct a hold.
CAN loss, power loss, motor faults, or firmware timeout can prevent holding.
This policy applies to `control`/`feedback`, not the legacy `native`/MIT controllers
or the standalone role-setting utility.

NEXT command inputs on can0 in teleop come from successfully sent command bytes:
SocketCAN does not echo the sender's own frames by default. In feedback, native
can0 master commands are received normally. Position/current always come from
real can0 telemetry. Input publishing uses a phase-preserving 50 Hz schedule.
The console prints NEXT ONLINE when the first valid prediction is accepted.

## Standalone role requests

`canframe_swap.py` defaults to feedback and implements the same zero-offset
`MasterSlaveConfig(0xFA/0xFC, 0, 0, 0)` frames as the installed SDK demos. Both
interfaces retain their normal IDs: roles do not literally exchange 0x1xx and
0x2xx frame families. The controller imports this helper directly, under one
writer lock, to avoid launching an overlapping CAN writer for each transition.

```bash
# Preview only; opens no CAN sockets:
.venv/bin/python scripts/canframe_swap.py --mode feedback --dry-run
.venv/bin/python scripts/canframe_swap.py --mode teleop --dry-run
# Actual role request (stop the automatic controller first):
.venv/bin/python scripts/canframe_swap.py --mode feedback
.venv/bin/python scripts/canframe_swap.py --mode teleop
```

Standalone requests leave joints disabled and do not verify firmware acceptance
or run forwarding. They fail if another cooperating controller holds the lock.

## Offline validation

```bash
python3 -m unittest discover -s scripts -p 'test_piper_x*.py'
```

Tests use simulated sockets, including role bytes/order, repeated contact cycles,
hysteresis/dwell, stale/invalid predictions, physical can0 routing, cache resets,
missing native targets and stale NEXT faults. They do not validate firmware hot
switching, torque accuracy in teaching mode, or physical feedback behavior.

Native startup feedback gaps: `native_feedback_timeout` in
`config/piper_x_haptics.yaml` is 0.5 seconds in all native controller phases.
After 0.2 seconds without fresh required feedback, new commands pause and the
alignment dwell resets; the arm can still hold its last target. At 0.5 seconds
the automatic controller enters HOLD. Normal TELEOP also uses this 0.5-second fault threshold.
This accommodates transient gaps; it does not repair persistent telemetry loss.

## Brief feedback and immediate release

Contact-on is now 2.2 (L1 sum of filtered NEXT joint torques); release remains 1.0
with zero release dwell. A fresh low estimate returns roles before another reverse
control tick, including during handover. This is first-detected-sample response,
not zero latency: sampling, smoothing, inference and firmware add delay.

Only initial startup aligns. Live teleop→feedback and feedback→teleop handovers
skip pose sampling and the alignment ramp; they require fresh targets, a <=15°
pose difference, enabled motors and a post-command CAN mode acknowledgement.
Gripper 0x159 payloads follow the active master to the active slave in both modes,
without reinterpretation. No gripper commands are forwarded during handover.

Feedback lasts at most `max_feedback_seconds: 0.5`, measured from the role
request, even if predictions disappear or the new master stays stationary.
Reverse-controller faults also request return to default teleop roles. A new
pulse requires a fresh release estimate after return; persistent contact cannot
immediately re-trigger and trap the operator in repeated role reversal. Failure
to establish default roles still enters HOLD; software cannot guarantee that
firmware makes can1 backdrivable. Inference may need to rewarm after a role gap,
so the pulse cap can cause return before any reverse movement occurs.

Role reversal still mirrors motion, not measured torque. A blocked can0 that
does not move may produce no tactile cue. NEXT accuracy in native master mode
and live firmware role-switch latency require physical validation.

Return-to-teleop mismatch recovery: if fresh default-role targets differ from
can0's measured pose by more than 15°, the controller now reports all six joint
differences and enters RECOVER TELEOP / WAIT_ALIGN instead of persistent fault
HOLD. It sends can0 its current pose as a hold target, then waits for the operator
to bring can1 within 15° and runs the startup ramp before forwarding again.
Normal close-pose switches still skip alignment, and feedback never aligns.
Other faults still use HOLD. This avoids a jump but does not guarantee automatic
recovery from incompatible firmware roles or explain a measured pose mismatch.
