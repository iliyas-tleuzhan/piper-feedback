# Continuous torque reflection

`bash scripts/run_piper_x_haptics.sh feedback` starts both arms as firmware
slaves. The master interface is the software leader in compliant MIT torque mode; can_slave follows
its measured joint angles in position mode at 250 Hz after startup alignment.
Continuous NEXT torque reflection and gripper following are enabled. There is
no master-speed shutdown and no runtime role swapping.

`torque-feedback` and `feedback` select the same controller. `native` explicitly
selects the older firmware-master teleop with NEXT observation only.
The default command (no argument) and `control` also select feedback with both
arms configured as firmware slaves. Explicit `teleop` selects plain native
teleop: can_master firmware master, can_slave follower, arm and gripper command forwarding,
no NEXT inference or force feedback.
No hardware execution was performed while implementing these changes.

The same NativeTeleop follower engine provides the 15-degree entry gate,
0.5-second pose sample, fixed-target alignment in 0.3-degree steps every 20 ms,
and direct joint target following at 250 Hz without a normal-motion slew limit.
The source differs: can_master runs MIT torque mode and its measured joint positions
replace native leader 0x155–0x157 targets. A Python adapter builds equivalent joint
payloads for the existing follower. Small encoder offsets accepted within the
startup calibration margin are clipped to the nominal command limits for both
alignment and subsequent following; gravity calculations retain measured angles.
Alignment completion compares against that same reachable command target.
The operator gripper motor is released at the start of alignment. Only fresh
measured gripper opening drives can_slave, using a 200-unit (0.2 Nm SDK command)
effort setting. Gripper feedback uses observed can_slave load and opening mismatch
to request effort-limited position resistance on can_master (5000 raw-unit effort cap).
It is not a NEXT estimate or a signed torque command; stale data/contact loss
releases the operator gripper. Physical feedback direction remains unverified.

Gripper resistance engages above a 0.5 mm opening mismatch and releases at
0.2 mm, reducing threshold chatter. Measured effort above the 0.02 deadband
is reflected at unity gain up to the legacy SDK ceiling of 5.0 (5000 raw units).
This ceiling does not request maximum effort continuously: a measured raw load
of 145 requests 125 after the 20-unit deadband. The follower command remains
200 raw units. These effort scales use
the legacy Piper SDK convention; they are not calibrated fingertip force.
Gripper feedback is independent of NEXT's six-joint contact flag. Commands
update at 50 Hz; stale/missing gripper telemetry (100 ms), gripper faults,
loss of load, or leaving TELEOP release resistance on the next control tick.
The `TORQUE DIAG` log includes both gripper openings, raw measured efforts,
status bytes, telemetry ages, the gate reason, and commanded effort. Use this
to distinguish missing telemetry from a load below the configured threshold.

Both arms are configured as firmware followers once at startup, while can_master is
always the operator master in software. Startup waits for firmware and fresh
telemetry without disabling joints. It seeds gravity support on can_master and a
measured-pose position hold on can_slave before requesting modes, then verifies
mode acknowledgement and enables only joints already disabled. The slave interface retains
its hold until alignment starts. Alignment remains required before following.
The transition has offline coverage but physical continuity is not verified;
mode changes and imperfect gravity compensation can still cause movement.

NEXT continuously estimates can_slave external joint torque from measured state,
measured effort, and the commands actually sent by the controller. The master
command combines modeled gravity, configured damping, and the signed NEXT
estimate after per-joint deadbands, gain scaling, torque limits and slew limits.
Reflection requires both a fresh NEXT residual and a fresh true contact signal.
Loss of contact, stale contact or stale torque immediately zeros reflection;
following continues. Reflection is also off during alignment. Gravity and light
damping remain active to support can_master. MIT kp and kd are commanded as zero;
damping is explicitly computed as -D * measured velocity and included in
feed-forward torque alongside gravity and reflection, before command scaling.
This removes dependence on the firmware velocity-loop gain for resistance.
The total-torque guard and diagnostics use the same requested sum. The
physical damping response still requires verification. NEXT subtracts predicted free-motion
torque, but model error can still cause false contact: this is not an independent
force sensor. Contact release is subject to inference smoothing and hysteresis.

Settings are in `config/piper_x_simulate_torque.yaml`. Arm reflection uses
negative unity gains and zero deadbands (reflection = -NEXT estimate before
limits; gravity and damping keep their original signs), with contact gating and a 6 Nm/s buildup ramp.
Torque reductions are immediate; reversals cancel the old sign before ramping
the new sign. Reflection caps are [8,14,10,3,3,2] Nm, matching the existing configured
total-torque ceilings at user request; these are not hardware-rated maxima. The 6 Nm/s buildup ramp, immediate reductions,
and motion suppression remain active after prior delayed acceleration.
Raising a cap only increases output when the requested reflection reaches it.
These are software settings, not certified hardware maxima. The total guard
checks gravity plus reflection, and the sum including damping, before sending
any MIT packet; it may fault before reflection reaches its own cap.
Direction, physical torque units, gravity model,
friction and closed-loop stability are NOT calibrated or physically verified.
Do not infer safe or realistic wall resistance from offline tests. Limits on
reflected torque do not replace validation of the physical response. Master torque encoding is pinned to
S-V1.8-9. Total torque limits remain. Master speed no longer triggers a shutdown; the
legacy slave startup speed guard remains. Master pose-limit shutdown is removed
in both startup and following. The slave interface targets remain clipped to nominal joint
limits; telemetry, torque and slave pose guards remain active.

On Ctrl-C or fault, the controller requests measured-position holds on both arms
instead of disabling motors. If fresh state is unavailable it uses the last
known master pose/slave command. These are firmware-retained targets after exit,
not an independently verified stop. The watchdog requests analogous holds on
heartbeat loss. Communication, mode, power or motor failure can prevent holding.

Offline validation:

```bash
bash scripts/run_piper_x_haptics.sh check
python3 -m unittest discover -s scripts -p 'test_piper_x*.py'
```

Response changes: inference EMA alpha is 0.6 at 50 Hz (about 60 ms to 90%
of a step, excluding model/transport delay). A per-joint latch suppresses
reflection if torque acts in the direction of measured master motion at or
above 0.05 rad/s. It rearms on renewed master motion opposing the reflected torque, or after a
fresh explicit no-contact reading. A stationary blocked joint does not rearm
merely because it stopped; this avoids repeatedly restarting driving torque.
Stale predictions alone do not rearm it. This may suppress legitimate assisting
feedback and is not a hand-presence detector or a proof of closed-loop stability.
Gravity/damping and teleoperation continue while reflection is suppressed.
