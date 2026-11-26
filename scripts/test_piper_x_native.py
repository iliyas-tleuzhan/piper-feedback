import math
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np

from piper_x_native_teleop import NativeTeleop
from piper_x_haptic_core import joint_frames
from test_piper_x_haptics import CFG


def frames(sock):
    return [struct.unpack('=IB3x8s', c.args[0]) for c in sock.send.call_args_list]


def state(c, now, master, slave=0., enabled=False):
    q=[master,0,0,0,0,0];qs=[slave,0,0,0,0,0]
    for i,d in joint_frames(q):c.receive(0,struct.pack('=IB3x8s',i,8,d),now)
    for i,d in joint_frames(qs):c.states[1].frames[i+0x150]=(now,d)
    for i in range(0x261,0x267):c.states[1].frames[i]=(now,bytes([0,0,0,0,0,0x40 if enabled else 0,0,0]))
    c.states[1].frames[0x2a1]=(now,bytes([1,0,1,0,0,0,0,0]))
    c.last_tick=now-.004


class NativeTests(unittest.TestCase):
    def test_basic_startup_holds_without_disabling_then_aligns(self):
        c=self.make();c.setup()
        self.assertFalse(any(i==0x471 for i,_,d in frames(c.sockets[1])))
        state(c,10.01,math.radians(30),slave=.1,enabled=True)
        c.tick(10.01)
        self.assertEqual(c.phase,'HOLD_STARTUP')
        np.testing.assert_allclose(c.command,[.1,0,0,0,0,0],atol=1e-5)
        state(c,10.02,math.radians(30),slave=.1,enabled=True)
        c.tick(10.02)
        self.assertEqual(c.phase,'WAIT_ALIGN')
        held=c.command.copy()
        for now in (10.1,10.7):
            state(c,now,math.radians(30),slave=.11,enabled=True)
            c.tick(now)
            self.assertEqual(c.phase,'WAIT_ALIGN')
            np.testing.assert_array_equal(c.command,held)
        state(c,10.71,.1,slave=.1,enabled=True);c.tick(10.71)
        self.assertEqual(c.phase,'ENABLING')
        state(c,10.72,.1,slave=.1,enabled=True);c.tick(10.72)
        self.assertEqual(c.phase,'ALIGNING')
        state(c,10.73,.1,slave=.1,enabled=True);c.tick(10.73)
        self.assertEqual(c.phase,'TELEOP')
        self.assertFalse(any(i==0x471 for i,_,d in frames(c.sockets[1])))

    def test_startup_seeds_hold_before_enabling_disabled_slave_without_master_target(self):
        c=self.make();c.setup()
        state(c,10.01,0.,slave=.1,enabled=False);c.target=None
        c.tick(10.01)
        self.assertFalse(any(i==0x471 for i,_,d in frames(c.sockets[1])))
        state(c,10.02,0.,slave=.1,enabled=False);c.target=None
        c.tick(10.02)
        enables=[d for i,_,d in frames(c.sockets[1]) if i==0x471]
        self.assertEqual([d[:2] for d in enables],[bytes([j,2]) for j in range(1,7)])
        state(c,10.03,0.,slave=.1,enabled=True);c.target=None
        c.tick(10.03)
        self.assertEqual(c.phase,'WAIT_ALIGN')
        state(c,10.04,0.,slave=.1,enabled=True);c.target=None
        c.tick(10.04)
        np.testing.assert_allclose(c.command,[.1,0,0,0,0,0],atol=1e-5)

    def test_pose_fault_identifies_source_joint_and_angle(self):
        for source in ('measured', 'target'):
            c=self.make();state(c,10.01,0.,enabled=True)
            q=np.zeros(6);q[4]=math.radians(100)
            if source == 'measured':
                for identifier,payload in joint_frames(q):
                    c.states[1].frames[identifier+0x150]=(10.01,payload)
            else:
                c.target=q
            with self.assertRaises(RuntimeError) as error:
                c.tick(10.01)
            message=str(error.exception)
            self.assertIn('can_slave measured' if source == 'measured' else 'leader target',message)
            self.assertIn('joint 5: angle=100.000 deg',message)
            self.assertIn('limits=[-89.000, 89.000] deg',message)
            self.assertIn('tolerance=2.865 deg',message)

    def test_alignment_finishes_with_small_zero_offset_at_joint_limit(self):
        c=self.make()
        # Joint 2 reads -1.15 degrees near its nominal zero. This is inside
        # the accepted calibration margin, but outside the commanded limits.
        for step in range(120):
            now=10.01+step*.02
            state(c,now,0.,enabled=True)
            q=np.array([0.,-.02,0.,0.,0.,0.])
            for identifier,payload in joint_frames(q):
                c.receive(0,struct.pack('=IB3x8s',identifier,8,payload),now)
            c.tick(now)
            if c.phase=='TELEOP':
                break
        self.assertEqual(c.phase,'TELEOP')
        self.assertEqual(c.command[1],0.)

    def test_close_holds_slave_without_disabling_or_commanding_master(self):
        c=self.make();c.owned=True;c.command=np.zeros(6)
        c.close()
        c.sockets[0].send.assert_not_called()
        ids=[i for i,_,_ in frames(c.sockets[1])]
        self.assertNotIn(0x471,ids)
        self.assertIn(0x151,ids)
        self.assertIn(0x155,ids)

    def make(self):
        with patch('piper_x_native_teleop.time.monotonic',return_value=10.):
            c = NativeTeleop([Mock(),Mock()],CFG)
            c.sample_since = 9.
            return c

    def test_master_never_configured_as_follower_or_mit(self):
        c=self.make();c.setup()
        master=frames(c.sockets[0])
        self.assertEqual([(i,d) for i,_,d in master],[(0x470,bytes([0xfa,0,0,0,0,0,0,0]))])
        self.assertIn((0x470,8,bytes([0xfc,0,0,0,0,0,0,0])),frames(c.sockets[1]))

    def test_outside_fifteen_degrees_never_enables_slave(self):
        c=self.make();state(c,10.01,math.radians(16))
        c.tick(10.01)
        self.assertEqual(c.phase,'WAIT_ALIGN')
        for sock in c.sockets:sock.send.assert_not_called()

    def test_only_slave_enabled_and_slowly_aligned(self):
        c=self.make();state(c,10.01,math.radians(9));c.tick(10.01)
        self.assertEqual(c.phase,'ENABLING')
        state(c,10.02,math.radians(9));c.tick(10.02)
        state(c,10.03,math.radians(9),enabled=True);c.tick(10.03)
        self.assertEqual(c.phase,'ALIGNING')
        state(c,10.034,math.radians(9),enabled=True);c.tick(10.034)
        self.assertAlmostEqual(c.command[0],math.radians(.3))
        c.sockets[0].send.assert_not_called()
        self.assertFalse(any(i in range(0x15a,0x160) for i,_,_ in frames(c.sockets[1])))

    def test_enable_confirmation_precedes_motion_mode(self):
        c=self.make();state(c,10.01,.01);c.tick(10.01)
        self.assertFalse(any(i == 0x151 for i,_,_ in frames(c.sockets[1])))
        c.sockets[1].reset_mock()
        state(c,10.02,.01);c.tick(10.02)
        self.assertEqual(frames(c.sockets[1]),
                         [(0x471,8,bytes([7,2,0,0,0,0,0,0]))])
        c.sockets[1].reset_mock()
        state(c,10.03,.01);c.tick(10.03)
        c.sockets[1].send.assert_not_called()
        state(c,10.04,.01,enabled=True);c.tick(10.04)
        self.assertEqual([i for i,_,_ in frames(c.sockets[1])],
                         [0x155,0x156,0x157,0x151])
        c.sockets[0].send.assert_not_called()

    def test_stale_enable_feedback_still_faults_without_more_commands(self):
        c=self.make();state(c,10.01,.01);c.tick(10.01)
        c.sockets[1].reset_mock()
        c.last_tick=10.216
        c.tick(10.22)
        self.assertEqual(c.phase, 'ENABLING')
        c.sockets[1].send.assert_not_called()
        c.last_tick=10.516
        with self.assertRaisesRegex(RuntimeError, 'stale/faulted during ENABLING'):
            c.tick(10.52)
        c.sockets[1].send.assert_not_called()

    def test_normal_teleop_uses_500ms_timeout(self):
        c=self.make();state(c,10.01,.01,enabled=True)
        c.phase='TELEOP';c.last_tick=10.216
        c.tick(10.22)
        c.sockets[1].send.assert_not_called()
        c.last_tick=10.516
        with self.assertRaisesRegex(RuntimeError, 'limit_ms=500'):
            c.tick(10.52)
        c.sockets[1].send.assert_not_called()

    def test_alignment_gap_pauses_then_recovers_on_fresh_state(self):
        c=self.make();state(c,10.01,.01,enabled=True)
        c.phase='ALIGNING';c.command=np.zeros(6);c.complete_since=10.
        c.align_target=np.array([.1,0,0,0,0,0])
        c.last_tick=10.216;c.tick(10.22)
        c.sockets[1].send.assert_not_called()
        self.assertIsNone(c.complete_since)
        state(c,10.23,.01,enabled=True);c.tick(10.23)
        self.assertTrue(c.sockets[1].send.called)

    def test_samples_before_enabling(self):
        c=self.make();c.sample_since=None
        for now in (10.01,10.2,10.4):
            state(c,now,.02);c.tick(now)
            self.assertEqual(c.phase,'WAIT_ALIGN')
            c.sockets[1].send.assert_not_called()
        state(c,10.52,.02);c.tick(10.52)
        self.assertEqual(c.phase,'ENABLING')

    def test_alignment_fixed_target_then_exact_payload_passthrough(self):
        c=self.make();state(c,10.01,math.radians(2));c.tick(10.01)
        state(c,10.02,math.radians(2),enabled=True);c.tick(10.02)
        sampled=c.align_target.copy()
        # Move master during the ramp: alignment target must remain sampled.
        for n in range(12):
            now=10.04+n*.021
            state(c,now,math.radians(3),slave=float(c.command[0]),enabled=True)
            c.tick(now)
            np.testing.assert_array_equal(c.align_target,sampled)
            if c.phase=='TELEOP':break
        self.assertEqual(c.phase,'TELEOP')
        c.sockets[1].reset_mock()
        state(c,now+.004,math.radians(6),slave=float(c.command[0]),enabled=True)
        c.tick(now+.004)
        self.assertEqual(frames(c.sockets[1]),
                         [(i,8,d) for i,d in joint_frames([math.radians(6),0,0,0,0,0])])
        self.assertAlmostEqual(c.command[0],math.radians(6),places=5)
        c.sockets[0].send.assert_not_called()

    def test_mode_matches_wireless_config_without_installation_write(self):
        c=self.make();state(c,10.01,.01);c.tick(10.01)
        state(c,10.02,.01,enabled=True);c.tick(10.02)
        modes=[d for i,_,d in frames(c.sockets[1]) if i==0x151]
        self.assertEqual(modes,[bytes([1,1,100,0xad,0,0,0,0])])

    def test_live_handover_skips_sample_and_ramp_but_waits_for_mode(self):
        c=self.make();c.phase='WAIT_ROLE';c.skip_alignment=True
        state(c,10.01,.01,enabled=True);c.tick(10.01)
        self.assertEqual(c.phase,'ENABLING')
        state(c,10.02,.01,enabled=True);c.tick(10.02)
        self.assertEqual(c.phase,'CONFIRM_ROLE')
        state(c,10.03,.01,enabled=True);c.tick(10.03)
        self.assertEqual(c.phase,'TELEOP')

    def test_gripper_payload_forwarded_exactly_only_after_teleop(self):
        c=self.make();payload=bytes([0,1,2,3,0,100,1,0])
        c.receive(0,struct.pack('=IB3x8s',0x159,8,payload),10.)
        state(c,10.01,.01,enabled=True)
        c.sockets[1].send.assert_not_called()
        c.phase='TELEOP';c.tick(10.01)
        self.assertEqual(frames(c.sockets[1])[-1],(0x159,8,payload))
        c.sockets[0].send.assert_not_called()

    def test_return_mismatch_waits_and_resumes_instead_of_fault_hold(self):
        c=self.make();c.phase='WAIT_ROLE';c.skip_alignment=True;c.recover_handover=True
        state(c,10.01,math.radians(20),enabled=True);c.tick(10.01)
        self.assertEqual(c.phase,'WAIT_ALIGN')
        self.assertFalse(c.skip_alignment)
        self.assertFalse(any(i==0x471 for i,_,_ in frames(c.sockets[1])))
        for now in (10.02,10.2,10.53):
            state(c,now,math.radians(2),enabled=True);c.tick(now)
        self.assertEqual(c.phase,'ENABLING')
        state(c,10.54,math.radians(2),enabled=True);c.tick(10.54)
        self.assertEqual(c.phase,'ALIGNING')

    def test_reverse_mismatch_keeps_guard_and_reports_joint_errors(self):
        c=self.make();c.phase='WAIT_ROLE';c.skip_alignment=True
        state(c,10.01,math.radians(20),enabled=True)
        with self.assertRaisesRegex(RuntimeError,'joint differences'):
            c.tick(10.01)
        c.sockets[1].send.assert_not_called()

    def test_stationary_native_target_is_not_reinterpreted_as_feedback(self):
        c=self.make();state(c,10.01,.02,enabled=True)
        c.phase='TELEOP';c.command=np.zeros(6);c.transition=10.
        saved=c.target.copy()
        # Refresh only slave telemetry; the last native target remains unchanged.
        for i,(stamp,data) in list(c.states[1].frames.items()):c.states[1].frames[i]=(11.,data)
        c.last_tick=10.996;c.tick(11.)
        np.testing.assert_array_equal(c.target,saved)
        c.sockets[0].send.assert_not_called()


if __name__=='__main__':unittest.main()

class DiagnosticTests(unittest.TestCase):
    def test_native_diagnostic_uses_actual_required_ids_and_timeout(self):
        from piper_x_haptics import State
        s = State()
        s.frames[0x261] = (9.85, bytes(8))
        s.frames[0x262] = (9.7, bytes(8))
        result = s.diagnostic(10., required=[0x261, 0x262, 0x263], max_age=.2)
        self.assertIn("stale=['0x262', '0x263']", result)
        self.assertIn('0x261:150.0', result)
        self.assertIn('0x262:300.0', result)
        self.assertIn('0x263:missing', result)
        self.assertIn('limit_ms=200', result)
