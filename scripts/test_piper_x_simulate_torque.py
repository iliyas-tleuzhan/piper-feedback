"""Continuous reflection integration without ROS or CAN hardware."""
import unittest
from unittest.mock import Mock, patch
import numpy as np
from piper_x_simulate_torque import TorqueTeleop
from piper_x_native_teleop import NativeTeleop
from test_piper_x_auto import fixture, update, packets
from piper_x_haptic_core import joint_frames


class TorqueTests(unittest.TestCase):
    def test_damping_is_feedforward_torque_not_firmware_velocity_gain(self):
        from piper_x_haptic_core import PIPER_X_TORQUE_C, uint
        c=self.make()
        velocity=np.array([.8,-.4,.3,-.2,.1,-.1])
        gravity=np.array([0.,2.,-4.,-.3,.05,0.])
        reflected=np.array([.1,0.,0.,0.,.02,0.])
        c.gravity.evaluate.return_value=(gravity,0.)
        c.compliant(0,np.zeros(6),velocity,reflected)
        expected=gravity+reflected-np.array(c.cfg['master_damping'])*velocity
        for joint,(_,_,payload) in enumerate(packets(c.sockets[0])):
            kd=(payload[5]<<4)|(payload[6]>>4)
            tau=((payload[6]&15)<<8)|payload[7]
            self.assertEqual(kd,uint(0.,-5.,5.,12))
            self.assertEqual(tau,uint(expected[joint]/PIPER_X_TORQUE_C[joint],-16.,16.,12))
        np.testing.assert_allclose(c.master_expected_torque,expected)

    def test_diagnostic_separates_base_torque_from_contact_reflection(self):
        c=self.make();update(c,10.)
        c.last_next=10.;c.last_contact=9.;c.contact=True
        c.next_tau=np.ones(6)
        c.tick(10.)
        report=c.feedback_diagnostic(10.,c.feedback.value)
        self.assertIn('contact age_ms=1000.0',report)
        self.assertIn('reflected Nm=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',report)
        self.assertIn('can1 gravity Nm=',report)
        self.assertIn('NEXT filtered residual Nm=',report)

    def test_startup_retains_enabled_joints_and_only_enables_disabled_joints(self):
        for enabled in (True,False):
            c=fixture('STARTUP');c.__class__=TorqueTeleop;c.owned=False
            update(c,10.,enabled=enabled)
            c.tick(10.)
            self.assertEqual(c.mode,'PREPARE')
            update(c,10.01,enabled=enabled,slave_mode=1)
            c.tick(10.01)
            if not enabled:
                update(c,10.02,enabled=True,slave_mode=1)
                c.tick(10.02)
            self.assertEqual(c.mode,'WAIT_ALIGN')
            for sock in c.sockets:
                enables=[d[1] for i,_,d in packets(sock) if i==0x471]
                self.assertEqual(enables,[] if enabled else [2]*6)
            self.assertFalse(any(i in range(0x15a,0x160) for i,_,_ in packets(c.sockets[1])))
            self.assertIn(0x155,[i for i,_,_ in packets(c.sockets[1])])

    def test_master_outside_pose_limits_does_not_stop_startup_or_following(self):
        for startup in (True, False):
            c=fixture('ENABLING') if startup else self.make()
            c.__class__=TorqueTeleop
            update(c,10.,slave_mode=1)
            q=np.array([3.1,0.,0.,0.,0.,0.])
            for identifier,payload in joint_frames(q):
                c.states[0].frames[identifier+0x150]=(10.,payload)
            c.tick(10.)
            self.assertEqual(c.mode,'WAIT_ALIGN' if startup else 'TELEOP')
            if not startup:
                self.assertLessEqual(c.following.target[0],c.following.limits[0,1])

    def test_no_contact_or_stale_contact_clears_reflection_keeps_following(self):
        for contact,stamp in ((False,10.),(True,9.)):
            c=self.make();update(c,10.,master=.01)
            c.contact=contact;c.last_contact=stamp;c.last_next=10.
            c.next_tau=np.full(6,10.);c.feedback.value[:]=.1
            c.tick(10.)
            np.testing.assert_array_equal(c.feedback.value,np.zeros(6))
            self.assertEqual(c.mode,'TELEOP')
            self.assertEqual([i for i,_,_ in packets(c.sockets[1])],[0x155,0x156,0x157])

    def test_gripper_follows_measured_opening_not_old_closed_command(self):
        c=self.make();update(c,10.)
        opening=(30000).to_bytes(4,'big')
        c.states[0].frames[0x2a8]=(10.,opening+bytes(4))
        c.states[0].frames[0x159]=(10.,bytes(8))
        c.tick(10.)
        gripper=[d for i,_,d in packets(c.sockets[1]) if i==0x159]
        self.assertEqual(gripper,[opening+(200).to_bytes(2,'big')+bytes([1,0])])

    def test_measured_leader_aligns_then_drives_follower_with_zero_offset(self):
        c=fixture('WAIT_ALIGN');c.__class__=TorqueTeleop
        for step in range(200):
            now=10.+step*.004
            update(c,now,slave_mode=6 if c.mode in ('WAIT_ALIGN','ENABLING') else 1)
            for identifier,payload in joint_frames([0.,-.02,0.,0.,0.,0.]):
                c.states[0].frames[identifier+0x150]=(now,payload)
            c.tick(now)
            if c.mode=='TELEOP':
                break
        self.assertEqual(c.mode,'TELEOP')
        self.assertEqual([d for i,_,d in packets(c.sockets[0]) if i==0x159],[bytes(8)])
        c.sockets[1].reset_mock()
        now+=.004
        update(c,now)
        for identifier,payload in joint_frames([.05,-.02,0.,0.,0.,0.]):
            c.states[0].frames[identifier+0x150]=(now,payload)
        c.tick(now)
        self.assertEqual([(i,d) for i,_,d in packets(c.sockets[1])],
                         joint_frames([.05,0.,0.,0.,0.,0.]))

    def test_launcher_defaults_to_software_leader_with_native_opt_in(self):
        from run_piper_x_auto import parse_args
        self.assertFalse(parse_args([]).native)
        self.assertTrue(parse_args(['--native']).native)
        self.assertFalse(parse_args(['--feedback']).native)
        self.assertFalse(parse_args(['--experimental-torque-master']).native)

    def test_master_startup_above_old_speed_limit_does_not_fault(self):
        c=fixture('ENABLING');c.__class__=TorqueTeleop
        update(c,10.,slave_mode=1)
        c.states[0].motor=Mock(return_value=(np.array([2.,0,0,0,0,0]),np.zeros(6)))
        c.tick(10.)
        self.assertEqual(c.mode,'WAIT_ALIGN')

    def test_master_motion_above_old_speed_limit_keeps_following(self):
        c=self.make();update(c,10.,master=.01)
        c.states[0].motor=Mock(return_value=(np.array([2.,0,0,0,0,0]),np.zeros(6)))
        c.tick(10.)
        self.assertEqual(c.mode,'TELEOP')
        self.assertEqual([(i,d) for i,_,d in packets(c.sockets[1])],joint_frames([.01,.01,0.,.01,.01,.01]))

    def make(self):
        c=fixture('TELEOP');c.__class__=TorqueTeleop
        c.following=NativeTeleop(c.sockets,c.cfg)
        c.following.states[1]=c.states[1]
        c.following.phase='TELEOP';c.following.started=9.;c.following.transition=10.
        c.following.last_tick=9.996
        c.guard_pipe=Mock()
        return c

    def test_direct_follow_and_reflected_torque_happen_in_same_tick(self):
        c=self.make();update(c,10.,master=.01,slave=.0)
        c.last_next=10.;c.next_tau=np.array([1,2,2,1,1,1.])
        c.last_contact=10.;c.contact=True
        c.tick(10.)
        self.assertEqual(c.mode,'TELEOP')
        self.assertTrue(np.any(c.feedback.value))
        self.assertEqual([(i,d) for i,_,d in packets(c.sockets[1])],joint_frames([.01,.01,0.,.01,.01,.01]))
        self.assertEqual([i for i,_,d in packets(c.sockets[0])],list(range(0x15a,0x160)))
        self.assertFalse(any(i==0x470 for sock in c.sockets for i,_,d in packets(sock)))

    def test_stale_prediction_removes_feedback_but_keeps_following(self):
        c=self.make();update(c,10.,master=.01)
        c.feedback.value[:]=.1;c.last_next=9.
        c.tick(10.)
        np.testing.assert_array_equal(c.feedback.value,np.zeros(6))
        self.assertEqual(c.mode,'TELEOP')

    def test_native_enable_phase_does_not_reenter_legacy_startup(self):
        c=self.make();c.mode='ENABLING';c.following.phase='ENABLING'
        c.following.align_target=np.zeros(6);c.following.transition=10.
        c.following.command=np.zeros(6)
        update(c,10.,slave_mode=6)
        c.tick(10.)
        self.assertEqual(c.mode,'ALIGNING')
        self.assertFalse(any(i==0x470 for sock in c.sockets for i,_,d in packets(sock)))

    def test_stop_requests_position_holds_not_disable(self):
        c=self.make();c.poses_for_hold=Mock(return_value=[[0]*6,[0]*6])
        c.stop('test')
        for sock in c.sockets:
            ids=[i for i,_,d in packets(sock)]
            self.assertNotIn(0x471,ids)
            self.assertIn(0x151,ids)
            self.assertEqual(ids.count(0x155),2)


if __name__=='__main__':unittest.main()
