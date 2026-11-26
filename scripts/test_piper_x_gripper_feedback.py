import unittest
from unittest.mock import Mock
from copy import deepcopy
from pathlib import Path
import yaml
import numpy as np
from piper_x_gripper_feedback import resistance, evaluate, validate
from piper_x_haptic_core import Feedback
from piper_x_simulate_torque import TorqueTeleop
from test_piper_x_auto import fixture, packets

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/piper_x_simulate_torque.yaml').read_text())
G=CFG['gripper_feedback']

def frame(opening,effort=0,status=0x40):
    return opening.to_bytes(4,'big',signed=True)+effort.to_bytes(2,'big',signed=True)+bytes([status,0])

class GripperTests(unittest.TestCase):
    def test_small_contact_and_hysteresis(self):
        command, reason = evaluate(frame(19000), frame(20000,80), G)
        self.assertEqual(reason, 'resisting')
        self.assertEqual(int.from_bytes(command[4:6], 'big'), 60)
        self.assertEqual(evaluate(frame(19700), frame(20000,80), G)[0], bytes(8))
        self.assertNotEqual(evaluate(frame(19700), frame(20000,80), G, True)[0], bytes(8))
        self.assertEqual(evaluate(frame(19800), frame(20000,80), G, True)[0], bytes(8))
        self.assertEqual(evaluate(frame(19700), frame(20000,0), G, True)[0], bytes(8))

    def test_release_bypasses_rate_limit_and_ignores_next_contact(self):
        for release in ('stale', 'phase', 'fault', 'disabled', 'load'):
            c=fixture('TELEOP');c.__class__=TorqueTeleop;c.cfg=deepcopy(CFG)
            c.states[0].frames[0x2a8]=(9.91,frame(10000))
            c.states[1].frames[0x2a8]=(9.91,frame(20000,80))
            c.update_gripper_feedback(10.)
            self.assertFalse(c.contact) # Gripper works independently of six-joint NEXT.
            self.assertEqual(packets(c.sockets[0])[-1][2][6],1)
            if release == 'phase':c.mode='ALIGNING'
            if release == 'fault':c.states[1].frames[0x2a8]=(10.,frame(20000,80,0x41))
            if release == 'disabled':c.cfg['gripper_feedback']['enabled']=False
            if release == 'load':c.states[1].frames[0x2a8]=(10.,frame(20000,0))
            c.update_gripper_feedback(10.011)
            self.assertEqual(packets(c.sockets[0])[-1][2],bytes(8),release)
            self.assertEqual(c.gripper_feedback_effort,0)

    def test_diagnostic_explains_missing_and_present_readings(self):
        c=fixture('TELEOP');c.__class__=TorqueTeleop;c.cfg=deepcopy(CFG)
        c.update_gripper_feedback(10.)
        self.assertIn('gate=missing or stale telemetry',c.gripper_diagnostic(10.))
        c.states[1].frames[0x2a8]=(10.,frame(20000,80))
        self.assertIn('can_slave opening_mm=20.000 effort_raw=80 status=0x40',c.gripper_diagnostic(10.))

    def test_invalid_hysteresis_rejected(self):
        for value in (-1, G['gap_um']+1, float('nan')):
            with self.assertRaises(ValueError):validate(dict(G,release_gap_um=value))

    def test_contact_resistance_both_directions_and_cap(self):
        validate(G)
        for master,slave in ((10000,20000),(30000,20000)):
            cmd=resistance(frame(master),frame(slave,500),G)
            self.assertEqual(int.from_bytes(cmd[:4],'big'),slave)
            self.assertEqual(int.from_bytes(cmd[4:6],'big'),480)
            self.assertEqual(cmd[6:],bytes([1,0]))
            for effort in (-10000,10000):
                capped=resistance(frame(master),frame(slave,effort),G)
                self.assertEqual(int.from_bytes(capped[4:6],'big'),5000)

    def test_sdk_effort_ceiling_is_enforced(self):
        with self.assertRaises(ValueError):
            validate(dict(G,max_effort_nm=5.001))

    def test_logged_load_is_no_longer_clipped_at_100(self):
        cmd=resistance(frame(33500),frame(20600,145),G)
        self.assertEqual(int.from_bytes(cmd[4:6],'big'),125)

    def test_no_load_no_gap_fault_or_bad_opening_releases(self):
        for m,s in ((frame(10000),frame(20000,0)),(frame(20000),frame(20000,500)),
                    (frame(10000),frame(20000,500,1)),(frame(-1000),frame(20000,500))):
            self.assertEqual(resistance(m,s,G),bytes(8))

    def test_stale_feedback_releases_master(self):
        c=fixture('TELEOP');c.__class__=TorqueTeleop;c.cfg=deepcopy(CFG)
        c.states[0].frames[0x2a8]=(10.,frame(10000))
        c.states[1].frames[0x2a8]=(10.,frame(20000,500))
        c.update_gripper_feedback(10.)
        self.assertEqual(packets(c.sockets[0])[-1][2][6],1)
        c.update_gripper_feedback(10.2)
        self.assertEqual(packets(c.sockets[0])[-1][2],bytes(8))

    def test_joint_mapping_opposes_estimate_but_retains_caps(self):
        f=Feedback(CFG['feedback'])
        residual=np.array([.02,-.02,.02,-.02,.02,-.02])
        np.testing.assert_allclose(f.update(residual,.1,True,True,True,np.ones(6)),-residual)
        for _ in range(150):out=f.update(np.full(6,16.7),.1,True,True,True,np.ones(6))
        np.testing.assert_allclose(out,-np.array(CFG['feedback']['limits']))
        np.testing.assert_array_equal(f.update(residual,.1,False,True,True,np.ones(6)),np.zeros(6))

    def test_increased_reflection_still_faults_before_excessive_total_is_sent(self):
        c=fixture('TELEOP');c.__class__=TorqueTeleop;c.cfg=deepcopy(CFG)
        c.gravity.evaluate.return_value=(np.array([-1.,0.,0.,0.,0.,0.]),0.)
        guard_cfg=deepcopy(CFG['feedback'])
        guard_cfg['limits']=CFG['master_total_torque_limit']
        f=Feedback(guard_cfg)
        with self.assertRaisesRegex(RuntimeError,'torque exceeds limit'):
            for _ in range(100):
                reflected=f.update(np.array([16.7,0.,0.,0.,0.,0.]),.1,True,True,True,np.ones(6))
                c.sockets[0].reset_mock()
                c.compliant(0,np.zeros(6),np.zeros(6),reflected)
        c.sockets[0].send.assert_not_called()

if __name__=='__main__':unittest.main()
