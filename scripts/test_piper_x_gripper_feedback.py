import unittest
from unittest.mock import Mock
from copy import deepcopy
from pathlib import Path
import yaml
import numpy as np
from piper_x_gripper_feedback import resistance, validate
from piper_x_haptic_core import Feedback
from piper_x_simulate_torque import TorqueTeleop
from test_piper_x_auto import fixture, packets

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/piper_x_simulate_torque.yaml').read_text())
G=CFG['gripper_feedback']

def frame(opening,effort=0,status=0x40):
    return opening.to_bytes(4,'big',signed=True)+effort.to_bytes(2,'big',signed=True)+bytes([status,0])

class GripperTests(unittest.TestCase):
    def test_contact_resistance_both_directions_and_cap(self):
        validate(G)
        for master,slave in ((10000,20000),(30000,20000)):
            cmd=resistance(frame(master),frame(slave,500),G)
            self.assertEqual(int.from_bytes(cmd[:4],'big'),slave)
            self.assertEqual(int.from_bytes(cmd[4:6],'big'),100)
            self.assertEqual(cmd[6:],bytes([1,0]))

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
