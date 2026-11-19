import unittest
import numpy as np
from piper_x_haptic_core import Feedback, ReflectionMotionGate

class ResponseTests(unittest.TestCase):
    def make(self):
        return Feedback(dict(gains=[-1]*6,deadband=[0]*6,limits=[8]*6,slew_nm_per_second=6))

    def test_release_reduces_immediately_even_while_contact_is_true(self):
        f=self.make();f.value[:]=3
        np.testing.assert_allclose(f.update(np.full(6,-.2),.004,True,True,True,np.ones(6)),.2)

    def test_reversal_cannot_keep_old_direction(self):
        f=self.make();f.value[:]=2.6556
        result=f.update(np.full(6,1.1437),.004,True,True,True,np.ones(6))
        np.testing.assert_allclose(result,-.024)

    def test_contact_loss_clears_immediately(self):
        f=self.make();f.value[:]=3
        np.testing.assert_array_equal(f.update(np.ones(6),.004,True,True,False,np.ones(6)),np.zeros(6))

    def test_motion_gate_latches_only_driving_joint_until_release(self):
        gate=ReflectionMotionGate(.05)
        torque=np.array([.3,.3,0,0,0,0])
        velocity=np.array([.1,-.1,0,0,0,0])
        np.testing.assert_allclose(gate.update(torque,velocity,True),[0,.3,0,0,0,0])
        np.testing.assert_allclose(gate.update(torque,np.zeros(6),True),[0,.3,0,0,0,0])
        gate.update(torque,np.zeros(6),False)
        np.testing.assert_allclose(gate.update(torque,np.zeros(6),True),torque)

    def test_motion_gate_recovers_resisting_motion_without_contact_release(self):
        gate=ReflectionMotionGate(.05)
        torque=np.full(6,.3)
        np.testing.assert_array_equal(gate.update(torque,np.full(6,.1),True),np.zeros(6))
        # Rest cannot repeatedly restart a torque-driven arm.
        for _ in range(100):
            np.testing.assert_array_equal(gate.update(torque,np.zeros(6),True),np.zeros(6))
        # Pushing against the reflection restores resistance without requiring
        # the slave to leave the obstacle first.
        np.testing.assert_allclose(gate.update(torque,np.full(6,-.1),True),torque)
        np.testing.assert_allclose(gate.update(torque,np.zeros(6),True),torque)
        np.testing.assert_array_equal(gate.update(torque,np.full(6,.1),True),np.zeros(6))

if __name__=='__main__':unittest.main()
