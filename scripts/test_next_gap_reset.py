"""Run with ROS sourced; no ROS node or hardware is created."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import JointState
from factr2_next.inference.inference_node import InferenceNode, TorqueFilter
from factr2_next.inference.history_buffer import HistoryBuffer


class GapTest(unittest.TestCase):
    def test_gap_clears_history_filter_and_contact(self):
        node = SimpleNamespace(cfg={'sync': {'max_input_gap_seconds': .04}},
                               last_input_stamp=1., loaded=SimpleNamespace(history=50),
                               buffer=HistoryBuffer(50), torque_filter=TorqueFilter({'mode':'ema'}),
                               contact_state=True, pubs={'contact': Mock()})
        for _ in range(49):node.buffer.append(np.zeros(6),np.zeros(6),np.zeros(6))
        node.torque_filter.y=np.ones(6)
        node._extract=lambda msg,key:np.array(msg.position)
        msgs=[]
        for _ in range(4):
            m=JointState();m.header.stamp.sec=2;m.position=[0.]*6;msgs.append(m)
        InferenceNode._callback(node,*msgs)
        self.assertEqual(len(node.buffer.rows),1)
        self.assertFalse(node.contact_state)
        self.assertIsNone(node.torque_filter.y)
        self.assertFalse(node.pubs['contact'].publish.call_args.args[0].data)


if __name__=='__main__':unittest.main()
