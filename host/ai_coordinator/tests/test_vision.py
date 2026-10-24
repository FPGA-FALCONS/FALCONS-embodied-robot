"""Known decoded geometry and class policy for the deployed original v17 graph."""
import unittest
import numpy as np
from ai_coordinator.worker import decode_outputs


class VisionContractTests(unittest.TestCase):
    def test_original_graph_letterbox_best_class_and_nms(self):
        # Two duplicate tape boxes and one screwdriver box. Second class score
        # on a tape box must not create a spurious screwdriver observation.
        output=np.array([[[200,200,400],[240,240,240],[100,100,80],[80,80,80],
                         [.9,.8,.1],[.7,.6,.95],[0,0,0],[0,0,0]]],dtype=np.float32)
        detections=decode_outputs([output],2,0,80,320,240)
        self.assertEqual([d['class'] for d in detections],['tape','screwdriver'])
        self.assertEqual(detections[0]['bbox'],[75,60,125,100])
        self.assertEqual(detections[1]['bbox'],[180,60,220,100])

    def test_wrong_graph_and_logits_rejected(self):
        with self.assertRaises(ValueError):decode_outputs([np.zeros((1,84,10))],1,0,0,640,640)
        x=np.zeros((1,8,1));x[0,4,0]=5
        with self.assertRaises(ValueError):decode_outputs([x],1,0,0,640,640)
