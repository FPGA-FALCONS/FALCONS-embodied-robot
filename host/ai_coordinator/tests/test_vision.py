"""Known decoded geometry and class policy for the deployed original v17 graph."""
import unittest
import numpy as np
from ai_coordinator.worker import decode_outputs
from ai_coordinator.schema import detection


class VisionContractTests(unittest.TestCase):
    def test_twenty_five_percent_boundary_applies_to_inference_and_frame_admission(self):
        output=np.zeros((1,8,3),dtype=np.float32)
        output[0,:4]=[[100,200,300],[100,100,100],[20,20,20],[20,20,20]]
        output[0,4]=[.249,.250,.251]
        found=decode_outputs([output],1,0,0,640,640)
        self.assertEqual([d['bbox'][0] for d in found],[290,190]) # score order after NMS
        for score,accepted in ((.249,False),(.250,True),(.251,True)):
            self.assertEqual(detection({'class':'tape','confidence':score,'bbox':[1,1,20,20]},640,640),accepted)

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


class TemporalVisionTests(unittest.TestCase):
    def test_new_frames_confirmation_dropout_and_stream_restart(self):
        import cv2, tempfile
        from pathlib import Path
        from ai_coordinator.worker import Vision
        from ai_coordinator.temporal_confirm import TemporalConfirmation
        class Engine:
            output=np.array([[[100],[240],[20],[20],[.3],[0],[0],[0]]],dtype=np.float32)
            def inference(self,**kwargs):return [self.output]
        model=Vision.__new__(Vision)
        model.size=640;model.contract='original';model.provider='rknn';model.rec=Engine()
        model.stream_id=None;model.gate=TemporalConfirmation()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'frame.jpg';cv2.imwrite(str(path),np.zeros((720,1280,3),dtype=np.uint8))
            def frame(n,stream=1):
                return model.run(dict(image=str(path),frame_id=n,captured_at_s=10+n*.1,stream_id=stream))
            a=frame(1);self.assertEqual(len(a['raw_detections']),1);self.assertEqual(a['detections'],[])
            self.assertEqual(frame(2)['detections'],[])
            confirmed=frame(3)['detections'];self.assertEqual(confirmed[0]['class'],'tape')
            self.assertEqual(confirmed[0]['confirmation_hits'],3)
            model.rec.output[0,4,0]=0
            self.assertEqual(frame(4)['detections'],[])  # 不填充历史框
            model.rec.output[0,4,0]=.3
            self.assertEqual(len(frame(5)['detections']),1)  # 0.6秒内恢复
            self.assertEqual(frame(1,stream=2)['detections'],[])  # 新采集不继承轨迹
            with self.assertRaises(ValueError):frame(1,stream=2)  # 同帧不能累计
            raw=model.run(dict(image=str(path)));self.assertTrue(raw['single_image_raw_only'])
            self.assertEqual(len(raw['detections']),1)
