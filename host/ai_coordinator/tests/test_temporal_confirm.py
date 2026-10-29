import unittest
from ai_coordinator.temporal_confirm import TemporalConfirmation
def d(c=1,box=(10,10,30,50),score=.8):return dict(cls=c,confidence=score,bbox=list(box))
class TestConfirmation(unittest.TestCase):
 def test_no_startup_exception(self):
  g=TemporalConfirmation();self.assertEqual(g.update([d()],0,0),[]);self.assertEqual(g.update([d()],1,.1),[]);self.assertEqual(len(g.update([d()],2,.2)),1)
 def test_flicker_is_not_accumulated(self):
  g=TemporalConfirmation();self.assertEqual(g.update([d()],0,0),[]);g.update([],1,.1);self.assertEqual(g.update([d()],2,.2),[]);self.assertEqual(g.update([d()],3,.3),[])
 def test_never_fills_missing_frames(self):
  g=TemporalConfirmation(min_hits=1);self.assertEqual(len(g.update([d()],0,0)),1);self.assertEqual(g.update([],1,.1),[])
 def test_class_does_not_inherit_confirmation(self):
  g=TemporalConfirmation(min_hits=2);g.update([d()],0,0);g.update([d()],1,.1);self.assertEqual(g.update([d(c=3)],2,.2),[])
 def test_two_boxes_in_one_frame_are_not_two_hits(self):
  g=TemporalConfirmation(min_hits=2);self.assertEqual(g.update([d(),d()],0,0),[])
 def test_expiry_requires_new_confirmation(self):
  g=TemporalConfirmation(min_hits=2);g.update([d()],0,0);g.update([d()],1,.1);self.assertEqual(g.update([d()],2,1),[])
 def test_duplicate_frame_rejected(self):
  g=TemporalConfirmation();g.update([d()],0,0)
  with self.assertRaises(ValueError):g.update([d()],0,.1)
 def test_time_reversal_rejected(self):
  g=TemporalConfirmation();g.update([],0,1)
  with self.assertRaises(ValueError):g.update([],1,.9)
 def test_far_background_is_separate(self):
  g=TemporalConfirmation(min_hits=2);g.update([d()],0,0);g.update([d()],1,.1);out=g.update([d(),d(box=(200,200,230,280))],2,.2);self.assertEqual(len(out),1);self.assertEqual(out[0]['bbox'],d()['bbox'])
 def test_size_change_resets_tracks(self):
  g=TemporalConfirmation(min_hits=2);g.update([d()],0,0,(720,1280));g.update([d()],1,.1,(720,1280));self.assertEqual(g.update([d()],2,.2,(1080,1920)),[])
 def test_one_frame_mode_preserves_current_detections(self):
  g=TemporalConfirmation(min_hits=1);p=d();out=g.update([p],0,0);self.assertEqual(out[0]['confidence'],p['confidence']);self.assertEqual(out[0]['bbox'],p['bbox']);self.assertNotIn('track_id',p)
if __name__=='__main__':unittest.main()
