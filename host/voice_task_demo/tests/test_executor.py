from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from executor import replay,valid_detection
from task_graph import legacy_command,parse

def frame(target):return [{'image':'synthetic_fixture','detections':[{'class':target,'confidence':.9,'bbox':[1,2,30,40]}]}]

class ExecutorTests(unittest.TestCase):
    def test_primary_does_not_run_fallback(self):
        result=replay({'i':'search','t':'tape','f':'screwdriver'},{'primary':'a','fallback':'b'},
                      {'a':frame('tape'),'b':frame('screwdriver')})
        self.assertEqual(result['trace'],['primary','primary_verify','done'])
        self.assertNotIn('fallback',result['evidence']);self.assertFalse(result['motion_authorized'])

    def test_fallback_and_return_need_actual_matching_class(self):
        result=replay({'i':'search','t':'tape','f':'screwdriver','r':True},
                      {'primary':'a','fallback':'b'},{'a':frame('ad_calcium_milk'),'b':frame('screwdriver')},True)
        self.assertEqual(result['trace'],['primary','fallback','fallback_verify','return_home','done'])
        self.assertFalse(result['physical_task_completed'])

    def test_missing_home_ack_fails(self):
        result=replay({'i':'search','t':'tape','r':True},{'primary':'a'},{'a':frame('tape')})
        self.assertEqual(result['status'],'failure');self.assertIn('return_home',result['trace'])

    def test_missing_detection_cannot_succeed(self):
        result=replay({'i':'search','t':'tape'},{'primary':'a'},{'a':frame('screwdriver')},True)
        self.assertEqual(result['status'],'failure')

    def test_unknown_confirm_and_stop_never_search(self):
        for intent,status in (('unknown','needs_clarification'),('confirm','confirmation_only'),('stop','stopped')):
            result=replay({'i':intent},{'primary':'a'},{'a':frame('tape')},True)
            self.assertEqual(result['status'],status);self.assertEqual(result['evidence'],{})

    def test_interrupt_stops_task(self):
        result=replay({'i':'search','t':'tape'},{'primary':'a'},{'a':frame('tape')},True,True)
        self.assertEqual(result['status'],'stopped');self.assertEqual(result['evidence'],{})

    def test_bad_detections_rejected(self):
        for changes in ({'confidence':float('nan')},{'confidence':.1},{'bbox':[0,0,0,1]},{'bbox':[0,0,float('inf'),1]}):
            det={'class':'tape','confidence':.9,'bbox':[1,2,30,40],**changes}
            self.assertFalse(valid_detection(det,'tape'))

    def test_bad_task_and_flattening_rejected(self):
        for raw in ('{"i":"search","t":"chair"}','{"i":"stop","i":"search"}'):
            with self.assertRaises(ValueError):parse(raw)
        with self.assertRaises(ValueError):legacy_command({'i':'search','t':'tape','r':True})

if __name__=='__main__':unittest.main()
