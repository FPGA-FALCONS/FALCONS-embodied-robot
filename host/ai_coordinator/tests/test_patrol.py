from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.patrol import PatrolMission
from ai_coordinator.ground_target import project_ground_pixel,base_to_map


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.core=PatrolMission(mode='simulation');self.core.start_patrol(['P1','P2'],['screwdriver'],1,1,[.2,.4])
        self.goto=self.core.drain()[0]
    def frame(self,t,seq,boxes=None):
        event={'task_id':self.core.task_id,'goal_id':self.core.goal_id,'sequence':seq,'captured_at_s':t,'width':640,'height':480,
               'detections':boxes if boxes is not None else [{'class':'screwdriver','confidence':.9,'bbox':[100,100,200,200]}]}
        self.core.on_frame(event,t)
    def approach(self):
        self.frame(1.1,0);self.frame(1.2,1);pause=self.core.drain()[0]
        self.assertEqual(self.core.state,'pausing_patrol')
        self.core.on_status({**pause,'kind':'patrol_paused'},1.3);return self.core.drain()[0]
    def test_interrupt_approach_manual_pickup_and_resume_same_waypoint(self):
        approach=self.approach();self.assertNotIn('map_x',approach)
        self.assertFalse(self.core.on_status({**approach,'kind':'approach_arrived'},1.4))
        self.assertTrue(self.core.on_status({**approach,'kind':'approach_arrived','metric_stop_verified':True,'stationary':True,'clearance_m':.3},1.4))
        self.assertEqual(self.core.state,'waiting_pickup')
        result=self.core.confirm_pickup(self.core.task_id,1.5);self.assertFalse(result['physical_pickup_verified'])
        resume=self.core.drain()[0];self.assertEqual(resume['waypoint'],'P1')
        self.core.on_status({**resume,'kind':'waypoint_arrived'},1.6);next_point=self.core.drain()[0]
        self.assertEqual(next_point['waypoint'],'P2')
        self.core.on_status({**next_point,'kind':'waypoint_arrived'},1.7);self.assertEqual(self.core.state,'completed')
    def test_disappearing_target_cancels_local_approach(self):
        self.approach();self.core.tick(2.3);self.assertEqual(self.core.state,'failed')
        self.assertEqual(self.core.drain()[-1]['kind'],'cancel')
    def test_same_class_at_different_location_does_not_satisfy_consensus(self):
        self.frame(1.1,0);self.frame(1.2,1,[{'class':'screwdriver','confidence':.9,'bbox':[400,300,500,400]}])
        self.assertEqual(self.core.state,'patrolling')
    def test_wrong_task_and_unmeasured_stop_do_not_enable_pickup(self):
        action=self.approach()
        for change in ({'task_id':'old'},{'metric_stop_verified':False},{'stationary':False},{'clearance_m':.05}):
            self.assertFalse(self.core.on_status({**action,'kind':'approach_arrived','metric_stop_verified':True,'stationary':True,'clearance_m':.3,**change},1.4))
        with self.assertRaises(ValueError):self.core.confirm_pickup(self.core.task_id,1.5)
    def test_voice_stop_cannot_be_overridden_by_late_arrival(self):
        action=self.approach();self.core.stop(1.4)
        self.assertFalse(self.core.on_status({**action,'kind':'approach_arrived','metric_stop_verified':True,'stationary':True,'clearance_m':.3},1.5))
        self.assertEqual(self.core.state,'stopped')
    def test_live_requires_measured_clearance_policy(self):
        with self.assertRaises(ValueError):PatrolMission(mode='live').start_patrol(['P1'],['screwdriver'],1,1)


class GroundTests(unittest.TestCase):
    K=[[500,0,320],[0,500,240],[0,0,1]]
    T=[[0,0,1,0],[-1,0,0,0],[0,-1,0,.2],[0,0,0,1]]
    def test_known_synthetic_ground_geometry(self):
        p=project_ground_pixel(320,340,self.K,self.T,pixel_on_ground_verified=True,max_range_m=2)['point_base_m']
        self.assertAlmostEqual(p[0],1);self.assertAlmostEqual(p[1],0);self.assertAlmostEqual(p[2],0)
        self.assertEqual(base_to_map(p,[2,3,0]),[3,3,0])
    def test_bbox_bottom_is_not_automatically_a_ground_contact(self):
        with self.assertRaises(ValueError):project_ground_pixel(320,340,self.K,self.T,max_range_m=2)
    def test_horizon_and_invalid_rotation_rejected(self):
        with self.assertRaises(ValueError):project_ground_pixel(320,240,self.K,self.T,pixel_on_ground_verified=True,max_range_m=2)
        bad=[list(r) for r in self.T];bad[0][2]=2
        with self.assertRaises(ValueError):project_ground_pixel(320,340,self.K,bad,pixel_on_ground_verified=True,max_range_m=2)
