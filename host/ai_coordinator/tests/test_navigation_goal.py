from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.navigation_goal import cell,parking_proposal


class NavigationGoalTests(unittest.TestCase):
    def test_exact_upstream_grid_edges_without_clamping(self):
        self.assertEqual(cell(-6400),0);self.assertEqual(cell(-.01),127)
        self.assertEqual(cell(0),128);self.assertEqual(cell(6399.9),255)
        for value in (-6400.1,6400,float('nan')):
            with self.assertRaises(ValueError):cell(value)

    def proposal(self,**change):
        metadata={'frame_id':'front:session:1','localization_epoch':'epoch1','current_epoch':'epoch1',
                  'captured_at_s':10,'pose_at_s':10,'now_s':10.2,**change}
        return parking_proposal([1,0,0],[0,0,0],.3,**metadata)

    def test_parking_point_is_in_front_of_object_not_object_center(self):
        p=self.proposal();self.assertAlmostEqual(p['goal_x_mm'],700);self.assertEqual(p['goal_y_mm'],0)
        self.assertEqual(p['goal_x_cell'],142);self.assertFalse(p['goal_valid']);self.assertFalse(p['motion_authorized'])

    def test_stale_image_pose_mismatch_and_reset_rejected(self):
        for change in ({'now_s':11.1},{'pose_at_s':9.9},{'current_epoch':'epoch2'}):
            with self.assertRaises(ValueError):self.proposal(**change)
