"""Offline checks of the design contract and existing geometry, never hardware."""
from pathlib import Path
import hashlib
import io
import json
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "host"))
from ai_coordinator.ground_target import project_ground_pixel, base_to_map
from ai_coordinator.navigation_goal import cell, parking_proposal

BASELINE = "437b48d11365fd9943988fa33efa72856560578c"
SOURCES = [
    "rtl/top/falcons_top.v", "rtl/ai/ai_rc_control.v", "rtl/ai/ai_display_link.v",
    "rtl/cam/cam_overlay.v", "rtl/ui/nav_map_view.v", "rtl/ai/ai_uart_tx.v",
    "host/ai_coordinator/ground_target.py", "host/ai_coordinator/navigation_goal.py",
    "host/ai_coordinator/patrol.py", "host/ai_coordinator/gateway.py",
    "host/ai_coordinator/frames.py", "host/ai_coordinator/schema.py",
    "vendor/verilog-reusable-modules/modules/auto_navigation/rtl/nav_map.v",
    "vendor/verilog-reusable-modules/modules/auto_navigation/rtl/nav_top.v",
    "vendor/verilog-reusable-modules/modules/auto_navigation/rtl/nav_map_arb.v",
    "vendor/verilog-reusable-modules/modules/auto_navigation/rtl/nav_planner.v",
]


class ExistingGeometryChecks(unittest.TestCase):
    # Deliberately fictional calibration, not a measured camera configuration.
    K = [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
    T = [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, .5], [0, 0, 0, 1]]

    def project(self, u=320, v=440, **kwargs):
        args = dict(pixel_on_ground_verified=True, max_range_m=3)
        args.update(kwargs)
        return project_ground_pixel(u, v, self.K, self.T, **args)

    def test_ground_intersection_has_expected_metric_position(self):
        value = self.project()
        for got, expected in zip(value["point_base_m"], [1.25, 0, 0]):
            self.assertAlmostEqual(got, expected)
        self.assertFalse(value["motion_authorized"])

    def test_image_center_does_not_supply_ground_distance(self):
        with self.assertRaises(ValueError):
            self.project(v=240)

    def test_unverified_contact_pixel_is_rejected(self):
        with self.assertRaises(ValueError):
            self.project(pixel_on_ground_verified=False)

    def test_near_horizon_and_upward_rays_are_rejected(self):
        for v in (241, 200):
            with self.subTest(v=v), self.assertRaises(ValueError):
                self.project(v=v)

    def test_outside_calibrated_range_is_rejected(self):
        with self.assertRaises(ValueError):
            self.project(max_range_m=1)

    def test_pose_at_capture_rotates_point_in_navigation_frame(self):
        value = base_to_map(self.project()["point_base_m"], [2, 3, math.pi / 2])
        for got, expected in zip(value, [2, 4.25, 0]):
            self.assertAlmostEqual(got, expected)

    def test_grid_edges_are_half_open_without_clamping(self):
        self.assertEqual([cell(x) for x in (-6400, -.01, 0, 6399.9)], [0, 127, 128, 255])
        for value in (-6400.1, 6400, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                cell(value)

    def test_localization_reset_invalidates_old_spatial_proposal(self):
        with self.assertRaises(ValueError):
            parking_proposal([1, 0, 0], [0, 0, 0], .3,
                frame_id="synthetic:1", localization_epoch="old", current_epoch="new",
                captured_at_s=10, pose_at_s=10, now_s=10.1)

    def test_parking_proposal_is_not_an_obstacle_or_motion_authorization(self):
        result = parking_proposal([1, 0, 0], [0, 0, 0], .3,
            frame_id="synthetic:1", localization_epoch="e1", current_epoch="e1",
            captured_at_s=10, pose_at_s=10, now_s=10.1)
        self.assertAlmostEqual(result["goal_x_mm"], 700)
        self.assertFalse(result["goal_valid"])
        self.assertFalse(result["motion_authorized"])


def check_contract():
    draft = json.loads((HERE / "接口草案.json").read_text(encoding="utf-8"))
    assert draft["document_kind"] == "architecture_draft_not_wire_protocol"
    assert draft["baseline_commit"] == BASELINE
    assert draft["implementation_status"] == "design_offline_planning_and_standalone_rtl"
    assert draft["production_navigation_integrated"] is False
    assert draft["hardware_deployed"] is False
    assert draft["motion_authorized"] is False and draft["hardware_tested"] is False
    assert draft["requirements"] == [f"R{i}" for i in range(1, 9)]
    assert draft["ui"]["existing_modes_preserved"] == [0, 1, 2]
    assert draft["ui"]["view_switch_changes_mission"] is False
    assert draft["navigation"]["return_policy"] == "replan_to_saved_home_on_current_fused_map"
    assert draft["navigation"]["outbound_history_required"] is False
    assert draft["navigation"]["production_route_directions"] == 8
    assert draft["obstacles"]["lidar_clear_removes_vision"] is False
    assert draft["object_record_example"]["position_valid"] is False
    assert draft["object_record_example"]["position_mm"] is None
    assert draft["physical_action_opcode_assignments"] is None
    assert draft["limits_proposed_not_implemented"]["max_speed_mm_s"] is None
    assert {m["name"] for m in draft["logical_messages_proposed"]} == {
        "capture_pose", "inspection_control", "visual_layer_transaction",
        "inspection_status", "inspection_ui_snapshot"}
    doc = (HERE / "README.md").read_text(encoding="utf-8")
    for req in draft["requirements"]:
        assert f"| {req} |" in doc, f"unmapped requirement {req}"
    return draft


def main():
    check_contract()
    output = io.StringIO()
    result = unittest.TextTestRunner(stream=output, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(ExistingGeometryChecks))
    report = {
        "date": "2026-10-26", "scope": "design_contract_and_existing_synthetic_geometry_only",
        "baseline_commit": BASELINE, "status": "PASS" if result.wasSuccessful() else "FAIL",
        "contract_check": "PASS", "tests_run": result.testsRun,
        "failures": len(result.failures), "errors": len(result.errors),
        "hardware_tested": False, "new_inspection_controller_tested": False,
        "camera_calibration_measured": False, "board_contacted": False,
        "models_loaded": False, "motion_commands_sent": 0,
        "python_version": sys.version.split()[0],
        "source_sha256": {name: hashlib.sha256((REPO / name).read_bytes()).hexdigest() for name in SOURCES},
        "design_sha256": {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                           for name in ("README.md", "接口草案.json", "verify_design.py")},
        "test_log": output.getvalue(),
    }
    (HERE / "核对与验证.json").write_bytes((json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(f"{report['status']}: contract + {result.testsRun} existing geometry checks; hardware_tested=false")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
