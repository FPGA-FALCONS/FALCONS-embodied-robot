import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from inspection_reference.route_cli import plan_snapshot
from inspection_reference.planning import Grid, line_of_sight

HERE = Path(__file__).resolve().parents[1]


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = json.loads((HERE / "examples/costmap.json").read_text())

    def test_return_uses_current_visual_obstacles_and_saved_home(self):
        result = plan_snapshot(self.snapshot, returning=True)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["path"][-1], tuple(self.snapshot["home"]))
        self.assertEqual(result["map_revision"], 2)
        self.assertFalse(result["motion_authorized"])
        grid = Grid(20, 14, frozenset(map(tuple, self.snapshot["vision_cost"])))
        self.assertTrue(all(line_of_sight(grid, a, b) for a, b in zip(result["path"], result["path"][1:])))
        self.assertGreater(len(result["path"]), 2)

    def test_invalid_contract_is_rejected(self):
        for key, value in (("configuration_space_inflated", False), ("revision", True),
                           ("revision", -1), ("cell_size_mm", True),
                           ("localization_epoch", ""), ("schema", "other"),
                           ("vision_cost", [[20, 0]]), ("width", 257)):
            bad = copy.deepcopy(self.snapshot)
            bad[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                plan_snapshot(bad)
        self.snapshot["home_epoch"] = "old"
        with self.assertRaises(ValueError):
            plan_snapshot(self.snapshot, returning=True)
        with self.assertRaises(ValueError):
            plan_snapshot([])

    def test_input_hash_tracks_map_changes_and_eight_direction_option(self):
        before = plan_snapshot(self.snapshot, algorithm="astar8_euclidean")
        self.assertTrue(before["current_fpga_direction_compatible"])
        self.snapshot["vision_cost"].append([10, 8])
        self.assertNotEqual(before["input_sha256"], plan_snapshot(self.snapshot)["input_sha256"])

    def test_cli_writes_explicit_no_path_and_refuses_input_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            source, target = Path(folder)/"map.json", Path(folder)/"route.json"
            self.snapshot["vision_cost"] = [[10, y] for y in range(14)]
            source.write_text(json.dumps(self.snapshot))
            cmd = [sys.executable, str(HERE/"route_cli.py"), str(source), "--output"]
            completed = subprocess.run(cmd+[str(target)], capture_output=True, timeout=20)
            self.assertEqual(completed.returncode, 2, completed.stderr)
            self.assertEqual(json.loads(target.read_text())["status"], "no_path")
            original = source.read_bytes()
            self.assertEqual(subprocess.run(cmd+[str(source)], capture_output=True, timeout=20).returncode, 1)
            self.assertEqual(source.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
