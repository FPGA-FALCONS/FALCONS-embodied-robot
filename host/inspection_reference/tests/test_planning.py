from fractions import Fraction
from pathlib import Path
import math
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from inspection_reference.planning import (
    Grid, fused_grid, line_of_sight, supercover, plan, length, replan_home, current_fpga_directions)


def hits_closed_square(a, b, cell):
    """Independent exact-rational segment/AABB oracle (not grid traversal)."""
    lower, upper = Fraction(0), Fraction(1)
    for axis in (0, 1):
        lo, hi = Fraction(2 * cell[axis] - 1, 2), Fraction(2 * cell[axis] + 1, 2)
        delta = b[axis] - a[axis]
        if delta == 0:
            if not lo <= a[axis] <= hi:
                return False
        else:
            u, v = (lo - a[axis]) / delta, (hi - a[axis]) / delta
            lower, upper = max(lower, min(u, v)), min(upper, max(u, v))
            if lower > upper:
                return False
    return True


class PlanningTests(unittest.TestCase):
    def test_supercover_matches_independent_collision_geometry(self):
        rng = random.Random(10626)
        for _ in range(600):
            a, b = tuple(rng.randrange(9) for _ in range(2)), tuple(rng.randrange(9) for _ in range(2))
            actual = set(supercover(a, b))
            expected = {(x, y) for x in range(9) for y in range(9) if hits_closed_square(a, b, (x, y))}
            self.assertEqual(actual, expected, (a, b))
            self.assertEqual(actual, set(supercover(b, a)))

    def test_diagonal_cannot_squeeze_through_blocked_corner(self):
        grid = Grid(4, 4, frozenset({(1, 0), (0, 1)}))
        self.assertFalse(line_of_sight(grid, (0, 0), (1, 1)))
        for algo in ("dijkstra8_23", "astar8_euclidean", "theta"):
            self.assertEqual(plan(grid, (0, 0), (3, 3), algo).status, "no_path")

    def test_empty_field_theta_takes_arbitrary_angle(self):
        grid = Grid(32, 24, frozenset())
        start, goal = (2, 2), (27, 18)
        theta = plan(grid, start, goal)
        self.assertEqual(theta.path, (start, goal))
        self.assertAlmostEqual(length(theta.path), math.dist(start, goal))
        self.assertFalse(current_fpga_directions(theta.path))
        self.assertLess(length(theta.path), length(plan(grid, start, goal, "astar8_euclidean").path))
        self.assertFalse(theta.summary()["motion_authorized"])

    def test_eight_neighbor_baseline_already_has_diagonals(self):
        p = plan(Grid(8, 8, frozenset()), (1, 1), (6, 6), "dijkstra8_23")
        self.assertTrue(any(a[0] != b[0] and a[1] != b[1] for a, b in zip(p.path, p.path[1:])))
        self.assertTrue(current_fpga_directions(p.path))

    def test_return_can_change_corridor_when_old_route_is_blocked(self):
        grid = Grid(20, 14, frozenset())
        home, end = (2, 6), (17, 6)
        outward = plan(grid, home, end)
        updated = fused_grid(20, 14, (), {(10, 6), (10, 5), (10, 7)})
        self.assertFalse(line_of_sight(updated, end, home))
        returned = replan_home(updated, end, home)
        self.assertEqual(returned.status, "ok")
        self.assertNotEqual(returned.path, tuple(reversed(outward.path)))
        self.assertEqual((returned.path[0], returned.path[-1]), (end, home))
        self.assertTrue(all(line_of_sight(updated, a, b) for a, b in zip(returned.path, returned.path[1:])))

    def test_lidar_clear_does_not_remove_vision_or_overlap(self):
        lidar, vision = {(4, 4), (5, 5)}, {(4, 4), (6, 6)}
        self.assertEqual(fused_grid(9, 9, lidar, vision).blocked, frozenset({(4, 4), (5, 5), (6, 6)}))
        self.assertEqual(fused_grid(9, 9, (), vision).blocked, frozenset(vision))
        self.assertEqual(fused_grid(9, 9, lidar, ()).blocked, frozenset(lidar))

    def test_segment_paths_are_clear_by_independent_oracle(self):
        for seed in range(12):
            rng = random.Random(seed)
            blocked = {(x, y) for y in range(12) for x in range(16) if rng.random() < .12}
            blocked.difference_update({(1, 1), (14, 10)})
            grid = Grid(16, 12, frozenset(blocked))
            statuses = []
            for algorithm in ("dijkstra8_23", "astar8_euclidean", "theta"):
                result = plan(grid, (1, 1), (14, 10), algorithm)
                statuses.append(result.status)
                for a, b in zip(result.path, result.path[1:]):
                    self.assertFalse(any(hits_closed_square(a, b, c) for c in blocked), (seed, algorithm, a, b))
            self.assertEqual(len(set(statuses)), 1, (seed, statuses))

    def test_blocked_outside_and_no_path_are_explicit(self):
        grid = Grid(6, 6, frozenset((3, y) for y in range(6)))
        for algorithm in ("dijkstra8_23", "astar8_euclidean", "theta"):
            self.assertEqual(plan(grid, (-1, 0), (5, 5), algorithm).status, "invalid_endpoint")
            self.assertEqual(plan(grid, (0, 0), (3, 3), algorithm).status, "invalid_endpoint")
            self.assertEqual(plan(grid, (0, 0), (5, 5), algorithm).status, "no_path")

    def test_budget_exhaustion_is_not_reported_as_no_path(self):
        grid = Grid(32, 24, frozenset())
        self.assertEqual(plan(grid, (1, 1), (30, 22), expansion_budget=1).status, "budget_exhausted")

    def test_same_cell_and_validation(self):
        grid = Grid(4, 4, frozenset())
        result = plan(grid, (1, 1), (1, 1))
        self.assertEqual(result.path, ((1, 1),))
        self.assertEqual(length(result.path), 0)
        with self.assertRaises(ValueError):
            Grid(4, 4, frozenset({(4, 1)}))
        with self.assertRaises(ValueError):
            plan(grid, (True, 1), (2, 2))


if __name__ == "__main__":
    unittest.main()
