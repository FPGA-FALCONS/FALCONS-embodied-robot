"""Fixed synthetic-map comparison. No models, devices, or production changes."""
from pathlib import Path
import hashlib
import json
import random
import statistics
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from inspection_reference.planning import Grid, fused_grid, plan, replan_home, line_of_sight, current_fpga_directions

ALGORITHMS = ("dijkstra8_23", "astar8_euclidean", "theta")


def fixtures():
    result = [
        ("open_oblique", Grid(32, 24, frozenset()), (2, 2), (27, 18)),
        ("wall_detour", Grid(32, 24, frozenset((x, y) for x in range(13, 16) for y in range(5, 19))), (3, 12), (28, 12)),
        ("vision_only", fused_grid(32, 24, (), {(x, y) for x in range(13, 17) for y in range(9, 15)}), (3, 12), (28, 12)),
        ("sealed_wall", Grid(32, 24, frozenset((16, y) for y in range(24))), (3, 12), (28, 12)),
        ("corner_trap", Grid(4, 4, frozenset({(1, 0), (0, 1)})), (0, 0), (3, 3)),
        ("occupied_goal", Grid(12, 12, frozenset({(10, 10)})), (1, 1), (10, 10)),
    ]
    for seed in range(20):
        rng = random.Random(260000 + seed)
        blocked = {(x, y) for x in range(28) for y in range(22) if rng.random() < .12}
        blocked.difference_update({(1, 1), (26, 20)})
        result.append((f"random_{seed:02}", Grid(28, 22, frozenset(blocked)), (1, 1), (26, 20)))
    return result


def map_record(grid):
    return {"width": grid.width, "height": grid.height, "blocked": sorted(grid.blocked)}


def main():
    output = HERE / "evidence"
    output.mkdir(exist_ok=True)
    rows = []
    for name, grid, start, goal in fixtures():
        plans = [plan(grid, start, goal, algorithm) for algorithm in ALGORITHMS]
        for p in plans:
            if p.status == "ok":
                if p.path[0] != start or p.path[-1] != goal:
                    raise RuntimeError("endpoint invariant failed")
                if not all(line_of_sight(grid, a, b) for a, b in zip(p.path, p.path[1:])):
                    raise RuntimeError("blocked path generated")
        map_data = map_record(grid)
        map_hash = hashlib.sha256(json.dumps(map_data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        rows.append({"name": name, "synthetic": True, "map": map_data, "map_sha256": map_hash,
                     "start": start, "goal": goal, "plans": {p.algorithm: p.summary() for p in plans}})
    common = [r for r in rows if all(r["plans"][a]["status"] == "ok" for a in ALGORITHMS)]
    agreement = all(len({r["plans"][a]["status"] for a in ALGORITHMS}) == 1 for r in rows)
    means = {a: {"mean_length_m": statistics.mean(r["plans"][a]["length_m"] for r in common),
                 "mean_turns": statistics.mean(r["plans"][a]["turns"] for r in common),
                 "mean_expanded": statistics.mean(r["plans"][a]["expanded"] for r in common)} for a in ALGORITHMS}
    shorter, longer = [], []
    improvements = []
    for r in common:
        base = r["plans"]["astar8_euclidean"]["length_cells"]
        candidate = r["plans"]["theta"]["length_cells"]
        improvements.append((base - candidate) / base * 100)
        if candidate < base - 1e-9:
            shorter.append(r["name"])
        elif candidate > base + 1e-9:
            longer.append(r["name"])
    home, end = (2, 6), (17, 6)
    initial = Grid(20, 14, frozenset())
    outward = plan(initial, home, end)
    updated = fused_grid(20, 14, (), {(10, 5), (10, 6), (10, 7)})
    returned = replan_home(updated, end, home)
    return_ok = (returned.status == "ok" and not line_of_sight(updated, end, home)
                 and all(line_of_sight(updated, a, b) for a, b in zip(returned.path, returned.path[1:])))
    report = {
        "status": "PASS" if agreement and return_ok else "FAIL",
        "scope": "fixed_synthetic_maps_offline_reference_only", "maps": len(rows),
        "common_solved": len(common), "connectivity_agreement": agreement,
        "motion_authorized": False, "hardware_tested": False, "live_camera_tested": False,
        "fpga_latency_measured": False, "production_rtl_changed": False,
        "cell_size_mm": 50, "collision_policy": "closed_cell_supercover_on_already_inflated_cost_map",
        "means_common_solved": means,
        "theta_vs_astar8": {"shorter_cases": shorter, "longer_cases": longer,
            "mean_per_map_length_reduction_percent": statistics.mean(improvements),
            "worst_per_map_length_reduction_percent": min(improvements)},
        "return_replanning": {"status": "PASS" if return_ok else "FAIL", "map": map_record(updated),
            "home": home, "current": end, "outbound": outward.summary(), "return": returned.summary(),
            "previous_direct_return_is_blocked": not line_of_sight(updated, end, home)},
        "scenarios": rows,
        "implementation_sha256": hashlib.sha256((HERE / "planning.py").read_bytes()).hexdigest(),
        "benchmark_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "limits": ["not an RTL baseline reproduction", "Theta* is not guaranteed globally optimal",
                   "cell-center conservative adaptation differs from paper vertex convention",
                   "no continuous dynamics, braking, visibility or real sensor uncertainty simulation",
                   "any-angle paths are not directly compatible with current FPGA route validation"],
    }
    (output / "benchmark.json").write_bytes((json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(json.dumps({k: report[k] for k in ("status", "maps", "common_solved", "connectivity_agreement",
                                           "means_common_solved", "theta_vs_astar8")}, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
