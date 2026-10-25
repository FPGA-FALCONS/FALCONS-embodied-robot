"""File-in/file-out shadow planner; no serial ports, motion or device writes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

if __package__:
    from .planning import fused_grid, plan, replan_home
else:
    from planning import fused_grid, plan, replan_home


def plan_snapshot(snapshot, *, algorithm="theta", returning=False, expansion_budget=None):
    """Consume one immutable, already inflated cost-map snapshot."""
    if not isinstance(snapshot, dict):
        raise ValueError("snapshot must be a JSON object")
    if snapshot.get("schema") != "falcons.costmap.snapshot.v1":
        raise ValueError("unsupported snapshot schema")
    if snapshot.get("configuration_space_inflated") is not True:
        raise ValueError("cost cells must already include vehicle envelope and uncertainty")
    revision = snapshot.get("revision")
    if type(revision) is not int or not 0 <= revision < 2**32:
        raise ValueError("expected uint32 map revision")
    epoch = snapshot.get("localization_epoch")
    if not isinstance(epoch, str) or not epoch:
        raise ValueError("localization epoch required")
    if returning and snapshot.get("home_epoch") != epoch:
        raise ValueError("saved home belongs to another localization epoch")
    cell_size = snapshot.get("cell_size_mm")
    for axis in ("width", "height"):
        if type(snapshot[axis]) is not int or not 1 <= snapshot[axis] <= 256:
            raise ValueError("snapshot dimensions must be between 1 and 256")
    # summary() checks finite, positive numeric cell size; bool is rejected.
    grid = fused_grid(snapshot["width"], snapshot["height"],
                      snapshot["lidar_cost"], snapshot["vision_cost"])
    target = snapshot["home"] if returning else snapshot["goal"]
    solve = replan_home if returning else plan
    result = solve(grid, snapshot["current"], target, algorithm,
                   expansion_budget=expansion_budget)
    digest = hashlib.sha256(json.dumps(snapshot, sort_keys=True, separators=(",", ":"),
                                      allow_nan=False).encode("utf-8")).hexdigest()
    return {"schema": "falcons.shadow.route.v1", "map_revision": revision,
            "localization_epoch": epoch, "input_sha256": digest,
            "mission_leg": "return_home" if returning else "outbound",
            "coordinate_unit": "cell_center", "cell_size_mm": cell_size,
            "execution_interface": None, **result.summary(cell_size)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--algorithm", choices=("theta", "astar8_euclidean", "dijkstra8_23"), default="theta")
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--expansion-budget", type=int)
    args = parser.parse_args()
    try:
        if args.snapshot.resolve() == args.output.resolve():
            raise ValueError("output must not overwrite input snapshot")
        snapshot = json.loads(args.snapshot.read_text(encoding="utf-8-sig"))
        result = plan_snapshot(snapshot, algorithm=args.algorithm, returning=args.return_home,
                               expansion_budget=args.expansion_budget)
        args.output.write_bytes((json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n").encode("utf-8"))
        print(json.dumps({"status": result["status"], "motion_authorized": False}))
        return 0 if result["status"] == "ok" else 2
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(f"snapshot rejected: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
