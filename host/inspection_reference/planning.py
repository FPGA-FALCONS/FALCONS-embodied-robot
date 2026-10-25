"""Bounded grid planning references, never an FPGA or real-time controller.

All blocked inputs are ALREADY inflated configuration-space obstacles. Cells
are closed squares centered at integer coordinates: touching a blocked corner
counts as collision. Theta* is a conservative cell-center adaptation of Basic
Theta*, not a reproduction of the paper's grid-vertex collision convention.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

Point = tuple[int, int]
STEPS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))


def point(value):
    if not isinstance(value, (tuple, list)) or len(value) != 2 or any(type(v) is not int for v in value):
        raise ValueError("expected integer cell pair")
    return tuple(value)


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    blocked: frozenset[Point]

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0 or self.width * self.height > 65536):
            raise ValueError("expected positive grid with at most 65536 cells")
        values = frozenset(point(p) for p in self.blocked)
        if any(not self.inside(p) for p in values):
            raise ValueError("blocked cell outside map; clipping is not allowed")
        object.__setattr__(self, "blocked", values)

    def inside(self, p):
        return 0 <= p[0] < self.width and 0 <= p[1] < self.height

    def free(self, p):
        return self.inside(p) and p not in self.blocked


def fused_grid(width, height, lidar_cost, vision_cost):
    """Keep vision occupancy even when lidar reports free. No double inflation."""
    return Grid(width, height, frozenset(point(p) for p in lidar_cost)
                | frozenset(point(p) for p in vision_cost))


def supercover(a, b):
    """All closed unit cells touched by a center-to-center line, exact integers.

    At a vertex both side cells are included, preventing diagonal corner cuts.
    """
    x, y = point(a)
    bx, by = point(b)
    nx, ny = abs(bx - x), abs(by - y)
    sx = 1 if bx > x else -1 if bx < x else 0
    sy = 1 if by > y else -1 if by < y else 0
    ix = iy = 0
    yield x, y
    while ix < nx or iy < ny:
        cross_x, cross_y = (1 + 2 * ix) * ny, (1 + 2 * iy) * nx
        if cross_x == cross_y:
            yield x + sx, y
            yield x, y + sy
            x, y = x + sx, y + sy
            ix, iy = ix + 1, iy + 1
        elif cross_x < cross_y:
            x += sx
            ix += 1
        else:
            y += sy
            iy += 1
        yield x, y


def line_of_sight(grid, a, b):
    a, b = point(a), point(b)
    if not grid.free(a) or not grid.free(b):
        return False
    return all(grid.free(c) for c in supercover(a, b))


def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def turns(path):
    count = 0
    previous = None
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if not dx and not dy:
            continue
        divisor = math.gcd(abs(dx), abs(dy))
        direction = dx // divisor, dy // divisor
        count += int(previous is not None and previous != direction)
        previous = direction
    return count


def current_fpga_directions(path):
    """Current nav_segments/nav_top accept only horizontal/vertical/45 degrees."""
    return all(a[0] == b[0] or a[1] == b[1] or abs(a[0] - b[0]) == abs(a[1] - b[1])
               for a, b in zip(path, path[1:]))


@dataclass(frozen=True)
class Plan:
    algorithm: str
    status: str
    path: tuple[Point, ...]
    expanded: int
    visibility_checks: int

    def summary(self, cell_size_mm=50):
        if type(cell_size_mm) not in (int, float) or not math.isfinite(cell_size_mm) or cell_size_mm <= 0:
            raise ValueError("invalid cell size")
        return {"algorithm": self.algorithm, "status": self.status, "path": self.path,
                "length_cells": length(self.path), "length_m": length(self.path) * cell_size_mm / 1000,
                "turns": turns(self.path), "expanded": self.expanded,
                "visibility_checks": self.visibility_checks,
                "current_fpga_direction_compatible": current_fpga_directions(self.path) if self.path else None,
                "motion_authorized": False, "hardware_tested": False}


def plan(grid, start, goal, algorithm="theta", *, expansion_budget=None):
    """dijkstra8_23, astar8_euclidean, or theta; all use identical blocked cells.

    First is a software 2/3-weight eight-neighbor baseline, NOT a bit/cycle
    reproduction of the production RTL's tie breaks and centering pipeline.
    Second isolates Euclidean eight-neighbor search from Theta* shortcuts.
    """
    if algorithm not in ("dijkstra8_23", "astar8_euclidean", "theta"):
        raise ValueError("unknown reference algorithm")
    start, goal = point(start), point(goal)
    budget = grid.width * grid.height if expansion_budget is None else expansion_budget
    if type(budget) is not int or budget <= 0:
        raise ValueError("expected positive expansion budget")
    if not grid.free(start) or not grid.free(goal):
        return Plan(algorithm, "invalid_endpoint", (), 0, 0)
    g = {start: 0.0}
    parent = {start: start}
    heuristic = (lambda p: 0.0) if algorithm == "dijkstra8_23" else (lambda p: math.dist(p, goal))
    queue = [(heuristic(start), 0.0, start)]
    closed = set()
    checks = 0
    while queue:
        _, popped_g, current = heapq.heappop(queue)
        if current in closed or popped_g > g[current] + 1e-12:
            continue
        if len(closed) >= budget:
            return Plan(algorithm, "budget_exhausted", (), len(closed), checks)
        closed.add(current)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
                if len(path) > grid.width * grid.height:
                    raise RuntimeError("cycle in parent chain")
            path.reverse()
            return Plan(algorithm, "ok", tuple(path), len(closed), checks)
        for dx, dy in STEPS:
            neighbor = current[0] + dx, current[1] + dy
            if neighbor in closed or not grid.free(neighbor):
                continue
            checks += 1
            if not line_of_sight(grid, current, neighbor):
                continue
            edge = (3 if dx and dy else 2) if algorithm == "dijkstra8_23" else math.hypot(dx, dy)
            candidate_g, candidate_parent = g[current] + edge, current
            if algorithm == "theta":
                ancestor = parent[current]
                if ancestor != current:
                    checks += 1
                    if line_of_sight(grid, ancestor, neighbor):
                        shortcut_g = g[ancestor] + math.dist(ancestor, neighbor)
                        if shortcut_g <= candidate_g:
                            candidate_g, candidate_parent = shortcut_g, ancestor
            if candidate_g < g.get(neighbor, math.inf) - 1e-12:
                g[neighbor], parent[neighbor] = candidate_g, candidate_parent
                heapq.heappush(queue, (candidate_g + heuristic(neighbor), candidate_g, neighbor))
    return Plan(algorithm, "no_path", (), len(closed), checks)


def replan_home(grid, current, home, algorithm="theta", *, expansion_budget=None):
    """Return on CURRENT fused map; outbound history is deliberately not required."""
    return plan(grid, current, home, algorithm, expansion_budget=expansion_budget)
