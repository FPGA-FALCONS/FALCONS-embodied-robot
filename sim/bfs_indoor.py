"""Generate indoor map fixtures and an independent FIFO BFS reference (stdlib only).

Run from the repository root. Outputs go to logs/; --order fixed is for checking
the unmodified RTL baseline, not a hardware compatibility mode.
"""
import argparse
import json
import math
from collections import deque
from pathlib import Path

N = 256
CELL = 0.05
DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))
LEFT = (2, 3, 1, 0)
RIGHT = (3, 2, 0, 1)


def cell(point):
    return tuple(math.floor((value + 6.4) / CELL) for value in point)


def raster(predicate):
    return bytearray(predicate((x + 0.5) * CELL - 6.4,
                               (y + 0.5) * CELL - 6.4)
                     for y in range(N) for x in range(N))


def rectangle(x0, y0, x1, y1):
    return raster(lambda x, y: x0 <= x < x1 and y0 <= y < y1)


def combine(*masks):
    return bytearray(any(values) for values in zip(*masks))


def subtract(a, b):
    return bytearray(x and not y for x, y in zip(a, b))


def inflate(raw, radius):
    cost = bytearray(raw)
    for index, blocked in enumerate(raw):
        if not blocked:
            continue
        x, y = index % N, index // N
        x0, x1 = max(0, x - radius), min(N, x + radius + 1)
        fill = b'\x01' * (x1 - x0)
        for row in range(max(0, y - radius), min(N, y + radius + 1)):
            cost[row * N + x0:row * N + x1] = fill
    return cost


def perimeter(bounds):
    x0, y0, x1, y1 = bounds
    floor = rectangle(*bounds)
    return floor, subtract(floor, rectangle(x0 + .1, y0 + .1, x1 - .1, y1 - .1))


def polygon(points):
    def inside(x, y):
        hit = False
        for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1]):
            if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
                hit = not hit
        return hit
    return raster(inside)


def segment(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    def inside(x, y):
        t = max(0, min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
        return (x - a[0] - t * dx) ** 2 + (y - a[1] - t * dy) ** 2 <= .05 ** 2
    return raster(inside)


def corridor(angle, width, distance=5.5):
    theta = math.radians(angle)
    hx, hy = math.cos(theta), math.sin(theta)
    a, b = (.4 * hx, .4 * hy), (distance * hx, distance * hy)
    nx, ny = -hy * width / 2, hx * width / 2
    floor = combine(rectangle(-.9, -.9, .9, .9),
                    polygon([(a[0]+nx, a[1]+ny), (b[0]+nx, b[1]+ny),
                             (b[0]-nx, b[1]-ny), (a[0]-nx, a[1]-ny)]),
                    rectangle(b[0]-.75, b[1]-.75, b[0]+.75, b[1]+.75))
    return subtract(inflate(floor, 2), floor), cell(b)


def scenes():
    result = []
    def add(name, raw, goal, start=(128, 128), escape=0, cost=None):
        result.append(dict(name=name, raw=raw, cost=inflate(raw, 4) if cost is None else cost,
                           start=start, goal=goal, escape=escape))

    _, raw = perimeter((-2.2, -3.2, 5.9, 4.8))
    raw = combine(raw,
                  subtract(rectangle(1.45, -3.2, 1.55, 4.8), rectangle(1.4, -1.4, 1.6, -.1)),
                  subtract(rectangle(3.55, -3.2, 3.65, 4.8), rectangle(3.5, 2., 3.7, 3.3)),
                  rectangle(-1.8, 1.5, -.7, 2.3), rectangle(2.25, .15, 2.85, 1.15))
    add('rooms_doors', raw, cell((5., 4.)))

    a, b = (.5, .3), (4.9, 2.94)
    length = math.hypot(b[0]-a[0], b[1]-a[1])
    nx, ny = -(b[1]-a[1]) / length * .7, (b[0]-a[0]) / length * .7
    floor = combine(rectangle(-1.2, -1.2, 1.2, 1.2), rectangle(4.1, 2.05, 6., 4.),
                    polygon([(a[0]+nx, a[1]+ny), (b[0]+nx, b[1]+ny),
                             (b[0]-nx, b[1]-ny), (a[0]-nx, a[1]-ny)]))
    raw = combine(subtract(inflate(floor, 2), floor), rectangle(-.95, .55, -.45, .95))
    add('diagonal_corridor', raw, cell((5.25, 3.25)))

    _, raw = perimeter((-2.2, -3.2, 5.9, 4.2))
    door0 = (1. + 2. * (.65 + 3.15) / 7.3, .65)
    door1 = (1. + 2. * (1.95 + 3.15) / 7.3, 1.95)
    raw = combine(raw, segment((1., -3.15), door0), segment(door1, (3., 4.15)),
                  polygon([(3.25, -.1), (4.9, .65), (4.55, 1.45), (2.9, .7)]),
                  rectangle(-1.75, -2.55, -.55, -1.7))
    add('slanted_wall_furniture', raw, cell((4.9, 3.2)))

    for angle in (-45, -30, -15, 15, 30, 45):
        for width in (1.2, 1.5, 2.0):
            raw, goal = corridor(angle, width)
            add(f'corridor_{angle}_{width:.1f}', raw, goal)

    add('same_cell', bytearray(N*N), (128, 128))
    raw = bytearray(N*N)
    raw[128*N+128] = 1
    add('blocked_start', raw, (145, 128))
    add('blocked_goal', raw, (128, 128), start=(145, 128))
    raw = bytearray(N*N)
    for y in range(N):
        raw[y*N+135] = 1
    add('sealed_wall', raw, (145, 128))
    raw = bytearray(N*N)
    raw[128*N+124] = 1
    add('escape_disabled', raw, (145, 128))
    add('escape_enabled', raw, (145, 128), escape=1)
    add('map_edge', bytearray(N*N), (0, 0), start=(10, 10))
    # 窄通道单独覆盖 4 格的临界段，以及不足一个探针间距的尾段。
    for angle in (30, 45):
        raw, goal = corridor(angle, .75, 3.5)
        add(f'short_bends_{angle}', raw, goal)
    return result


def solve(scene, straight):
    start = scene['start'][1]*N + scene['start'][0]
    goal = scene['goal'][1]*N + scene['goal'][0]
    raw, cost = scene['raw'], scene['cost']
    def allowed(index):
        x, y = index % N, index // N
        relaxed = (scene['escape'] and cost[start] and
                   abs(x - start % N) <= 4 and abs(y - start // N) <= 4)
        return not raw[index] and (not cost[index] or relaxed)
    if not allowed(start) or not allowed(goal):
        return dict(path=[], steps=-1, turns=0, expanded=0, peak_queue=0)
    if start == goal:
        return dict(path=[start], steps=0, turns=0, expanded=0, peak_queue=1)
    q = deque([(goal, 0)])
    parent = {goal: goal}
    expanded, peak = 0, 1
    while q:
        index, heading = q.popleft()
        expanded += 1
        order = (heading, LEFT[heading], RIGHT[heading], heading ^ 1) if straight else range(4)
        for direction in order:
            dx, dy = DIRECTIONS[direction]
            x, y = index % N + dx, index // N + dy
            if not (0 <= x < N and 0 <= y < N):
                continue
            neighbour = y*N+x
            if neighbour in parent or not allowed(neighbour):
                continue
            parent[neighbour] = index
            if neighbour == start:
                path = [start]
                while path[-1] != goal:
                    path.append(parent[path[-1]])
                dirs = [b-a for a, b in zip(path, path[1:])]
                return dict(path=path, steps=len(dirs),
                            turns=sum(a != b for a, b in zip(dirs, dirs[1:])),
                            expanded=expanded, peak_queue=peak)
            q.append((neighbour, direction))
            peak = max(peak, len(q))
    return dict(path=[], steps=-1, turns=0, expanded=expanded, peak_queue=peak)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('logs/bfs_straight/fixtures'))
    parser.add_argument('--order', choices=('fixed', 'straight'), default='straight')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = scenes()
    records, manifest = [], [str(len(cases))]
    for index, scene in enumerate(cases):
        old, new = solve(scene, False), solve(scene, True)
        assert old['steps'] == new['steps'], scene['name']
        assert new['peak_queue'] < 8192, scene['name']
        selected = new if args.order == 'straight' else old
        (args.output / f'map_{index}.hex').write_text(
            ''.join(f'{2*r+c:x}\n' for r, c in zip(scene['raw'], scene['cost'])), encoding='ascii')
        (args.output / f'path_{index}.hex').write_text(
            ''.join(f'{p:04x}\n' for p in selected['path']) or '0000\n', encoding='ascii')
        manifest.append(' '.join(map(str, (index, *scene['start'], *scene['goal'], scene['escape'],
                                           selected['steps'], selected['turns']))))
        records.append(dict(index=index, name=scene['name'], start=scene['start'], goal=scene['goal'],
                            escape=scene['escape'], old={k:v for k,v in old.items() if k != 'path'},
                            straight={k:v for k,v in new.items() if k != 'path'}))
    (args.output / 'cases.txt').write_text('\n'.join(manifest)+'\n', encoding='ascii')
    (args.output / 'reference.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    print(f'Generated {len(cases)} indoor/boundary fixtures; reference order={args.order}')


if __name__ == '__main__':
    main()
