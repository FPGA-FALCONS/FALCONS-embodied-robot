"""Independent equal-cost eight-neighbour BFS fixtures; outputs only to logs/."""
import argparse
import json
from collections import deque
from pathlib import Path
from bfs_indoor import scenes, N

DIR = ((1, 0), (1, 1), (0, 1), (-1, 1),
       (-1, 0), (-1, -1), (0, -1), (1, -1))
ORDER = (0, 1, -1, 2, -2, 3, -3, 4)


def solve(scene):
    sx, sy = scene['start']
    gx, gy = scene['goal']
    raw, cost = scene['raw'], scene['cost']
    relax = scene['escape'] and cost[sy*N+sx]

    def allowed(x, y):
        return (0 <= x < N and 0 <= y < N and not raw[y*N+x] and
                (not cost[y*N+x] or (relax and abs(x-sx) <= 4 and abs(y-sy) <= 4)))

    if not allowed(sx, sy) or not allowed(gx, gy):
        return [], 0, 0
    parent = {(gx, gy): None}
    root_vector = ((sx > gx)-(sx < gx), (sy > gy)-(sy < gy))
    root_direction = DIR.index(root_vector) if root_vector != (0, 0) else 0
    q = deque([(gx, gy, root_direction)])
    peak, expanded = 1, 0
    while q and (sx, sy) not in parent:
        x, y, heading = q.popleft()
        expanded += 1
        for offset in ORDER:
            direction = (heading + offset) % 8
            dx, dy = DIR[direction]
            point = (x+dx, y+dy)
            if (point in parent or not allowed(*point) or
                    (dx and dy and not (allowed(x+dx, y) and allowed(x, y+dy)))):
                continue
            parent[point] = (x, y)
            if point == (sx, sy):
                break
            q.append((*point, direction))
            peak = max(peak, len(q))
    path = []
    if (sx, sy) in parent:
        point = (sx, sy)
        while point is not None:
            path.append(point[1]*N+point[0])
            point = parent[point]
    return path, peak, expanded


def fixtures():
    cases = scenes()
    for direction, (dx, dy) in enumerate(DIR):
        cases.append(dict(name=f'open_direction_{direction}', raw=bytearray(N*N),
                          cost=bytearray(N*N), start=(128, 128),
                          goal=(128+dx*24, 128+dy*24), escape=0))
    # Only the two diagonally adjacent cells are free: a diagonal must not leak through.
    for side in (0, 1, 2):
        cost = bytearray([1])*(N*N)
        for x, y in [(128, 128), (129, 129)]:
            cost[y*N+x] = 0
        if side == 1:
            cost[128*N+129] = 0
        if side == 2:
            cost[128*N+129] = cost[129*N+128] = 0
        cases.append(dict(name=f'corner_sides_{side}', raw=bytearray(cost), cost=cost,
                          start=(128, 128), goal=(129, 129), escape=0))
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('logs/eight_direction/fixtures'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = fixtures()
    manifest, records = [str(len(cases))], []
    for index, case in enumerate(cases):
        path, peak, expanded = solve(case)
        assert peak < 8192
        (args.output/f'map_{index}.hex').write_text(
            ''.join(f'{2*r+c:x}\n' for r, c in zip(case['raw'], case['cost'])), encoding='ascii')
        (args.output/f'path_{index}.hex').write_text(
            ''.join(f'{p:04x}\n' for p in path) or '0000\n', encoding='ascii')
        manifest.append(' '.join(map(str, (index, *case['start'], *case['goal'],
                                          case['escape'], len(path)-1))))
        records.append(dict(index=index, name=case['name'], steps=len(path)-1,
                            peak=peak, expanded=expanded))
    (args.output/'cases.txt').write_text('\n'.join(manifest)+'\n', encoding='ascii')
    (args.output/'reference.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    print(f'Generated {len(cases)} eight-direction indoor/corner fixtures')


if __name__ == '__main__':
    main()
