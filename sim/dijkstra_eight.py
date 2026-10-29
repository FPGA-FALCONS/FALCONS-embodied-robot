"""Single-label turn-cost search checked with a full-distance heap; outputs only to logs/."""
import argparse
import json
from heapq import heappush, heappop
import random
from pathlib import Path
from bfs_indoor import scenes, N

DIR = ((1, 0), (1, 1), (0, 1), (-1, 1),
       (-1, 0), (-1, -1), (0, -1), (1, -1))
ORDER = (0, 1, -1, 2, -2, 3, -3, 4)
TURN_COST = 4


def route_cost(path):
    directions = [(b % N-a % N, b // N-a // N) for a, b in zip(path, path[1:])]
    distance = sum(3 if dx and dy else 2 for dx, dy in directions)
    turns = sum(a != b for a, b in zip(directions, directions[1:]))
    return distance, turns, distance + TURN_COST * turns


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
    distance = {(gx, gy): 0}
    heading = {}
    settled = set()
    root_vector = ((sx > gx)-(sx < gx), (sy > gy)-(sy < gy))
    heading[gx, gy] = DIR.index(root_vector) if root_vector != (0, 0) else 0
    q = [(0, 0, (gx, gy))]
    sequence, peak, expanded = 0, 1, 0
    while q:
        value, _, point = heappop(q)
        if point in settled or distance[point] != value:
            continue
        settled.add(point)
        if point == (sx, sy):
            break
        x, y = point
        expanded += 1
        for offset in ORDER:
            direction = (heading[point] + offset) % 8
            dx, dy = DIR[direction]
            child = (x+dx, y+dy)
            candidate = value + (3 if dx and dy else 2) + TURN_COST * int(
                point != (gx, gy) and direction != heading[point])
            old = distance.get(child, 1 << 30)
            if (child in settled or not allowed(*child) or candidate >= old or
                    (dx and dy and not (allowed(x+dx, y) and allowed(x, y+dy)))):
                continue
            parent[child] = point
            distance[child] = candidate
            heading[child] = direction
            sequence += 1
            heappush(q, (candidate, sequence, child))
            peak = max(peak, len(q))

    path = []
    if (sx, sy) in parent:
        point = (sx, sy)
        while point is not None:
            path.append(point[1]*N+point[0])
            point = parent[point]
    if path:
        assert route_cost(path)[2] == distance[sx, sy]
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
    for dx, dy in [(40,12), (40,-12), (-40,12), (-40,-12)]:
        cases.append(dict(name=f'unequal_diagonal_{dx}_{dy}', raw=bytearray(N*N),
                          cost=bytearray(N*N), start=(128,128), goal=(128+dx,128+dy), escape=0))
    for seed in range(12):
        rng = random.Random(seed)
        raw = bytearray([1]) * (N*N)
        for y in range(110, 144):
            for x in range(110, 146):
                raw[y*N+x] = int(rng.random() < .15)
        for x, y in [(112,112), (143,139)]:
            raw[y*N+x] = 0
        cases.append(dict(name=f'room_furniture_random_{seed}', raw=raw, cost=bytearray(raw),
                          start=(112,112), goal=(143,139), escape=0))
    # More than 255 turns must still be charged; no diagonal corner cuts.
    raw = bytearray([1])*(N*N)
    cursor = [4,2]
    raw[cursor[1]*N+cursor[0]] = 0
    def line_to(x,y):
        dx=(x>cursor[0])-(x<cursor[0]);dy=(y>cursor[1])-(y<cursor[1])
        while cursor != [x,y]:
            cursor[0]+=dx;cursor[1]+=dy;raw[cursor[1]*N+cursor[0]]=0
    for x,y in [(6,2),(6,0),(8,0),(8,2),(12,2),(12,0),(14,0),(14,2),(20,2)]:
        line_to(x,y)
    for row in range(1,126):
        line_to(cursor[0],2+2*row)
        line_to(4 if row%2 else 20,2+2*row)
    cases.append(dict(name='many_turns_corridor',raw=raw,cost=bytearray(raw),
                      start=(4,2),goal=tuple(cursor),escape=0))
    # Same room geometry, different endpoints: exercise fewer turns without artificial mazes.
    for index, start, goal in [(0, (224,112), (176,112)), (2, (144,88), (208,80))]:
        cases.append(dict(cases[index], name=cases[index]['name']+'_turn_cost',
                          start=start, goal=goal))
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('logs/dijkstra/fixtures'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = fixtures()
    manifest, records = [str(len(cases))], []
    for index, case in enumerate(cases):
        path, peak, expanded = solve(case)
        assert peak < 2048
        (args.output/f'map_{index}.hex').write_text(
            ''.join(f'{2*r+c:x}\n' for r, c in zip(case['raw'], case['cost'])), encoding='ascii')
        (args.output/f'path_{index}.hex').write_text(
            ''.join(f'{p:04x}\n' for p in path) or '0000\n', encoding='ascii')
        manifest.append(' '.join(map(str, (index, *case['start'], *case['goal'],
                                          case['escape'], len(path)-1))))
        weighted, turns, total = route_cost(path) if path else (-1, -1, -1)
        if case['name'].startswith('unequal_diagonal_'):
            ax=abs(case['start'][0]-case['goal'][0]); ay=abs(case['start'][1]-case['goal'][1])
            assert weighted == 2*max(ax,ay)+min(ax,ay)
            directions = [(b%N-a%N,b//N-a//N) for a,b in zip(path,path[1:])]
            assert sum(a != b for a,b in zip(directions,directions[1:])) <= 1
            assert total == weighted + TURN_COST
        if case['name'].startswith('open_direction_'):
            assert turns == 0 and total == weighted
        if case['name'] == 'room_furniture_random_1':
            # This room trades the old movement cost 93 / eight turns for 97 / five.
            assert weighted == 97 and turns == 5 and total == 117
        if case['name']=='many_turns_corridor':
            directions=[(b%N-a%N,b//N-a//N) for a,b in zip(path,path[1:])]
            assert sum(a!=b for a,b in zip(directions,directions[1:]))>255
            assert total > weighted + TURN_COST * 255
        records.append(dict(index=index, name=case['name'], steps=len(path)-1,
                            cost=weighted, turns=turns, total_cost=total, peak=peak, expanded=expanded))
    (args.output/'cases.txt').write_text('\n'.join(manifest)+'\n', encoding='ascii')
    (args.output/'reference.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    print(f'Generated {len(cases)} Dijkstra indoor/corner/unequal diagonal fixtures')


if __name__ == '__main__':
    main()
