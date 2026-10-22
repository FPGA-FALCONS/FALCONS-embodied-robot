"""Plot actual old/new RTL waypoints after both nav_planner_tb runs.

Run from the repository root; requires matplotlib and numpy, not needed by run_sim.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

ROOT = Path('logs/bfs_straight')
NEW = ROOT / 'fixtures'
OLD = ROOT / 'baseline'
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def rows(directory, filename):
    with (directory / filename).open(encoding='ascii') as stream:
        return [{key: int(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=OLD)
    parser.add_argument('--current', type=Path, default=NEW)
    parser.add_argument('--output', type=Path, default=ROOT)
    parser.add_argument('--labels', nargs=2, default=('原固定顺序 BFS（保留现有居中处理）',
                                                    '直行优先 BFS（保留现有居中处理）'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reference = json.loads((args.current / 'reference.json').read_text(encoding='utf-8'))
    metrics = [rows(directory, 'rtl_center_1.csv') for directory in (args.baseline, args.current)]
    waypoints = [rows(directory, 'waypoints_center_1.csv') for directory in (args.baseline, args.current)]
    titles = ['房间与错位门洞', '斜走廊连接两间房', '斜隔墙、门洞与斜放长桌']
    bounds = [(-2.6, -3.6, 6.3, 5.2), (-1.65, -1.65, 6.4, 4.45), (-2.6, -3.6, 6.3, 4.6)]
    result = []
    for index, scene in enumerate(reference):
        entry = dict(scene=scene['name'], old_rtl=metrics[0][index], new_rtl=metrics[1][index])
        result.append(entry)
        if index >= 3:
            continue
        data = np.array([int(value, 16) for value in
                         (args.current / f'map_{index}.hex').read_text(encoding='ascii').split()], dtype=np.uint8).reshape(256, 256)
        image = np.ones((256, 256, 3))
        image[(data & 1) != 0] = (.91, .81, .77)
        image[(data & 2) != 0] = (.16, .18, .20)
        fig, axes = plt.subplots(1, 2, figsize=(13.4, 6.8))
        fig.subplots_adjust(left=.065, right=.985, top=.86, bottom=.17, wspace=.13)
        for mode, (ax, color, label) in enumerate(zip(axes, ('#d77924', '#246fbc'), args.labels)):
            points = [scene['start']] + [[row['x'], row['y']] for row in waypoints[mode] if row['case'] == index]
            path = (np.array(points) + .5) * .05 - 6.4
            metric = metrics[mode][index]
            ax.imshow(image, origin='lower', extent=(-6.4, 6.4, -6.4, 6.4), interpolation='nearest')
            ax.plot(path[:, 0], path[:, 1], color=color, linewidth=2., zorder=4)
            ax.scatter(*path[0], s=65, color='#12a46a', edgecolors='white', zorder=6)
            ax.scatter(*path[-1], s=115, color='#b82e44', marker='*', edgecolors='white', zorder=6)
            ax.annotate('S', path[0], xytext=(0, -16), textcoords='offset points', ha='center')
            ax.annotate('G', path[-1], xytext=(0, 10), textcoords='offset points', ha='center')
            x0, y0, x1, y1 = bounds[index]
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_aspect('equal')
            ax.set_xlabel('世界 x / m')
            ax.set_ylabel('世界 y / m')
            ax.grid(alpha=.12)
            ax.set_title(f"{label}\n{metric['steps']*.05:.2f} m，"
                         f"{metric['turns']} 次转弯，{metric['cycles']/50000:.3f} ms")
        fig.suptitle(titles[index] + '｜实际 RTL 输出路线对比', y=.97, fontsize=14)
        fig.legend(handles=[Patch(facecolor=(.16, .18, .20), label='墙 / 家具实际障碍'),
                            Patch(facecolor=(.91, .81, .77), label='现有方形膨胀 4 格（200 mm）'),
                            Line2D([], [], color='#777', linewidth=2, label='RTL 接收成功的航点连线')],
                   loc='lower center', bbox_to_anchor=(.5, .055), ncol=3, frameon=False)
        fig.text(.5, .02, '256×256、50 mm/格、中心原点；静态室内地图，含读口延迟与输出背压；不代表实车运动。',
                 ha='center', fontsize=9)
        fig.savefig(args.output / (scene['name'] + '_rtl.png'), dpi=160, bbox_inches='tight', pad_inches=.15)
        plt.close(fig)
    (args.output / 'rtl_comparison.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(f'Wrote 3 actual RTL comparison figures and {len(reference)} scenario metrics')


if __name__ == '__main__':
    main()
