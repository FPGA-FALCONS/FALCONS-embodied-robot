"""Plot four-direction RTL waypoints and eight-direction paths checked cell by cell."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from bfs_eight import fixtures

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
ROOT = Path('logs/eight_direction')


def metric(points):
    delta = np.diff(np.asarray(points), axis=0)
    directions = np.sign(delta)
    turns = np.count_nonzero(np.any(directions[1:] != directions[:-1], axis=1))
    return np.linalg.norm(delta, axis=1).sum()*.05, turns


def main():
    with Path('logs/eight_direction/four_direction/waypoints_center_0.csv').open() as f:
        old = list(csv.DictReader(f))
    titles = ['房间与错位门洞', '斜走廊连接两间房', '斜隔墙、门洞与斜放家具']
    bounds = [(-2.6, 6.3, -3.6, 5.2), (-1.65, 6.4, -1.65, 4.45), (-2.6, 6.3, -3.6, 4.6)]
    records = []
    for index, scene in enumerate(fixtures()[:3]):
        data = np.array([int(x, 16) for x in (ROOT/f'fixtures/map_{index}.hex').read_text().split()]).reshape(256, 256)
        image = np.ones((256, 256, 3))
        image[data & 1 != 0] = (.92, .84, .78)
        image[data & 2 != 0] = (.15, .18, .21)
        first = [scene['start']] + [(int(r['x']), int(r['y'])) for r in old if int(r['case']) == index]
        cells = [int(x, 16) for x in (ROOT/f'fixtures/path_{index}.hex').read_text().split()]
        second = [(x % 256, x // 256) for x in cells]
        fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.7))
        fig.subplots_adjust(left=.065, right=.985, bottom=.16, top=.85, wspace=.15)
        for ax, path, label, color in zip(axes, (first, second),
                ('原四方向：实际 RTL 航点', '八方向：RTL 已逐格核对'), ('#cc7722', '#1677c8')):
            distance, turns = metric(path)
            points = (np.array(path)+.5)*.05-6.4
            ax.imshow(image, origin='lower', extent=(-6.4, 6.4, -6.4, 6.4), interpolation='nearest')
            ax.plot(points[:, 0], points[:, 1], color=color, lw=2.2)
            ax.scatter(*points[0], color='#10a567', s=70, edgecolors='white', zorder=4)
            ax.scatter(*points[-1], color='#bc3050', s=135, marker='*', edgecolors='white', zorder=4)
            ax.set(xlim=bounds[index][:2], ylim=bounds[index][2:], xlabel='世界 x / m', ylabel='世界 y / m')
            ax.set_aspect('equal')
            ax.grid(alpha=.12)
            ax.set_title(f'{label}\n{distance:.2f} m · {turns} 次方向变化')
            records.append(dict(case=index, mode=label, distance_m=distance, turns=int(turns)))
        fig.suptitle(titles[index]+'｜同一膨胀地图、直行优先，均未做居中', fontsize=14)
        fig.legend(handles=[Patch(color=(.15,.18,.21), label='原始墙体 / 家具'),
                            Patch(color=(.92,.84,.78), label='规划预留区：4格 / 200 mm')],
                   loc='lower center', bbox_to_anchor=(.5,.05), ncol=2, frameon=False)
        fig.text(.5,.015,'256×256 · 50 mm/格 · 中心原点；八邻域等权 BFS 禁止穿墙角；本图展示搜索结果，不代表实车运动。',ha='center',fontsize=9)
        target=ROOT/f'case_{index}_eight.png'
        fig.savefig(target,dpi=150,bbox_inches='tight')
        plt.close(fig)
        print(target.resolve())
    (ROOT/'image_metrics.json').write_text(json.dumps(records,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(records,ensure_ascii=False))


if __name__ == '__main__':
    main()
