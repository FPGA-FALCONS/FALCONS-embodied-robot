# 巡检规划参考：八方向、任意角度、自主返航

状态：**可运行的离线规划与文件接口，配套独立RTL线段检查器已仿真；未接生产FPGA、相机或实车。**

用户2026-10-26修订：允许自主选择返程路线，不再要求沿去程原路返回。总体方案及接口见 [巡检架构R3](../../doc/AI自主巡检_20261026/README.md)。

## 已经实现什么

| 算法 | 实现与用途 |
| --- | --- |
| `dijkstra8_23` | 八邻接、正向2/斜向3代价的软件基线；已经能走45°斜线 |
| `astar8_euclidean` | 八邻接欧氏A*，用于区分启发式搜索和任意角度收益 |
| `theta` | 基于Basic Theta*祖先视线连接的保守格中心改编，可产生任意斜率线段 |
| `replan_home()` | 在当前融合地图上从当前位置重规划到保存的任务起点，不依赖去程轨迹 |
| `fused_grid()` | 只读融合已膨胀雷达/视觉cost集合，雷达清空不会抹掉视觉障碍 |

碰撞约定：格中心为整数，障碍为闭合单位方格；线段碰到障碍边角也拒绝。supercover会列出整段触及的格，包含45°对角穿越时的两侧格。地图边界外阻挡。输入必须是包含车体包络、误差和不可见危险区域的**配置空间障碍**；本包不根据真实图像生成这些信息，也不把point robot路径当成车体安全证明。

算法依据：[Daniel, Nash, Koenig, Felner, *Theta*: Any-Angle Path Planning on Grids*, JAIR 39, 2010](https://idm-lab.org/bib/abstracts/Koen10r.html)，[论文预印本入口](https://arxiv.org/abs/1401.3843)。这是成熟参考，不是“2026最新算法”。本实现的格中心/碰角拒绝规则与论文顶点模型不同；Theta*不保证全局最优或最少转弯。

## 复现

Python 3.10+标准库，无需安装包、模型或设备：

```text
python host/inspection_reference/verify_reference.py
```

单独运行测试或对照：

```text
python -m unittest discover -s host/inspection_reference/tests -v
python host/inspection_reference/benchmark.py
```

基准输出 `host/inspection_reference/evidence/benchmark.json`，保存固定地图、输入哈希、每条路径、欧氏长度、转折、扩展节点和碰撞策略。若任一返回路径违规或可达性不一致会失败。地图为固定6个场景+20个固定随机种子；不是采集地图，不用于模型训练或精度评价。

总验证输出 `evidence/verification.json` 和 `python_tests.txt`；算法/输入哈希可复核。可选图示 `evidence/算法对照.png` 来自实际保存的合成路径，重画脚本 `render_comparison.py` 需要Pillow及显式中文字体；标准库验证本身不需要这两个依赖。

测试共14项，包含600条固定随机线段与**独立有理数线段/AABB几何判定器**对照；另在12张随机图中对三个算法返回的每段做独立几何检查。覆盖禁止切角、封路、占用终点、地图范围、预算耗尽与无路区分、起终点重合、返程绕行、快照合同、epoch失效及命令行退出码。

## 队友可直接试的文件接口

```text
python host/inspection_reference/route_cli.py host/inspection_reference/examples/costmap.json --return-home --output route.json
```

示例是合成的返程地图：新视觉障碍封住旧直线，重新规划回保存的home。`route.json`是用户指定的输出位置；没有串口或网络动作。Python模块入口为 `plan_snapshot(snapshot, algorithm="theta", returning=True)`。

输入合同 `falcons.costmap.snapshot.v1`：地图宽高1～256、格长mm、uint32 revision、定位epoch、current/goal/home格坐标、home_epoch、两层已膨胀cost坐标列表。必须显式声明 `configuration_space_inflated=true`；返程要求home_epoch与当前定位epoch相同。不负责把视觉原始点变成带车体包络的障碍，不接受越界裁剪。

输出带输入SHA256、地图版本、定位epoch、规划状态与格中心路径；`motion_authorized=false`、`hardware_tested=false`、`execution_interface=null`。退出码0=成功得到路径、2=有结构化结果但无可用路线（含预算耗尽/非法端点）、1=输入合同或文件错误。调用者必须检查状态，不能把缺失/旧的输出当成功。

`--algorithm astar8_euclidean` 可生成几何上符合现有八方向的路线供离线对照，仍不代表已有上板命令协议。默认Theta路径不能直接送给八方向生产路线链。

## FPGA任意角度基础组件

[nav_supercover_check](../../rtl/inspection/README.md) 接收起终格，逐格查询整条任意斜率线段，支持读图背压、取消、地图版本失效和旧读排空，完成结果带版本。没有直接改动生产导航行为。

```text
python host/inspection_reference/verify_rtl.py
```

当前461例RTL仿真全部通过，实际接受8,771次地图读；与独立有理数几何预期对照。结果保存在 `evidence/rtl_verification.json`、`rtl_tests.txt`。此脚本需要已有Icarus，不安装或升级任何工具；仿真不证明综合资源、时序或实车表现。

## 当前固定基准结果

26张地图上三算法可达性判断一致；23张共同可达，另有完全隔断、角落被封和终点占用场景。共同可达图统计：

| 软件参考 | 平均欧氏路长 | 平均转折次数 | 平均扩展节点 |
| --- | ---: | ---: | ---: |
| 八方向2/3 Dijkstra | 1.6977m | 9.87 | 557.22 |
| 八方向欧氏A* | 1.6977m | 11.22 | 195.70 |
| 保守格中心Theta* | 1.6125m | 4.00 | 115.35 |

Theta*相对八方向A*逐图平均路长减少5.02%，这23图均缩短，最小缩短2.94%；该固定小集合不证明任意环境都更短。两种八方向参考的转折差异来自本包确定性优先队列及代价/启发函数，不意味着正式RTL会有相同差异。

返程固定对照：开始图为空，去程可直达；返程前在旧直线中间加入视觉障碍，原直返线被拒绝，新路径仍回到原任务起点。完全无路时返回 `no_path`，预算耗尽返回 `budget_exhausted`，不能互相冒充。起终点占用/越界返回 `invalid_endpoint`。

## 不得直接替换生产导航

正式main `437b48d`早已实现八方向四桶Dijkstra、等代价少转弯、居中和航点后处理。本包的八方向基线**没有复现这些RTL后处理、时钟周期或tie-break**。上表是软件参考对照，不能声称相对正式整车路径减少5%或FPGA提速。

`Plan.summary()`输出 `current_fpga_direction_compatible`；任意角度路径通常为false，所有结果都包含 `motion_authorized=false`。当前 `nav_segments`、路线复核、前探与段方向接口只支持八方向，不能将任意角度路径强塞进去。

部署顺序：

1. 用现有八方向执行链实现视觉融合地图和自由返航，目标只由巡检任务入口提交。
2. Theta*先运行shadow，保存同一实测地图上的路径对照，验证未知区域、最小间隙和视野限制。
3. 若要执行任意角度，扩展段向量/整段supercover查询/近场前探与握手，再做RTL、动力学、综合/PnR和实车验收。
4. 连续曲线跟踪另行实现前视跟踪、曲率/制动速度约束与跟踪偏差碰撞检查。仅减少折点不代表速度、加速度或曲率连续，不使用未经检查的样条穿障。

不更换模型、不升级板端环境，不打开串口、不下载位流。规划步数/扩展量不等于RK/NPU/FPGA延迟；本轮没有实时性或制动实验。
