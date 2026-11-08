# FALCONS-embodied-robot

高云ACG720四驱差速机器人，GW5AT-138B、50MHz。复用模块直接维护在本项目`vendor/verilog-reusable-modules/`。

## 项目概览

FPGA具有七个菜单项：运行数据、摄像头、雷达建图、避障导航、非避障导航、参数整定、AI模块。基础采显、雷达建图、八方向Dijkstra、点位执行、航位推算、四轮PID、遥控及字库由FPGA独立完成；车载屏幕配合手柄可脱离PC运行，RK断连不影响基础功能。

AI模块是RK3588承担的扩展，含语义导航、目标识别、聊天机器人、视觉巡检。RK使用独立UVC相机；FPGA摄像头只本地显示。巡检由RK管理多路径点、视觉目标、图层、页面、播报及录像，复用FPGA导航并按最新地图返航。

当前交付、最近部署记录与待办统一见[交接单](doc/交接单.md)，运行参数见[参数总表](doc/参数总表与待定清单.md)，各轮测试与现场反馈见[验证记录](doc/AI巡检验证记录.md)。2026-11-04摄像头三帧缓存候选已构建验证、尚未烧录，不能据源码更新推断板上已运行该版本。

## 文档入口

| 目的 | 文档 |
| --- | --- |
| 接手与当前待办 | [交接单](doc/交接单.md)、[文档索引](doc/README.md)、[设计落实待办](doc/设计落实待办.md) |
| AI操作、功能依赖 | [AI模块交互](doc/AI模块页面与遥控交互_20261024.md)、[主机模块](host/README.md)、[协调器](host/ai_coordinator/README.md) |
| 独立规划参考 | [任意角度规划/文件接口](host/inspection_reference/README.md)、[RTL线段检查器](rtl/inspection/README.md)、[设计与三视图](doc/AI自主巡检_20261026/README.md)；保留主分支成果，尚未接入生产导航 |
| 多点视觉巡检 | [使用与架构](doc/AI视觉巡检.md)、[录像与报告](doc/AI巡检录像与报告.md)、[验证记录](doc/AI巡检验证记录.md) |
| 标定 | [去畸变H及独立验证](doc/七点地面单应性标定_20261028.md)、[Windows采图/标定](host/usb_camera_win/README.md)、[H接口](host/ai_coordinator/inspection/HOMOGRAPHY.md) |
| 基础导航 | [避障导航操作](doc/避障导航操作指南.md)、[导航控制](doc/导航控制方案.md)、[Dijkstra验证](doc/Dijkstra导航验证_20261025.md) |
| 遥控、接线与参数 | [HT10A接线](doc/HT10A接线与验收.md)、[约束说明](constr/README.md)、[参数总表](doc/参数总表与待定清单.md)、[遥控/ToF待办](TODO_RC_CAR.md) |
| 构建与上板 | [工具](tools/README.md)、[应用仿真](sim/README.md)、[基础上板操作](doc/上板指南_v1.md)、[手柄独立LED测试](doc/手柄LED上板测试.md) |
| 需求与历史 | [用户原始需求](doc/使用FPGA实现的差速小车功能设计.md)、[实现说明](doc/使用FPGA实现的差速小车功能设计_ai优化版.md)、[工作记录](doc/工作记录.md) |

日期化的发布/验证记录按对应输入阅读；带日期但列在现行入口中的操作文档持续维护。当前操作优先读上表，原始测量、二进制证据和模块例程保留各自范围，不重写历史结果。

## 构建与测试

```powershell
# 全部应用自检；包内单测另按 vendor/ORIGIN.md 运行
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1
# 只检查巡检导航控制接口
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Only inspection_control
# 无ModelSim时可选 -Simulator Icarus

# 整车全量构建；源码改变后不能用旧位流签核
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -GowinSh '<IDE/bin/gw_sh.exe>'
# 引脚核对
python tools/check_pin_constraints.py
# RK主机侧测试（需要已有Python/OpenCV等依赖）
$env:PYTHONPATH = (Join-Path $PWD 'host')
python -m unittest discover -s host/ai_coordinator/tests -v
```

工具路径集中在`tools/lib_paths.ps1`；`build_syn.tcl`只综合，`build_pnr.tcl`只布线。下载使用[工具说明](tools/README.md)中的SRAM操作，`.fs`、波形和本机构建日志不入Git。

## 目录与维护约定

`rtl/`是应用层，`vendor/`是唯一复用实现，`host/`是RK/电脑软件，`sim/`是应用集成自检，`constr/`是整车约束，`tools/`是构建/下载工具，`doc/`存操作、设计和证据。`deps/modules.list`与`scripts/vendor_sources.tcl`管理依赖。

1. 用户最新指令优先；不改原始需求来迁就实现。基础功能、独立例程和AI验收分别记录。
2. 保留无关本地及队友工作。J6按应用映射，不能套用独立模块例程。
3. 修改RTL/约束需跑受影响仿真并核对整车PnR；修改vendor按其[维护约定](vendor/ORIGIN.md)回归。文档整理只做静态核对，不制造新的硬件验收结论。
4. 一次任务一次中文提交，代码与必要文档同提交；提交前拉取当前共享分支。未推送的遗漏用amend，已推送的提交不改写。
5. 分支按当前任务使用`codex/`开发分支；`main`与被排除的队友导航方案不因整理文档自动合入。只在用户明确要求时推送到唯一项目仓库。
6. 根README只作摘要；交接单汇总当前交付与待办，参数总表解释代码/配置来源，验证记录维护日期、范围、结果；旧过程原文查Git，测量文件/证据保持原始日期及SHA，不维护随下一次提交即过期的HEAD/ahead字段。
