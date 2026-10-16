# FALCONS-embodied-robot

高云ACG720四驱差速机器人主工程，GW5AT-138B、50MHz。模块库已内嵌到`vendor/verilog-reusable-modules`并在本项目维护，旧源库不再更新。

## 先看哪里

| 目的 | 入口 |
| --- | --- |
| 明天先测试手柄与电平转换，不开电机 | [手柄LED上板测试](doc/手柄LED上板测试.md) |
| 手柄通道、针号、标定与失联限制 | [HT10A接线与验收](doc/HT10A接线与验收.md) |
| 当前审查发现、测试和构建证据 | [2026-10-17审查与上板验证](doc/审查与上板验证_20261017.md) |
| 整车烧录、屏幕、IMU与电机操作 | [整车上板指南](doc/上板指南_v1.md) |
| 功能与交互的最高依据 | [需求原文](doc/使用FPGA实现的差速小车功能设计.md) |
| 实现细节、硬件规划 | [实现说明](doc/使用FPGA实现的差速小车功能设计_ai优化版.md)（冲突时服从需求原文） |
| 参数与遗留任务 | [参数总表](doc/参数总表与待定清单.md)、[设计待办](doc/设计落实待办.md) |
| 历史过程与完整文档索引 | [工作记录](doc/工作记录.md)、[文档索引](doc/README.md) |

## 当前状态（2026-10-17）

- 已实现：中文菜单与数据页、HT10A输入、四路速度闭环、IMU偏航链路、整车12列CSV；这些仍需要整车实物验收。
- 手柄：CH2油门、CH4转向、CH5确认、CH7 select、CH6模式；CH8闲置。只有两个按钮，没有返回键。接收机通道不等于J6针号，接线只查上表文档。
- 新增独立手柄测试顶层：方向/模式LED、半秒事件指示、原始脉宽CSV；电机13个输出恒0。
- 修复：冷启动复位计数器未初始化、合并后Icarus UI测试初始激励丢失。
- 当前Icarus回归：**9个测试台 / 241项检查通过**；整车及独立手柄目标均已重新构建，证据见审查记录。
- 未完成：摄像头、ToF、雷达与导航系统集成；整车顶层集成测试仍待补。测试版不运行这些模块。
- 整车仍存在HDMI跨域/复位时序告警，不能由“已生成位流”推导出全部时序收敛。手柄独立目标的时序与整车分开记录。
- 尚未执行本轮实物测试、烧录或落地。关闭发射机后接收机是否继续输出旧值必须实测。

## 构建与测试

```powershell
# 全部应用/集成自检；工具未加入PATH时先配置Icarus
powershell -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Simulator Icarus
# ModelSim机器使用同一清单，每个测试独立进程
powershell -ExecutionPolicy Bypass -File sim/run_sim.ps1 -ModelSim '<ModelSim win64目录>'

# 主工程：impl/pnr/falcons.fs
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -GowinSh '<IDE/bin/gw_sh.exe>'
# 独立手柄：impl/rc_led_test/impl/pnr/rc_led_test.fs
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_rc_led.tcl -ArtifactDirectory 'impl/rc_led_test/impl/pnr' -GowinSh '<IDE/bin/gw_sh.exe>'

python tools/check_pin_constraints.py
```

下载方法见各上板指南。`.fs`、仿真库、波形和日志都是本地产物，不提交Git；队友拉取后需要本机构建。工具默认路径集中在`tools/lib_paths.ps1`，也可通过脚本参数覆盖，避免将某台电脑路径散写进代码。

## 目录职责

| 目录/文件 | 职责 |
| --- | --- |
| `rtl/top` | 整车及手柄测试顶层 |
| `rtl/rc` | 本车遥控配置、输入适配、解锁与LED观察逻辑 |
| `rtl/motor` | 差速和PID应用层 |
| `rtl/debug` | 整车命令/CSV及独立RC诊断上报 |
| `rtl/ui`、`rtl/common` | UI与字库、复位、按键 |
| `vendor/verilog-reusable-modules` | 可复用模块唯一实现，保留独立包结构 |
| `deps/modules.list` | 主工程依赖包清单 |
| `scripts/vendor_sources.tcl` | 派生源文件和约束清单；测试目标可选择现有依赖的子集 |
| `constr` | 主工程唯一管脚与时序约束；测试约束从这里生成 |
| `sim` | 应用与集成测试；包内单测仍在vendor各包的sim目录 |
| `tools` | 构建、下载、字库生成、引脚检查等工具 |
| `doc` | 需求、执行指南、参数、证据和历史工作记录 |
| `build.tcl` / `build_rc_led.tcl` | 整车 / 独立手柄构建入口 |

## 维护约定

1. 用户明确指令优先。宏观功能与交互以需求原文为准；实现侧不改写需求来迁就代码。
2. 复用模块只维护项目内`vendor`的一份实现；旧源库保持归档。板级取值放应用层，模块保持独立。
3. 改RTL后跑`sim/run_sim.ps1`；改顶层后重新综合与布局布线。改vendor还要跑对应包单测。仿真不能代替实物，生成位流不能代替时序复核。
4. `.cst`与`.sdc`成对使用，引脚逐针核对硬件手册，不按相邻管脚猜测。测试位流与整车位流用途不同。
5. `.ps1`使用UTF-8 BOM与CRLF，其它文本按`.gitattributes`保持LF；读取中文显式指定UTF-8。源码改后必须重建，不使用旧位流冒充新实现。
6. 任务完成包含文档同步：README和工作记录每次更新；参数、接口、测试、操作和目录内容改变时同步对应说明。证据以工具实际输出为准，参数值与状态不在多处复制。
7. 保留已有用户改动和历史。未经授权不做破坏性删除、强制回退或强制推送，不把本地产物加入仓库。
8. 推送需要用户明确授权。本轮用户要求在现有`main`整理、不新建分支；远端是否同步以`git status -sb`为准，提交信息用`git log`查询。

模块来源与维护细节见[vendor/ORIGIN.md](vendor/ORIGIN.md)；过去每轮的资源、时序和测试数量保留在工作记录中，不在首页并排列作“当前值”。
