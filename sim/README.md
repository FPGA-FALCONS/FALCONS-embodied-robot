# 应用层自检与跨模块集成

本目录测试本车应用和真实模块组合，包内独立单测在vendor各包sim中。入口为run_sim.ps1；最近已保存的完整回归/多点接口结果见[巡检验证](../doc/AI巡检验证记录.md)，历史各轮计数不再作为当前清单。

## 运行

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Only inspection_control
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Only inspection_control,inspection_application -Library work_inspection
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Simulator Icarus
```

ModelSim路径可通过-ModelSim覆盖。脚本按每台独立仿真进程执行，避免$finish使后续台静默跳过；-Library选择独立工作库。完整日志在logs/sim_<test>.log，只在屏幕汇总判据。

## 覆盖范围

| 范围 | 检查重点 |
| --- | --- |
| 复位/遥控/底盘 | 冷启动、六路映射、开关/摇杆、回中守卫、手动接管、差速/PID及实时停机 |
| 数值/UI | 生成字模、14字段格式化、页面逐像素扫描、菜单冻结、大字高亮和原页面回归 |
| IMU/航位 | 初始化/校准、角度/坐标口径、采样与里程积分 |
| 基础导航 | 原非避障和避障单点执行、地图/Dijkstra、对角/路线复核、双缓冲及运行参数 |
| 语音运动 | MOT1/ASCII隔离、有界动作、转向、停止所有权和本地恢复 |
| 巡检控制/图层 | session/task/epoch/command、物理令牌、停车/图层版本、授权内连续GOTO、手动撤权 |
| 高速/DDR | AIF1容器、FX2时钟/PHY/回环、跨域FIFO、DDR仲裁/帧银行、地图/路线导出和UI门控 |

实际测试名、顶层与源依赖以脚本清单为准；新增台须加入入口，不能只放源码后声称全量已覆盖。向量生成工具位于sim/gen_inspection_vectors.py及gen_inspection_stream_vectors.py，主机/RTL交叉回放和自造RTL输入分别说明。

## 判定与边界

脚本核对退出码、PASS/FAIL行及已声明CHECKS数；日志里的“0 FAIL”不当作失败。中文日志若因工具编码显示问号，ASCII检查行仍按实际结果判定，不篡改原日志。

这些是软件/仿真证据，不代替整车PnR、实际USB吞吐、模型推理或实车停车/绕障。改vendor需同时遵循[ORIGIN回归约定](../vendor/ORIGIN.md)，改RTL/约束需核对对应输入的实际构建时序。

RK任务队列、H定位和录像测试在host/ai_coordinator/tests，见[协调器入口](../host/ai_coordinator/README.md)；现场已测/待测只在专题验证维护。

## 短段锁定和运行时规划窗口（2026-10-29）

新增 `nav_lock` 台验证真实规划器到控制输出，当前短段终点与最远同向目标分离，后台从前者规划。`w_max` 默认 5，改值当轮锁存；保留全部限长中继。`nav_route` 改为八方向全部短段、任意同向链前看；`nav_planner` 使用生产容量 1024 检查每段最多 4 格，另由 `nav_planner_limit` 检查小容量截断。整定页和数字格式化扩为 15 项，像素回归同步。完整验证与周期见 [验证记录](../doc/短段锁定与运行时规划窗口验证_20261029.md)。
