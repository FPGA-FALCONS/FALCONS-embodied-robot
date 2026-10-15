# sim\ —— 本工程的仿真（跨模块集成 + 应用层）

## 这里放什么

只放**跨模块的集成测试**与**应用层测试**，例如：

- 速度闭环：`speed_pid` + `vendor\motor_tb6612` 的 `motor_top` + 串口命令/上报，
  用电机对象模型（`motor_plant_model.v`）闭环验证收敛/阶跃/带载/饱和；
- 未来的 IMU + 速度环联调（直行纠偏）。

**单个模块的单元测试不在这里** —— 它们跟着模块住在
`vendor\verilog-reusable-modules\modules\<包>\sim\`，用 `tools\run_all_modules.ps1` 跑。
把单元测试复制到 `sim\` 会让"库自带测试当门槛"这条失效（副本会与库漂移）。

## 两条踩过的坑（写脚本时照做）

1. **ModelSim 在 `-c`（命令行）模式下，tb 里的 `$finish` 会终结整个 vsim 进程。**
   所以不能把多个 tb 串在同一个 `do` 脚本里跑 —— 只有第一个会执行，后面静默跳过且不报错。
   命令行请给**每个测试开一个独立进程**（`vsim -c -do "run -all; quit -f" work.<tb>`），
   参考 `class\car_fpga\sim\run_speed.ps1` 的结构。
   `do` 脚本里的 `vsim work.xxx / run -all / quit -sim` 内建写法只适用于 **GUI Transcript**。
2. **测试平台自身的错误会让"通过"失去意义。**
   曾出现：`wait_ticks` 早返回一个节拍导致期望值差 1；清零用例后多等一拍导致期望 0 实测 8；
   tb 里把有符号量声明成 unsigned 导致打印成 4294885376。
   **"改了 RTL 之后 tb 的检查值变了"本身就是信号**，要停下来查清楚是谁错了。

## 本工程现有的测试（2026-10-15）

| 文件 | 覆盖 | 检查项 |
| --- | --- | --- |
| `rc_input_tb.v` | 遥控 6 路归一化 / 死区 / 模式三档 / 按钮 / 丢信号 | 12 |
| `chassis_ctrl_tb.v` | 差速分解 / 限幅 / 安全逻辑 / 闭环方向 / rpm 换算 | 23 |
| `num_text_tb.v` | 实时数值格式化 | 22 |
| `ui_render_tb.v` | 中文界面（菜单/高亮/状态栏/动态数值/占位页） | 8 |
| `debug_rc_tb.v` | 最小调试台：单个 `rc_pwm_capture`（排查多路例化问题时用） | — |
| `run_sim.ps1` | 命令行批量入口：每测试独立 vsim 进程 + PASS/FAIL 汇总 | — |

```powershell
powershell -File sim\run_sim.ps1
```

## 命名

| 文件 | 用途 |
| --- | --- |
| `run_<集成名>.ps1` | 命令行跑该集成测试（每个 tb 独立进程 + PASS/FAIL 汇总表） |
| `<模块>_tb.v` | 测试平台本体 |
| `<对象>_model.v` | 行为级模型（如电机对象模型），不要放进 `rtl\` |

日志与仿真库（`work*\`、`transcript`、`*.log`）都被 `.gitignore` 忽略，不会进仓库。
