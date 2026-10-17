# RC PWM 模块验证记录

验证日期：2026-10-17，北京时间。生产 RTL 为 Verilog-2001；仿真使用 Icarus，静态检查使用 Verilator 5.053 `-Wall`。

| 检查 | 结果 |
| --- | --- |
| 生产 RTL Verilog-2001 编译 | PASS |
| 生产 RTL / 板级顶层 Verilator lint | PASS，0 warning / 0 error |
| `rc_receiver_tb` | 17 组、502 条断言，0 失败；真实 50 MHz、默认 100 ms 超时 |
| `rc_axis_normalize_tb` | 65536 种输入 × 两种标定，2031717 条检查，0 失败 |
| `rc_receiver_pipeline_tb` | 8 组、241 条检查，0 失败 |
| `tb_rc_debug` | 15 组、175389 条检查、1969 个外部 HC595 锁存字，0 失败 |

核心自检通过 `sim/run_test.ps1` 一键执行，lint 通过 `sim/run_lint.ps1`。板级仿真使用 `examples/acg720_rc_debug/run_debug_test.ps1`；它为缩短耗时使用 4 MHz、15 ms 超时，生产顶层仍为 50 MHz、100 ms。ModelSim `.do` 已提供，本轮未执行 ModelSim。

覆盖典型 1000/1500/2000 μs、独立通道变化、非对称/反向标定、死区边界、合法边界、非法脉宽、卡高/卡低、单路/双路超时、坏脉冲不刷新 watchdog、半脉冲复位、默认超时与恢复。归一化穷举包含整个 16-bit 输入域及宽范围标定 MIN=1/CENTER=12345/MAX=65535/DEADZONE=17/REVERSE=1。

流水线测试核验 27 周期计算及第 28 周期提交；普通新脉冲不导致 valid 每帧闪断；新更新覆盖在途旧快照；失效/超时与完成撞拍、恢复、复位均不能复活旧命令。板级测试从 DS/SH_CP/ST_CP 的物理边沿解码显示字，验证最终命令/原始脉宽、模式切换、LED、失效横杠及恢复，不读取内部 BCD 代替显示链路。

## Gowin 综合和布局布线

工具 Gowin EDA V1.9.12.03；器件 GW5AT-LV138PG484AC1/I0，版本 C。实际构建 `examples/acg720_rc_debug/build.ps1` 完成 synthesis + place & route，包含完整的 `rc_receiver` 命令路径；归一化输出全部 12 位均用于板级观察。

| 项目 | P&R 报告 |
| --- | --- |
| 时钟约束 | 50.000 MHz / 20 ns |
| 报告 Fmax | 83.358 MHz |
| Setup 违例端点 | 0 |
| Hold 违例端点 | 0 |
| Recovery / Removal 已报告最差 slack | +14.367 ns / +0.602 ns |
| 资源 | 1024 LUT + 50 ALU、500 FF、2 MULTALU27X18 |

报告路径：`examples/acg720_rc_debug/build/impl/pnr/project_tr_content.html`，位流 `project.fs` 位于同一目录。Gowin 未发布 recovery/removal 全局违例端点计数字段；以上为对应 worst-path 表的最差值，不将缺失计数解释为 0。SDC 仅含 `create_clock`，未用整段 false path 隐藏内部逻辑。GPIO/SW0 是异步输入，需结合首级同步器检查 CDC；本报告不构成外部电气接口签核。

构建脚本自动执行 `check_timing.ps1`，并生成含源文件、报告、位流 SHA256 的 `build/timing_manifest.json`。检查器对缺表、负 recovery slack、49 MHz Fmax 和 setup 违例四个负向样例均正确拒绝。此结果针对本次板级工程；集成到其他顶层必须重新跑时序。本次未向实板下载新版。

## 已有实物结果和未验证范围

先前用户在 HT-10A 发射机、F-10A 接收机、ACG720 上确认旧捕获/显示例程可读取 CH1–CH4，摇杆变化使数码管变化。用户报告标称 1500/1000 对应显示 1499/999 μs；输出按整数 μs 向下取整，没有人为统一加 1，未测原始波形，不能确定偏差来源。

当时接收机供电为实验性 3.3V，低于标称 4–9V；未测 PWM 高电平。该记录不证明额定供电、电气兼容或当前新版归一化/LED/数码管的硬件功能。新版尚未实测，未驱动电机，未完成射频失控保护或长期运行验收。

`signal_valid` 只说明捕获合法且新鲜并已有归一化命令；接收机射频失联后若继续输出合法保持/保护 PWM，无法据此识别无线失联。整车安全仲裁和急停由上层实现。
