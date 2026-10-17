# SAFE/WARN/STOP 上板例程验证记录

执行日期：2026-10-17；芯片 GW5AT-LV138PG484AC1/I0，版本C。

| 检查 | 结果 |
| --- | --- |
| 默认50MHz生产顶层 Verilog-2001 编译 | PASS |
| Formatter 全65536距离、状态/边界/溢出/复位自检 | 8组，656507检查，0失败 |
| 真实核心 + 物理 I²C 传感器模型 + HC595 引脚解码 | 14组，121301检查、1768模型检查，0失败 |
| 外部显示字/完整扫描帧 | 1376字 / 172帧，逐字核验16位移位及独热位选 |
| Verilator `-Wall` 集成检查 | PASS；仅1个已说明的定向复位结构豁免，0剩余诊断 |
| Gowin EDA V1.9.12.03 综合及P&R | PASS |
| 时钟约束 / 报告Fmax | 50.000MHz / 101.482MHz |
| Setup / Hold 违例端点 | 0 / 0 |
| Recovery / Removal 最差已报告slack | +14.535ns / +0.873ns |
| 资源 | 1919 LUT + 17 ALU、647 FF、0 latch |

Formatter覆盖所有16位距离、99/100/199/200严格阈值、9999/10000显示边界、65535、初始化、非法/陈旧状态、致命故障、快速改变输入和复位。正常距离范围显示SAFE，警告WARN、危险STOP；不使用无效的历史距离判断安全。

完整测试平台使用4MHz仿真时钟，通过实际开漏SCL/SDA/SHUT与现有传感器模型通信；未强制或替换核心内部数据。它从真实DS/SH_CP/ST_CP边沿解码显示，核验300/150/80/99/100/199/200mm、无效样本、stale恢复、设备NACK、S4重新初始化、LED与心跳。正常帧必须一致；失效发生时允许剩余位立即转横杠，随后整帧无旧距离。生产顶层仍是50MHz、100kHz I²C、500μs每位扫描。

集成lint的 `verilator_reset.vlt` 只匹配 `tof_debug_top.sensor.reset_pipe` 在 `tof_vl53l0x.v` 的 SYNCASYNCNET：该同步释放复位既用于同步控制门控，也用作I²C子模块复位，属于有意结构。没有全局屏蔽诊断，没有为了消警修改核心。两个新显示模块各自 `-Wall` 无诊断。

Gowin实际执行 `build.ps1`，退出码0，输出 `TOF_DEBUG_BUILD_FINISHED`，并通过 `check_timing.ps1`。约束仅包含20ns `create_clock`，没有宽泛false path。Recovery/Removal各解析25条worst-path表，负slack均0；工具未提供全局违例端点计数字段，不将缺失字段当0。时序只涵盖该工程内部路径，不替代异步输入CDC或电气兼容评估。

日志：`build/sim/tb_tof_formatter.log`、`build/sim/tb_tof_debug.log`、`build/lint.log`、`build/gowin_build.log`。P&R报告及位流：`build/impl/pnr/project_tr_content.html`、`project.fs`。`build/timing_manifest.json`记录源码、约束、报告和位流SHA256。构建脚本拒绝早于本次构建开始时间的报告；源文件新于位流也会拒绝通过。

J7映射依据用户提供RevI手册第3页，并与官方管脚表交叉核对，指纹见 `pin_sources.json`。随后使用本次位流成功执行SRAM Program，用户确认ToF实际测距和SAFE/WARN/STOP显示已工作，记录见 [../../docs/HARDWARE_TEST.md](../../docs/HARDWARE_TEST.md)。电源/信号幅度尚未用仪表测量，未做精度、边界误差或长期运行验收。**STOP只用于显示和LED，不驱动电机。**
