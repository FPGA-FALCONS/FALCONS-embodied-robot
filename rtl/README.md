# rtl\ —— 应用层 RTL（本工程自己写的代码）

这里只放**本工程特有的应用层**：顶层、控制律、判决逻辑、上报格式。

## 外部依赖不在这里

所有可复用模块（电机驱动、IMU 驱动、偏航角积分、UART…）都从
`vendor\verilog-reusable-modules\modules\<包>\rtl\` 直接参与构建，
**不再往这里拷副本**。

> 这是与 `class\car_fpga` 最大的结构差异：`car_fpga` 用的是"只拷 RTL 副本"（方案里的方式 A），
> 于是有了 `car_fpga\ext\` 四个串口副本、以及"两侧哈希不同实为行尾差异"的手工比对。
> 本工程用**内嵌快照**（2026-10-15 起改为**本项目直接维护**，见 `vendor\ORIGIN.md`）：
> 同一份文件只有一处、不存在"副本与原库漂移"的问题；改了 `vendor\` 里的模块就跑 `sim\run_sim.ps1` 回归。

## 子目录

| 目录 | 放什么 |
| --- | --- |
| `rc\` | 本车手柄配置ht10a_input、输入适配rc_input、手动解锁rc_drive_guard；只复用vendor捕获，不维护副本 |
| `top\` | 顶层与上板测试顶层（管脚进出、按键/LED/串口接线）。`build*.tcl` 的 `-top_module` 指这里的模块 |
| `motor\` | 本工程特有的电机侧逻辑：速度闭环 PID、相位/方向仲裁、未来的位置环 |
| `imu\` | IMU 侧的应用层逻辑（航向保持、直行纠偏等）；纯驱动在 vendor 的 `imu660ra` / `gyro_tilt_comp` / `yaw_integrate` |
| `debug\` | 调试通路：串口命令解析、CSV 上报、状态字 |
| `common\` | 跨模块的小件：复位同步、去抖、跨时钟同步器等 |

## 从 car_fpga 搬代码时的对应关系（L0：速度闭环）

| car_fpga 里的位置 | 本工程目标位置 | 备注 |
| --- | --- | --- |
| `rtl\motor\speed_pid.v` | `rtl\motor\speed_pid.v` | 应用层，原样搬 |
| `rtl\debug\uart_cmd_rx.v` | `rtl\debug\uart_cmd_rx.v` | 应用层，原样搬 |
| `rtl\debug\speed_report.v` | `rtl\debug\speed_report.v` | 应用层，原样搬 |
| `rtl\top\speed_loop_top.v` | `rtl\top\` | 顶层，接线要按本工程的实例化风格调整 |
| `rtl\common\reset_sync.v` | `rtl\common\reset_sync.v` | 应用层，原样搬 |
| `ext\uart_byte_rx.v` 等 4 个副本 | **不搬** | 直接用 vendor 的 `modules\uart`、`modules\sync_fifo` |
| `rtl\motor\motor_top.v` 等 6 个 | **不搬** | 直接用 vendor 的 `modules\motor_tb6612` |
| `rtl\imu\*.v` | **不搬** | 直接用 vendor 的 `modules\imu660ra` 等（L1 起） |

搬的时候注意：`car_fpga` 的应用层模块是**在模块库副本旁边**写的，个别文件可能在参数默认值里
带了本车标定值。搬进本工程前先核对一遍（本车标定值应当写在 `rtl\top\` 的例化参数里，
不要写进 vendor 的模块）。
