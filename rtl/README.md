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
| `top\` | 顶层与上板测试顶层（管脚进出、按键/LED/串口接线）。`build*.tcl` 的 `-top_module` 指这里的模块。★ 当前还含遥控参数覆盖、两段式切页、跨域同步、数值换算 —— 与"顶层只连线"仍有距离（见 `doc\设计落实待办.md` §九 ④）；底盘使能已搬到 `rc\rc_drive_guard.v` |
| `rc\` | 遥控输入三件套：`ht10a_input.v`（**板级配置层**：J6 槽位 ↔ 接收机通道映射 + 全部标定参数）、`rc_input.v`（通用适配层：单路捕获 ×6、每轴标定、两档模式、开关迟滞/帧确认、按钮与菜单事件、**信号有效性掩码**）、`rc_drive_guard.v`（手动解锁守卫：回中解锁一次后保持） |
| `ui\` | 显示与交互：`ui_render.v`（逐像素叠加渲染，无帧缓存；**大字通路 24 px 组只读 ROM、数字通路 16 px 组只读 RAM**）、`num_text.v`（9 字段数值格式化，负号跟随最高位）、`dyn_text_ram.v`（字符缓冲，sys 写 / pixel 读）、`ui_text_index.vh`（**自动生成，勿手改**） |
| `motor\` | 本工程特有的电机侧逻辑：速度闭环 PID、相位/方向仲裁、未来的位置环 |
| `imu\` | IMU 侧的应用层逻辑（航向保持、直行纠偏等）；纯驱动在 vendor 的 `imu660ra` / `gyro_tilt_comp` / `yaw_integrate` |
| `debug\` | 调试通路：串口命令解析、CSV 上报、状态字 |
| `common\` | 跨模块的小件：复位同步、按键去抖、字库与字符串 ROM —— `font_rom.v`（16×16 小字）/ `font_rom_lg.v`（24×24 大字）/ `font_num.v`（16×24 紧凑数字）/ `ui_text_rom.v`，**四者均为自动生成** |
| `cam\` | **摄像头画面叠加层** `cam_overlay.v`（2026-10-17 新增）：把 `vendor` 的 `ov5640_capture` 与 DDR3 帧缓存串起来，并在**输出端二选一**（摄像头页且落在菜单右侧 → 用摄像头像素，否则用 UI 像素）。纯采集驱动在 vendor 的 `ov5640_capture` |
| `ip\` | **Gowin IP 生成文件**（2026-10-17 从 `ov5640_capture` 示例拷入）：`ddr3_ctrl_2port.v` / `fifo_ddr3_adapter.v` / `ddr3_memory_interface.v` / `ddr_pll.v` / `gowin_pll.v` / `rd_data_fifo.v` / `wr_data_fifo.v`。⚠ **勿手改**；换器件必须用 Gowin EDA 对目标器件**重新生成** |

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

## 独立手柄测试（2026-10-17）

`top/rc_led_test_top.v`只例化生产HT10A配置、按键/复位、`rc/rc_led_observer.v`与`debug/rc_probe_report.v`。13个电机输出恒为0，未例化电机或HDMI。只通过`build_rc_led.tcl`选择这个顶层，正常build.tcl仍选择falcons_top。
