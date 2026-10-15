# M10 雷达读取与点云显示模块

`m10_radar` 从 M10 的 **460800、8N1** 串口数据流中接收固定 **102 字节 M10_GPS 包**，完成帧格式检查、距离归一化和角度分配，输出带 ready/valid 握手的测量槽。接收入口包含两级输入同步和 UART 错误恢复；双包缓存允许消费端短暂暂停。

同目录的 `m10_point_cloud_core` 在读取模块上增加极坐标转换、双缓冲点云位图、状态文字、四档按键缩放及原始字节串口镜像。它接受外部视频坐标和同步信号，输出 RGB888 及对齐后的 HS/VS/DE。`examples/acg720_m10_hdmi/` 提供 ACG720 的 PLL、HDMI 和完整管脚约束。

## 目录与依赖

```text
m10_radar/
├── rtl/
│   ├── m10_radar.v                 公共读取接口：UART、解析、诊断
│   ├── m10_packet_parser.v         固定 GPS102 格式与双包缓存
│   ├── m10_point_cloud_core.v      读取、缩放、点云与原始串口镜像
│   ├── polar_to_xy.v               毫米距离/角度转屏幕坐标
│   ├── sin_lut.vh                  Q14 正弦查表
│   ├── point_bitmap.v              两幅 512×512 单色位图
│   ├── point_cloud_video.v         RGB、文字、距离环及同步对齐
│   └── zoom_control.v              两键消抖与四档缩放
├── sim/
│   ├── m10_radar_tb.v              公共读取接口自检
│   ├── m10_capture.hex             719 个真实完整包的回放数据
│   ├── *_tb.v                     协议、UART、缩放、坐标、位图、视频自检
│   ├── run_test.do                 ModelSim 交互式完整回归
│   └── run_tests.do                编译入口，供独立进程回归脚本使用
├── constraints/
│   └── m10_radar.sdc              独立读取默认 50 MHz 时序约束模板
├── tools/
│   ├── run_sim.ps1                 每项自检使用独立 ModelSim 进程
│   ├── gen_sin_lut.py              生成正弦查表
│   ├── analyze_m10_dump.py         原始字节包长与字段分析
│   └── capture_radar.ps1           读取 USB 串口镜像并保存原始数据
├── docs/
│   ├── protocol_notes.md           协议及厂商源码依据
│   ├── hardware_notes.md           ACG720 接线与显示时序核对记录
│   └── point_cloud_sim.png         模拟 RGB 输出示意图
└── examples/
    └── acg720_m10_hdmi/            ACG720 1024×600 HDMI 示例
```

读取模块依赖更新后的公共 `../uart/rtl/uart_byte_rx.v`，接收器须支持 `RECOVERY_EN` 参数；显示核心还依赖 `../uart/rtl/uart_byte_tx.v`。这两个文件由模块库的 `uart` 模块统一维护，公共 UART 的 `RECOVERY_EN=0` 默认行为供原有工程使用，`m10_radar` 显式设置为 1。示例使用 `../hdmi_1024x600/rtl/` 中的 `display_pll.v`、`disp_driver.v`、`dvi_tx.v` 及显示时序头文件；HDMI 依赖属于板级示例，独立读取不需要 PLL 或 DVI IP。

把所需 `.v` 加入工程，并将本模块 `rtl/` 加入 include 路径；`sin_lut.vh` 是包含文件，不能作为独立 Verilog 编译单元。示例直接引用公共模块，不另存一套 `vendor` 副本。

## ACG720 RevI 接线

示例接线于 **2026-10-14** 按 `ACG720_引脚接线手册_RevI.md` 的 **Rev I / 028 雷达 TX 修正**映射更新；手册快照日期为 2026-10-11。完整摘要见[硬件记录](docs/hardware_notes.md)，板级约束见[ACG720 示例](examples/acg720_m10_hdmi/README.md)。独立 `m10_radar` 接口本身不绑定某个 FPGA 引脚。

| RevI 028 雷达接口 | 连接 |
| --- | --- |
| J3.1，M10 VCC | 5 V，对应 J1.30；独立供电或确认容量后使用板上电源 |
| J3.2，M10 GND | GND，对应同网的 J1.11 / J1.29 |
| J3.3，M10 RX | NC，留空 |
| J3.4，M10 TX | **C14（J1.24）**，FPGA 输入，460800 8N1 |
| J3.5 / J3.6 | NC，留空 |

**A15 为 RevI J2.6 的 IMU CS（J1.38），不能连接雷达。** 027 及更早已打板版本把 J3.3 连到 C14，使用前需核对实物并纠正为雷达第 4 针 TX 接 C14、第 3 针 RX 留空，不能直接按 028 接口插接。S0 F15、S1 A20、S2 B20、HDMI 与 USB 串口 M15 保持原分配。

M10 供电要求为 4.75～5.25 V、约 400 mA，表列最大 450 mA；手册原全板 **5 V / 50 mA** 预算不涵盖此负载。应独立供电或先核算主板供电、转接板走线及连接器容量。**J3.1 与 J1.30 的 5 V 直连**，独立供电时须在接线中隔离雷达 VCC 与这条直通路径，再由外置 5 V 单供雷达、两套电源仅共地；TX 仍接 C14，不能把两个独立 5 V 输出直接并接。GPIO0 bank 为 3.3 V，TX 电平须适配 FPGA 输入。本次不修改电源或 PCB 设计。旧 A15 直连实板记录属于历史配置，RevI C14 的构建及上板结果另见示例验证记录。

## 公共读取接口

`rtl/m10_radar.v` 全部输出均属于 `clk` 域。调用方应把低有效 `reset_n` 同步释放到该时钟；`radar_rx` 在模块内经过两级同步。

| 信号 | 方向 / 位宽 | 含义 |
| --- | --- | --- |
| `clk` | 输入 | 系统时钟，频率须与参数一致 |
| `reset_n` | 输入 | 低有效复位，清除接收状态、缓存和诊断 |
| `radar_rx` | 输入，异步 | 雷达 TX，空闲高，固定 460800 8N1 |
| `point_valid` / `point_ready` | 输出 / 输入 | 测量槽 ready/valid 握手 |
| `point_angle_cdeg` | 输出，16 位 | 角度，单位 0.01°，范围 0…35999 |
| `point_distance_mm` | 输出，16 位 | 归一化后的毫米距离；FFFF 为填充，0 为无有效距离 |
| `point_index` | 输出，6 位 | 原始包内槽索引 0…41 |
| `packet_valid` | 输出 | 完整格式检查通过并成功入队，单周期脉冲 |
| `packet_error` / `packet_overflow` | 输出 | 错误事件 / 双包缓存满而丢包事件，单周期脉冲 |
| `packet_error_reason` | 输出，3 位 | 最近解析错误原因，保持至下一错误或复位 |
| `packet_speed` | 输出，16 位 | 最近成功入队包的原始 speed 计时字段 |
| `rx_data` / `rx_valid` / `rx_error` | 输出，8 位 / 1 位 / 1 位 | 原始解码字节及单周期字节有效/接收错误事件，不经过包过滤 |
| `data_active` | 输出 | 最近一秒内有成功入队的有效包 |
| `error_sticky` / `overflow_sticky` | 输出 | 错误 / 溢出锁存状态，复位清除 |
| `packet_count` | 输出，32 位 | 成功入队包数，循环计数 |
| `rx_byte_count` | 输出，16 位 | 有效 UART 字节数，循环计数 |
| `uart_error_count` / `bad_packet_count` | 输出，各 16 位 | UART 错误 / 解析错误事件数，饱和至 FFFF |
| `last_rx_byte` | 输出，8 位 | 最近成功解码的字节 |
| `last_error_reason` | 输出，3 位 | 最近错误原因，供状态显示 |

`CLK_FREQ_HZ` 默认 **50000000**。UART 波特率固定 460800，解析字节间超时为 10 ms；修改系统时钟时必须同步修改此参数和时序约束。

```verilog
m10_radar #(.CLK_FREQ_HZ(50_000_000)) u_radar (
    .clk(clk), .reset_n(reset_n), .radar_rx(radar_tx),
    .point_valid(point_valid), .point_ready(point_ready),
    .point_angle_cdeg(angle_cdeg), .point_distance_mm(distance_mm),
    .point_index(point_index), .packet_valid(packet_valid),
    .packet_error(packet_error), .packet_overflow(packet_overflow)
);
```

`point_valid && point_ready` 时消费一个槽。背压时，角度、距离和索引保持不变。每个提交包输出 42 个槽，包括零距离及填充；消费方按用途过滤 0/FFFF，不能通过删槽重新排列角度。原始字节输出没有 ready 输入，消费方须及时接收每个 `rx_valid` 脉冲。

双包缓存满时丢弃新整包，不覆盖待消费数据。UART 数据流无法用此 ready 信号暂停；长时间背压仍会丢包并记录 OVERFLOW。`packet_valid` 只表示成功入队，不表示这一包已由下游消费。

## 固定 GPS102 协议

| 包内偏移 | 内容 |
| --- | --- |
| 0…1 | A5 5A 帧头 |
| 2…3 | 起始角，高字节在前，单位 0.01° |
| 4…5 | speed，原始旋转计时量 |
| 6…89 | 42 个高字节在前的距离槽 |
| 90…99 | 10 字节 GPS 时间扩展：年月日时分秒、毫秒、微秒余量 |
| 100…101 | FA FB 帧尾 |

时间字段会被接收并跳过，点云不要求 GPS 授时或连接 PPS/REC。90…99 不是 checksum；本协议没有 CRC。完整长度、帧尾和角度范围检查通过后才提交点，这些检查不能识别仍满足格式的内容损坏。

原始 FFFF 保留为填充，其他距离清除 bit15 强度标志。若原始索引 39 为 FFFF，或索引 40 的原始高字节大于等于 F5，则索引 40 也按 FFFF 处理。令 N 为处理后非 FFFF 槽数，0 仍计入 N；索引 i 的角度为 `(start + floor(i*1500/N)) mod 36000`。N=0 时输出填充槽，不生成有效显示点。

本次雷达实测 719 包全部为 102 字节，每包 40 或 41 个测量，FFFF 均为尾部填充。所附早期资料的 92 字节帧曾导致持续 TAIL；模块现固定为实物确认的 GPS102。规则依据厂商 [ROS1 驱动 commit 7b586f1f2439adf3dfba1c882764b1322625ee8e](https://github.com/Lslidar/Lslidar_ROS1_driver/blob/7b586f1f2439adf3dfba1c882764b1322625ee8e/lslidar_driver/src/lslidar_driver.cc)，完整统计见 `docs/protocol_notes.md`。

## 诊断与 UART 恢复

错误原因：0=无错误、1=UART、2=TAIL、3=ANGLE、4=TIMEOUT、5=OVERFLOW。有效包不会清除历史错误，`bad_packet_count` 包含 UART 错误及溢出事件，与 `uart_error_count` 有重叠；不要把两者相加当成独立错误总数。

`m10_radar` 通过公共 UART 的 `RECOVERY_EN=1` 启用恢复策略：在复位或帧错误后等待连续 1.5 位高电平，再以下降沿识别起始位；正常收包允许只有一个停止位的连续字节。持续低电平 break 只记录一次停止位错误，然后等待恢复。复位落在连续数据中间时可能丢弃启动的不完整包并记录历史 UART 错误，判断当前链路应同时观察有效包和错误计数是否继续增加。

## 公共点云显示接口

`rtl/m10_point_cloud_core.v` 接收 `clk/reset_n/radar_rx`、低有效 `zoom_in_n/zoom_out_n`、`video_x/y[11:0]`、`video_de/hs/vs` 和单周期 `frame_start`。它输出 `rgb888[23:0]`、三拍对齐的 `out_de/hs/vs`、921600 8N1 原始字节镜像 `debug_uart_tx`，以及 `data_active/error_sticky/overflow_sticky/front_points[15:0]/bitmap_ready` 状态。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `CLK_FREQ_HZ` | 49000000 | 核心及视频坐标所在时钟域 |
| `KEY_DEBOUNCE_CLKS` | `CLK_FREQ_HZ/50` | 按键消抖 20 ms |
| `REFRESH_FRAMES` | 12 | 位图积累窗口，约 60 Hz 显示时为 200 ms |

绘图区为中心 `(512,300)` 的 512×512 位图，0°向上、90°向右、顺时针增加。两幅位图每 bank 为 8192×32 bit，使用同步 RAM 读及读改写；清屏为 8192 拍，49 MHz 下约 167 µs。视频帧起点交换前后台。显示核心需要 RAM 推断和乘法运算支持，移植后应检查 RAM/DSP 映射及 RGB 与视频控制的流水对齐。

放大/缩小键经过两级同步和消抖，每次按下切换一档，长按只触发一次，上下限保持，同时按下忽略。档位 0…3 为 10/20/40/80 mm/像素，半径量程 2.5/5/10/20 m；复位默认档位 2。缩放请求在 `frame_start` 生效，只清极坐标流水和双位图；读取模块、包队列、诊断和视频时序继续运行。

## 仿真

在本模块目录执行：

```powershell
& .\tools\run_sim.ps1
```

可用 `-ModelSimBin` 指定工具目录。交互式 ModelSim 在 `sim/` 执行 `do run_test.do`。测试脚本检查进程退出码、失败文本及每项 PASS 标记。

| 自检 | 覆盖 |
| --- | --- |
| `m10_radar_tb` | 公共读取接口、测量握手和诊断输出 |
| `uart_rx_recovery_tb` | 启动位相、波特率 ±1%、连续字节及 break 恢复 |
| `m10_gps_packet_parser_tb` | 固定 GPS102、719 个真实包回放、填充/强度、角度、坏包和背压 |
| `zoom_control_tb` | 消抖、长按一次、帧边界生效、上下限和同时按键 |
| `polar_to_xy_tb` | 四档坐标/量程、四象限、过滤和背压 |
| `point_bitmap_tb` | RAM 同步读、读改写、清屏和前后台交换 |
| `point_cloud_video_tb` | RGB、诊断文字、动态比例及控制信号对齐 |
| `m10_point_cloud_core_tb` | 串行输入到整帧 RGB、错误恢复、缩放期间收包及原始转发 |

`docs/point_cloud_sim.png` 是模拟 RGB 输出，不是实机雷达截图。封装后的回归、器件重建和时序结果以示例 `docs/validation.md` 为准。

## 单时钟与移植

独立读取接口和显示核心使用一个 `clk`，UART 引脚及按键输入在各自入口同步。它们不自动跨域传送点流；若消费端使用另一个时钟，应在输出处加入异步 FIFO。独立读取的时钟与 HDMI 无关；使用显示核心时，核心时钟必须与视频坐标/控制所在时钟一致。

`constraints/m10_radar.sdc` 是独立读取默认 50 MHz 的模板，约束周期为 20 ns，仅豁免外部异步 RX 到同步入口及外部复位路径。嵌入其他顶层时须按端口/实例名称合并并检查约束，修改时钟时同步调整周期；ACG720 HDMI 示例使用自身完整约束，不同时加载这份 50 MHz 模板。

读取和点云 RTL 不包含器件专用 PLL/DVI IP。ACG720 示例的 49/245 MHz PLL、加密 DVI 和管脚映射属于 GW5AT-138B，换器件时需重生成对应 IP 并重做约束；换分辨率时还需调整固定显示时序和绘图区布局。完整构建应检查 setup、hold、recovery、removal 和 TNS，尤其保留 HDMI 串行器复位时序检查。
