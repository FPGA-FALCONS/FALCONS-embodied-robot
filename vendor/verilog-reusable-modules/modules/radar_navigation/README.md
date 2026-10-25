# radar_navigation 自动导航入口

整车例化 `radar_navigation`，读取 M10 GPS 串口和定位，输出目标点任务、紧急停车、任务状态、地图、路线及原始点云。共 36 个正式端口，18 输入、18 输出。内部适配器保留诊断能力，外层不引出诊断接口。

完整接口、运行时前探格数/车宽和编译时规划膨胀参数、坐标单位与方向、初始化、选点、启动、控制器握手、暂停恢复和显示接收方法见 [交接文档](docs/delivery_handoff.md)。来源与验证见 [ORIGIN.md](ORIGIN.md)。

## 工程文件

- [本模块清单](rtl/files.f)：2 个文件，正式入口和内部适配器。
- [导航清单](../auto_navigation/rtl/files.f)：17 个文件。
- [m10_radar.v](../m10_radar/rtl/m10_radar.v) 与 [m10_packet_parser.v](../m10_radar/rtl/m10_packet_parser.v)：2 个公共雷达文件，支持随点保存位姿。
- [uart_byte_rx.v](../uart/rtl/uart_byte_rx.v)：1 个公共 UART 接收器，整车已加入时不重复加入。

共 22 个 RTL，不存在另一套私有雷达或 UART。若通过整车 deps/modules.list 与 scripts/vendor_sources.tcl 接入，应加入 auto_navigation、radar_navigation 和 m10_radar，并复用原有 uart；m10_radar 包的原点云显示文件继续保留，相关显示依赖按该包说明加入。

固定 50 MHz，M10 GPS UART 460800 / 8N1。控制接口为 cmd_valid、target_x_mm、target_y_mm、emergency_stop、ctrl_done、ctrl_linear_active；控制器自行转向和直行，整个目标点任务完成且停稳后确认。地图 256×256、50 mm/格、中心原点，未知允许规划通过。显示端可另取 `target_line_dir`（0 水平、1 正45°、2 竖直、3 负45°），结合有效命令、当前位置和当前目标补画八方向直线，忽略横向偏差；该字段不参与控制器握手。

本次为模块库交付，尚未加入整车顶层或编译包列表。整车后续需连接选点、定位、控制器握手、手动/自动授权及显示，并重新进行集成与上板验证。本交付不含临时文件、仿真测试、测试工具、测量顶层和媒体。

搜索使用直步 2、斜步 3 的四桶 Dijkstra，同代价优先累计转弯少，再保留直行扩展的发现顺序。控制、地图、显示端口和分段锁定规则不变；内部搜索详见 [auto_navigation](../auto_navigation/README.md)。

前探配置输入 `forward_check_cells[7:0]`、`vehicle_width_mm[15:0]` 由整车 UI 在线提供，默认 4 格 / 180 mm；从定位中心点前探，总宽度就是车宽，无前缘偏移或额外横向余量。车宽按 10 mm 调整；每次扫描开始锁存配置，下一次扫描应用修改。
