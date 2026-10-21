# 导航交付来源与验证

- 来源：find_road_test，提交 `f3809b246ef34e2e9856e3a74baaef891f52ada7`。
- 目标模块库：vendor/verilog-reusable-modules/modules；交付日期 2026-10-21。
- 目标仓库在整理交付前已拉取至 `6ba0434`。
- 新增 auto_navigation 的 13 份 RTL、radar_navigation 的 2 份 RTL及清单；RTL 从来源复制；正式入口文件仅调整交接文档路径注释。
- 对应更新公共 m10_radar 中 m10_radar.v、m10_packet_parser.v；原点云显示核心只增加未使用旁带端口的明确绑定。
- 公共 uart_byte_rx.v 与来源逐字节相同，整个 UART 包不修改、不新增文件。
- 交付不复制仿真测试、临时文件、测试工具、测量工程、综合报告或媒体；原模块库已有测试及显示示例保留。
- 原工程已完成 30/30 功能自检及额外参数验证。原交付入口独立综合为 2940 Register / 6356 LUT / 46 BSRAM；内部测量顶层 PnR 为 55.851 MHz，50 MHz setup/hold 违例 0/0。以上不能代替整车集成后验证。

## 交付说明

完整交接文档位于本模块的 [docs/delivery_handoff.md](docs/delivery_handoff.md)。本次按来源 RTL 复制到对应模块目录，尚未接入整车顶层。按用户要求不追加验证，也不交付仿真测试或临时产物。

## 来源说明

M10 读取、解析和 UART 的原始来源是 verilog-reusable-modules，导航新增随点位姿保存功能；没有复制厂商雷达驱动代码。导航及依赖文件未添加新的许可条款，模块库的 [来源说明](../../THIRD_PARTY_NOTICES.md)继续适用。
