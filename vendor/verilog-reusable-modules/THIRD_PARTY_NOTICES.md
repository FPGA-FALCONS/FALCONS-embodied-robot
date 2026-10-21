# 第三方代码与生成 IP 说明

本仓库同时保存自行编写的封装、第三方参考代码和 Gowin EDA 生成的器件相关 IP。保留这些文件是为了复现实测工程；使用或再分发前仍需分别确认原作者和工具供应商的许可条件。

## 武汉芯路恒科技有限公司（小梅哥）参考代码

下列文件头声明来源为武汉芯路恒科技有限公司、小梅哥或 `www.corecourse.cn`，原始创建时间为 2019 年：

- `modules/ov5640_capture/rtl/DVP_Capture.v`
- `modules/ov5640_capture/rtl/camera_init.v`
- `modules/ov5640_capture/rtl/i2c_bit_shift.v`
- `modules/ov5640_capture/rtl/i2c_control.v`
- `modules/ov5640_capture/rtl/ov5640_init_table_rgb.v`
- `modules/hdmi_1024x600/rtl/disp_driver.v`
- `modules/hdmi_1024x600/rtl/disp_parameter_cfg.vh`

这些文件来自 ACG720 官方示例工程。本仓库没有发现随源文件提供的明确开源许可证，因此不能仅凭本说明推定获得了再分发或商业使用授权。后续对这些文件的本地修订会保留原文件头，并在提交历史中记录修改。

## Gowin EDA 生成文件

下列文件或目录由 Gowin EDA/IP Core Generator 生成，面向 `GW5AT-LV138PG484AC1/I0`：

- `modules/hdmi_1024x600/rtl/display_pll.v`
- `modules/hdmi_1024x600/rtl/dvi_tx.v`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/ip/ddr_pll/`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/ip/system_pll/`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/ip/ddr3_memory_interface/`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/ip/rd_data_fifo/`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/ip/wr_data_fifo/`

这些文件受 Gowin 软件及 IP 的相应许可条款约束。移植到其他器件时应使用目标器件对应版本的 Gowin EDA 重新生成。

## 本仓库封装代码

以下文件是为本仓库新增的连接和封装层：

- `modules/ov5640_capture/rtl/ov5640_capture.v`
- `modules/hdmi_1024x600/rtl/hdmi_1024x600.v`
- `modules/ov5640_capture/examples/acg720_ddr3_hdmi/rtl/acg720_ov5640_ddr3_hdmi_top.v`

## 已知本地修改

相对参考工程，当前仓库已经包含以下工程适配：

- OV5640 2×9 模块直插 ACG720 2×10 接口的管脚映射。
- 1024×600 MPI7002 显示时序与 49/245 MHz 显示 PLL 配置。
- 摄像头、DDR3 帧缓存和 HDMI 输出的公共封装层。


## M10 雷达封装与公共 UART 恢复

`modules/m10_radar/rtl/` 为新增雷达读取、协议解析、极坐标、位图及界面封装。GPS102 格式与角度/填充规则依据实物串口捕获及厂商 Lslidar 官方驱动核对，来源链接见模块 `docs/protocol_notes.md`。未复制厂商驱动源码。

M10 HDMI 示例直接引用 `modules/hdmi_1024x600/rtl/` 中现有显示时序、PLL 和 DVI 文件，保留本文件列出的原作者说明和 Gowin IP 条款。

`modules/uart/rtl/uart_byte_rx.v` 增加可选 `RECOVERY_EN` 参数：默认 0 保留既有起收和帧错误处理；设为 1 时在复位/帧错误后等待连续 1.5 位高电平并以下降沿起收。M10 读取模块显式启用该模式。接口及原有参数保留，公共 UART 回环与 M10 启动恢复测试均已执行。

## TOF200C / VL53L0X

modules/tof_vl53l0x的初始化/tuning依据Pololu和ST实现转写；来源及指纹见该包docs/SOURCES.md，Pololu/ST许可原文保留在third_party/Pololu_ST_LICENSE.txt。该包在本项目vendor内直接维护，不再向旧源仓库推送。

## 2026-10-21 自动导航与 M10 随点信息

新增 modules/auto_navigation 与 modules/radar_navigation 的 RTL 和交接说明，来源 find_road_test 的 f3809b246ef34e2e9856e3a74baaef891f52ada7。公共 M10 读取与解析增加固定 97 位随点信息，在 UART 接收到距离低字节时保存，随双包距离缓存同步读取；这是导航本地实现，不是厂商驱动代码。公共 UART 接收器与来源相同，未作修改。GPS102 协议与原有第三方来源说明继续适用。
