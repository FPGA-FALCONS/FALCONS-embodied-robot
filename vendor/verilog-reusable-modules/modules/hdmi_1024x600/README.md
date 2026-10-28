# HDMI 1024×600 输出模块

`hdmi_1024x600`接受 RGB888 像素，产生 1024×600、约 60 Hz 的显示时序和 HDMI 兼容 TMDS 输出。内置 PLL 从 50 MHz 输入产生 49 MHz 像素时钟和 245 MHz 串行时钟。

## 像素接口

| 信号 | 方向 | 时钟域 | 含义 |
| --- | --- | --- | --- |
| `pixel_clk` | 输出 | — | 49 MHz 显示像素时钟 |
| `pixel_req` | 输出 | `pixel_clk` | 为高时模块正在请求有效 RGB888 像素 |
| `pixel_rgb888` | 输入 | `pixel_clk` | 24 位 `{R,G,B}` 像素数据 |
| `pixel_x/y` | 输出 | `pixel_clk` | 当前有效区坐标，有效区外输出 0 |
| `pixel_x_ui/y_ui`、`pixel_x_ai/y_ai` | 输出 | `pixel_clk` | 与原坐标同拍的独立寄存副本，供多渲染器分摊扇出；不使用时可不连接 |
| `frame_start` | 输出 | `pixel_clk` | 每帧一个周期的起始脉冲 |
| `frame_sync` | 输出 | `pixel_clk` | 内部低有效 VS，完整示例用它同步 DDR 读帧；与 HDMI 线上正有效 VS 区分 |
| `video_active` | 输出 | `pixel_clk` | 有效显示区电平，与 `pixel_req`一致 |
| `pll_lock` | 输出 | — | 显示 PLL 锁定状态 |

像素源应在 `pixel_req`有效时提供当前像素。同步 BRAM 可按 `pixel_x/y`寻址；异步摄像头数据必须先经过异步 FIFO 或 DDR 帧缓存。

把`rtl/`内的`.v`文件加入 Gowin 工程；`disp_parameter_cfg.vh`由`disp_driver.v`包含，不要再作为独立 Verilog 源添加。该头文件固定为实测 MPI7002 1024×600 时序：总周期 1312×622，像素时钟 49 MHz。

MPI7002 EDID的同步标志为`0x1C`：水平负极性、垂直正极性。扫描器内部仍产生低有效VS供`frame_sync`与帧起点逻辑使用，仅在DVI发送器的VS输入端反相。原负极性HDMI VS会在该屏上触发与像素序列有关的绿色竖纹；排查与整车验证状态见主工程`doc/AI巡检验证记录.md`。

`display_pll.v`和加密的`dvi_tx.v`面向 GW5AT-LV138PG484AC1/I0；移植到其他器件时需要重生成这两个 IP。ACG720 管脚见`constraints/acg720_hdmi.cst`。

## 仿真

`sim/disp_driver_tb.v`连续检查三帧的总周期、有效像素数、有效区坐标范围和`frame_start`脉宽。在 ModelSim Transcript 中执行：

```tcl
cd D:/path/to/verilog-reusable-modules/modules/hdmi_1024x600/sim
do run_test.do
```
