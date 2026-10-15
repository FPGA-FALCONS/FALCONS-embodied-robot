# OV5640 DVP 采集模块

`ov5640_capture`封装了 SCCB/I²C 初始化和 8 位 DVP 到 RGB565 的采集过程。默认配置是 1024×600 RGB565，配置时钟为 50 MHz，SCCB 时钟为 400 kHz。

## 公共接口

| 信号 | 方向 | 时钟域 | 含义 |
| --- | --- | --- | --- |
| `cfg_clk` | 输入 | 配置时钟 | 默认 50 MHz，用于初始化摄像头 |
| `reset_n` | 输入 | 异步 | 低有效复位；模块分别在 `cfg_clk`和`camera_pclk`域同步释放内部复位 |
| `init_done` | 输出 | `cfg_clk` | OV5640 寄存器表写入完成 |
| `init_error` | 输出 | `cfg_clk` | 初始化超时后锁存为高，保持到外部复位 |
| `pixel_clk` | 输出 | — | 直接转发 `camera_pclk`，接收端用其上升沿采样 |
| `pixel_valid` | 输出 | `pixel_clk` | `pixel_rgb565`有效；每个像素拉高一个周期 |
| `pixel_rgb565` | 输出 | `pixel_clk` | RGB565 像素 |
| `line_valid` | 输出 | `pixel_clk` | 当前处于有效行 |
| `frame_sync` | 输出 | `pixel_clk` | 与实测工程一致的帧电平，等于采样后 VSYNC 的反相；启动时丢弃前 10 帧 |
| `capture_clear` | 输出 | `pixel_clk` | 首个 VSYNC 到来前为高，可用于清空外部采集 FIFO |
| `pixel_x/y` | 输出 | `pixel_clk` | 从 1 开始的像素/行计数 |

`INIT_TIMEOUT_MS`默认是 1000 ms。摄像头未插入、SDA 被拉死或持续返回 NACK 时，模块停止重复初始化、释放 SCCB/I²C 总线并置位`init_error`。重新拉低`reset_n`可再次初始化。

`CAMERA_PWDN_LEVEL`设置摄像头正常工作期间的 PWDN 输出，默认值 0 与 BY_OV5640_V1.1 实测连接一致。`camera_rst_n`和`camera_pwdn`均由`cfg_clk`寄存器驱动，避免组合比较器直接连接外部控制脚。

模块不会生成 XCLK。配套 BY_OV5640_V1.1 模块带 24 MHz 晶振，直插 ACG720 时不需要 FPGA 提供 XCLK。

把`rtl/`内全部 Verilog 文件加入工程。若将本模块直接作为顶层或顶层沿用相同端口名，可加入`constraints/acg720_by_ov5640_direct.cst`和`constraints/ov5640_capture.sdc`；嵌入其他顶层时应按顶层端口名合并约束。

摄像头 PCLK 与系统/显示时钟异步。连续显示时应使用异步 FIFO 或 DDR 帧缓存跨时钟域。`examples/acg720_ddr3_hdmi/`给出了已经上板验证的 DDR3 帧缓存方案。

## 仿真

`sim/i2c_bit_shift_tb.v`自检 SCCB/I²C 字节发送、从机 ACK/NACK 采样以及`Trans_Done`单周期脉冲。在 ModelSim Transcript 中执行：

```tcl
cd D:/path/to/verilog-reusable-modules/modules/ov5640_capture/sim
do run_test.do
```
