# ACG720 OV5640 → DDR3 → HDMI 示例

这个工程把`ov5640_capture`和`hdmi_1024x600`组合起来：OV5640 输出的 RGB565 写入板载 DDR3，HDMI 端在独立的 49 MHz 像素时钟域读取，并转换为 RGB888 输出。它与已经上板验证的工程保持相同的寄存器表、DDR3 控制、显示时序和管脚映射。

## 硬件

- ACG720，GW5AT-LV138PG484AC1/I0。
- BY_OV5640_V1.1 2×9 摄像头模块，带 24 MHz 晶振。
- 摄像头直接插在开发板 2×10 接口上：GND/3V3 行对齐，开发板末端 NC/NC 行留空。模块 FLASH 对应 FPGA N20，本工程不驱动该脚。
- HDMI 显示器支持 1024×600 输入。

## 构建

用 Gowin EDA 打开`acg720_ov5640_ddr3_hdmi.gprj`，设置顶层为`acg720_ov5640_ddr3_hdmi_top`并执行综合、布局布线和生成位流。也可在已配置 Gowin 环境的命令行运行：

```powershell
gw_sh build.tcl
```

生成文件在`impl/pnr/acg720_ov5640_ddr3_hdmi.fs`。LED 从高到低依次显示 HDMI PLL 锁定、摄像头初始化完成、DDR3 校准完成和系统 PLL 锁定。

约束文件`constraints/acg720_ov5640_ddr3_hdmi.cst`包含 DDR3、HDMI、摄像头、系统时钟、复位和 LED 的完整 ACG720 管脚映射。
