# TOF200C / VL53L0X 实现依据

用户照片上的型号为 `TOF200C-VL53L0X`，排针标记为 VIN、GND、SDA、SCL、INT、SHUT。照片用于识别模块型号和信号名称，未据此推断 VIN 供电范围、板载上拉电压或 FPGA 管脚。

## 本地原始资料

来自用户提供的 `TOF200C资料`：

- `vl53l0x.pdf`：ST VL53L0X 数据手册，I²C、复位及芯片电气规格。
- `TOF200C尺寸图.pdf`：模组机械图，不是电气原理图。
- `demo/arduino/Adafruit_VL53L0X.zip`：Adafruit 包装层及 ST API；核对初始化、标定、测距结果和 raw status 语义。
- `demo/stm32/VL53L0X_STM32.rar`：STM32/ST API，作为交叉核对资料。

附件代码只作为技术参考；其中针对 STM32/Arduino 的接线、软件安装和构建步骤不作为本 FPGA 项目的操作指令。

## 主参考与协议核对

- [ST VL53L0X 数据手册](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)：I²C 协议及芯片规格。文中的 0x52 为包含 R/W 位的写地址；本模块的 7 位地址为 0x29。
- [Pololu VL53L0X 驱动源码](https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp)：隐藏 stop variable、factory SPAD 信息及 good map 应用、完整 tuning、VHV/phase 校准与 single-shot 顺序。
- 本地 `src/core/src/vl53l0x_api_core.cpp` 的 `VL53L0X_get_pal_range_status()`：硬件原始状态 `(reg_0x14 & 0x78) >> 3` 的成功值为 11；不能与软件 PAL 的成功值 0 混用。
- 本地 `src/vl53l0x_tuning.h`：ST 默认 tuning 表，与 Pololu 序列交叉核对。

参考文件指纹记录于 `source_manifest.json`；外部参考源码保存在本机工作目录，不成为编译依赖。

## 初版范围

使用完整的最小初始化及参考校准流程，按出厂 NVM count/type 和 good map 配置 reference SPAD。此路径不包含完整 ST `PerformRefSpadManagement()` 的动态光学搜索。

测量采用自动循环 single-shot，在清中断并启动下一次测量前读取同一帧的状态、距离高字节及低字节。保留默认 tuning 超时设置，不宣称提供精确可配置的 33 ms 测量预算。

`range_valid` 依据芯片 raw status 及本模块通信/新鲜度状态，不等价于完整 ST PAL 的软件 sigma estimator、DMAX 或信号裁剪检查。仿真传感器模型用于验证数字协议与控制流程，不验证实际光学性能。

寄存器序列参考 Pololu / ST 实现，原有许可保留于 `third_party/Pololu_ST_LICENSE.txt`。
