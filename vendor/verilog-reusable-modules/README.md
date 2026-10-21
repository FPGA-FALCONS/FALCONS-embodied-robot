# FPGA 可复用模块

本目录收集可以在不同FPGA工程中复用的RTL模块、ModelSim测试和最小上板示例。

## `sync_fifo`

单时钟同步 FIFO 模块，用于在同一个时钟域内缓存数据，协调数据生产者和消费者速度不一致的问题。

- `sync_fifo/rtl/sync_fifo.v`：同步FIFO的可综合实现。
- `sync_fifo/sim/sync_fifo_tb.v`：FIFO写满、读空和数据顺序测试。
- `sync_fifo/sim/run_test.do`：ModelSim自动编译与运行脚本。

该模块的读写两侧必须使用同一个时钟。跨时钟域传输应使用异步 FIFO。

## `uart`

UART收发模块包。带缓冲队列依赖公共的`sync_fifo/rtl/sync_fifo.v`，UART目录不再保存FIFO副本。

- `uart/rtl/uart_byte_tx.v`：基础UART字节发送器。
- `uart/rtl/uart_byte_rx.v`：基础UART字节接收器。
- `uart/rtl/uart_tx_queue.v`：使用公共FIFO的UART发送队列。
- `uart/rtl/uart_rx_queue.v`：使用公共FIFO的UART接收队列。
- `uart/sim/uart_queue_loopback_tb.v`：UART收发队列回环测试。
- `uart/sim/run_test.do`：ModelSim自动编译与运行脚本。
- `uart/examples/`：可直接打开的ACX720串口回显上板示例。

## 模块关系

```text
用户发送数据
    ↓
uart_tx_queue
    ├── sync_fifo
    └── uart_byte_tx
            ↓
          UART TX

          UART RX
            ↓
uart_rx_queue
    ├── uart_byte_rx
    └── sync_fifo
            ↓
用户读取数据
```

只需要单字节收发时，可以直接使用`uart_byte_tx`或`uart_byte_rx`；需要连续收发或临时缓存数据时，优先使用`uart_tx_queue`和`uart_rx_queue`，并同时把公共`sync_fifo.v`加入工程。


## `imu660ra`

逐飞 IMU660RA 模块（内部芯片 Bosch BMI270）六轴驱动。上电自动完成初始化：读 CHIP_ID → 关闭高级省电 → 灌入 8192 字节官方 config file → 校验 INTERNAL_STATUS → 配置量程/ODR → 使能；之后按周期 burst 读取六轴数据并输出物理量。

- `imu660ra/rtl/imu_top.v`：驱动顶层，输出角速度（0.01°/s，int32）与加速度（mg，int16）。
- `imu660ra/rtl/spi_master_mode0.v`：通用 SPI 主机（字节级），可复用于其它 SPI 外设。
- `imu660ra/rtl/bmi270_cfg_rom.v`：8192 字节配置 ROM，数据已内嵌，由 `tools/gen_cfg_rom.py` 生成。
- `imu660ra/sim/`：SPI 引擎与初始化流程测试（含 BMI270 SPI 行为模型）。
- `imu660ra/constraints/`：ACG720 管脚与时序约束（引脚按硬件手册 **J2 表**逐针抄录）。
- `imu660ra/examples/acg720_imu_uart/`：读六轴并经串口输出 CSV 的最小上板示例。

该模块**不依赖其它任何模块**，可单独加入工程。接口为 SPI Mode0；实测杜邦线连接下 SCK 建议 1~5 MHz（8.33 MHz 会导致 8 KB 配置加载失败）。

输出两个有效脉冲：`o_data_valid`（**仅 RUN 态产生**）与 `o_acc_valid`（**标定态与 RUN 态都产生**）。两者语义不同，下游按需选择，详见模块 README。

## `gyro_tilt_comp`

陀螺 Z 轴「固定安装倾斜」补偿。静止时用加速度计测出竖直轴在本体坐标系中的方向，算出增益 `k = |a_B| / a_z = 1 / cos α`，用它把角速度换算到竖直轴方向，消除偏航积分的 `cos α` 标度误差（倾斜 30° 差 13.4%，45° 差 29.3%，90° 完全测不到）。

- `gyro_tilt_comp/rtl/gyro_tilt_comp.v`：单文件，无子模块。标定全自动，复用上游零偏标定的静止窗口，运行期只需**一次乘法**。
- `gyro_tilt_comp/sim/gyro_tilt_comp_tb.v`：单元测试 9 个用例（增益计算/倒置符号/超范围限幅/样本不足退化）。
- `gyro_tilt_comp/sim/tilt_integration_tb.v`：★ 与 `imu660ra` 的**集成测试**，覆盖"标定窗口内能否采到加速度样本"这一跨模块契约。
- `gyro_tilt_comp/sim/run_test.do`、`run_integration.do`：ModelSim 脚本。

`k` **自动带符号**（IMU 倒置安装时符号自动翻回），不需要安装方向配置位。**只补偿固定倾斜**：载体动态侧倾/俯仰仍有残差，且要求旋转轴竖直。它**不解决 yaw 的长期漂移** —— 重力对偏航无约束，那是原理性限制。

## `yaw_integrate`

把陀螺 Z 轴角速度积分成偏航角，输出 ±180° 回绕角（int32，单位 0.01°）。纯定点、**零乘法器**、无浮点。

- `yaw_integrate/rtl/yaw_integrate.v`：单文件，无子模块。
- `yaw_integrate/sim/yaw_integrate_tb.v`：单元测试 11 个用例（积分标度/回绕/死区边界/ZOH 锁存/清零/冻结）。
- `yaw_integrate/sim/run_test.do`：ModelSim 脚本。

积分节拍取 **1024 Hz**（而不是 1000 Hz），使 `dt = 2⁻¹⁰ s` 恰好是 2 的幂，于是每周期增量**就是输入的 `gz` 本身**：

```verilog
acc <= acc + gz;        // 零乘法、零除法、零截断误差
```

改用 1000 Hz 则每拍要做 `gz × 65.536` 的定点乘法，既占 DSP 又引入截断误差。内置静止死区，可显著压住静止漂移。

积分时基与上游采样节拍**完全解耦**（零阶保持），因此上游改采样率不会破坏积分正确性。

## 三者的配合

```text
imu660ra.o_gyro_z  ──┐
                     ├──▶ gyro_tilt_comp ──▶ yaw_integrate ──▶ o_yaw
imu660ra.o_acc_xyz ──┘          ▲   ▲
                                │   └──── imu660ra.o_calib_busy
                                └──────── imu660ra.o_acc_valid
```

连接要点：`gyro_tilt_comp` 的 `i_sample_valid` 必须接 `imu660ra` 的 **`o_acc_valid`**（**不是** `o_data_valid`），`i_calib_start` 接 `o_calib_busy`。

> 这三者的完整上板链路（六轴 + 倾斜补偿 + 偏航角 + 串口 CSV 输出）已在 ACG720 实板验证通过。

## `motor_tb6612`

面向 TB6612FNG 类双/四路电机驱动板的完整电机控制模块包，提供「给占空比 → 电机转 → 读编码器 → 算转速 → 卡住时自动保护」的全链路。纯定点、无浮点、无厂商 IP。

- `motor_tb6612/rtl/motor_pwm.v`：单路 TB6612 三线驱动（占空比 / 方向 / 刹车 / 使能）。
- `motor_tb6612/rtl/quad_decoder.v`：正交编码器解码（两级同步 + 数字去毛刺 + 4 倍频 + 32 位位置计数）。
- `motor_tb6612/rtl/speed_measure.v`：定窗口测速（M 法）。
- `motor_tb6612/rtl/stall_detect.v`：★ 堵转检测与保护（**净位移判据**，锁存停机）。
- `motor_tb6612/rtl/motor_unit.v`：单路完整封装（上四者合一，开箱即用）。
- `motor_tb6612/rtl/motor_top.v`：四路集成 + STBY 门控。
- `motor_tb6612/sim/`：5 个测试平台 + `run_test.do`（GUI）/ `run_motor.ps1`（命令行）。
- `motor_tb6612/constraints/`：ACG720 管脚约束（21 根电机/编码器信号逐针抄自硬件手册 J4/J5 表）。

**分层而非合并**：4 个原子模块可单独复用、单独替换（换驱动芯片只改 `motor_pwm`，换编码器类型只改 `quad_decoder`）；`motor_unit` 只是把手动拼装的内联接线收敛到一个文件，避免跨模块契约（`delta` / `delta_valid` / 使能通路）在顶层接错。

**★ 堵转判据用「净位移」而不是「转速为 0」**：60000 计数/转时 1° = 167 计数，而堵转时电机的机械抖动与低速正常转动的单窗口增量**在同一量级**，单窗口阈值无法区分（取小则漏报、取大则误报）。累加 `delta` 后振动来回抵消、转动单向累积，判据变成"连续 200ms 内净位移不足 300 计数（≈1.8°）即判堵转"，两侧都有 10 倍以上余量。

**极性需按硬件自行标定**：`INV_DIR_VEC`（电机方向）与 `ENC_INV_DIR_VEC`（编码器计数方向）是两个**正交**参数，默认全 0。左右电机镜像安装时两侧计数符号天然相反，必须把其中一侧反相。**先校电机方向、再校编码器符号，顺序不能反**（反电机会同时翻转转向与符号）。

已在 ACG720 + WHEELTEC D24A + MG513P30_12V + GMR 500PPR 上完成实板验证：四路驱动、编码器测速与极性统一、堵转保护（锁存停机）、上电不窜车。

## `m10_radar`

M10 雷达串口读取及点云显示模块，固定使用实物确认的 **102 字节 M10_GPS** 格式，460800 / 8N1。输出角度（0.01°）、距离（mm）及槽索引，提供 valid/ready 背压、双包缓存、错误原因和诊断计数。

- [`m10_radar/rtl/m10_radar.v`](modules/m10_radar/rtl/m10_radar.v)：独立读取封装，仅依赖公共 UART RX 与本模块解析器。
- [`m10_radar/rtl/m10_point_cloud_core.v`](modules/m10_radar/rtl/m10_point_cloud_core.v)：读取、极坐标转换、双位图、诊断和四档缩放显示核心。
- [`m10_radar/sim/`](modules/m10_radar/sim/)：八项自检，含 719 个真实包回放、480 个 UART 启动位相及缩放期间持续接收。
- [`m10_radar/examples/acg720_m10_hdmi/`](modules/m10_radar/examples/acg720_m10_hdmi/)：ACG720 / GW5AT-138B 读取并显示 HDMI 1024×600 点云的完整工程。
- [`m10_radar/README.md`](modules/m10_radar/README.md)：公共接口、协议、诊断、移植和仿真说明。

示例引用公共 `uart`、`hdmi_1024x600` 文件，S1 放大、S2 缩小，半径量程 2.5/5/10/20 m，默认 10 m。公共 UART 新增 `RECOVERY_EN` 参数，默认 0 保留既有接收方式；雷达显式启用 1，以提供启动及帧错误恢复。

## `rc_pwm_receiver`

航模遥控接收机 RC PWM 输入模块包。输入两路异步 PWM，输出有符号 `steering`、`throttle`（−1000…+1000）和 `signal_valid`；支持独立校准、中位死区、方向反转、非法脉宽过滤及超时归零。

- [`rc_pwm_receiver/rtl/rc_pwm_capture.v`](modules/rc_pwm_receiver/rtl/rc_pwm_capture.v)：单路脉宽捕获，两级同步、完整边沿计数、有效状态和更新事件；可独立实例化增加通道。
- [`rc_pwm_receiver/rtl/rc_receiver.v`](modules/rc_pwm_receiver/rtl/rc_receiver.v)：双通道命令封装，只依赖包内捕获模块。
- [`rc_pwm_receiver/sim/`](modules/rc_pwm_receiver/sim/)：17 组自检，提供 ModelSim 与 Icarus 入口。
- [`rc_pwm_receiver/README.md`](modules/rc_pwm_receiver/README.md)：接口、参数、实例化代码、通道映射和标定说明。
- [`rc_pwm_receiver/VERIFICATION.md`](modules/rc_pwm_receiver/VERIFICATION.md)：仿真及 HT-10A/F-10A → ACG720 实物 PWM 捕获测试记录。

生产 RTL 为 Verilog-2001，无厂商 IP、无其他公共模块依赖。两路默认逻辑角色为转向/油门，物理接收机通道由主工程接线决定；例如接 CH4 转向、CH2 油门。任一路无效时两路命令均输出 0。

实物已观察到接收机 CH1–CH4 PWM 可被捕获并使数码管随摇杆变化；正常供电/信号电压仍需确认，组合归一化路径需在主工程中完成时序检查。接收机持续输出合法失控保护脉冲时，`signal_valid` 不等于射频在线。

## `ov5640_capture`

面向并行 DVP OV5640 的初始化与 RGB565 采集模块。封装层输出像素时钟、像素有效、RGB565、行/帧状态和像素坐标。

- `ov5640_capture/rtl/ov5640_capture.v`：公共封装接口。
- `ov5640_capture/constraints/`：ACG720 + BY_OV5640_V1.1 直插方式的摄像头管脚与 PCLK 时序约束。
- `ov5640_capture/sim/`：SCCB/I²C 字节发送、ACK/NACK 和完成脉冲自检。
- `ov5640_capture/examples/acg720_ddr3_hdmi/`：摄像头经 DDR3 帧缓存输出到 HDMI 的完整 Gowin 工程，已在实板验证。

## `hdmi_1024x600`

固定 1024×600、约 60 Hz 的 HDMI/DVI 输出模块。模块产生 49 MHz 像素时钟和 245 MHz 串行时钟，向外请求 RGB888 像素，并输出 TMDS 差分信号。

- `hdmi_1024x600/rtl/hdmi_1024x600.v`：公共封装接口。
- `hdmi_1024x600/constraints/`：ACG720 板载 HDMI 管脚与输入时钟约束。
- `hdmi_1024x600/sim/`：1024×600 帧周期、有效像素数、坐标范围和帧起始脉冲自检。

这两个模块中的 PLL、DDR3 与 DVI TX 文件面向 GW5AT-LV138PG484AC1/I0。移植到其他 FPGA 时需要替换相应厂商 IP 和管脚约束。

## 第三方代码

摄像头、显示时序和 Gowin IP 中包含第三方来源文件。来源、修改状态和许可注意事项见[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

## 运行ModelSim测试

在ModelSim Transcript中先进入相应`sim`目录，再执行：

```tcl
cd D:/path/to/verilog-reusable-modules/modules/sync_fifo/sim
do run_test.do
```

其他测试把路径换成以下目录之一：

- `modules/uart/sim`
- `modules/ov5640_capture/sim`
- `modules/hdmi_1024x600/sim`
- `modules/m10_radar/sim`
- `modules/imu660ra/sim`
- `modules/gyro_tilt_comp/sim`（`run_test.do` 跑单元测试；`run_integration.do` 跑与 `imu660ra` 的集成测试）
- `modules/yaw_integrate/sim`
- `modules/motor_tb6612/sim`（5 个测试平台；**命令行请用 `run_motor.ps1`**，会汇总 PASS/FAIL）
- `modules/rc_pwm_receiver/sim`（`run_test.do`；Icarus 命令行可用 `run_test.ps1`）

> ⚠️ 这些脚本用的是 `vsim work.xxx / run -all / quit -sim` 的**内建写法**，只适用于 ModelSim **GUI 的 Transcript**。
> 在 `-c`（命令行）模式下，测试平台里的 `$finish` 会**终结整个 vsim 进程** —— 因此
> **不能**把多个测试串在同一个 do 脚本里跑（那样只有第一个会执行，后面静默跳过、且不报错）。
> 命令行下请给每个测试开独立进程：
>
> ```
> vsim -c -do "run -all; quit -f" work.<测试平台名>
> ```

## 2026-10-18 RC / ToF补充

- [RC新版核心与观察例程](modules/rc_pwm_receiver/README.md)：三文件归一化核心、四轴10秒轮换和CH5/6独立LED。
- [TOF200C/VL53L0X](modules/tof_vl53l0x/README.md)：独立驱动、测试及SAFE/WARN/STOP板级例程。

本vendor的唯一维护仓库为FALCONS-embodied-robot；应用接线映射以工程doc/HT10A接线与验收.md为准，不能照搬独立观察例程。

## 自动导航

新增 [radar_navigation](modules/radar_navigation/README.md) 正式入口与 [auto_navigation](modules/auto_navigation/README.md) 内部模块；直接复用本库更新后的 m10_radar 和原有 uart。完整接口及使用流程见 [导航交接文档](modules/radar_navigation/docs/delivery_handoff.md)。本次仅交付 RTL 与文档，尚未加入整车顶层。
