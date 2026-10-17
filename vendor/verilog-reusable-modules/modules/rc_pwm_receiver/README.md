# rc_pwm_receiver — RC PWM 捕获与双通道遥控命令

把航模接收机的两路 RC PWM 转换成有符号 `steering`、`throttle` 和 `signal_valid`。可直接在主工程中实例化；需要三个 RTL 文件，不依赖其他公共模块或厂商 IP。附 ACG720 数码管与 LED 调试工程，无车也能观察真实命令。

接收的是 GPIO 上的 PWM 脉宽，默认 50 MHz 系统时钟。输出命令为无量纲的 −1000…+1000，上层再换算成机器人速度、角速度。

## 目录

```text
rc_pwm_receiver/
├── rtl/
│   ├── rc_pwm_capture.v       单路捕获，可单独实例化扩展通道
│   ├── rc_axis_normalize.v    多周期归一化，避免长组合除法路径
│   └── rc_receiver.v          两路捕获、原子提交、失效归零
├── sim/
│   ├── rc_receiver_tb.v       自检测试平台，17 组测试
│   ├── rc_axis_normalize_tb.v 全部 65536 种输入、两种标定穷举
│   ├── rc_receiver_pipeline_tb.v 更新/取消/恢复跨模块安全测试
│   ├── run_test.do            ModelSim GUI 入口
│   └── run_test.ps1           Icarus 命令行入口
├── constraints/
│   └── rc_receiver.sdc        50 MHz 顶层时钟约束示例
├── examples/acg720_rc_debug/  Gowin 工程、LED/数码管、约束与自检
└── VERIFICATION.md            验证范围、时序结果与实物边界
```

## 直接接入

把 `rtl/rc_pwm_capture.v`、`rtl/rc_axis_normalize.v`、`rtl/rc_receiver.v` 全部加入工程，在主顶层实例化：

```verilog
wire signed [11:0] steering;
wire signed [11:0] throttle;
wire signal_valid;
wire [15:0] steering_width_us, throttle_width_us;
wire steering_valid, throttle_valid;
wire steering_timeout, throttle_timeout;

rc_receiver #(
    .CLK_HZ(50000000),
    .CH1_MIN_US(1000), .CH1_CENTER_US(1500), .CH1_MAX_US(2000),
    .CH2_MIN_US(1000), .CH2_CENTER_US(1500), .CH2_MAX_US(2000),
    .CH1_DEADZONE_US(25), .CH2_DEADZONE_US(25)
) u_rc (
    .clk(clk_50m), .rst_n(system_rst_n),
    .rc_ch1(pwm_steering), .rc_ch2(pwm_throttle),
    .steering(steering), .throttle(throttle), .signal_valid(signal_valid),
    .ch1_width_us(steering_width_us), .ch2_width_us(throttle_width_us),
    .ch1_valid(steering_valid), .ch2_valid(throttle_valid),
    .ch1_timeout(steering_timeout), .ch2_timeout(throttle_timeout)
);
```

`pwm_steering`、`pwm_throttle` 是主工程选择的接收机信号：当前默认对应物理 CH1、CH2；也可以把接收机 CH4 接到逻辑 `rc_ch1`、CH2 接到逻辑 `rc_ch2`，实现左摇杆左右转向、右摇杆上下油门，不需要修改模块。HT-10A 通道映射仍以当前设置和实测为准。

不要把有符号命令接成无符号量；各个输出总线在上层声明时也应带 `signed`。

## 接口

### `rc_receiver`

| 信号 | 方向 / 类型 | 含义 |
| --- | --- | --- |
| `clk` | 输入，1 bit | 系统时钟，默认 50 MHz |
| `rst_n` | 输入，1 bit | 低有效复位，内部异步拉低、同步释放 |
| `rc_ch1` / `rc_ch2` | 输入，1 bit | 异步 RC PWM，模块内部两级同步 |
| `steering` / `throttle` | 输出，signed [11:0] | −1000…+1000，任一路无效时两者均为 0 |
| `signal_valid` | 输出，1 bit | 两路捕获有效、均未超时且已有本次有效会话的归一化结果；不代表射频在线 |
| `ch1_width_us` / `ch2_width_us` | 输出，[15:0] | 最近合法脉宽，单位 μs，向下取整；失效后保留历史值 |
| `ch1_valid` / `ch2_valid` | 输出，1 bit | 独立通道脉宽有效且新鲜 |
| `ch1_timeout` / `ch2_timeout` | 输出，1 bit | 对应通道超过指定时间没有合法更新 |

### `rc_pwm_capture`

单独使用时接 `clk`、`rst_n`、`rc_pwm`，读取 `pulse_width_us`、`pulse_valid`、`timed_out`、`pulse_update`。`pulse_update` 是每个合法完整脉冲结束后的一拍事件；`pulse_valid` 是保持状态。复位后先观察低电平，再接受完整上升沿，避免把半个脉冲识别为命令。

过短、过长、卡高、失联均会使有效状态清零。非法脉冲不会刷新超时计时；计数器饱和，卡高不会回绕成合法脉宽。

增加 CH3、CH4 等通道时，可以独立实例化 `rc_pwm_capture`，不需要复制其 RTL。

## 参数与标定

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `CLK_HZ` | 50000000 | 必须为正整数 MHz；50 MHz 分辨率为 20 ns |
| `MIN_PULSE_US` / `MAX_PULSE_US` | 900 / 2100 | 合法脉宽窗，按原始时钟计数比较 |
| `TIMEOUT_US` | 100000 | 无合法更新超过 100 ms 后失效 |
| `CH1/CH2_MIN_US` | 1000 | 该逻辑通道负端点 |
| `CH1/CH2_CENTER_US` | 1500 | 该逻辑通道中位 |
| `CH1/CH2_MAX_US` | 2000 | 该逻辑通道正端点 |
| `CH1/CH2_DEADZONE_US` | 25 | 中位左右 ±25 μs 输出 0 |
| `CH1/CH2_REVERSE` | 0 | 设为非零反转命令符号 |

端点之间按死区外距离线性映射；合法窗内、超出标定端点的脉宽限幅到 ±1000。两路独立标定。

F-10A 模拟舵机模式可使用 50 Hz、1000–2000 μs；数字舵机模式可能使用 330 Hz、500–2500 μs。后一模式应相应设置 `.MIN_PULSE_US(500)`、`.MAX_PULSE_US(2500)`，并重新填写各轴实际标定值。

约束：`1 <= MIN_PULSE_US <= MAX_PULSE_US <= 65535`，`TIMEOUT_US > MAX_PULSE_US`；标定必须满足 `MIN < CENTER < MAX`，死区小于中位到任一端点的距离。时钟换算后的计数参数须在正的 32 位整数范围内，详见 RTL 头部。

这些默认值是初始参考，不是已完成的 HT-10A/F-10A 标定。实测曾出现遥控器指令 1500/1000、显示 1499/999 μs；模块保留测量值，没有人为统一加 1。

## 仿真

Windows / Icarus（工具位于 PATH）：

```powershell
powershell -ExecutionPolicy Bypass -File .\sim\run_test.ps1
```

工具不在 PATH 时显式传入：

```powershell
powershell -ExecutionPolicy Bypass -File .\sim\run_test.ps1 -Iverilog C:/iverilog/bin/iverilog.exe -Vvp C:/iverilog/bin/vvp.exe
```

Linux / macOS / Icarus，在本模块根目录执行：

```sh
mkdir -p build
iverilog -g2012 -Wall -s rc_receiver_tb -o build/rc_receiver_tb.vvp rtl/rc_pwm_capture.v rtl/rc_axis_normalize.v rtl/rc_receiver.v sim/rc_receiver_tb.v
cd build
vvp rc_receiver_tb.vvp
```

PowerShell 入口依次运行全部三个自检，Linux 示例只运行基础自检；另两项可用同样命令替换顶层名和 TB 文件。ModelSim GUI：进入 `sim` 后执行 `do run_test.do`。生产 RTL 为 Verilog-2001，测试平台因使用 `$fatal` 按 SystemVerilog 编译。ModelSim 入口已提供，实际复验使用 Icarus，详见验证记录。

## 没有车时观察控制命令

打开 [examples/acg720_rc_debug/rc_debug.gprj](examples/acg720_rc_debug/rc_debug.gprj)，顶层 `rc_debug_top`。默认接收机物理 CH4 为转向、CH2 为油门；替代 CST 支持 CH1 转向。具体已核验引脚、构建步骤及信号接法见 [例程说明](examples/acg720_rc_debug/README.md)。

SW0=0 显示左右两路原始 μs；SW0=1 显示最终命令绝对值，符号由 D0/D1（转向负/正）和 D2/D3（油门负/正）表示。D4 是命令模式，D5 是超时，D6 是命令有效，D7 是心跳。无效数码管显示横杠。例程使用真实接收和归一化链路，覆盖全部 12 位命令，不需要电机。

## 时序与安全接入

单个时钟域，无派生时钟。约束文件仅在本模块独立作为顶层且端口名为 `clk` 时可直接使用；集成到主工程时使用主工程的时钟与 GPIO 约束。

归一化改为两路并行的迭代除法。接受一次快照后第 27 个时钟完成单轴计算，第 28 个时钟原子提交两路命令（50 MHz 下 560 ns，从归一化接受沿算起，不包含 PWM 捕获时间）。普通有效更新计算期间保持上一组命令及 valid；首次捕获或真实失效后恢复，须等待新计算完成才置 valid。新更新优先于旧结果完成；非法脉宽、超时及复位会取消旧计算并将两路输出归零。

包含完整命令路径的本例程已在 GW5AT-LV138PG484AC1/I0 完成 50 MHz P&R，报告 Fmax 83.358 MHz，setup/hold 违例端点均为 0。移入更大的主工程后仍应重新跑时序，不能复用此数值替代整车工程签核。

正常供电、电平适配和共地由板级设计负责。F-10A 标称供电为 4–9V；目前 3.3V 低压试接成功只是实验结果，尚未测 PWM 高电平。GPIO 输入同步不会保护过压，按 FPGA IO 电压确认兼容或使用电平转换。

接收机射频失联后可能继续输出合法的保持/失控保护 PWM；模块无法仅靠脉宽和超时识别这种无线失联。整车仍需独立急停、接收机失控保护测试与安全仲裁。

## 四轴和遥控开关调试扩展（2026-10-17）

- [四轴数码管例程](examples/acg720_rc_4ch/README.md)：CH1～CH4同时捕获，每10秒轮换显示，SW0冻结；用户确认四轴直连捕获工作。生产编译/lint及50MHz P&R通过，10秒版Fmax73.653MHz、setup/hold违例0，已执行SRAM下载。
- [CH5/CH6独立LED例程](examples/acg720_rc_6ch_led/README.md)：D4对应CH5，D5对应CH6，默认高档点亮，可配置低档有效；其余四轴显示保留。新扩展的生产编译/lint通过，P&R和实板按键映射仍待验证，详见其VERIFICATION.md。

只有映射到接收机通道的遥控按键/开关才可观察；这不是板载实体按键或电机急停实现。CH7～CH10尚未扩展。供电和GPIO兼容性沿用用户确认的当前接法，未补造电压测量数值。
