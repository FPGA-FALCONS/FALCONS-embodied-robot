# rc_pwm_receiver — RC PWM 捕获与双通道遥控命令

把航模接收机的两路 RC PWM 转换成有符号 `steering`、`throttle` 和 `signal_valid`。可直接在主工程中实例化；只需要两个 RTL 文件，不依赖其他公共模块或厂商 IP。

接收的是 GPIO 上的 PWM 脉宽，默认 50 MHz 系统时钟。输出命令为无量纲的 −1000…+1000，上层再换算成机器人速度、角速度。

## 目录

```text
rc_pwm_receiver/
├── rtl/
│   ├── rc_pwm_capture.v       单路捕获，可单独实例化扩展通道
│   └── rc_receiver.v          两路捕获、标定、死区、反向、失效归零
├── sim/
│   ├── rc_receiver_tb.v       自检测试平台，17 组测试
│   ├── run_test.do            ModelSim GUI 入口
│   └── run_test.ps1           Icarus 命令行入口
├── constraints/
│   └── rc_receiver.sdc        50 MHz 顶层时钟约束示例
└── VERIFICATION.md            验证范围、实物结果与已知限制
```

## 直接接入

把 `rtl/rc_pwm_capture.v`、`rtl/rc_receiver.v` 加入工程，在主顶层实例化：

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
| `signal_valid` | 输出，1 bit | 两路脉宽合法且均未超时；不代表已确认射频在线 |
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
iverilog -g2012 -Wall -s rc_receiver_tb -o build/rc_receiver_tb.vvp rtl/rc_pwm_capture.v rtl/rc_receiver.v sim/rc_receiver_tb.v
cd build
vvp rc_receiver_tb.vvp
```

ModelSim GUI：进入 `sim` 后执行 `do run_test.do`。生产 RTL 为 Verilog-2001，测试平台因使用 `$fatal` 按 SystemVerilog 编译。ModelSim 入口已提供，实际复验使用 Icarus，详见验证记录。

## 时序与安全接入

单个时钟域，无派生时钟。约束文件仅在本模块独立作为顶层且端口名为 `clk` 时可直接使用；集成到主工程时使用主工程的时钟与 GPIO 约束。

归一化函数含组合乘除运算，加入系统后必须检查输出命令路径的综合和布局布线时序；捕获/数码管演示的 Fmax 不能代替该路径的时序签核。

正常供电、电平适配和共地由板级设计负责。F-10A 标称供电为 4–9V；目前 3.3V 低压试接成功只是实验结果，尚未测 PWM 高电平。GPIO 输入同步不会保护过压，按 FPGA IO 电压确认兼容或使用电平转换。

接收机射频失联后可能继续输出合法的保持/失控保护 PWM；模块无法仅靠脉宽和超时识别这种无线失联。整车仍需独立急停、接收机失控保护测试与安全仲裁。
