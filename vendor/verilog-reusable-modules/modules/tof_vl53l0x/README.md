# TOF200C / VL53L0X FPGA 测距模块

可综合 Verilog-2001；默认 50 MHz 时钟，通过 100 kHz I²C 自动初始化并循环单次测距。适用于用户照片中标注 `TOF200C-VL53L0X` 的传感器。独立运行，不依赖 RK3588、Arduino 或 STM32 软件。

## 接口与使用

把 `rtl/tof_i2c_reg_master.v` 和 `rtl/tof_vl53l0x.v` 加入工程，实例化 `tof_vl53l0x`：

```verilog
wire [15:0] distance_mm;
wire range_valid, sample_pulse, initialized, busy;
wire [3:0] raw_range_status;
wire [4:0] fault_code;
tof_vl53l0x #(.CLK_HZ(50000000)) tof (
    .clk(clk_50m), .rst_n(rst_n), .retry(retry),
    .scl(tof_scl), .sda(tof_sda), .xshut_n(tof_shut),
    .distance_mm(distance_mm), .range_valid(range_valid),
    .sample_pulse(sample_pulse), .initialized(initialized), .busy(busy),
    .raw_range_status(raw_range_status), .fault_code(fault_code)
);
```

`clk_50m`、`rst_n`、`retry` 和三个双向端口由上层提供；示例不是带管脚约束的板级顶层。`rst_n` 低有效，可异步置位，释放在控制器内同步。`retry` 与 `clk` 同步，一次上升沿会使已有结果失效、取消当前事务并从 SHUT 重新初始化。

| 端口 | 含义 |
|---|---|
| `distance_mm[15:0]` | 最近一次完整读取的距离，单位 mm；复位/retry 为 0 |
| `range_valid` | 最近样本 raw status 为 11，且没有致命故障/陈旧超时 |
| `sample_pulse` | 一拍脉冲；状态、距离高低字节读取且清中断成功后一起提交。无效测量也发脉冲 |
| `raw_range_status[3:0]` | `RESULT_RANGE_STATUS[6:3]`；硬件成功为 **11**，不是软件 PAL 的成功码 0 |
| `initialized` | 完成初始化、VHV 与 phase 两次参考校准；致命故障时清零 |
| `busy` | 初始化或正在获取样本；等待下次采样及锁定故障时为 0 |
| `fault_code[4:0]` | 见下表；0 不代表距离有效，仍须使用 `range_valid` |
| `scl`, `sda`, `xshut_n` | 仅驱动低电平或高阻；需要合适电压的外部上拉 |

下游应只在 `sample_pulse && range_valid` 时采用新距离，并在 `!range_valid` 时采取上层定义的安全行为。无效测距或陈旧时仍保留最后提交的距离用于诊断，不要单凭数值判断安全。

## 已实现的流程

SHUT 复位、上电等待 → 型号 ID 0xEE 检查 → 读取隐藏 stop variable → 读取出厂 SPAD count/type 和完整 48-bit good map，保留孔洞、aperture 时跳过前 12 位并选够指定数量 → 完整 80 条参考 tuning → IRQ 配置 → VHV/phase 校准 → 循环恢复 stop variable、single-shot、有限等待、读取同一帧 status/high/low、清 IRQ 后原子提交。

I²C 7 位地址固定 **0x29**，线上写/读地址字节为 0x52/0x53。所有逻辑使用输入时钟和时钟使能，不产生内部派生时钟。SCL/SDA 输入均有两级同步；支持从机拉低 SCL 等待。每次读操作以重复 START 切换读地址，并以 NACK 结束最后一个字节。

| 故障码 | 含义 |
|---|---|
| 0 | 无控制器故障 |
| 1 | I²C NACK |
| 2 | SCL 被拉低超时 |
| 3 | 总线 SDA 卡低或总线状态错误 |
| 4 | 型号 ID 不匹配 |
| 5 | SPAD 信息就绪超时 |
| 6 | VHV/phase 校准就绪超时 |
| 7 | single-shot START 位未清除 |
| 8 | 测距结果未就绪 |
| 9 | 出厂 SPAD count 为 0 或 good map 不足以选够数量 |
| 10 | 样本过期；非致命，下一次有效样本自动恢复 |

故障 1–9 清除 `range_valid`、`initialized`，并持续拉低 SHUT；恢复接线或供电后由 `retry` 上升沿或复位重新启动。不会自动把故障前的距离标为有效。

## 参数

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `CLK_HZ` | 50000000 | 系统时钟频率 |
| `I2C_HZ` | 100000 | SCL 目标频率上限；同步/状态开销会使实际略低 |
| `I2C_TIMEOUT_US` | 2000 | 单次等待 SCL/SDA 恢复的期限 |
| `BOOT_DELAY_US` | 10000 | SHUT 释放后的等待 |
| `RESET_LOW_US` | 1000 | SHUT 复位低电平时间 |
| `POLL_INTERVAL_US` | 1000 | 未就绪两次读取之间的等待 |
| `INIT_TIMEOUT_US` | 100000 | 每阶段 SPAD/校准轮询期限 |
| `MEAS_TIMEOUT_US` | 100000 | 每次测距 START 与 ready 共用的轮询期限 |
| `SAMPLE_PERIOD_US` | 50000 | 连续测量启动之间的最小间隔，首次就绪后立即测量 |
| `STALE_TIMEOUT_US` | 200000 | 距离最后有效样本超过此时长则清 valid |

支持 `CLK_HZ` 为至少 1 MHz 的整数 MHz，`I2C_HZ` 不超过 400 kHz，且 `CLK_HZ / I2C_HZ / 2 >= 4`。所有时间参数为正，计数器乘积须适合 32 位。轮询期限在一笔已发起事务结束后检查，因此最坏故障上报延迟还包含一笔 I²C 事务或其等待超时，不是逐周期硬截止。

## 验证和上板边界

在本目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_test.ps1
```

支持 PATH 中的 Icarus Verilog，也可用 `-Iverilog`、`-Vvp` 指定路径。测试日志生成在 `sim/artifacts/`。Questa/ModelSim 可执行 `do /absolute/path/to/sim/run_test.do`；该 `.do` 提供便携入口，本次实际执行的是 Icarus。

数字仿真已验证，细节见 [VERIFICATION.md](VERIFICATION.md)。**已完成本次实板测距与SAFE/WARN/STOP显示链路验证，尚未进行光学精度标定**；实板结果由用户观察确认，见 [docs/HARDWARE_TEST.md](docs/HARDWARE_TEST.md)。初始化使用固定参考 tuning，未完整实现 ST PAL 的软件 sigma estimator、DMAX、动态参考 SPAD 搜索或测量预算重算 API；`range_valid` 不等价于完整 ST PAL 的全部质量检查。

新增 [ACG720 SAFE / WARN / STOP 数码管上板例程](examples/acg720_tof_debug/README.md)：真实测距输入，≥200mm显示SAFE，100–199mm显示WARN，<100mm显示STOP，右侧保留毫米距离；未初始化、非法/陈旧样本和通信故障分别显示。七段管用近似字形显示英文，同时有状态LED。含完整 `tof_debug.gprj`、已核验J7约束和一键仿真/构建入口。该完整例程已通过50MHz布局布线，Fmax 101.482MHz，setup/hold违例端点均0；随后已成功下载到ACG720 SRAM，用户确认真实距离变化及SAFE/WARN/STOP显示工作。该结果不构成精度、长期稳定性或电气额定范围验收。

核心的 Gowin **仅综合**入口仍为：`powershell -ExecutionPolicy Bypass -File scripts/run_synth.ps1 -GowinShell /path/to/IDE/bin/gw_sh.exe`。它不布局布线、不生成可下载硬件文件；上板须使用上述例程的 `build.ps1`，不能混淆两个入口。器件均为 GW5AT-LV138PG484AC1/I0，SDC含50MHz时钟约束。

照片只确定型号和 VIN/GND/SDA/SCL/INT/SHUT 丝印，不能确定该模组 VIN 允许范围、上拉电压或板载电平转换。接线须按该模组电气资料确认。SCL/SDA/SHUT 应使用双方兼容电压的上拉；本模块无需使用 INT，通过 I²C 轮询。板级例程的CST依据用户提供的ACG720转接板RevI手册第3页与官方管脚表，未猜测GPIO；其他板卡集成时应替换为自身约束。

参考、版本指纹与许可见 [docs/SOURCES.md](docs/SOURCES.md) 和 `third_party/Pololu_ST_LICENSE.txt`。
