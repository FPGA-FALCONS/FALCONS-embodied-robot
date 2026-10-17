# 四路遥控器链路测试记录

日期：2026-10-17，北京时间。该例程同时捕获CH1～CH4，经已有HC595显示驱动观察四轴，不生成电机命令。

| 验证 | 实际结果 |
| --- | --- |
| Icarus默认50MHz生产顶层，Verilog-2001 `-Wall` | PASS |
| Verilator `--lint-only -Wall` | PASS，无警告/错误，无额外豁免 |
| 4MHz自检、独立物理PWM与HC595引脚解码 | 19040检查，0失败，6299个16-bit HC595锁存字 |
| Gowin V1.9.12.03、GW5AT-LV138PG484AC1/I0版本C | 综合及P&R通过 |
| 50MHz时钟约束 / 报告Fmax | 50.000MHz / 73.653MHz（10秒轮换版） |
| Setup / Hold违例端点 | 0 / 0 |
| 已报告Recovery / Removal最差slack | +16.966ns / +1.132ns（10秒轮换版） |
| 资源 | 1188 LUT，471 FF，0 latch（10秒轮换版） |
| 板载Type-C SRAM Program | 工具返回Finished，退出0；用户码0x0000F983，状态码0x00026230（10秒轮换版） |

测试覆盖四路1000/1500/2000/1250μs、单轴独立变化、500μs非法脉冲、单路超时不影响其他三路、恢复、SW0冻结和自动轮换、复位重获及心跳。通过DS/SH_CP/ST_CP实际引脚解码所选通道编号、脉宽及横杠，不以内部BCD替代显示验证。生产默认超时100ms、轮换10s、心跳翻转500ms；TB缩短这些时间以加速仿真，详见测试平台参数。10秒版仅修改CHANNEL_US默认值，生产参数编译、lint及P&R重新通过，并成功下载SRAM；原参数化TB结果仍适用于未修改的轮换/捕获逻辑。

P&R只约束50MHz时钟，没有宽泛false path；Recovery/Removal为各25条worst-path表结果，工具未提供其全局违例端点计数。时序门禁生成`build/timing_manifest.json`，保存源文件、位流、报告SHA256。内部门禁通过不代替GPIO电气兼容确认。

10秒轮换版位流SHA256：`51fc0e67b187c11c53f279124f0af248c1a64d53348b20cf8c80b1fbfdacf43a`；日志`build/sram_program_10s.log`，回执`build/program_receipt_10s.json`。初版2秒位流SHA256为`718b8f0b314530a87b8d39043721c02fa73f0bae7da420df6bd1c07fc4e7a0e0`。用户反馈移除转换模块后能读取摇杆PWM，并报告较遥控器设定值少1μs；这属于用户确认的直连捕获结果，未测PWM电压，不能作为电气兼容证明。10秒轮换的实际观察仍待确认。
