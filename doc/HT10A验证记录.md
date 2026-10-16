# HT10A接口对接验证记录（2026-10-16）

> ## ★ 本文件记录的是**合并前**的分支基线（历史证据）
>
> 下表的 170 项、位流 13:41:28 / SHA256 `4b3b3a…` 都对应分支 `feat/ht10a-channel-integration`
> （基点 `701144b`）自己那一版源码。**2026-10-16 晚已把该分支合并进 `main`**
> （工作分支 `merge/ht10a-into-main`，合并方式与逐条裁定见 `doc\工作记录.md` §二十一），
> 合并后：
> - **遥控通道口径改了**：J6.4/bit3 由"返回"改为 **select**，J6.6/bit5 **闲置**（接线表见 `HT10A接线与验收.md`）；
> - `ht10a_input_tb` 由 77 项扩到 **79 项**（掩码双向用例 + CH7→select 改名）；
> - 全套仿真 **6 台 / 203 项全 PASS**（ModelSim 实测）；
> - 位流重新综合为 **2026-10-16 20:51:52**，SHA256 `4FA40D37…3493`，Logic 9112 / Register 4227 / DSP 43；
> - 此后又经一轮**全项目审查修复**（2026-10-16 20:51:52 重新综合），仿真因补断言由 193 项增至 203 项。
>
> **下面原表中的数字请当"合并前基线"读，不要与当前状态混用。**

## 范围和版本

- 基线：`701144bb2dce4109c5c859bdb782037bddd45a0c`，本地分支`feat/ht10a-channel-integration`。
- 新仓库：HITSZ-FPGA-FALCONS/FALCONS-embodied-robot；用户确认的CH2/CH4及四开关分配见[接线与验收](HT10A接线与验收.md)。**（★ 合并时该分配中"CH7/SWC 返回、CH8/SWD select"已改为"CH7/SWC = select、CH8 闲置"）**
- 仅改应用层、测试和相关工具/说明；vendor模块及两份需求原文没有修改。首次本地交付阶段未推送；没有烧录或实物操作。功能分支发布按后续用户明确授权执行，记录见工作记录。
- 机器路径：`D:/HITSZ_FALCONS/FALCONS-embodied-robot`；工具Gowin V1.9.12.03、Icarus 12。本机未安装ModelSim，本轮未实跑ModelSim；同一测试清单保留其入口。

## 实际执行结果

| 检查 | 结果 |
| --- | --- |
| ht10a_input_tb | 77项检查PASS |
| rc_input_tb | 26项检查PASS |
| chassis_ctrl_tb | 23项检查PASS |
| num_text_tb | 28项检查PASS |
| ui_render_tb | 12项检查PASS |
| speed_report_tb | 4项检查PASS |
| 应用层合计 | **170项 / 6个测试台PASS** |
| 原vendor rc_receiver_tb | 50MHz真实时钟、17组502断言PASS（含20ns边界、50Hz帧和默认100ms超时） |
| 引脚复核 | RevI手册J2/J4/J5/J6/J7共35根信号一致，60个IO无重复 |
| 整车构建 | 综合、布局布线、位流生成完成，工具正常退出；不是实物验证 |

本机原全量测试首次执行时，UI测试的手动状态文字统计为0：测试只用了声明初始化，没有给无复位组合状态逻辑一个实际输入跳变。补上明确的模式切换后，原状态栏断言通过（手动138像素）；未修改UI生产逻辑或降低断言。chassis测试的`before`是SystemVerilog保留字，改名`duty_before`使原测试可由Icarus -g2012运行。

引脚脚本原PyMuPDF回退逐单元格提取，五张表均解析为0针；改为`sort=True`保留行布局，并增加各连接器针数门槛。本轮不改变电气约束，仅补槽位注释。

## 测试反向检查

在临时副本分别交换两根摇杆输入、恢复旧的持续回中门禁，两种变异均被新增集成测试检出并非零退出。没有将变异写回项目源码。

## 当前下载文件

最终构建报告时间：`Fri Oct 16 13:41:28 2026`；源码在构建前已按仓库规则归一化行尾。

- 路径：`impl/pnr/falcons.fs`
- 配置：GW5AT-138B / GW5AT-LV138PG484AC1/I0。实际芯片版本与转换模块尚需现场核对。
- 大小：**35882493字节**。
- SHA256：`4b3b3a7ed139fc316c56cc59f130e29d0127efd4384d4db6c73cfdf15fb4fd4a`。
- 资源：Logic 9096/138240；Register 4267/139095；BSRAM 9/340；DSP 43/298。
- 原始本地日志：`logs/ht10a_regression.log`、`logs/ht10a_vendor_capture.log`、`logs/ht10a_build.log`；均被仓库忽略规则排除。
- 已通过Verilog-2005语法编译（配置壳无timescale的提示不影响可综合逻辑）；SRAM下载脚本仅DryRun通过，未实际下载。
- 结构化证据及输入指纹：[HT10A验证数据.json](HT10A验证数据.json)。源码改动后该位流不再代表新源码，必须重建。

## 时序限制：没有宣称全系统关闭

下表是本次布线后报告中所列路径的统计，不等同于所有端点的穷尽检查。

| 类型 | 报告列出路径数 | 其中负slack数 | 最差slack/ns |
| --- | ---: | ---: | ---: |
| Setup | 25 | 0 | 0.325 |
| Hold | 25 | 13 | -1.019 |
| Recovery | 25 | 25 | -4.502 |
| Removal | 25 | 0 | 0.28 |

> ★ **2026-10-16 21:18 后注（口径已查清；下文原样保留作历史证据）**：
> 本文件是**合并前分支基线**，其中 `Numbers of Setup Violated Endpoints=98` 是队友那轮更早提交的数字；
> 合并后（20:51 / 21:18 两次独立布线）该值稳定为 **4**，且**已查明身份**：
> 就是 `Numbers of Falling Endpoints = 4` 的那 4 个**下降沿捕获端点** ——
> `u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_{clk,b,r,g}/RESET`（由 245 MHz CLKOUT1 下降沿捕获），
> 属**结构性伪违规**；**上升沿数据路径 0 条违规**。证据与完整推理见 `doc\工作记录.md` §21.9。

报告同时仍显示`Numbers of Setup Violated Endpoints=98`，与所列Setup路径统计口径不同。本轮没有验证其形成原因，不能凭前25条Setup为正就认定全部端点通过。Hold最差路径在跨域同步级；Recovery最差路径在复位到HDMI OSER10。未经跨域/复位结构及约束专项审查，不能将这些直接视为无风险。原仓库已有此类遗留，本轮未改vendor/HDMI，也未添加false path隐藏它们。

## 必须留待实物验证

CH2/CH4实际行程和方向、CH5/6/7/8档位/极性、转换模块电气接法、芯片B/C版本、架空电机行为以及关闭发射机的failsafe。`signal_valid`只证明PWM形式合法且新鲜，不保证无线在线。实测记录表见接线与验收文档。
