# 手柄LED上板测试（2026-10-17）

## 明天先下载哪一个

下载 **`impl/rc_led_test/impl/pnr/rc_led_test.fs`**，顶层是`rc_led_test_top`。
这是手柄输入专用测试：不运行电机控制、IMU、HDMI或导航；**STBY、四路IN1、四路IN2、四路PWM共13个电机输出恒为0**。S1在此版本中是切换LED视图，不是整车的停机按钮。使用SRAM下载，不先固化Flash。

文件已在本机构建，校验值和测试结果见[审查与上板验证](archive/hardware/遥控与装车接口验证_20261016-24.md#report-02)。板卡仍沿用工程GW5AT-138B；下载前核对芯片丝印，用户旧资料的138C字样不能代替实物确认。

## 接线

沿用[HT10A接线与验收](HT10A接线与验收.md)第2节的唯一接线表：
接收机CH4/CH2/CH5/CH7/CH6分别经电平转换接J6.1..5；J6.6闲置。当前只有确认与select两个按钮，没有返回键。要获得有效指示，五路都需要连接；转换模块也需要覆盖这五路信号。

J6仅为信号，接收机单独按规格供电，各模块共地。转换板的高低压供电和方向要按实际型号确认，不能凭外形假定。先断开电机动力；如果驱动板仍连着，按原整车指南拔掉STBY跳线，保留外部下拉。先完成手柄验收再换整车固件。

## LED对照表

LED高电平点亮。S0复位；S1每按一次切换两个视图，默认方向视图。

| LED | 方向视图（默认） | 事件视图（按S1一次） |
| --- | --- | --- |
| LED7 | 慢闪：半秒亮、半秒灭 | 每秒双闪，表示事件视图 |
| LED6 | 五路输入均有效时亮 | 同左 |
| LED5 | 光标模式亮，手动模式灭；无有效输入也灭 | 同左 |
| LED4 | 油门正向超过阈值 | 菜单向下事件亮半秒 |
| LED3 | 油门负向超过阈值 | 菜单向上事件亮半秒 |
| LED2 | 转向正向超过阈值 | CH7 select事件亮半秒 |
| LED1 | 转向负向超过阈值 | CH5确认事件亮半秒 |
| LED0 | 任意按钮/菜单事件亮半秒 | 常亮，作为事件视图标记 |

方向灯默认要求归一化值超过±100，因此很小的摇杆动作可能只在串口数值上变化。正负方向只表示解码值，不代表实车前后左右已标定。

**确认/select/菜单事件仅在光标模式有效。** 切入光标后先把摇杆回中、按钮释放，再操作。保持按钮不应重复闪；保持摇杆端点不应连翻，必须回中后再拨。无信号后事件灯清空。

## 建议按这个顺序验收

1. 打开遥控器，确认转换与供电接线，下载测试位流。无需给电机供电。
2. 复位后看LED7慢闪；LED6亮表示五路PWM均合法、新鲜。LED6灭时不要先怀疑代码，打开串口看哪一位缺失。
3. 方向视图下单独推CH2上下，应该只亮LED4或LED3；单独拨CH4左右，只亮LED2或LED1。回中时这四灯都灭。
4. 拨模式开关，看LED5随手动/光标变化。物理档位极性未标定，亮灯的档位就是当前解码的光标模式。
5. 按S1进入事件视图；在光标模式测试确认、select、摇杆上下，分别看LED1/2/3/4。长按不重复，释放后重按才再亮。
6. 拔下一根正在使用的信号线，最迟100ms左右LED6应灭；重新接回后等待输入恢复。J6.6未接是正常状态，不需要补接。
7. **单独关闭发射机并记录现象。** 如果接收机继续输出合法旧脉冲，LED6可能仍亮；这证明PWM有效不能等同于无线在线。先配置/确认failsafe，再进行电机测试。
8. 记录各方向、档位、缺线与关发射机的现象。测试位流本身不会动车；通过后再换整车`falcons.fs`，架空验收后才低速落地。

## 串口查看原始脉宽

板载USB转串口，**115200、8N1**，每秒约10行。不需要发送命令。

```text
RC,view,ok,mode,steer,throttle,ch4_us,ch2_us,ch5_us,ch7_us,ch6_us,spare_us,valid_mask,timeout_mask,events
RC,+00000,+00001,+00001,-01000,+00500,+01500,+02000,+01500,+02000,+01000,+00000,+00031,+00032,+00005
```

第一行是列说明，程序只发第二种数据行；每个数值是符号加5位十进制数。例如`+01500`就是1500us。每行对全部字段做同一时刻快照，不会在半行中混入新输入。

- `view`：0方向、1事件；`ok`：有效输入；`mode`：0手动、1光标。
- 六个脉宽按J6槽位排列；`spare_us`是闲置第六路，可为0或旧值。
- `valid_mask`/`timeout_mask`：十进制位掩码，bit0..5对应J6.1..6。正常五路有效可见`valid_mask=31`；闲置第六路超时可见`timeout_mask=32`，不影响ok。
- `events`：显示保持中的事件位，确认=1、select=2、上=4、下=8，可相加。
- 非法/超时后脉宽可能保留上次合法值，必须同时看有效掩码。测试CSV与整车原来的12列CSV是不同格式。

## 重新构建与下载

在工程根目录运行（本机实际Gowin安装路径如下；其它电脑改工具路径）：

```powershell
powershell -ExecutionPolicy Bypass -File sim/run_sim.ps1 -Simulator Icarus
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tcl build_rc_led.tcl -ArtifactDirectory 'impl/rc_led_test/impl/pnr' -GowinSh 'D:/FALCONS_workspace/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'
```

测试约束从`constr/falcons.cst`筛选生成31个管脚，产物全部放在`impl/rc_led_test/`，不覆盖主工程位流，也不把额外约束放进生产`constr/`目录。

首次只预览下载命令：

```powershell
powershell -ExecutionPolicy Bypass -File tools/program_flash.ps1 -Action Sram -Fs 'impl/rc_led_test/impl/pnr/rc_led_test.fs' -Programmer 'D:/FALCONS_workspace/Gowin/Gowin_V1.9.12.03_x64/Programmer/bin/programmer_cli.exe' -DryRun
```

确认芯片型号与现场接线后，实际下载才去掉`-DryRun`。也可在Gowin Programmer里打开该文件，选择SRAM Program。不要选择主工程的`falcons.fs`来做本说明的LED验收。

## 当前边界

软件和构建已验证，实物尚未操作。没有看到转换模块型号，不能认证其具体接法。这里不完成无线failsafe标定、电机方向或PID整定。测试顶层复用当前`ht10a_input`默认标定，当前与主工程显式默认值相同；以后现场改主工程覆盖值时，也应同步测试配置，不能将未标定默认值当作实测值。
