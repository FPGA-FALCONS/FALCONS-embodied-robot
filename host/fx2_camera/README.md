# FPGA → FX2 → RK3588 图像链路

摄像头接 FPGA，由 FPGA 独立完成采集和本地显示；RK3588 读取 FPGA 转发的图像做 AI 拓展。该边界采用用户 2026-10-24 的明确项目要求，不将摄像头直接接 RK3588。

主通路：`OV5640 DVP → FPGA → 板载 FX2／J5 Type-C → USB-A Host／RK3588`。连接使用 USB-A 公头转 USB-C 公头数据线。板载 J2 Type-C 接 CH340E 串口，用于命令／状态；J5 接 CY7C68013A-56LFC。这里的 J2/J5 是原理图中的 USB 连接器编号，不是扩展排针编号，接线时要按对应的 USB 接口和芯片识别。

## 当前可用工具

| 文件 | 功能 | 尚未覆盖 |
|---|---|---|
| `probe.py` | Linux 标准库读取 USB sysfs，输出 VID/PID、速度、接口、驱动和端点 | 不加载固件、不读取流，不证明 FX2 已正常传图 |
| `raw565.py` | 将一个完整的原始 RGB565 帧转换成 PPM，支持明确大小端 | 不从任意 USB 字节流寻找帧边界、不处理帧头或压缩格式 |
| `receive.py` | 使用已有 libusb 接收 FIMG，重组/校验并保存 RGB565、PPM 和接收记录；也可离线解码 | 不加载固件、不解绑驱动；尚未实机接收 |
| `selection.py` | 核验实际480M、接口、512字节Bulk IN端点；歧义设备需显式选择拓扑路径 | 描述符不能证明固件内部FIFO配置正确 |
| `verify_rtl.py` | FPGA仿真总线字节与Python FIMG编码/解析交叉验证 | 不代表USB线缆和芯片时序通过 |
| `tests/` | 21项描述符、图像、USB模拟、分片/CRC/序号/断连测试 | 不代表 Linux／RK3588／FX2 实物通过 |

这些工具使用Python 3标准库；实机接收额外调用系统已有的libusb-1.0，不安装或升级包。PPM可由OpenCV/Pillow读取。匹配的FPGA发送模块已新增到 `vendor/verilog-reusable-modules/modules/camera_fx2_stream/`，电脑端已完成RTL/接收器交叉验证；固件、接线和真实板端传输尚待验收。

## 本轮新增的匹配发送/接收实现

### 直传FIMG v2的当前接收合同

新增直传工程 `camera_fx2_direct` 使用 **FIMG magic、version=2、帧尾CRC**，不是实验性FIM2逐行协议。配置从 `usb.fimg2.example.json` 复制，实际USB标识仍必须填写；`protocol=fimg2`要求版本2，`expected_size=[640,375]`匹配当前5/8候选。改变发送尺寸时必须同步改配置，不能靠接收器猜测。

接收器核对版本、尺寸和CRC后才发布图像。`fimg`允许v1/v2自动识别，历史`fimg1`参数作为兼容别名仍可接收两种FIMG版本；`fim2`仅指另一个实验协议。新直传验收请显式使用`fimg2`。

```sh
python3 host/fx2_camera/receive.py --config verified_fimg2_usb.json --out sessions/fimg2_rate --frames 100000 --seconds 30 --benchmark
```

测速模式不写RAW/PPM、不加载YOLO，只统计收到的完整帧率、线速、序号缺口和CRC错误。它测的是接收帧率，不能据此算采集延迟。实际USB不存在时不会自动挑选麦克风或替换设备。

下面的整帧缓存说明属于保留的早期snapshot/v1方案，不代表新的直传RTL已上板。

独立RTL在原相机输出处分出支路，以完整帧双时钟RAM隔离USB背压，FIMG v1头和像素一起做CRC32，显式提交USB短包。默认保持1024×600采集和原有显示，AI支路隔点抽取为512×300。它不会向摄像头施加ready或停止原DDR/HDMI。该抽取是链路初验方案，小目标识别效果仍需真实相机画面验证。

`build_integration_candidate.py`从固定GitHub版本生成独立候选目录，不覆盖本地旧且有未提交工作的顶层。代码和实测范围见[摄像头传输实现与验收](../../doc/摄像头到FPGA再到RK3588实现与验收_20261024.md)。

真实板上先运行前述probe，将实际信息填到 `usb.example.json` 的副本。VID/PID/接口/端点均为JSON整数；示例的null是刻意保留的未配置状态。然后：

```sh
python3 host/fx2_camera/receive.py --config verified_usb.json --out sessions/camera_first --frames 10 --seconds 20
```

输出目录必须全新。工具不发送USB OUT命令、不复位设备、不改配置/备用接口、不自动解绑驱动。配套FX2固件必须已经工作于下述匹配模式。收到帧不等于运动准入：FIMG v1没有源采集时间，结果保持 `source_timestamp_verified=false`。

电脑离线解码：

```sh
python3 host/fx2_camera/receive.py --input sampled_wire.bin --out sessions/wire_replay --frames 2
```

离线结果始终标记replay，不冒充实机。FPGA复位导致序号归零时，应明确重启接收器建立新会话；同一接收器不自动接受倒退序号。自然uint32回绕允许前进，重复与倒退帧拒绝。

## 接好两板后先做这一步

保持现有供电方案，并依据实板版本确认电源选择开关 S5、USB VBUS 供电路径；不要把两个 Type-C 口视为完全等效。把 J5 高速数据口接 RK3588 的 USB-A Host。若这两个文件已复制到板上项目的相同目录，在 RK3588 运行：

```sh
lsusb
lsusb -t
python3 host/fx2_camera/probe.py > fx2_probe.json
```

也可以只复制独立的 `probe.py`，在其目录运行 `python3 probe.py > fx2_probe.json`。不需要 sudo；若有权限限制或设备插拔，结果会记录可见错误。不执行安装、驱动解绑、USB 复位、配置切换或固件写入。

需要核对：

1. 插拔 J5 前后新增哪个设备。CH340 串口枚举不是 FX2 已枚举的证据。
2. `speed` 是否为 `480`（USB 2.0 High Speed）；如果只有 `12`，先排查线缆、端口和固件。
3. 实际接口的 `bInterfaceClass` 与 IN／OUT 端点；`bulk_in_candidate=true` 只表示候选读取端点，具体用途要匹配固件。
4. 记录实际 VID/PID，再用 `--vid 04b4 --pid 实际PID` 筛选。04b4 是 Cypress 的提示，不代表一定是 FX2；不硬编码教材示例 PID 或设备名称。

若只看到引导设备、没有预期数据端点，下一步是核对并加载与 FPGA 时序匹配的 FX2 固件。不能仅靠安装 Python 包解决；也不要把不同实验的固件随意交叉使用。

## 收到完整原始帧后转换

以下示例只适用于无帧头、无行填充、逐行排列的单帧 RGB565：

```sh
python3 host/fx2_camera/raw565.py --input frame.raw --width 1024 --height 600 --byteorder little --output frame.ppm
```

大小端必须用发送端定义和已知红绿蓝彩条确认，不从 USB/CPU 的大小端猜测。输出独占创建，不覆盖旧文件。1024×600 RGB565 必须正好 1,228,800 字节，多余或不足都会报错；实际格式不是 RGB565 时需要相应解码器。文件长度正确不等于帧边界、顺序或颜色正确，还须用帧号、校验及彩条验证。

## FPGA 接入方案与当前缺件

现有 `rtl/cam/cam_overlay.v` 中已有 `camera_pixel_clk`、`camera_pixel_valid`、`camera_pixel_rgb565` 和 `camera_frame_sync`；采集模块还提供 `pixel_x/pixel_y`。首选从摄像头采集输出处分出只读支路，保留原有 DDR3／HDMI 显示。不要修改已有显示 `rfifo_rden = pixel_req` 连续读取规则，也不要将菜单叠加后的画面当作 AI 原始输入。

新增发送支路使用完整帧RAM和所有权握手跨时钟，并实现长度/格式/CRC32、FX2满标志及短包收尾。USB断开或接收暂停时，保持待发送帧并跳过新帧，不阻塞摄像头；损坏帧不提交。恢复读取可能先读到此前缓存的旧帧，因此不能将主机收到时间当成采集时间。停止AI支路不得停止FPGA显示/控制/导航。

当前 FPGA 分辨率参数为 1024×600，先保持不变。先用计数器和彩条验证 USB，再验证原尺寸单帧；稳定后可从 FPGA 内部分出 512×300 支路（约 307,200 字节／帧，5 fps 约 1.536 MB/s），保持原宽高比。简单隔点抽取容易混叠，正式推理应评估低通缩小或完整帧方案。目标帧率只是工程起点，不是已测结果。

本地已有厂家第 64／68／70 章 PDF。2026-10-24 后续核验已通过高云官网下载第 68／70 章 FPGA 源码，固件仍待核对：

- `ch68_FX2_Stream_IN.rar`：递增测试流工程，已取得。
- `ch70_fx2_stream_in_ov5640_rgb.rar`：RGB 摄像头传输工程，已取得；实际为 RAR，教材名称写 ZIP。
- 配套 FX2 固件及描述符／端点配置；第 70 章提到 `slave_for_adc.iic`、`slave_for_adc_clk_not.iic`，须核对实际工程和板卡版本后选择。

官网直接下载入口：[高云 ACG720](https://www.gowinsemi.com.cn/university/board/17)。源码及下载校验记录位于 `D:/HITSZ_FALCONS/ai部分/acg720_official_20261024/`。原论坛入口直接打开超时，后续改从官网取得源码；现有本地教材中有复制和表述错误，应以原理图、固件描述符和实际时序为准；例如 USB 2.0 High Speed 不采用教材末尾所写的 8b/10b 编码。

用户反馈实板没有 DVP 接口。原理图第 11 页存在 P4、2×10 CMOS 接口，但尚未完成实物 PCB 版本／焊接状态比对。此前 2×9 描述是模块插头，不能等同于开发板连接器；新下载的厂家摄像头约束与现有 BY 模块约束不同，不直接覆盖应用。详见[摄像头物理接口核验](../../doc/摄像头物理接口核验_20261024.md)。

## 验收顺序

| 阶段 | 必须取得的证据 |
|---|---|
| 设备枚举 | 实际 VID/PID、480M、接口及端点；区分引导和工作固件 |
| 递增数据 | 连续计数正确、无丢字／重复／错序，包含暂停和重连 |
| FPGA 彩条 | 完整帧长度、帧号／校验、RGB 次序与行列正确 |
| OV5640 单帧／低帧率 | 可恢复完整图像，拔掉 RK3588 后 FPGA 显示与基础功能继续工作 |
| AI 扩展 | 使用上述真实图像推理，结果有时效；执行端仍由 FPGA 仲裁 |

测试：`python3 -m unittest discover -s host/fx2_camera/tests -v`。目前完成新增模块及隔离候选的电脑验证；未修改工作区生产顶层/现有约束，未加载固件、烧录或升级RK3588，实机链路未验收。

## FIMG v2（2026-10-24 直传相机）

`ai_coordinator/frames.py` 同时接受 FIMG v1 与 v2。v2 的 24 字节头字段 20 是头 20 字节的 CRC32，payload 后追加 4 字节尾 CRC32（头 20 字节 + payload）；坏尾帧会被丢弃。直传 FPGA 模块以 5/8 抽选取 640×375，用 128k 字弹性 FIFO 吸收帧内突发，不再冻结整帧。协议细节和验证证据见 `vendor/verilog-reusable-modules/modules/camera_fx2_direct/README.md` 与 `doc/摄像头直传高分辨率_20261024.md`。
