# RK3588 AI协调器

状态核对：2026-11-09。生产服务覆盖语义导航、独立目标识别、聊天机器人、视觉巡检四个功能；各模型复用RK已有资产。FPGA运行基础相机、雷达、导航、闭环和最终手动仲裁，RK运行模型、AI任务、图像渲染及记录，不安装ROS2/Nav2。

## 当前入口与数据流

| 功能 | 数据和执行 |
| --- | --- |
| 语义导航 | USB麦克风 → SenseVoice → 规则/C1b → 有界MOT1动作 → FPGA运动桥；不依赖相机或FX2 |
| 目标识别 | RK独立UVC → K541 P02 RKNN → 连续3帧确认 → 当帧框/中文类别 → FX2显示；只识别，不授予巡检导航权限 |
| 聊天机器人 | 语音 → 现有Qwen3.5 0.8B RKLLM → Kokoro → USB扬声器；不进入运动解析 |
| 视觉巡检 | RK有序路径点与停车后三帧H定位 → 独立视觉障碍层/当前目标 → FPGA现有导航 → 真实回执；两路10Hz录像、报告及发现缓存播报 |

FPGA OV5640图像不传RK；RK使用自己的UVC相机。旧FIMG输入、FAM1找物及`patrol.py`人工拾取回放属于保留的独立工具，不是当前视觉巡检的执行通路。其历史合同见[FULLSTACK](FULLSTACK.md)、[研究依据](RESEARCH.md)。

模型在既有worker中加载，协调器共享唯一串口和异步任务循环。RK保留最近2秒位姿，在送入推理前按原生取帧`received_at_s−0.3s`选取最近位姿并随帧冻结，用H粗略投影后按同类别、地图距离<30cm过滤已确认目标；不使用识别完成时刻估计取帧时刻，粗略目标坐标不进入正式记录。新目标取得停车回执后等0.3秒并用三帧正式定位；不使用20ms门槛或传感器年龄门禁。去重贯穿本次任务的去程、手动暂停恢复和返程，新任务清空。相机模式1280×720、focus260、去畸变H及正式定位有效凸包仍须匹配配置。

## 操作与文档

左侧AI模块确认开启总开关，CH8选择四项，SWA确认。SWB下拨录音、上拨结束，进入/恢复后须新的上拨→下拨。SWC手动始终由FPGA最高优先处理，AI留页保留总开关；回光标不自动续跑。

- [AI页面和原三个功能](../../doc/AI模块页面与遥控交互_20261024.md)
- [多点巡检使用/架构](../../doc/AI视觉巡检.md)、[录像与报告](../../doc/AI巡检录像与报告.md)、[已测/待测和部署证据](../../doc/AI巡检验证记录.md)
- [地面H接口](inspection/HOMOGRAPHY.md)、[Windows标定](../usb_camera_win/README.md)、[UVC工具](../usb_camera_v4l2/README.md)
- [模型自启](../../doc/archive/camera/RK相机与自启验证_20261025-27.md#report-02)、[固定任务播报工具](SPEECH.md)、[模型与缓存](../MODELS.md)

## 板端与检查

运行根：`/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1`。`active_release.json`给出代码release，`config.json`给出实际模型、设备身份、功能和标定，依赖在`python/`，运行结果在`runs/`。生产代码release另位于`/home/orangepi/workspace/falcons_ai_interactive_20261024/releases/`；不要假定都在运行根的releases下。

`falcons-ai-coordinator.service`为已启用linger的用户服务，开机加载既有ASR、语义、聊天、TTS及视觉模型，完成后尝试播放一次就绪音。`config.example.json`是shadow示例，包含历史v17模型路径，不能直接当作当前生产配置。新环境按[模型资产](../MODELS.md)配置K541 P02并核对实际路径；示例的关闭开关保持用于离线操作。不能凭示例的关闭开关断言板端未启用模型或串口。

```bash
systemctl --user status falcons-ai-coordinator.service
journalctl --user -u falcons-ai-coordinator.service -n 30 --no-pager
curl -s http://127.0.0.1:18765/status

FALCONS_AI_ROOT=/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1
FALCONS_RELEASE=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["release"])' "$FALCONS_AI_ROOT/active_release.json")
python3 "$FALCONS_RELEASE/host/ai_coordinator/main.py" --config "$FALCONS_AI_ROOT/config.json" status
```

HTTP仅监听`127.0.0.1:18765`。现有`/text`、`/audio`、`/capture`、`/image`、`/stop`、`/warmup`等入口由同一Runtime处理，`--plan-only`只分析；巡检开始/继续仍使用物理SWA，不让文字请求替代该确认。

## 源码分工与验证

`runtime.py/gateway.py`负责worker和唯一串口，`ai_module.py`负责原三个AI功能，`startup.py/install_autostart.py`负责开机加载，`inspection/`独立负责巡检。`deploy_inspection.py`核对源码清单、编译原有UVC工具、测试后按事务切服务，保留已有模型/配置，不升级系统包。

```powershell
$env:PYTHONPATH = (Join-Path $PWD 'host')
python -m unittest discover -s host/ai_coordinator/tests -v
```

各轮主机和板端测试按日期、范围及跳过项记录在[验证记录](../../doc/AI巡检验证记录.md)，不在此复制发布号、测试总数或某次外设缺失状态。成果拍摄和实物反馈不代表所有故障边界均已验收；当前设备/服务状态需实际连接RK检查。

FAM1仍为独立任务协议参考，生产FPGA不提供完整FAM1找物/拾取回执；语义MOT1串口写入也不等于物理动作完成。巡检所用保留事务接口确有真实FPGA状态回传，三者不能混为同一协议。
