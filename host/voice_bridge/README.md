# USB 麦克风与离线中文语音模块

> 本目录为独立离线语音工具；下面“麦克风未到货”是初版测试背景。项目正式USB录音、ASR和播音已接协调器，当前状态见[协调器](../ai_coordinator/README.md)。

本目录将USB音频或已有WAV转成中文转写和任务预览，保留独立离线工具及其设备/故障测试。车辆正式录音、ASR及播放由ai_coordinator运行，其验收状态单独记录。

数据路径：录音／WAV → 选择音频通道 → 抗混叠重采样到 16 kHz → 能量检查 → Sherpa-ONNX 中文 ASR → 规则或可选语言模型 → JSON 预览。

## 现在可以运行：不需要麦克风

在 PowerShell 中运行以下命令。使用工作区已经存在的独立 Python 环境和模型，不安装、升级任何包：

```powershell
Set-Location D:/FALCONS_workspace/FALCONS-embodied-robot
$voicePython = 'D:/FALCONS_workspace/ai部分/FALCONS_AI_Agent_24h_20261018/FALCONS_AI_Agent_24h_20261018/.venv/Scripts/python.exe'
& $voicePython host/voice_bridge/voice.py doctor
& $voicePython host/voice_bridge/voice.py transcribe --wav 'D:/FALCONS_workspace/ai部分/ai_dual_model_poc/weights/asr/commands/find_tape.wav'
```

每次结果写入独立的 `recordings/时间戳_随机标识/`，含 `result.json` 和正常进入识别时的 `asr16k.wav`；录音模式另存 `capture.wav`。默认不向网络传输音频或文本。录音、模型、缓存不进入 Git。

文件输入只接受最长 60 秒、最多 8 通道、8–192 kHz 的 PCM16 WAV。默认选择通道 0，不将双通道直接平均，避免参考通道或反相音频抵消。非 16 kHz 输入通过 SciPy 重采样。返回码：0 为已产生非空转写（不保证内容正确），2 为错误，3 为静音能量或空转写，130 为用户中断。

## 到货后：先枚举，再录音

Windows 下：

```powershell
& $voicePython host/voice_bridge/voice.py devices
# 将 0 改成枚举得到的 USB 麦克风 id，不要直接采用电脑默认输入。
& $voicePython host/voice_bridge/voice.py record --device 0 --seconds 5
& $voicePython host/voice_bridge/voice.py listen --device 0 --seconds 5 --once
```

`listen` 是**回车触发、固定时长录音**，默认 5 秒，允许 0.2–30 秒；不是持续监听、流式端点检测或松手即结束。去掉 `--once` 可逐次回车录音，输入 `q` 退出。自动尝试单声道 16 kHz／48 kHz／44.1 kHz；若设备只支持双声道，可显式增加 `--rate 48000 --channels 2 --channel 0`（`record` 不接受 `--channel`，它保存所有录入通道）。通道号从 0 开始，软件选择通道不等于控制物理双麦算法。

RK3588/Linux 中仅使用板上已有环境和 `arecord`。先检查，不执行系统或包升级：

```sh
python3 host/voice_bridge/voice.py devices --backend alsa
python3 host/voice_bridge/voice.py doctor --backend alsa --model-dir /实际模型目录
# CARD 名称以 devices 列出的 PCM 名称为准，USB 只是示例。
python3 host/voice_bridge/voice.py listen --backend alsa --device 'plughw:CARD=USB,DEV=0' --model-dir /实际模型目录 --seconds 5 --once
```

优先用 `plughw:CARD=设备名,DEV=0`，避免 USB 插拔后声卡数字编号变化。若需硬件原生格式，显式传入 `--rate 48000 --channels 2 --channel 0`。缺少依赖时停止并记录版本，后续在独立目录／环境选择与板上系统兼容的包；不要替换现有 Python、NPU 驱动、RKNN 或竞赛依赖。

## 识别、超时与任务范围

CPU 推理使用现有的 `tokens.txt`、`encoder-epoch-99-avg-1.int8.onnx`、`decoder-epoch-99-avg-1.int8.onnx`、`joiner-epoch-99-avg-1.int8.onnx`。模型路径优先级：`--model-dir` → `FALCONS_ASR_MODEL_DIR` → 模块内 `models/asr/` → 当前工作区已有 ASR 模型目录。本轮没有重新下载或重新分发模型，来源和许可沿用原资产记录，正式分发前需核验。

识别依赖 NumPy、Sherpa-ONNX；非 16 kHz 输入还需 SciPy。电脑验证环境为 Python 3.13.7、NumPy 2.5.2、SciPy 1.18.1、Sherpa-ONNX 1.13.8；这些是电脑实测版本，不是 RK3588 安装要求。`devices`、`doctor` 可在没有这些包时运行并给出诊断。Windows 采集使用标准库 WinMM，Linux 采集调用已有 `arecord`。

`--asr-timeout 15` 默认限制私有 ASR 子进程的运行时间，**包含加载模型，不包含录音**。超时会终止并回收该进程；进程启动、音频处理和结果落盘仍有少量开销，因此不是整个命令的硬实时期限。每段录音重新加载模型，便于隔离故障，但有固定延迟，板端性能需单独实测。

简单规则只完整匹配“找胶带”“请帮我寻找螺丝刀”等明确句型；识别目标为胶带、螺丝刀、AD 钙奶、营养快线。用户要求禁用“交代”后，默认先做领域纠错：明确搜索语境中的“找交代”“找到交代”等改为胶带，否定和条件保留；普通或含糊用法拒识为 `unknown`。原始转写保存在 `text`，规范化结果另存 `task_text`，拒识时为 `null`；`domain_vocab` 记录原因和纠错。这不是修改声学词表。诊断时可用 `--disable-domain-normalization`。否定、多个目标、附加移动动作和不支持的抓取操作默认返回 `unknown`；“平”不会强行改成停止。语音停止识别不能代替实体急停。

默认仍为 `--decoding-method greedy_search`。可选 `modified_beam_search`，或 `--hotwords-file D:/实际热词.txt --hotwords-score 1.5`；提供热词时自动选择 beam、最多 4 条活动路径。热词文件为 UTF-8，每行一个词，分数允许 0–5。四类热词文件为 `D:/FALCONS_workspace/ai部分/voice_domain_adaptation_20261024/hotwords.txt`，热词不能保证专名识别正确。

可选 `--llm-url http://127.0.0.1:8080` 指向自己启动的 OpenAI 兼容服务根地址，访问其 `/v1/chat/completions`。只有需要解析的复杂句才调用，最大输出 80 token，请求 `enable_thinking=false`；服务器是否遵从该参数仍须验证。转写发送到显式指定地址。结果检查 JSON、键、目标白名单、目标在转写中出现以及完成状态；检查不能证明理解正确。`--llm-timeout 5` 是客户端等待设置，不保证服务端终止推理，也不是交互总期限。另一个已实跑的[四类专项回放入口](../voice_task_demo/README.md)管理自己的私有模型进程，实际串联 ASR、Qwen 和图片检测，并在超时终止自己的模型进程；它不改变此通用接口的超时语义。

所有结果始终带有：

```json
{"type":"voice_result","requires_confirmation":true,"execution_allowed":false,"motion_authorized":false,"observation_only":true}
```

模块不连接 FPGA／串口／导航，不执行车辆动作；说“确认”也只产生确认意图预览，没有当前任务执行状态机。能量检查仅过滤低能量输入，不是完整 VAD，电机噪声仍可能进入 ASR。本版未实现唤醒词、回声消除、声源定位，也未验证硬件宣传的降噪、增益或波束成形。

## 验证与待验收

```powershell
& $voicePython -m unittest discover -s host/voice_bridge/tests -v
```

2026-10-24：最初 25 项自动测试通过，随后新增热词透传和领域政策检查，目前 27 项通过，任务回放另有 13 项。另完成 9 个初期离线回放／异常用例，CLI、设备枚举及无识别依赖诊断均通过。ALSA 成功与故障为模拟，Windows 只做设备枚举，未做真实采集。ASR 在旧 TTS 指令中有明显错字，不能据此宣布精度合格；后续四类对照结果见回放报告。

本机报告：`D:/FALCONS_workspace/ai部分/voice_module_validation_20261024/验证报告.md`。电脑耗时不能直接当作 RK3588 延迟，也不代表 USB 硬件性能。

到货后固定同一组指令，覆盖 1／2／3 米、停车／电机运行、不同说话人，分别记录原始转写、字错率、完整任务正确率、误触发和延迟。特别单独验证停止、否定、纠正及多目标；先完成录音回放验收，再决定是否更换 ASR 或引入热词／流式识别。
