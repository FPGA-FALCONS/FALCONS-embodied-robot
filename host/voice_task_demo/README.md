# 四类物体中文语音任务回放

> 本目录保留合成音频/图片回放，不代表当前板端尚未部署语音或相机。生产四功能AI及多点巡检见[协调器](../ai_coordinator/README.md)。

本机已跑通：Windows 离线中文合成音频 → 真实 ASR → 关闭思考的 Qwen 专项模型 → 真实 YOLO 静态图片检测 → 条件任务回放。目标为胶带、螺丝刀、AD 钙奶、营养快线。

**音频是合成声音，图像是已有开发图片，导航和返回起点是模拟。没有接麦克风、实时相机、RK3588 或车辆，也没有输出运动命令。** `success` 仅表示脚本场景的预期结果匹配；所有结果保留 `physical_task_completed=false`。

## 一条命令试用

在 PowerShell 中使用工作区现有环境，无需安装包：

```powershell
Set-Location D:/FALCONS_workspace/FALCONS-embodied-robot
$voicePython = 'D:/FALCONS_workspace/ai部分/FALCONS_AI_Agent_24h_20261018/FALCONS_AI_Agent_24h_20261018/.venv/Scripts/python.exe'
& $voicePython host/voice_task_demo/demo.py --text '先找胶带，找不到再找螺丝刀，找到后返回起点。' --scene fallback --simulate-home-arrival
```

该入口已实际运行，原始 ASR 为“先找交代找不到再找螺丝刀找到后返回起点”，规范化后为“先找胶带找不到再找螺丝刀找到后返回起点”。模型输出：

```json
{"i":"search","t":"tape","f":"screwdriver","r":true}
```

回放路径为 `primary → fallback → fallback_verify → return_home → done`。其中 `fallback_verify` 是检测类别、置信度和框结构检查，未调用独立视觉语言模型；`--simulate-home-arrival` 只提供模拟导航回执。没有此参数时，要求返回起点的任务因缺少回执失败。

用 `--wav D:/实际文件.wav` 替换 `--text` 可使用已有 PCM16 音频；`--scene primary|fallback|absent` 选择固定图片回放。默认声音为 Microsoft Huihui Desktop，可通过 `--voice 'Microsoft Kangkang'` 选男声。TTS 直接写 WAV，不播放后重新录制，不经过麦克风。Windows 使用系统 SAPI；Linux 使用 `--wav`。

每次输出保存在 `D:/FALCONS_workspace/ai部分/voice_domain_adaptation_20261024/demos/随机标识/`，包含音频、原始转写、纠错记录、模型输出、检测标框和任务路径。也可指定一个尚不存在的 `--out` 目录。

## “交代”禁用策略

由用户明确要求加入，默认启用：

- 明确搜索语境的“找交代”“寻找交代”“找到交代”归一化为胶带。
- 否定和后续条件保留，例如“不要找交代”变为“不要找胶带”，不转为搜索。
- 仍含“交代”的含糊或普通表达，例如“把事情交代清楚”，不调用任务模型，返回 `unknown`。
- 原始 `text` 不改；供任务解析使用的 `task_text` 单独保存，含糊时为 `null`。

实现位于 `host/voice_bridge/domain_vocab.py`。这是识别后的领域纠错／拒识，**没有修改声学模型或从 ASR 解码词表中删除“交代”**。不将“概念”或单独“奶”强行解释为 AD 钙奶。

## 模型与证据

语言产物位于 `D:/FALCONS_workspace/ai部分/voice_domain_adaptation_20261024/`：

| 文件 | 用途 |
| --- | --- |
| `adapter/`、`merged_hf/` | 本轮 LoRA 与合并权重 |
| `domain-Q8_0.gguf` | 本机测试模型，811,842,976 字节 |
| `protocol.json`、`train.json`、`training_metadata.json` | 冻结配置、555 条训练数据及实际训练记录 |
| `export_audit.json` | 权重、tokenizer 和 GGUF 校验 |
| `e2e_run_1/` | 首轮对照，失败保留 |
| `policy_after_user_request/` | 首次政策回放，含“找到交代”未匹配的已知缺陷 |
| `policy_after_user_request_v2/` | 修复后重放，未覆盖旧结果 |
| `spoken_alias_probe/` | 事后中文简称探测，另计 |
| `验证报告.md` | 最终证据、失败与部署边界 |

从前次 `qwen_offline_distill_pilot_20261023/merged_hf` 继续做最终短答案 SFT，关闭思考；rank 8、alpha 16、batch 4、128 步、学习率 5e-5，5,411,328 个可训练参数，5090 D 上训练约 56.7 秒。样本为合成模板和旧训练回放，不是本轮新蒸馏的推理链。视觉权重未训练；153 个视觉张量不变不能证明视觉能力保持，本模型本轮只作文本解析。

首轮合成开发回放：16 个场景 × 两种声音，共 32 段音频。

| ASR 配置 | 旧专项学生 pilot | 本轮四类专项 domain |
| --- | ---: | ---: |
| 14M greedy | 21/32 | 27/32 |
| 同一 beam，不加词 | 19/32 | 25/32 |
| 同一 beam，加四类热词 | 23/32 | 29/32 |
| 纯文字诊断，另计 | 15/16 | 16/16 |

加入禁词政策的 v2 回放中，同一热词 ASR 文字下分别为 **27/32 和 29/32**。两个表的处理条件不同，不能跨表把差值都归因于微调。

首轮 domain 热词模式三处失败：女声 AD→“概念”、男声 AD→“奶”、男声纠正→“沼罗斯刀”，均未执行猜测目标。事后简称探测 domain 为 5/10，当前 14M ASR 的专名识别仍不足。音频／图片用于开发、诊断或方案选择，不是独立盲测；pilot 是已微调的父模型，**不是官方原版基线**。

视觉使用已有 `field_adaptation_20261021_v16_lab122/runs/v17s_replay/weights/best.pt`，每类排序前两张旧图片，640 输入、置信度 0.3。没有本轮重训 YOLO。`background` 含 AD 钙奶，并非真实空房间；一组图片未检测到不代表完成全场搜索。

## 可选 SenseVoice 对照

保留小型 transducer 默认路径，另提供 `sensevoice.py`，使用已有 sherpa-onnx 的 CPU 推理。模型数据从 sherpa-onnx 维护者 Hugging Face 仓库固定 revision 下载并校验；只取 ONNX、tokens、许可说明和 README，不运行上游代码：

```powershell
& $voicePython host/voice_task_demo/download_sensevoice.py --out 'D:/FALCONS_workspace/ai部分/sensevoice_int8_20261024/新的模型目录'
& $voicePython host/voice_task_demo/demo.py --text '请帮我寻找AD钙奶。' --asr-engine sensevoice --asr-model 'D:/FALCONS_workspace/ai部分/sensevoice_int8_20261024/hf_snapshot'
```

来源：[sherpa-onnx 文档](https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html)、[维护者模型仓库](https://huggingface.co/csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17)。包内 LICENSE 只指向 FunASR 许可说明，不能据此宣称权重可按 MIT／Apache 自由再分发。最终结果见本机验证报告。

此新增对照已实跑：同一原32段音频中，pilot为30/32、domain为32/32；事后简称10段两者均10/10。SenseVoice的原集CER为1.76%，本机CPU识别处理中位数1.377秒（含进程／加载）；14M热词配置为11.27%、1.082秒。SenseVoice模型文件约228.2MiB，板端RAM与速度未知，暂不据此替换默认引擎。AD钙奶的SenseVoice单命令入口已串联实际YOLO成功回放。分数来自已看过的开发音频，不能称独立验收。

复现该对照时运行 `evaluate_asr.py`，参数为 `--work`、新的 `--out`、`--asr-model` 和 `--pilot`，路径同上。该实验沿用首轮图片检测证据；`demo.py`则重新执行实际图片推理。

## 复现与部署边界

```powershell
& $voicePython -m unittest discover -s host/voice_bridge/tests -v
& $voicePython -m unittest discover -s host/voice_task_demo/tests -v
& $voicePython host/voice_task_demo/evaluate_policy.py --work 'D:/FALCONS_workspace/ai部分/voice_domain_adaptation_20261024' --pilot 'D:/FALCONS_workspace/ai部分/qwen_offline_distill_pilot_20261023' --asr-model 'D:/FALCONS_workspace/ai部分/ai_dual_model_poc/weights/asr' --out 'D:/FALCONS_workspace/ai部分/voice_domain_adaptation_20261024/新的政策结果目录'
```

27 项语音模块与 13 项任务／词汇政策测试通过。`prepare.py`、`train_domain.py`、`validate_domain.py` 保留训练与导出审计流程；重跑应使用新目录。`run_eval.py` 保留未加政策的原始对照，当前使用入口是 `demo.py`。

本机 Qwen 使用私有回环 llama.cpp 进程、2048 上下文、最大输出 80 token、关闭思考、每请求 15 秒上限，超时终止自己的进程。ASR 同样独立进程并限制 15 秒，包含模型加载；这些不是含录音、视觉、导航的全链总时限。GPU 约 0.047 秒的解析中位数不能当作 RK3588 耗时。GGUF Q8 不等于 RKLLM W8A8，后者尚未转换／实测。

麦克风到货后，用未用于开发的真人指令在 1／2／3 米、停车／电机运行条件下验证。板端分别测 ASR、解析、视觉、冷启动和并发资源；当前结果支持继续开发，尚不支持实车语音控制已验收的结论。
