# 模型与缓存资产

GitHub保存模块源码、训练/评估工具、提示词、配置与测试；发布包包含Kokoro女声A的48段缓存音频及聊天模板。运行权重沿用现有RK3588文件，SHA256与大小见`doc/evidence/ai_release_20261024/MODEL_MANIFEST.json`，Release中也附同名清单。

| 模型 | 板端基线 | 说明 |
| --- | --- | --- |
| Qwen3.5-0.8B C1b | c1b-Q8_0.gguf，811842976字节 | 中文领域特化，关闭思考；本项目权重，不冒充官方直接下载版本 |
| SenseVoiceSmall | model.int8.onnx，239233841字节 | CPU INT8中文ASR，配套tokens.txt |
| YOLO v17 | v17_640_fp16.rknn，21812469字节 | 四类目标，RK3588 RKNN FP16；新检测头候选未达门槛，不替换基线 |
| 播报 | Kokoro zf_001/sid3缓存 | 48段PCM16/16kHz，板端直接播放，不需加载TTS模型 |

现有板子的权重已就位。本次不重新下载/覆盖，不升级驱动或Python环境。迁移新板时按清单从现有板备份这些文件，保留版本与SHA；自定义C1b和v17不能用名字相似的官方模型替代。上游架构来源分别为Qwen、FunAudioLLM/SenseVoice及项目YOLO训练；本项目的实际文件与接口合同以清单为准。

RKLLM W8A8转换尚未完成，本次语言模型仍是CPU GGUF。是否解除HOLD、启用模型/摄像头/运动，需要现场稳定性与实车验收；GitHub模块发布不代表这些项目已经完成。
