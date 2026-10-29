# 模型与缓存资产

GitHub保存模块源码、训练/评估工具、提示词、配置与测试；发布包包含Kokoro女声A的48段缓存音频及聊天模板。运行权重沿用现有RK3588文件，SHA256与大小见`doc/evidence/ai_release_20261024/MODEL_MANIFEST.json`，Release中也附同名清单。

| 模型 | 板端基线 | 说明 |
| --- | --- | --- |
| Qwen3.5-0.8B C1b | c1b-Q8_0.gguf，811842976字节 | 中文领域特化，关闭思考；本项目权重，不冒充官方直接下载版本 |
| SenseVoiceSmall | model.int8.onnx，239233841字节 | CPU INT8中文ASR，配套tokens.txt |
| YOLO v17 | v17_640_fp16.rknn，21812469字节 | 四类目标，RK3588 RKNN FP16；新检测头候选未达门槛，不替换基线 |
| 聊天 | 原版Qwen3.5 0.8B RKLLM | 与C1b语义GGUF独立，使用已有RKLLM环境 |
| 播报 | 既有固定短句缓存及Kokoro动态TTS | 聊天实时合成；巡检四句“发现xx”预合成缓存于inspection_speech/，启动也加载TTS模型 |

现有板子的权重已就位。本次不重新下载/覆盖，不升级驱动或Python环境。迁移新板时按清单从现有板备份这些文件，保留版本与SHA；自定义C1b和v17不能用名字相似的官方模型替代。上游架构来源分别为Qwen、FunAudioLLM/SenseVoice及项目YOLO训练；本项目的实际文件与接口合同以清单为准。

C1b语义分支仍使用既有GGUF；聊天分支已使用原版RKLLM，不能用旧“RKLLM尚未完成”概括全部语言功能。模型自启、相机与串口已部署，具体实物验收及最近代码见[验证记录](../doc/AI巡检验证记录.md)。10-24模型清单是对应资产的历史指纹，新板迁移时仍需核对实际config.json和资产清单。
