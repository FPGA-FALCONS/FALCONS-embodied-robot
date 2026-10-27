# FALCONS AI与主机模块

所有源码随本项目发布，FPGA实现见`rtl/`及`vendor/verilog-reusable-modules/modules/`。先读[整车与板端交付入口](../doc/全栈上板与队友下载_20261024.md)。

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| AI协调器 | ai_coordinator/ | 语音/文本、找物、巡航任务状态、视觉入口、播报、FPGA任务网关与部署工具 |
| 中文语音适配 | voice_bridge/ | 音频录制/转换、领域词表、中文策略与验收辅助 |
| 语义运动 | semantic_nav/ | 相机无关的运动口令、MOT1、显式串口测试工具；协调器默认只预览 |
| 视觉与原观察协议 | vision_bridge/ | 目标观测、显示适配、串口导出与回放；保留最新上游版本 |
| FX2相机 | fx2_camera/ | FIMG v1/v2解析、设备选择、离线回放与真实USB吞吐工具 |
| USB相机 | usb_camera_v4l2/ | UVC模式探测和独立取帧；协调器适配位于ai_coordinator/v4l2_camera.py |
| USB相机(Windows) | usb_camera_win/ | 本机 DirectShow 固定焦距/曝光 + 9×6 棋盘格针孔标定；MSMF 后端不生效 |
| 视觉训练研究 | vision_training/ | 语义蒸馏、检测头/P3候选与对照评估；未通过候选不替换v17 |
| 语音任务研究 | voice_task_demo/ | ASR与领域训练、评估、任务图和演示；正式入口为协调器 |

模型版本与实际文件校验由板端配置`artifacts`记录，并随发布导出`MODEL_MANIFEST.json`。C1b、SenseVoice、YOLO和女声缓存均沿用已核验基线；发布源码不默认加载或下载任何模型。新环境应按各模块说明安装独立依赖，RK3588现有竞赛环境不得upgrade。
