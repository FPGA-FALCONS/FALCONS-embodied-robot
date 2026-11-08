# FALCONS AI与主机模块

所有源码随本项目发布，FPGA实现见`rtl/`及`vendor/verilog-reusable-modules/modules/`。当前状态见[交接单](../doc/交接单.md)，AI操作与巡检见[巡检指南](../doc/AI视觉巡检.md)。10-24全栈包是历史发布。

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| AI协调器 | ai_coordinator/ | 四功能AI、语音/文本、独立UVC视觉、多点巡检/返航、双路录像、播报、唯一串口及部署工具 |
| 中文语音适配 | voice_bridge/ | 音频录制/转换、领域词表、中文策略与验收辅助 |
| 语义运动 | semantic_nav/ | 相机无关的运动口令、MOT1、显式串口测试工具；协调器默认只预览 |
| 视觉与原观察协议 | vision_bridge/ | 目标观测、显示适配、串口导出与回放；保留最新上游版本 |
| FX2相机 | fx2_camera/ | 历史FIMG v1/v2解析与回放；当前AI高速显示/地图使用ai_coordinator/inspection的AIF1 |
| USB相机 | usb_camera_v4l2/ | UVC模式探测和独立取帧；正式原生采集为ai_coordinator/native_v4l2_camera.py，focus260、720p；另保留独立探测工具 |
| USB相机(Windows) | usb_camera_win/ | 本机 DirectShow 固定焦距/曝光 + 9×6 棋盘格针孔标定；MSMF 后端不生效 |
| 视觉训练研究 | vision_training/ | 语义蒸馏、检测头/P3候选与对照评估；10-24 v17基线的历史研究；生产视觉已于10-29切换K541 P02，见[模型资产](MODELS.md) |
| 语音任务研究 | voice_task_demo/ | ASR与领域训练、评估、任务图和演示；正式入口为协调器 |

模型版本与实际文件校验由板端配置`artifacts`记录，并随发布导出`MODEL_MANIFEST.json`。C1b、SenseVoice、YOLO和女声缓存均沿用已核验基线；源码不携带权重；本车用户服务已配置启动加载既有模型。新环境应按各模块说明安装独立依赖，RK3588现有竞赛环境不得upgrade。
