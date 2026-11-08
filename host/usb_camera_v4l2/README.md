# RK独立UVC相机与原生采集

当前AI视觉只使用RK直连UVC；FPGA OV5640只在原摄像头页显示。正式协调器后端为ai_coordinator/native_v4l2_camera.py，使用本目录native/capture.c辅助程序；独立探测工具不会改变运行服务或加载模型。

## 正式配置

相机身份0bda:5856、序列200901010001，图像功能video-index0；具体video编号和USB插座可变。模式1280×720 MJPG，目标30fps，focus260、自动对焦关闭。内参/H绑定此模式，不能仅按相机节点名判断相同视野。

每次打开/重连先协商模式、设置并读回手动对焦，等待镜头稳定；逐帧附focus_value、focus_locked及V4L2读回证明。原生工具调用为`capture DEVICE WIDTH HEIGHT FPS FIXED_0_OR_1 [MANUAL_FOCUS]`，48字节头保存驱动序号/时间来源和图像长度。

主机只保留最新待处理帧，正在推理的JPEG有独立租期。驱动时间戳表示其来源，不等于已测曝光时刻；当前巡检按相机延迟≤0.3秒、停车后等待0.3秒并处理三帧，不使用精确动态对时门槛。

## 独立探测与取帧

在已有Python/OpenCV环境、仓库根目录执行：

```bash
PYTHONPATH=host python3 -m ai_coordinator.uvc_prepare probe --out uvc_probe.json
PYTHONPATH=host python3 -m ai_coordinator.uvc_prepare capture \
  --device /dev/v4l/by-id/usb-Generic_USB_Cam_200901010001-video-index0 \
  --width 1280 --height 720 --fps 30 --fourcc MJPG --frames 60 --out uvc_capture
```

先核对probe实际身份；同型号有多台时显式区分，不自动用video0。`--fixed-frame-rate`可临时比较动态降帧影响，采集工具完成后恢复其负责的临时控制。收帧率不等于推理率或端到端显示时延。

本目录probe_v4l2.py/capture_v4l2.py也保留独立实验能力；生产使用上述身份选择和原生后端，不从旧按index脚本推断实际相机。

## 标定与状态

当前720p K/D、14点去畸变H及40cm起凸包见[H接口](../ai_coordinator/inspection/HOMOGRAPHY.md)，采图方法见[Windows工具](../usb_camera_win/README.md)。只对投影点去畸变，显示及模型输入仍为原图。

原生取帧、focus260、实际三视图已验证；最新多点任务和实车范围见[巡检验证](../../doc/AI巡检验证记录.md)。不得以独立取帧成功代替自动绕障或动态定位验收。
