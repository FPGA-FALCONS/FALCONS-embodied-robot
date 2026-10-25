# USB/UVC 摄像头直连 RK3588（周末调试）

## 2026-10-25 已接入实物，当前推荐入口

已识别 `0bda:5856` UVC相机，图像节点为稳定by-id路径的 `video-index0`；`video-index1`是元数据节点，不能视为第二只相机。新增 `python3 -m ai_coordinator.uvc_prepare probe/capture`，支持不打开流的探测、稳定设备选择、协商核验、独立进程超时和临时固定帧率测试。实测720p MJPG在关闭动态帧率时约30fps，测试后恢复原设置。

当前用户约定覆盖下面历史流程：FPGA摄像头只在原摄像头页显示，不上传RK；AI图像仅来自RK直连相机。现有AI服务本次未切换，新工具在独立目录准备。命令、设备、候选配置及验证边界见 [UVC准备与验证](../../doc/RK独立UVC摄像头准备与验证_20261025.md)。

下面保留2026-10-24独立探测脚本说明。旧脚本按video索引探测，当前整车系统有ISP/CIF等非USB节点，不宜直接使用默认index0；使用上面的稳定路径工具。

## 2026-10-24 历史流程

2026-10-24。用户已购买 USB 摄像头，到货后优先作为 AI 主链路；FPGA 直传保留为可选演示。
本目录只在 RK3588 上做**只读探测/取帧**，不安装包、不升级、不改协调服务、不加载模型。

## 到货后先探测

```bash
cd /path/to/usb_camera_v4l2_20261024
python3 probe_v4l2.py --json-out probe.json
```

- 有 `v4l2-ctl` 时会列出 `--info` 和 `--list-formats-ext`；没有就只读 sysfs/OpenCV，不安装 `v4l-utils`。
- 看是否出现 `MJPG 1920x1080@30`（USB2 必须用 MJPEG；USB3 可用 YUYV）。
- `lsusb -t` 确认摄像头挂在哪一级：优先板载 USB3.0 口。

## 取帧与测速

```bash
python3 capture_v4l2.py --index 0 --width 1920 --height 1080 --fps 30 \
  --fourcc MJPG --frames 120 --out capture_1080p
```

输出 `summary.json` 和三张 PNG。`measured_fps` 是主机收帧率，不是传感器时间戳；
如果低于 25 fps，先换 USB3.0 口/确认 MJPEG，再考虑降分辨率。

## 接入协调器

- 先保持协调器 HOLD/shadow，不改运行服务。
- 将 V4L2 取帧接到视觉 worker（OpenCV `VideoCapture` 或 GStreamer），保留现有 FX2/FIMG 解析器作为可选 FPGA 支路。
- 不覆盖 Codex 正在复核的 `host/ai_coordinator/` 接口；等相机型号/取帧结果出来后再定接口。
- 任何板端部署/激活需用户明确批准，且先与 Codex 在 `ai部分/AI_总集成协同_20261024.md` 写清版本与时间。

## 常见故障

| 现象 | 处理 |
|---|---|
| 无 `/dev/video*` | 换 USB 口/线；`dmesg | tail` 看枚举 |
| 打开失败/权限 | 确认用户 video 组；不要改系统服务 |
| 只有 5–10 fps | USB2 + YUYV 带宽不够，改 MJPG；或换 USB3 口 |
| 画面过暗/太近 | 手动对焦/曝光；机器人近距建议广角低畸变镜头 |
| 颜色异常 | OpenCV 默认 BGR；送 YOLO 前按训练时的 RGB/BGR 约定转换 |

## 边界

- 本目录没有真实相机测试记录；到货后把 `probe.json`、`capture_*/summary.json` 和 PNG 放进 `evidence/`。
- 板端环境禁止 upgrade；不要为了 V4L2 安装/升级 OpenCV。
- FPGA 直传位流 FX2 时序未闭合（40.804 MHz < 48 MHz，5 ns 暂定预算），不得直接烧录；本目录与 FPGA 路径相互独立。
