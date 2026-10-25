# RK独立UVC相机准备证据

对应 [准备与验证说明](../../RK独立UVC摄像头准备与验证_20261025.md)。这些记录仅证明本次UVC设备识别、真实取帧及相机worker接入，不证明目标检出、地面坐标、双目同步、FPGA视频回传或巡检运动。

- `SOURCE.json`：独立源码包SHA256、193项文件清单规模及新增工具/测试的哈希；源码包无模型权重，正式服务未激活该目录。
- `probe.json`：真实by-id/by-path、VID/PID、内核QUERYCAP、支持模式及曝光/对焦控制。只有video-index0是图像入口，index1是元数据。
- `board_tests.txt`：RK上22项相关软件测试全部通过；`local_tests.txt`为本机22项，其中1项Linux专用检查跳过。
- `initial/`：首次三种分辨率在原动态帧率设置下的约20fps记录，以及临时关闭后的30.01fps对照；未达到速率阈值的结果保留为失败，未改写为成功。
- `capture_720p_fixed_rate/`：新版工具自动临时固定帧率，300帧主机实测29.96fps，控制项读回恢复成功。含关闭状态的候选配置和必须单独核验的控制前置条件。
- `worker_smoke.json`和`camera_worker_smoke.py`：直接使用现有Camera类读取3张真实1280×720图像，不加载模型、不使用运动串口；源时间仍标记未验证。
- `before.json`、`after.json`：正式配置、active_release及服务文件哈希、服务PID/重启数及相机状态完全一致；`VALIDATION.json`为最终核对回执。

原始PNG不纳入Git：保存在RK独立工具目录，以及本机 `build/rk_uvc_20261025/evidence_v2/capture_720p_fixed_rate/`；图片SHA256保留在summary中。已下载并查看图像，当前为桌面/地面视野，不是最终安装标定图。

本次未安装/升级软件，未改FPGA逻辑、构建或烧录，未发送运动指令。现有模型服务保持运行，正式相机仍disabled。新源码仅准备在 `/home/orangepi/workspace/falcons_uvc_prepare_20261025_v2`。
