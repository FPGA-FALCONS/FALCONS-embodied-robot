# FPGA摄像头帧缓冲验证（2026-11-04）

- 候选修复：外部DDR三帧银行，完整帧发布，垂直空白期排空切换；未烧录。
- `regression_summary.json`：原81台加新增2台最终结果，合计83台、10698项场景检查；两台摄像头另有逐像素比较。
- `sim_camera_frame_buffer*.log`：最终的异步/错误注入和完整1024×600仿真日志。FIFO是仿真行为模型，不代表厂商IP/实板验证。
- `pnr_summary.json`、`falcons.rpt.txt`、`falcons_tr_content.html`：Gowin实际布线报告。16行同钟域setup/hold TNS均为0；系统最差0.004ns；全局仍有58个setup和18个hold违例端点。
- `build_inputs.json`：134项实际构建输入及SHA256，标明隔离目录从当前提交补齐的4份既有缺失调试源码；没有撤销原工作区的删除。
- 位流：`build/falcons_camera_frame_buffer_20261104.fs`，SHA256 `de424baa0ef8b271f6de29f31e303e86f5971e3a9adb0d1136199fa53b449a84`。原始全量日志和隔离源码在`build/camera_frame_fix_20261104/`。

实现与限制见[帧缓冲说明](../../FPGA摄像头帧缓冲与防撕裂.md)。仿真和布局布线完成，运动画面实屏结果待后续烧录验收。
