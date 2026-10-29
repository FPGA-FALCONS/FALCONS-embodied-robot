# AI巡检：录像、报告与管理

状态核对：2026-10-28。当前任务操作见[视觉巡检](AI视觉巡检.md)，测试和最近部署见[验证记录](AI巡检验证记录.md)。

本功能仅记录RK承担的AI视觉巡检，不记录原FPGA摄像头页、雷达页、基础导航或其他基础任务。全部改动在RK侧；基础功能继续脱离RK运行。

## 录像内容和时基

- 任务确认开始后，先建立录像并确认两路已有可读编码片段，再放行第一条巡检GOTO。编辑、选点、普通预览不会创建录像。
- 摄像头与地图均在后台按10Hz生成，使用与屏幕相同的巡检渲染器。摄像头保留实拍图像、框及类别；地图保留地图、车位/朝向、路线、目标红点、编号、图例和任务文字。CH8切换三视图不影响两路记录。
- 两个816×420内容画面各追加28像素时间条，输出816×448。一个FFmpeg输入同时分成两路H.264 MP4，PTS与帧编号共用，每帧间隔100ms；编码CRF24，默认60秒一段。录像时间使用RK单调时钟调度，UTC由同一次起点换算，系统校时不会让视频倒退。
- 文件夹名为UTC时间加task编号和随机标识。时间条包含UTC、相对时间与帧编号；`frames.jsonl`保存逐帧时间、阶段、暂停/手动状态、当前路径点序号/总数/完成数、当前目标及相机/地图数据年龄。数据年龄用于录像说明，不是巡检定位的传感器时效判据。采样到相同源帧是允许的，源序列和年龄会保留，不伪称每帧都完成一次新推理。
- 两路在后台持续渲染；手动接管和暂停时保留专用相机/推理所有权，同时原有停止逻辑照常优先执行。录制不授权运动；当前巡检定位采用停车后固定等待与三帧平均，不以旧动态对时门槛作为准入条件。

## 开始、终止及异常

| 情况 | 处理 |
| --- | --- |
| 去程、跨路径点、停车识别、返程、暂停、手动接管 | 持续同一会话的双路录像；回光标不自动恢复运动 |
| 完成、取消 | 关闭编码输入、完成文件收尾，保存manifest和本次报告 |
| 退出巡检页/AI总关/切换到其他AI功能 | 取消当前任务，结束录像并释放专用相机所有权 |
| FPGA下电、reset、定位清零、控制串口断联 | 撤销旧任务，结束录像，保存旧坐标系报告；恢复连接不续录旧任务 |
| 相机、地图或高速链路缺失 | 短时仍记录暂停状态和明确的STALE提示；持续超过默认5秒取消任务并收尾，避免长期录制失效画面 |
| 空间/配额不足、编码失败、无法维持10Hz | 停车取消任务并尝试保存错误报告；不放行没有就绪录像的GOTO |
| RK服务正常退出 | 请求取消，关闭编码器并等待有界收尾 |
| RK断电或进程被杀 | 下次服务启动检查未完成记录，对齐两路完整MP4片段、裁掉不完整尾部并修正逐帧索引，标记interrupted；保存中断报告 |

分片和定期同步用于降低突然断电的损失；物理断电前尚未同步的尾部可能丢失，不能保证最后一帧落盘。恢复记录区分估计结束时间与恢复时间。SD卡不可读时保留已有字节，不执行在线文件系统修复。

默认保留至少2GiB空闲，单任务最多4GiB、总录像最多16GiB、单任务最长2小时（含暂停和手动时间）。达到限制会取消并收尾，不自动删除历史。可在 `inspection.recording` 调整这些正数限额、`source_timeout_s`和分段长度。正在录像的目录由进程锁保护，管理命令拒绝删除；结束后可由用户显式清理。

## 路径和自行检查、回放、清理

正式目录：

```text
/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1/inspection_recordings/
  20261027T040000Z_task1_0123abcd/
    camera-00000.mp4, camera-00001.mp4, ...
    map-00000.mp4, map-00001.mp4, ...
    manifest.json
    frames.jsonl
    report.latest.json
    report.json
    encoder.log
```

`report.json`与该次录像同目录，保存巡检报告、路径点队列及各点到达时间/位姿、目标、轨迹、路线版本、证据引用和终止原因；录制期间另有每秒更新的`report.latest.json`。分段MP4可单独复制读取；按manifest中的顺序和时基回放。

服务同时提供只读回放页面，默认仅监听RK的 `127.0.0.1:18766`。电脑建立SSH转发后打开 `http://127.0.0.1:18766`，选择记录并点击打开；可同时播放/暂停、跨分段拖动，并下载报告、清单和逐帧时间。示例：

```powershell
ssh -N -L 18766:127.0.0.1:18766 falcons-rk3588
```

在RK上，按当前发布目录运行管理命令：

```bash
FALCONS_AI_ROOT=/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1
FALCONS_RELEASE=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["release"])' "$FALCONS_AI_ROOT/active_release.json")
export PYTHONPATH="$FALCONS_AI_ROOT/python:$FALCONS_RELEASE/host"
python3 -m ai_coordinator.inspection.recording_cli --config "$FALCONS_AI_ROOT/config.json" list
python3 -m ai_coordinator.inspection.recording_cli --config "$FALCONS_AI_ROOT/config.json" show 记录ID
python3 -m ai_coordinator.inspection.recording_cli --config "$FALCONS_AI_ROOT/config.json" delete 记录ID --yes
```

删除要求完整记录ID，不接受任意路径；目录含未知文件或符号链接时会拒绝，需用户检查后自行处理。`recover`可显式再次检查中断录像；`serve --port 其他端口`可单独运行回放服务。基础页面不会产生录像文件，历史巡检报告和录像也不会因切页而自动删除。

## 报告中的路径点

`mission.waypoints_m`为按访问顺序排列的米制地图坐标，`waypoint_index`从0开始；去程全完成后等于点数。`waypoint_arrivals`逐点保存编号、目标坐标、实际位姿、单调时钟`at_s`和相对任务启动的`elapsed_s`。录像manifest提供单调时间与UTC对应关系；`frames.jsonl`的`relative_ms`每帧增加100，两路采用相同帧号。取消保留已完成与未完成点，不把部分完成写成全部成功。

另有运行目录下`runs/.../inspection_reports/`报告和`inspection_evidence/`关键帧/障碍层证据；录像同目录的`report.json`便于整次拷贝。后台摄像头与地图一直生成，当前屏幕选择的视图不会决定录制哪一路。

本功能已通过真实MP4、10Hz/同PTS、分段、跨点/手动/返航连续记录、进程中断恢复及同步回放测试；真实整车巡检全程录像仍待现场验收。测试与证据统一保存在[验证记录](AI巡检验证记录.md)，不在此复制逐轮发布包号。


手动接管仅暂停运动，保留相机采集和录像时间，不清空相机接收状态；真实断联/断流仍会按记录终止规则收尾。运行目录报告按`session-会话-task-任务-epoch-纪元.json`命名，证据也按这三个字段共同筛选，防止新任务混入旧图片及复位后覆盖报告。

存储修复或清理后可在页面按SWA重新检查，成功后再次SWA确认继续或新建任务。检查过程不自动发出导航命令。
