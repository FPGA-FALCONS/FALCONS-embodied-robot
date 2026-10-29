# AI → 运动控制：仿真链路（2026-10-19）

> 本文为独立视觉伺服建议的历史仿真链，未成为现行巡检执行器。现行巡检复用FPGA导航，不发送该建议驱动车，见[指南](../../doc/AI视觉巡检.md)。

> 状态：**宿主侧视觉伺服建议 + 真实 `chassis_ctrl` 自动端口的 RTL 联合仿真通过；未接串口、未改生产顶层、未上板、未实车。**
> 这不是“视觉已经控制实车”，而是把 AI 观测安全地变成可检查的运动建议，并在仿真中验证它能被底盘自动端口接收、超时清零、手动/急停优先。

## 1. 数据链

```text
schema-v2 JSONL (YOLO+Qwen)
  → VisionBridge（类别/状态/TTL/冲突校验）
  → MotionPlanner（bbox 中心→差速，bbox 面积→中心速度）
  → motion_vectors.txt
  → vision_motion_sink（RTL 观测邮箱 + TTL）
  → chassis_ctrl 的 i_auto_valid/i_auto_center/i_auto_diff
  → o_tgt_left/o_tgt_right（真实底盘差速目标）
```

- 生产 `falcons_top` 仍把自动三端口接 0；本链路没有改它。
- 主机不打开串口，不产生真实电机命令。
- `motion_authorized=false`；只有显式 `--allow-motion` 的仿真向量才置 `fpga_auto_valid=1`。

## 2. 规划规则

`motion.py`：

- `x_err = (bbox_center_x / image_width - 0.5) * 2`
- `diff = clamp(-turn_gain * x_err * max_diff, ±max_diff)`；目标在左 → `diff>0` → 左轮慢/右轮快 → 左转；
- `area = bbox_w*bbox_h/(image_w*image_h)`
- `area <= min_area`（太远）→ 中心速度 0，继续观察；
- `area >= stop_area`（太近）→ 中心速度 0，停车；
- 中间区域按 `target_area` 线性给慢速前进；
- 无目标/类别不允许/框非法/TTL 过期 → 全 0；
- 默认参数：`max_center=512`、`max_diff=256`、`target_area=0.10`、`min_area=0.01`、`stop_area=0.30`、`deadband=0.08`；**未实车整定**。

## 3. 仿真结果（v7 真实模型输出）

命令：

```powershell
cd D:/HITSZ_FALCONS/FALCONS-embodied-robot/host/vision_bridge
python run_motion_integration.py `
  --predictions 'D:/HITSZ_FALCONS/ai部分/ai_dual_model_poc/deliverables/real_ad_v7/predictions.jsonl' `
  --output evidence/motion_ad_v7 --allow-motion
python run_motion_integration.py `
  --predictions 'D:/HITSZ_FALCONS/ai部分/ai_dual_model_poc/deliverables/real_bull_tape_v7/predictions.jsonl' `
  --output evidence/motion_tape_v7 --allow-motion
```

| 输入 | 预测 | 桥接受 | 运动建议有效 | 结果 |
|---|---:|---:|---:|---|
| 真实 AD 钙奶 v7 | 14 | 14 | 14（9 前进/调速，5 到距） | RTL PASS |
| 真实公牛黑胶带 v7 | 10 | 10 | 10（10 前进/调速） | RTL PASS |

RTL 检查：`vision_motion_tb` 固定场景 12 项 + 真实向量回放；`chassis_ctrl` 输出 `left=center-diff`、`right=center+diff`；TTL 299 ms 仍有效、300 ms 清零；无效事件、手动接管、急停均优先。证据：

- `evidence/motion_ad_v7/summary.json`、`motion_events.jsonl`、`motion_vectors.txt`、`simulation.txt`
- `evidence/motion_tape_v7/...`

主机测试：`python -m unittest discover -s tests -v` → **46 tests OK**（含原有 bridge/serial 36 + 新增 motion 10）。

## 4. 与生产安全的边界

- `falcons_top` 的 `i_auto_valid/i_auto_center/i_auto_diff` 仍为 0；本链路只在独立 TB 中驱动它们。
- 没有串口发送，没有 RK3588S/RKNN/RKLLM 部署，没有摄像头实时链路。
- 类别不等于速度：本规划器只是“目标准星”启发式；真实自动导航还需要路径规划、避障、定位和速度限制。
- bbox 面积不是可靠距离；透视、俯仰、遮挡都会破坏它；未做相机标定/深度。
- 当前 JSONL 没有可信采集时间/序号；回放采用合成 100 ms 时间轴，TTL 不能当端到端延迟。
- `chassis_ctrl` 的 PID/电机/制动参数未在环；真实停车距离未知。
- 手动接管、RC 失效、ToF/急停仍是更高优先级；本链路没有削弱它们。

## 5. 下一步

1. 在真实视频生产者上加单调时间戳和 `seq`，用 `VisionBridge` 的 `require_source_metadata=True`；
2. 把 `vision_motion_sink` 做成板端接收器（UART/以太网 + CRC + seq 防重放 + watchdog），但先保持自动三端口可门控；
3. 用相机标定/深度或目标尺寸估计替代 bbox 面积距离；
4. 把 `safety_filter_top` 的安全上限接到自动中心速度，形成“AI 目标 → 视觉伺服 → 安全滤波 → 底盘”的完整仿真链；
5. 实车只从**低速、空载、有人急停**开始，先验证手动接管和 TTL 清零。
