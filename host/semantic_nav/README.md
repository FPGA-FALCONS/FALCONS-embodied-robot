# 语义运动指令（无摄像头）— 2026-10-24

中文固定运动短语 → 受限 JSON → MOT1 二进制帧 → 协调器共享串口 → FPGA
`voice_motion_bridge`。这是不依赖摄像头的低层运动入口；找螺丝刀等任务仍交给
原有 C1b/视觉链路。

## 支持的命令

| 说法示例 | 输出 |
|---|---|
| 前进 / 后退 | `{"cmd":"forward/backward","duration_s":0.8}` |
| 前进2秒 / 后退半秒 | 带 `duration_s`；0.1–5 s |
| 前进1米 / 后退两米 | 保留 `distance_m`；按固定0.3m/s换算为 `timed_estimate` 时长，必要时分段发送 |
| 左转 / 右转 / 左转90度 | `{"cmd":"turn","direction":"left/right","angle_deg":90}` |
| 原地顺时针转一圈 / 逆时针转一圈 | `{"cmd":"spin","angle_deg":±360}` |
| 顺时针转半圈 / 转180度 | 带整数角度 |
| 开启巡航 / 关闭巡航 | `{"cmd":"patrol_start/patrol_stop"}`；本版巡航启动未接真实航点，协调器会拒绝 |
| 停 / 停下 / 停止 / 别动 / 急停 | `{"cmd":"stop"}` |
| 找螺丝刀 | `None`（交给 C1b） |

限制：距离 0.05–2 m、角度 5–360°、时长 0.1–5 s；超限直接拒绝，不静默截断或拆成多次运动。
否定、引号、条件句、复合动作不进入运动执行。

## 距离（阶段 1：定时估算）

距离指令支持0.05–2m，固定按0.3m/s估算时长，使用速度码229；不要求实测标定开关。1m约3.333s、2m约6.667s。RK把超过5s的距离指令拆为每段不超过5s的MOT1帧，保持心跳，任务末尾发送STOP；取消或接管后不再发送后续段。普通定时指令仍限0.1–5s。

UI/日志明确“按设定速度估算、到达未验证”。当前不使用里程计闭环控制距离。

## MOT1 帧

```text
MOT1 (4B) + seq (1B) + cmd (1B) + value (int16 BE) + speed (1B) + flags (1B) + CRC16 (2B)
```

- `value`：前进/后退为 duration_ms；转向/自旋为角度（spin 带符号，正=顺时针）。
- CRC-16/CCITT-FALSE 覆盖前 10 字节。
- 心跳 `HEARTBEAT=8`；FPGA `voice_motion_bridge` 500 ms 看门狗。
- 串口 115200 8N1，显式 `/dev/serial/by-id/...`，独占 `flock`；协调器与独立 sender 不能同时占用。

## 协调器执行入口

- 文本 `POST /text` 和真实 ASR `POST /audio` 走同一个 `motion_intents` / `MotionBackend`。
- `plan_only=true` 只预览，不写串口。
- 后端不另开串口；MOT1 与 FAM1 共用 `Gateway` 的同一个 `SerialPort`，整帧排队。
- 专用心跳任务每 `heartbeat_s`（默认 0.2 s，最大 0.25 s）发送，不被 ASR/视觉/LLM/TTS 阻塞。
- 停止/取消会丢弃旧心跳并优先排队 STOP；旧动作的迟到任务不会重启。
- FPGA 当前没有 MOT1 任务级回执：串口写入完成、主机计时器和收到的任意 ASCII/CSV 都不算动作完成或物理停车。状态字段明确为 `device_reply_verified=false`、`action_complete_verified=false`、`physical_stop_verified=false`。

## 运行

```bash
python -m unittest -v test_voice_nav.py test_sender.py
python voice_nav.py "左转90度"
python send_motion.py --dry-run "原地顺时针转一圈"
# 距离按固定0.3m/s估算：
python send_motion.py --dry-run "前进一米"
python send_motion.py --dry-run "后退两米"
# Linux 板端，确认串口独占后：
python3 send_motion.py --device /dev/serial/by-id/usb-... --speed 10 "前进"
```

独立 sender 只是测试工具；正式入口是协调器。板端默认 `voice_motion.enabled=false`、
`live_commissioned=false`、shadow、FAM1 `allow_motion=false`。

## 现在能做什么 / 不能做什么

- ✅ 规则、限值、MOT1 编解码、CRC/重同步、共享串口整帧调度、心跳/停止/取消测试。
- ✅ FPGA `voice_motion_bridge` 已随本发布 RTL 仿真/综合（见 vendor README 和
  `doc/运动桥生产接入验证_20261024.md`）；协调器有正式执行后端。
- ⚠️ 尚未做真实麦克风到电机、轮子架空、落地、偏航/轮速标定；这些不能由仿真替代。
- ❌ `patrol_start` 未接真实航点，不能声称自动巡航已工作。
- ❌ ToF 仍是生产工程占位，不能声称已有实物防撞。
- ❌ 没有 MOT1 完成/停车回执；禁止把串口写入成功或定时结束称为物理完成。

安全边界：语音不能覆盖硬件急停、遥控手动、ToF/规划急停、堵转；默认 shadow 只记录。
板端禁止 upgrade，禁止写 Flash；只使用 SRAM 实验位流。
