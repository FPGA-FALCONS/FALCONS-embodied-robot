# 语义运动指令（无摄像头）— 2026-10-24

这是不依赖摄像头的第一层：中文运动短语 → 受限 JSON → MOT1 二进制帧。

发布整合版更新：协调器已经接好文本/ASR转写→运动规则入口，默认只做预览；FPGA桥及整车集成随本次发布一并提供。下方旧“未有RTL/未接ASR”为最初开发记录，以`doc/全栈上板与队友下载_20261024.md`和`doc/无相机语音运动接入与验收_20261024.md`的最终状态为准。独立sender使用同一审核规则；明确米数不再换算为时长，否定/复合/越界拒绝，短写会补齐，退出会尝试STOP。串口收回字节不称为动作完成回执。
固定短语走规则解析（低延迟、可控）；无法匹配时返回 `None`，交给 C1b 做任务语义。

## 支持的命令

| 说法示例 | 输出 |
|---|---|
| 前进 / 后退 | `{"cmd":"forward/backward","duration_s":0.8}` |
| 前进2秒 / 后退1米 | 带 duration/ distance；米按 0.2 m/s 估算，最多 5s |
| 左转 / 右转 / 左转90度 / 向左转九十度 | `{"cmd":"turn","direction":"left/right","angle_deg":90}` |
| 原地顺时针转一圈 / 逆时针转一圈 | `{"cmd":"spin","angle_deg":±360}` |
| 顺时针转半圈 / 转180度 | 带角度 |
| 开启巡航 / 关闭巡航 | `{"cmd":"patrol_start/patrol_stop"}` |
| 停 / 停下 / 停止 / 别动 / 急停 | `{"cmd":"stop"}` |
| 找螺丝刀 | `None`（交给 Qwen） |

限制：距离 0.05–2 m、角度 5–360°、时长 0.1–5 s；超限直接拒绝。

## MOT1 帧

```text
MOT1 (4B) + seq (1B) + cmd (1B) + value (int16 BE) + speed (1B) + flags (1B) + CRC16 (2B)
```

- `value`：前进/后退为 duration_ms；转向/自旋为角度（spin 带符号）。
- CRC-16/CCITT-FALSE 覆盖前 10 字节。
- 这是**低层运动原语**，与高层 FAM1 JSON 任务协议分开，避免把任意任务直接塞进电机路径。
- 心跳命令 `HEARTBEAT=8`，FPGA 侧 300–500 ms 收不到任何帧就自动停车。

## 现在能做什么 / 不能做什么

- ✅ 规则解析、MOT1 编解码、CRC/重同步、单元测试（8 项）都已通过。
- ✅ `send_motion.py --dry-run` 可查看实际帧；Linux 下可发到显式 `/dev/serial/by-id/...`。
- ❌ 还没有 FPGA `voice_motion_bridge` RTL，所以**现在发帧不会让车动**。
- ❌ 巡航 start/stop 目前只是协调器意图；真实航点/导航仍需 FPGA 侧能力。

## 运行

```bash
python -m unittest -v test_voice_nav.py
python voice_nav.py "左转90度"
python send_motion.py --dry-run "原地顺时针转一圈"
# Linux 板端：
python3 send_motion.py --device /dev/serial/by-id/usb-... "前进"
```

## 下一步

1. FPGA 加 `voice_motion_bridge`：
   - 解析 MOT1/CRC；前进/后退定时；转向/自旋用 `yaw_deg` 闭环；300 ms 看门狗。
   - 输出 `i_auto_valid/i_auto_center/i_auto_diff` 到 `chassis_ctrl`；`i_manual/estop/ToF` 始终优先。
2. 先架空轮子测前进/后退/停；再落地低速测 90°/360°。
3. 再接协调器巡逻/找物流程；不把 MOT1 与 FAM1 混用。

安全边界：语音不能覆盖急停/遥控/ToF/堵转；默认 shadow 只记录。板端禁止 upgrade/烧录 Flash。
