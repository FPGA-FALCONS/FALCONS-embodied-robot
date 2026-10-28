# AI 与整车联调开发包

> 本目录为独立视觉观察/回放开发包；其协议未接生产顶层的结论不等于整个项目尚无AI。当前生产见[协调器](../ai_coordinator/README.md)和[巡检指南](../../doc/AI视觉巡检.md)。

状态：**离线回放可运行；Python 与独立 UART RTL 可验证；未接整车顶层、未做摄像头实时显示、未实车。**

完整分工与接口见 [AI与整车联调开发指南](../../doc/AI与整车联调开发指南_20261023.md)。

## 队友先运行这两条

从仓库根目录运行，Python 3.10+，演示与主机测试只用标准库，无需安装依赖或模型：

```text
python host/vision_bridge/team_demo.py
python host/vision_bridge/verify_handoff.py
```

第一条生成 `host/vision_bridge/build/team_demo/preview.html`，浏览器打开即可看到四类合成结果的框、文字和失效场景。背景和文字都是固定测试样例，未调用 YOLO/Qwen。另有 `display.jsonl`、`observations.jsonl`、19 字节帧文件 `frames.hex` 和带内容哈希的 `summary.json`。

第二条要求 `iverilog`、`vvp` 在 PATH；运行全部 Python 测试、协议单测及 Python 编码→50 MHz/115200 8N1 UART 位级仿真。结果在 `build/verification/verification.json`。缺工具或任何失败均返回非零，不会把未执行记作通过。可用 `--output <目录>` 更改结果路径。

只运行 Python 测试：

```text
python -m unittest discover -s host/vision_bridge/tests -v
```

## 开发入口

| 文件 | 给谁使用 | 当前能力 |
| --- | --- | --- |
| `bridge.py` | AI/融合 | schema-v2 → 有时效、可撤销的单目标观察；输入冲突和超时不保持旧目标 |
| `display_adapter.py` | 屏幕/UI | 对应帧的原图像素框、中文类别、状态文字；语义文字仅用于显示 |
| `serial_export.py` | FPGA/通信 | 归一化观察→19 字节 CRC16 帧，只写文件 |
| `team_demo.py` | 所有人 | 无硬件演示及可复现样例 |
| `verify_handoff.py` | 所有人 | 不依赖历史提交或私有数据的自检入口 |
| `motion.py` | 控制算法参考 | 旧视觉伺服启发式，默认不授权运动；框面积不是距离，不能直接接车 |
| `../../rtl/ai/ai_vision_link.v` | FPGA | 独立 UART 观察解析器，CRC、序号、状态与超时检查；未接主工程 |

类别编号固定：0 胶带 `tape`；1 螺丝刀 `screwdriver`；2 AD钙奶 `ad_calcium_milk`；3 营养快线 `nutri_express`。

## 接入实际推理结果

实时生产者在**采集图像时**记录 `time.monotonic()` 与递增序号。推理后把原图像素坐标（撤销 letterbox/缩放后的框）写入 schema-v2。YOLO 单独工作时使用 `TRACKING/accept`、`vlm_called=false`、`qwen=null`；不能伪造 `VERIFIED`。

```python
from bridge import BridgeConfig, VisionBridge
from display_adapter import display_packet

bridge = VisionBridge(BridgeConfig(require_source_metadata=True))
# row 采用 examples/schema2_sample.jsonl 中的 schema-v2 结构。
# captured_at_s、now_s 来自同一机器的同一单调时钟。
event = bridge.ingest(row, now_s, source_seq=seq, captured_at_s=captured_at_s)
packet = display_packet(event, frame_id=row["frame_id"],
                        image_size=(original_width, original_height), now_s=now_s)
# UI 把 packet["boxes"] 叠加到同一 frame_id 的原图上。
# 无新图/新推理结果时仍须由独立定时器调用 bridge.poll(time.monotonic())。
```

`frame_id` 必须包含摄像头/流及本次启动会话身份，例如 `front:session42:123`；不同流使用不同桥实例。显示同一帧需要缓存该帧，不能拿当前实时画面直接配旧框。若选择跟踪预测，必须另做验证，当前适配器对不匹配帧直接不画框。

默认接收后 TTL 300 ms、采集年龄上限 1 s 是开发参数，不是实车延迟保证。Qwen 慢回复只能先留作文字/候选证据，再用当前帧确认目标；不得给旧图结果换新时间戳重新授权。跨机器不能直接相减 `monotonic()`。

## 提交边界

- 不包含模型权重、训练图像、私有路径、板端凭据、RKNN 运行时或系统安装脚本。
- Windows 可运行主机自检；Linux 可用相同脚本。RKNN 转换/真实推理在独立兼容环境做。
- 不打开串口、不下载位流、不自动启动车。生成的 HEX 只供回放测试。
- 不修改比赛环境，不执行任何系统或 pip upgrade。
- 本次不替代导航、不更改现有遥控/急停/ToF/电机仲裁。无需为本开发包重新综合整车。
