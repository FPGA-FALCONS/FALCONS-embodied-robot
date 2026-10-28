# 独立找物/人工拾取任务与回放工具

本文件说明保留的FAM1任务研究接口，不是生产视觉巡检说明。现行四功能AI见[协调器](README.md)，生产多点巡检见[巡检指南](../../doc/AI视觉巡检.md)。

## 工具边界

`PatrolMission`用于路径点巡航 → 检出指定目标 → 暂停 → 局部靠近 → 人工确认取走 → 恢复原点位的独立合同。请求包括goto_waypoint、pause_patrol、approach_target、target_observation，回执绑定任务/动作；人工拾取不表示机械执行器成功。

完整FAM1找物/拾取回执没有接入生产FPGA。`reference_peer.py`和patrol_replay.py提供参考/模拟器；其到点、停车与拾取状态不能计入实车完成率，也不通过放宽旧协议给现行巡检授权。

生产视觉巡检管理的是记录沿途全部支持目标、绕行、返回实际起点，使用独立的InspectionMission、保留MOT1事务和真实FPGA回执。两者任务、录音和页面生命周期分别管理。

## 模型及回放

- 复用SenseVoice、C1b任务解析和v17视觉模型；检测头/P3研究候选未替换生产模型。
- `computer_profile.py`建立隔离电脑配置，`doctor.py --deep`核对本地资产；`acceptance.py`/`patrol_replay.py`使用明确输入和模拟回执。
- 文件图片始终标为replay；回放结果的physical_task_completed=false不能改写为实车成功。
- 固定短句和真实动态TTS属于不同使用方式，见[语音说明](SPEECH.md)。

## 发布与迁移

`package_release.py`、`adb_deploy.py`和`deploy_board.py`保留早期通用打包/ADB安装能力；当前巡检使用deploy_inspection.py和既有事务安装器。迁移从实际config.json和资产清单开始，保留模型、USB角色及标定，不从旧HOLD/shadow示例推断当前服务状态。

10-24的模型回放、失败尝试和旧release成绩见[历史整合记录](../../doc/全栈整合与巡航找物交付_20261024.md)及Git历史。最近部署和现场范围只在[巡检验证](../../doc/AI巡检验证记录.md)维护，不在这里累计v5/v6/v7等阶段日志。
