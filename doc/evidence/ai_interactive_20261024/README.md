# AI 交互验证证据

对应说明：[AI模块页面与遥控交互](../../AI模块页面与遥控交互_20261024.md)。日期包含2026-10-24至2026-10-25。

- `VALIDATION.json`：软件、应用仿真、本地字库兼容性与现场验证的分项状态。
- `coordinator_tests.txt`、`application_regression_summary.txt`：板端156项测试原始结果和最终43项应用仿真汇总。
- `BUILD_STATUS.json`：本轮输入、位流与时序报告哈希；只有新鲜输入及同钟域时序均通过才产生可审阅的SRAM候选。DDR/串行器IP跨钟域报告仍按原工程单独核对，不能据此宣称所有时序路径签核。
- `BUILD_INPUTS.json`：本轮109个构建输入的逐文件哈希。发布归档哈希对应实际部署包；Git文本换行规范化可能使检出字节与归档不同，不能据此替换实际部署包哈希。
- `SRAM_PROGRAMMING.json`、`sram_program.txt`：新位流已于2026-10-25 02:10通过SRAM加载，Flash未写入；包含位流与加载日志哈希。
- `activation_receipt.json`、`activation_status.json`、`post_activation_check.json`：RK新服务激活与回滚备份位置；新FPGA遥控快照、显示确认、服务状态与串口错误检查。FAM1任务固件握手与MOT1语义运动/AI显示链路分别统计，不能把FAM1等待握手误判为后两者不可用。
- `OPERATOR_ACCEPTANCE.json`：用户反馈“都成功”，确认新版菜单及标签、CH8/SWA选择、SWB录音/转写/聊天播报、SWC手动关闭AI和停止播报，以及切回光标档不自动恢复。该记录为现场观察反馈，未附屏幕实拍。
- `HOST_CANDIDATE.json`、`candidate_source.json`：RK候选源码归档及逐文件校验依据，非模型权重。
- `LOCAL_FONT_COMPATIBILITY.json`：原三套FPGA字形逐行一致性。
- `UI_SIMULATION.png`、`UI_RK_ABSENT_SIMULATION.png`：真实RTL仿真截图，使用示例任务数据，不能作为屏幕实拍或硬件验收。
- `asset_smoke.json`、`tts_archive_verified.json`：既有Qwen与指定完整Kokoro归档的校验/合成/USB播放。用户已确认人声清晰；该步骤未打开麦克风或发送运动指令。
- `device_inventory.json`：扫描时设备清单。高速图像数据线缺席只影响目标识别，不封锁页面、语义导航或聊天。

新位流加载、RK服务激活及上述三项现场交互已有回执。新版语义运动、运动中制动距离/摇杆接管、RK实物断连后原功能逐项回归及目标识别未单独验收。MOT1发送与计时结束不代表实物停车或动作完成。
