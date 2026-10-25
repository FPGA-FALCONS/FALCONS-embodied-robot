# RK前进/后退固定速度码30部署记录

- `HOST_RELEASE.json`与`stage_drive30_receipt.json`：从上一已部署包制作独立release，只修改RK构帧策略、正式后端、独立发送入口及对应测试，增加操作说明；未修改FPGA。
- `board_tests_drive30_20261025.txt`：RK板端176项测试通过；`motion_policy_tests_local.txt`包含本地策略与内存串口构帧回归。
- `drive30_install_receipt.json`：新用户服务切换及旧配置/服务/发布来源备份位置。
- `drive30_deployment_check.json`：运行中服务报告直行速度码30、模型已就绪、控制链路正常；六条API预览均为速度字节30且未发送。原JSON配置逐项保持一致。
- `VALIDATION.json`：本次未构建、修改或烧录FPGA，109个构建输入与此前部署版本完全一致；未发出真实前进/后退测试指令。

归档说明保存部署前快照，当前结果以这些回执及仓库文档为准。此验证证明RK实际构帧策略，不是新实车速度或位移标定。
