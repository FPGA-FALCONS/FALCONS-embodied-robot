# 模型自启、AI手动暂停与相机方向的验证

本次先本地修改，后恢复电脑热点连接并完成配套部署。本目录包含当前版本的板端测试、SRAM加载、系统重启自启及现场反馈，不沿用上一版本的通过结论。

- `coordinator_tests_windows.txt`：174项软件测试，164项通过、10项Linux平台检查跳过。本地隔离测试环境补充OpenCV，未更改板端Python、驱动、SDK或权重。
- `application_regression_summary.txt`：44项整车应用仿真全部通过，含253项相机初始化寄存器比较、23项AI遥控状态检查、语音运动桥手动最高优先回归。
- `BUILD_INPUTS.json`与`BUILD_STATUS.json`：逐文件构建输入、位流哈希及同钟域时序结果。系统50.090MHz、像素49.697MHz，所列时钟建立/保持违规端点均为零；生成IP跨钟域路径仍按原工程单独核对，不是全部路径签核。
- `HOST_CANDIDATE.json`：包含开机启动脚本、安装器、模型加载管理器与手动暂停处理的新候选源码包，逐文件校验通过，不含模型权重。
- `board_tests_boot_pause_20261025.txt`：RK板端174项测试全部通过。
- `stage_boot_pause_receipt.json`与`autostart_install_20261025.json`：源码清单校验、服务安装及旧配置/服务/发布来源备份位置。
- `SRAM_PROGRAMMING.json`与`sram_program.txt`：当前位流加载成功，用户代码`0x0000DE68`，未写Flash。
- `pre_reboot_boot_pause_check.json`、`post_reboot_boot_pause_check.json`与`post_interaction_boot_pause_check.json`：不同boot ID、模型在SSH登录前就绪、单次提示音、服务与内存状态，以及手动保持AI/新录音的遥控快照。
- `OPERATOR_ACCEPTANCE.json`：用户对提示音、摄像头180°方向和聊天页手动暂停/新录音反馈“都正常”，范围不含运动中制动距离或目标识别实物验收。
- `VALIDATION.json`：区分已完成与尚未验证的现场项目；系统重启已验证，整机断电冷启动未单独验证。

已部署的源码归档及哈希保持不变，归档中的说明是部署前快照；本仓库文档与本目录回执随现场结果更新。

功能与安装步骤见[开机加载与手动暂停](../../RK模型开机加载与手动暂停_20261025.md)。新FPGA控制确认位与RK解析器应配套部署，继续只按SRAM实验流程验证；本次未写Flash。
