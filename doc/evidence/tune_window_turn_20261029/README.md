# 规划窗口与独立巡检转向参数验证

规划窗口默认128、范围2～255；导航转向和AI巡检转向独立。测试、综合、布局布线结论见[验证记录](VALIDATION.json)、[时序结果](fpga_validation.json)和[关键路径](path_review.json)。同钟域通过，系统余量0.029ns；全局跨域违例仍在，本轮未烧录。

下图由实际ui_render及本地ROM经ModelSim渲染，数值取tune_ctrl默认显示值；状态为光标档、参数页编辑态，不是实车截图。

- [选中规划窗口](window_selected.png)
- [选中AI转向](inspection_turn_selected.png)

应用完整回归初轮仅参数页旧指纹失败；布局与其他页面核对后更新基准，显示两台重跑通过。原始完整回归摘要及显示重跑摘要分别保存，不将初轮失败改写为通过。
