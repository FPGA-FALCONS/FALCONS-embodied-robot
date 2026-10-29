# 应用层RTL

这里维护本车的顶层、控制、交互和协议；可复用模块直接从项目内`vendor/verilog-reusable-modules/`参与构建，不复制第二份。维护约定见[vendor/ORIGIN.md](../vendor/ORIGIN.md)。

| 目录 | 当前职责 |
| --- | --- |
| inspection/ | 独立任意角度整段检查器nav_supercover_check，支持取消和地图版本失效；尚未接入生产导航，见[模块说明](inspection/README.md) |
| top/ | 整车与独立测试顶层，实际参数覆盖、引脚、时钟及仲裁 |
| rc/ | J6配置、六路捕获适配、手动停车回中和解锁守卫 |
| motor/ | 本车差速、四轮速度PID及自动命令仲裁 |
| imu/、odom/ | IMU应用接入和航位积分，驱动/偏航积分核心在vendor |
| nav/ | CORDIC、原非避障nav_ctrl及避障单点nav_point_ctrl |
| ui/ | 七项菜单、基础页、参数/导航交互；动态14字段，每字段5字符，顶层RAM128字节 |
| ai/ | 四功能交互、MOT1巡检事务、真实执行回执、视觉图层、高速AIF1、地图/路线导出及DDR内容显示 |
| cam/ | 原OV5640采集与DDR本地显示、180°画面方向及显示合成 |
| common/ | 同步复位、按键、252槽本地字模和字符串ROM |
| debug/ | ASCII调参/MOT1路由、CSV和状态上报 |
| ip/ | Gowin DDR/PLL/FIFO等IP与适配器，按对应文件的生成/维护约定修改 |

构建依赖来自deps/modules.list及scripts/vendor_sources.tcl。默认build.tcl选择falcons_top；独立手柄目标由build_rc_led.tcl选择，不将其针脚/有效掩码覆盖整车。

RK模型、H定位、点位队列、AI排版和录像在host/ai_coordinator；FPGA只复用原导航和显示/图层通路。基础字形和功能不依赖RK。职责及接口入口见[模块速查](../doc/模块库接口速查.md)、[巡检指南](../doc/AI视觉巡检.md)。

num_text、参数页、显示RAM及顶层字段数须一并核对；字体产物通过tools/font_chars.txt和gen_font_rom.py生成，不手改ROM内容。新FPGA数据路径从设计起分拍、寄存，改变RTL/约束后按受影响测试及实际PnR验证。
