# build.tcl —— FALCONS-embodied-robot 完整构建（综合 + 布局布线）
#
# 用法（在工程根目录执行）：
#   powershell -ExecutionPolicy Bypass -File tools\build.ps1    （推荐，能拿到退出码与产物清单）
#   & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl
#
# 产物：impl/pnr/falcons.fs
#
# 只想跑一个阶段时用同级脚本，不要改这里：
#   build_syn.tcl   只综合（约 2 分钟，看资源与综合告警）
#   build_pnr.tcl   只布局布线（约 6.5 分钟，改了 .cst/.sdc 但没改 RTL 时用）
#
# 注意 A：结尾必须显式 exit。gw_sh 跑完脚本会掉进交互式 Tcl shell 并一直不退出，
#         调用方的 & gw_sh 就永远等不到退出码（2026-10-20 实测白等 12 分钟）。
#
# 注意 B：.cst 与 .sdc 必须【同时】给出。只给 .cst 不给 .sdc 时工具按默认 100 MHz 估算，
#         会报出假的时序违规（见 class/工作交接总结.md 第六节第 5 条）。
#
# 注意 C：芯片与时钟 —— GW5AT-138B（GW5AT-LV138PG484AC1/I0），系统时钟 50 MHz（管脚 Y18）。
#         引脚约束必须逐针抄自《ACG720_引脚接线手册 RevI》，不要按"相邻引脚递增"推测。
#
# 源文件列表全部由 scripts/vendor_sources.tcl 派生、在 scripts/build_common.tcl 里装配，
# 本脚本里【不出现】任何具体模块路径。

source scripts/build_common.tcl

if {[catch {run all} err]} {
    puts "BUILD FAILED (run all): $err"
    exit 1
}
exit 0