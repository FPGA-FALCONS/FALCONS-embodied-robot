# build_pnr.tcl —— 只跑布局布线，复用上一次的综合网表 impl/gwsynthesis/falcons.vg
#
# 用途：只改了约束（constr/*.cst 引脚、constr/*.sdc 时钟）而 RTL 一行没动时用，
#       跳过约 2 分钟的综合，直接约 6.5 分钟出位流。
#
# 注意 1：前提是 falcons.vg 存在且比所有 rtl/**/*.v 都新。RTL 改过就必须走 build.tcl，
#         否则会把【旧网表】和【新约束】拼在一起 —— 本脚本开头会检查并拒绝执行。
# 注意 2：本文件同样重新 add_file 全部源文件（gw_sh 需要知道文件清单），
#         但 run pnr 阶段读的是综合产物，不重新综合。
# 注意 3：结尾的 exit 不能删，理由见 build.tcl。

source scripts/build_common.tcl

# ---- 前置检查：网表必须存在，且不早于任何一个 RTL 源文件 ----
set netlist [file join $build_root impl gwsynthesis falcons.vg]
if {![file exists $netlist]} {
    puts "ERROR: 找不到综合网表 $netlist —— 先跑一次 build.tcl（或 build_syn.tcl）。"
    exit 1
}
set netlist_time [file mtime $netlist]
set stale {}
foreach source [get_app_sources] {
    if {[file mtime $source] > $netlist_time} { lappend stale $source }
}
if {[llength $stale] > 0} {
    puts "ERROR: 以下 RTL 比综合网表新，必须先跑 build.tcl 重新综合："
    foreach s $stale { puts "   $s" }
    exit 1
}

if {[catch {run pnr} err]} {
    puts "BUILD FAILED (run pnr): $err"
    exit 1
}
exit 0