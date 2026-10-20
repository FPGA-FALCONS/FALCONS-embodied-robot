# build_syn.tcl —— 只跑综合（不布局布线、不产出位流）
#
# 用途：想快速知道"资源涨了多少 / 有没有新告警"时用，约 2 分钟。
# 产物：impl/gwsynthesis/falcons_syn_resource.html（资源）、falcons_syn.rpt.html（综合报告）、
#       falcons.vg（网表，随后 build_pnr.tcl 可用）
#
# 注意 1：本脚本【不更新】impl/pnr/falcons.fs —— 仓库/板上的位流仍是上一次 run all 的。
# 注意 2：综合不跑 PnR，发现不了"布不通 / 时序不收敛"这类问题。
# 注意 3：结尾的 exit 不能删，理由见 build.tcl。

source scripts/build_common.tcl

if {[catch {run syn} err]} {
    puts "BUILD FAILED (run syn): $err"
    exit 1
}
exit 0