# build_common.tcl —— 生产构建（falcons）的公共配置，被三个入口脚本 source
#
# 三个入口（都在工程根，用 run 的阶段参数区分，公共配置只有这一份）：
#   build.tcl       run all  —— 综合 + 布局布线，产出 impl/pnr/falcons.fs（默认、全量）
#   build_syn.tcl   run syn  —— 只综合，约 2 分钟，产出 impl/gwsynthesis/falcons_syn_resource.html
#   build_pnr.tcl   run pnr  —— 只布局布线，约 6.5 分钟，复用 impl/gwsynthesis/falcons.vg 网表
#
# 为什么要分入口：Gowin 1.9.12 的 Tcl 【没有】增量综合 / 增量 PnR / 并行开关，
# 实测 set_option -incremental* / -parallel / -threads 全部 unknown option；
# run 也只认 all / syn / pnr 三个阶段名。详见 doc/工作记录.md 第 67 节。
#
# 注意 1：本文件【不执行 run】—— 阶段由入口文件决定。
# 注意 2：源文件列表只在本文件里装配一次，三个入口共享。
# 注意 3：vendor_sources.tcl 里全是相对工程根的路径，所以这里先 cd 到工程根。

set build_root [file dirname [file dirname [file normalize [info script]]]]
cd $build_root

source [file join $build_root scripts vendor_sources.tcl]

require_top_source

set sources [concat [get_vendor_sources] [get_app_sources] [get_constraint_sources]]

foreach source $sources {
    add_file $source
    puts "  + $source"
}

set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0

set_option -top_module falcons_top
set_option -verilog_std v2001
set_option -output_base_name falcons
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1
# Explicit pin registers need IOB placement for the 48MHz external FX2 budget.
# SUG1220: placement-only options, no logical latency or clock changes.
set_option -oreg_in_iob 1
set_option -ioreg_in_iob 1
set_option -ireg_in_iob 1
