# build.tcl —— FALCONS-embodied-robot 完整构建（骨架）
#
# 用法（在工程根目录执行）：
#   & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl
#
# 产物：impl/pnr/<output_base_name>.fs
#
# 关于源文件列表：
#   全部由 scripts/vendor_sources.tcl 从 deps/modules.list + rtl\ 目录派生，
#   本脚本里【不出现】任何具体模块路径 —— 路径只在 lib_paths.ps1 / modules.list /
#   vendor_sources.tcl 三处各定义一次（分别对应 PowerShell、清单、Tcl）。
#
# ⚠ .cst 与 .sdc 必须【同时】给出：只给 .cst 不给 .sdc 时工具按默认 100 MHz 估算，
#   会报出假的时序违规（见 class\工作交接总结.md 第六节第 5 条）。
#
# ⚠ 芯片与时钟：GW5AT-138B（GW5AT-LV138PG484AC1/I0），系统时钟 50 MHz（管脚 Y18）。
#   引脚约束必须逐针抄自《ACG720_引脚接线手册 RevI》，不要按"相邻引脚递增"推测。

source scripts/vendor_sources.tcl

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

run all
