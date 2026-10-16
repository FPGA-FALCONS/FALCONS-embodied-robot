# ============================================================================
# sta_timing_probe.tcl —— 全量时序路径"探针"构建（一次性调查用）
#
# 目的：PnR 默认只把每类分析最差的 25 条路径写进报告（report_timing -max_paths 25），
#       而报告头部的 "Numbers of Setup Violated Endpoints = 4" 需要一个【全量】列表来定位。
#       本脚本把每类分析的路径条数放大到 500，并把结果单独落盘。
#
# ★ 零覆盖保证：output_base_name 用 falcons_sta（不是 falcons），所以全部产物都是
#   impl/pnr/falcons_sta* / impl/gwsynthesis/falcons_sta.vg，
#   impl/pnr/falcons.fs（当前【正式】位流；★ 本工程尚未上板过，别写成"已上板验证"）
#   与 falcons_tr_content.html（已有证据）
#   【不会被改动】。这是刻意的：位流不可逐字节复现，覆盖即永久丢失。
#
# 用法（在工程根目录执行）：
#   & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" tools\sta_timing_probe.tcl
#
# 产物：
#   impl/pnr/falcons_sta.fs                （探针位流，调查用，**不是**上板用的那一版）
#   impl/pnr/falcons_sta_tr_content.html   （探针时序报告：每类 25 → 500 条路径）
#   impl/pnr/falcons_sta_setup_all.rpt     （setup 全量，report_timing -file 导出）
#   impl/pnr/falcons_sta_hold_all.rpt
#   impl/pnr/falcons_sta_recovery_all.rpt
#   impl/pnr/falcons_sta_removal_all.rpt
#   impl/pnr/falcons_sta_oser_reset.rpt    （定向：4 个 OSER10 RESET 端点的 setup 路径）
#   impl/pnr/falcons_sta_clocks.rpt        （report_clocks，确认 CLKOUT1 的沿关系）
# ============================================================================

source scripts/vendor_sources.tcl

require_top_source

set sources [concat [get_vendor_sources] [get_app_sources] [get_constraint_sources]]

foreach source $sources {
    add_file $source
}
puts "  sources added: [llength $sources]"

set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0

set_option -top_module falcons_top
set_option -verilog_std v2001
set_option -output_base_name falcons_sta
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1

run all

# ---------------------------------------------------------------------------
# 全量路径导出。max_paths 取 500：足够覆盖"所有负 slack 路径"，
# 又不会让报告膨胀到不可读（当前设计分析路径总数 13711）。
# report_timing 的 -file 若不被本版本支持，则退回打印到 stdout（日志里照样能看到）。
# ---------------------------------------------------------------------------
set MAXP 500

proc dump {args} {
    set tag   [lindex $args 0]
    set cmd   [lindex $args 1]
    set file  "impl/pnr/falcons_sta_${tag}.rpt"
    puts "=== report_timing $cmd -> $file ==="
    if {[catch {eval report_timing $cmd -file $file} msg]} {
        puts "    (-file 不被支持或报错: $msg)"
        puts "    ---- 退回 stdout 输出 ----"
        catch {eval report_timing $cmd}
    }
}

dump setup    "-setup    -max_paths $MAXP -max_common_paths 1"
dump hold     "-hold     -max_paths $MAXP -max_common_paths 1"
dump recovery "-recovery -max_paths $MAXP -max_common_paths 1"
dump removal  "-removal  -max_paths $MAXP -max_common_paths 1"

# ---------------------------------------------------------------------------
# 定向查询：报告头部说 Setup 有 4 个违规端点，而全文只有 4 个"下降沿捕获端点"
# （u_OSER10_{clk,b,r,g}/RESET，由 245 MHz CLKOUT1 下降沿捕获）。
# 这里直接对这 4 个端点各要一条 setup 路径，验证"4 个违规端点 == 4 个下降沿端点"。
# get_pins 若不被支持，catch 会兜住，不影响前面的产物。
# ---------------------------------------------------------------------------
set oser_pins {
    u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_clk/RESET
    u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_b/RESET
    u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_r/RESET
    u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_g/RESET
}

set fh [open "impl/pnr/falcons_sta_oser_reset.rpt" w]
foreach pin $oser_pins {
    puts $fh "================ $pin ================"
    if {[catch {set txt [report_timing -setup -to [get_pins $pin] -max_paths 5 -max_common_paths 1]} msg]} {
        puts $fh "  (fail: $msg)"
    } else {
        puts $fh $txt
    }
}
close $fh

catch {report_clocks -file impl/pnr/falcons_sta_clocks.rpt}

puts "=== sta_timing_probe done ==="
