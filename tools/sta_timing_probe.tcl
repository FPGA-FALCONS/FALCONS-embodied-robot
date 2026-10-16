# ============================================================================
# sta_timing_probe.tcl —— "独立复布"探针（一次性调查用）
#
# ★ 实测结论：本脚本的用途**不是**"把每类 25 条放大到 500 条" —— 那做不到。
#   `report_timing` / `get_pins` / `report_clocks` **都不是 `gw_sh` 的 tcl 命令**
#   （实测 `info commands *report*` 只返回 `report_property`；`*timing*`/`*slack*`/`*analysis*` 全为空）。
#   报告里那句 "Report Command: report_timing -setup -max_paths 25 -max_common_paths 1"
#   是 PnR 引擎的**内部调用**，不是能敲的命令；`set_option` 的完整选项表（约 130 项）里
#   也没有任何一项能调这个条数（`-num_critical_paths` 等属**综合/Synplify 侧**）。
#   → 脚本下半部分的 dump / 定向查询**预期会失败**（已用 catch 兜住，不影响前面的构建产物）。
#
# ★ 它真正的用途：对**同一份 RTL 再独立布一次线**，拿第二份统计与主版本对比 ——
#   若逐项相同，说明那些统计是**结构性**的（与布局无关），而不是随机落点。
#   2026-10-16 实测：20:51 主版 vs 21:18 探针版，Paths/Endpoints/Falling/Setup Violated/
#   Hold Violated 以及 25 条 Setup/Hold 的最差 slack（+0.032 / −1.026）**全部逐项相同**
#   → 这是"4 个 Setup 违规端点 = 4 个下降沿端点"的独立证据之一（见 doc\工作记录.md §21.9.4）。
#   副产品：证明 Gowin 在本设计上的 PnR 是**确定性**的。
#
# ★ 零覆盖保证：output_base_name 用 falcons_sta（不是 falcons），所以全部产物都是
#   impl/pnr/falcons_sta* / impl/gwsynthesis/falcons_sta.vg，
#   impl/pnr/falcons.fs（当前【正式】位流；★ 本工程尚未上板过，别写成"已上板验证"）
#   与 falcons_tr_content.html（已有证据）【不会被改动】。
#   这是刻意的：位流不可逐字节复现，覆盖即永久丢失。
#
# 用法（在工程根目录执行）：
#   & "D:\Apps\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" tools\sta_timing_probe.tcl
#
# 实际产物（2026-10-16 首次运行的实况）：
#   impl/pnr/falcons_sta_tr_content.html   ★ 证据本体：完整时序报告（816 KB）
#   impl/pnr/falcons_sta.timing_paths      ★ 证据：25+25 条路径的 slack（57 KB）
#   impl/pnr/falcons_sta.fs / .pr / .db …  PnR 标准产物，**调查用**、不是上板位流
#   impl/gwsynthesis/falcons_sta.vg …      综合产物
#   ★ 上述非证据产物已于 2026-10-16 按需求方授权清理（共 142.4 MB），
#     只保留上面两个证据文件；需要时重跑本脚本即可复现（约 2 分钟）。
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
# ★ 本段**预期失败**（实测报 `invalid command name "report_timing"`）。
# 保留它是为了把"gw_sh 没有这个命令"这一事实**固化在可执行代码里**，而不只是写在文档里
# —— 下次谁又想"跑全量 report_timing"，看到这里就够了。
# （`set_option` 的约 130 项选项里也没有能调 PnR 路径条数的开关。）
# 真正有用的产物是上面 `run all` 生成的那一整套报告 → 见文件头"实际产物"。
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
# 定向查询（★ 实测：本段**会整体失败**，保留是为了把"工具能力边界"固化在可执行代码里）：
# 想法是从这 4 个端点各要一条 setup 路径，直接验证"4 个违规端点 == 4 个下降沿端点"。
# 但 `report_timing` 与 `get_pins` 都不是 gw_sh 的命令 → 只会写出一份空壳 .rpt。
# 实际结论由统计 + 排他性推理给出（doc\工作记录.md §21.9.3），不依赖本段。
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
