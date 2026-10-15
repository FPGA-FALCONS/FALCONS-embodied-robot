# vendor_sources.tcl —— 被 build*.tcl source：把源文件列表拼出来
#
# 设计要点（对应内嵌方案的"路径只在一处定义"）：
#   * 依赖哪些模块包 → 只读 deps/modules.list（与 tools\*.ps1 用同一份清单）；
#   * vendor 的根路径 → 只在下面的 $VENDOR_ROOT 里出现一次；
#   * 应用层 → 递归 glob rtl\*\*.v。
# 改目录结构时只改这一个文件 + deps/modules.list，不要在各 build 脚本里另抄一份路径。
#
# 用法（在工程根的 build*.tcl 里）：
#     source scripts/vendor_sources.tcl
#     set sources [concat [get_vendor_sources] [get_app_sources] [get_constraint_sources]]

set VENDOR_ROOT "vendor/verilog-reusable-modules"
set APP_RTL_DIR "rtl"
set CONSTRAINTS_DIR "constr"

# 读出 deps/modules.list 里被工程引用的包名（忽略注释与空行）
proc read_referenced_packages {} {
    set pkgs {}
    set fh [open "deps/modules.list" r]
    while {[gets $fh line] >= 0} {
        set line [string trim $line]
        if {$line eq ""} { continue }
        if {[string match "#*" $line]} { continue }
        set tok [lindex [split $line] 0]
        # 只接受合法包名，挡掉中文续行说明（否则综合会去找一个中文名目录）
        if {![regexp {^[A-Za-z0-9_]+$} $tok]} { continue }
        lappend pkgs $tok
    }
    close $fh
    return $pkgs
}

# 内嵌模块库侧：每个引用包的 rtl\*.v（按包名排序，保证构建可复现）
proc get_vendor_sources {} {
    global VENDOR_ROOT
    set out {}
    foreach pkg [lsort [read_referenced_packages]] {
        set dir "$VENDOR_ROOT/modules/$pkg/rtl"
        if {![file isdirectory $dir]} {
            puts "ERROR: deps/modules.list references '$pkg' but $dir does not exist (run tools/sync_vendor.ps1 first)."
            puts "错误：deps/modules.list 引用了 '$pkg'，但 $dir 不存在（先跑 tools/sync_vendor.ps1）。"
            exit 1
        }
        foreach f [lsort [glob -nocomplain "$dir/*.v"]] { lappend out $f }
    }
    return $out
}

# 应用层：rtl\<子目录>\<文件>.v
proc get_app_sources {} {
    global APP_RTL_DIR
    set out {}
    foreach f [lsort [glob -nocomplain "$APP_RTL_DIR/*/*.v"]] { lappend out $f }
    return $out
}

# 约束：.cst 与 .sdc 必须【同时】给出
#   只给 .cst 不给 .sdc 时，工具按默认 100 MHz 估算时钟，会报出假的时序违规。
proc get_constraint_sources {} {
    global CONSTRAINTS_DIR
    set out {}
    foreach f [lsort [glob -nocomplain "$CONSTRAINTS_DIR/*.cst"]] { lappend out $f }
    foreach f [lsort [glob -nocomplain "$CONSTRAINTS_DIR/*.sdc"]] { lappend out $f }
    return $out
}

# 顶层存在性检查：骨架阶段 rtl\top 还是空的，这里给一句人话而不是让综合器报天书
# 中英双语：Gowin 的中文输出经管道会被转成 '?'，英文那半仍可读。
proc require_top_source {} {
    global APP_RTL_DIR
    if {[llength [glob -nocomplain "$APP_RTL_DIR/top/*.v"]] == 0} {
        puts "ERROR: rtl/top/ has no top-level RTL yet -- this project is still a SKELETON."
        puts "       Step 1 (see README.md section 7): port the board-verified speed loop from"
        puts "       class/car_fpga -- speed_pid + uart_cmd_rx + speed_report + speed_loop_top."
        puts "错误：rtl/top/ 下还没有顶层 RTL —— 本工程目前是【骨架】。"
        puts "       见 README.md 第七节分步计划第一步：把 class/car_fpga 里已仿真+综合的"
        puts "       速度闭环（speed_pid + uart_cmd_rx + speed_report + speed_loop_top）搬进来。"
        exit 1
    }
}
