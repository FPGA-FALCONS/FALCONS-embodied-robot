# 独立综合 nav_polar，保留公开端口供 UART 链路混合仿真替换。
set project_root [file dirname [file dirname [file normalize [info script]]]]
set output_dir [file join $project_root logs radar_polar_gate]
file mkdir $output_dir
cd $output_dir
add_file [file join $project_root vendor verilog-reusable-modules modules auto_navigation rtl nav_polar.v]
set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0
set_option -top_module nav_polar
set_option -verilog_std v2001
set_option -output_base_name nav_polar_gate
if {[catch {run syn} err]} {
    puts "RADAR SYNTHESIS FAILED: $err"
    exit 1
}
exit 0
