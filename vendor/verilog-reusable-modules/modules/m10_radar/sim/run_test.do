onerror {quit -code 1 -force}
# Start in this sim directory. run_sim.ps1 runs each test in a fresh process.
set root [file normalize [file join [pwd] ..]]
set modules [file normalize [file join $root ..]]
if {![file isdirectory work]} {vlib work}
vmap work work
vlog -sv +incdir+$root/rtl +incdir+$modules/hdmi_1024x600/rtl \
    $modules/uart/rtl/uart_byte_rx.v \
    $modules/uart/rtl/uart_byte_tx.v \
    $modules/hdmi_1024x600/rtl/disp_driver.v \
    $root/rtl/m10_packet_parser.v \
    $root/rtl/m10_radar.v \
    $root/rtl/zoom_control.v \
    $root/rtl/polar_to_xy.v \
    $root/rtl/point_bitmap.v \
    $root/rtl/point_cloud_video.v \
    $root/rtl/m10_point_cloud_core.v \
    $root/sim/m10_gps_packet_parser_tb.v \
    $root/sim/m10_point_cloud_core_tb.v \
    $root/sim/point_bitmap_tb.v \
    $root/sim/point_cloud_video_tb.v \
    $root/sim/polar_to_xy_tb.v \
    $root/sim/uart_rx_recovery_tb.v \
    $root/sim/zoom_control_tb.v \
    $root/sim/m10_radar_tb.v

vsim -t 1ps -onfinish stop work.m10_radar_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.uart_rx_recovery_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.m10_gps_packet_parser_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.zoom_control_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.polar_to_xy_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.point_bitmap_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.point_cloud_video_tb
run -all
quit -sim
vsim -t 1ps -onfinish stop work.m10_point_cloud_core_tb
run -all
quit -sim
