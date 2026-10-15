# Run from any directory: gw_sh.exe build.tcl
cd [file dirname [file normalize [info script]]]
set sources {
    ../../../uart/rtl/uart_byte_rx.v
    ../../../uart/rtl/uart_byte_tx.v
    ../../../hdmi_1024x600/rtl/display_pll.v
    ../../../hdmi_1024x600/rtl/disp_driver.v
    ../../../hdmi_1024x600/rtl/dvi_tx.v
    ../../rtl/m10_packet_parser.v
    ../../rtl/m10_radar.v
    ../../rtl/zoom_control.v
    ../../rtl/polar_to_xy.v
    ../../rtl/point_bitmap.v
    ../../rtl/point_cloud_video.v
    ../../rtl/m10_point_cloud_core.v
    rtl/acg720_m10_hdmi_top.v
    constraints/acg720_m10_hdmi.cst
    constraints/acg720_m10_hdmi.sdc
}
foreach source $sources { add_file $source }
set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0
set_option -top_module acg720_m10_hdmi_top
set_option -verilog_std v2001
set_option -include_path {../../rtl;../../../hdmi_1024x600/rtl}
set_option -output_base_name acg720_m10_hdmi
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1
run all
