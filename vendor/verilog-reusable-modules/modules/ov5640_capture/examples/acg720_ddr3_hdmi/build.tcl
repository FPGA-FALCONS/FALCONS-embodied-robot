set sources {
    ../../rtl/DVP_Capture.v
    ../../rtl/camera_init.v
    ../../rtl/i2c_bit_shift.v
    ../../rtl/i2c_control.v
    ../../rtl/ov5640_init_table_rgb.v
    ../../rtl/ov5640_capture.v
    ../../../hdmi_1024x600/rtl/display_pll.v
    ../../../hdmi_1024x600/rtl/disp_driver.v
    ../../../hdmi_1024x600/rtl/dvi_tx.v
    ../../../hdmi_1024x600/rtl/hdmi_1024x600.v
    ip/ddr3_ctrl_2port/ddr3_ctrl_2port.v
    ip/ddr3_ctrl_2port/fifo_ddr3_adapter.v
    ip/ddr3_memory_interface/ddr3_memory_interface.v
    ip/ddr_pll/ddr_pll.v
    ip/system_pll/gowin_pll.v
    ip/rd_data_fifo/rd_data_fifo.v
    ip/wr_data_fifo/wr_data_fifo.v
    rtl/acg720_ov5640_ddr3_hdmi_top.v
    constraints/acg720_ov5640_ddr3_hdmi.cst
    constraints/acg720_ov5640_ddr3_hdmi.sdc
}

foreach source $sources {
    add_file $source
}

set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0
set_option -top_module acg720_ov5640_ddr3_hdmi_top
set_option -verilog_std v2001
set_option -output_base_name acg720_ov5640_ddr3_hdmi
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1
run all
