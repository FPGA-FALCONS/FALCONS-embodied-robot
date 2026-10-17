set example_root [file dirname [file normalize [info script]]]
set module_root [file dirname [file dirname $example_root]]
set output_root [file join $example_root build]
file mkdir $output_root
cd $output_root
set_device -name GW5AT-138C GW5AT-LV138PG484AC1/I0 -device_version C
foreach name {tof_i2c_reg_master.v tof_vl53l0x.v} {
    add_file [file join $module_root rtl $name]
}
foreach name {tof_display_formatter.v tof_hc595_display.v tof_debug_top.v tof_debug.cst tof_debug.sdc} {
    add_file [file join $example_root $name]
}
set_option -top_module tof_debug_top
set_option -global_freq 50
run all
puts "TOF_DEBUG_BUILD_FINISHED"
