set example_root [file dirname [file normalize [info script]]]
set module_root [file dirname [file dirname $example_root]]
set output_root [file join $example_root build]
file mkdir $output_root
cd $output_root
set_device -name GW5AT-138C GW5AT-LV138PG484AC1/I0 -device_version C
foreach name {rc_pwm_capture.v rc_axis_normalize.v rc_receiver.v} {
    add_file [file join $module_root rtl $name]
}
foreach name {rc_hc595_display.v rc_debug_top.v rc_debug.cst rc_debug.sdc} {
    add_file [file join $example_root $name]
}
set_option -top_module rc_debug_top
run all
puts "RC_DEBUG_BUILD_FINISHED"
