set example_root [file dirname [file normalize [info script]]]
set module_root [file dirname [file dirname $example_root]]
file mkdir [file join $example_root build]
cd [file join $example_root build]
set_device -name GW5AT-138C GW5AT-LV138PG484AC1/I0 -device_version C
add_file [file join $module_root rtl rc_pwm_capture.v]
add_file [file join $example_root .. acg720_rc_debug rc_hc595_display.v]
foreach name {rc_4ch_top.v rc_4ch.cst rc_4ch.sdc} {
    add_file [file join $example_root $name]
}
set_option -top_module rc_4ch_top
set_option -global_freq 50
run all
puts "RC_4CH_BUILD_FINISHED"
