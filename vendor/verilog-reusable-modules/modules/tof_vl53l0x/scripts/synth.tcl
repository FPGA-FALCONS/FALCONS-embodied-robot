set tof_root [file dirname [file dirname [file normalize [info script]]]]
set tof_build [file join $tof_root build synth]
file mkdir $tof_build
cd $tof_build
set_device -name GW5AT-138C GW5AT-LV138PG484AC1/I0 -device_version C
add_file [file join $tof_root rtl tof_i2c_reg_master.v]
add_file [file join $tof_root rtl tof_vl53l0x.v]
add_file [file join $tof_root constraints tof_50mhz.sdc]
set_option -top_module tof_vl53l0x
# GowinSynthesis uses global_freq; frequency is the Synplify option and does
# not change GowinSynthesis's generated project.prj global_freq setting.
# MHz only; this is not post-route timing signoff. SDC is consumed by PnR.
set_option -global_freq 50
run syn
puts "TOF_SYNTH_SCRIPT_FINISHED"
