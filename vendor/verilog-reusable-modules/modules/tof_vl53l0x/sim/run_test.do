# Run in Questa/ModelSim: do /absolute/path/to/sim/run_test.do
# This is a simulation script, not a Gowin synthesis file.
onerror {quit -code 1}
onbreak {quit -code 1}
set module_root [file normalize [file join [file dirname [info script]] ..]]
cd $module_root
if {![file exists sim/work]} {vlib sim/work}
vmap work sim/work
vlog rtl/tof_i2c_reg_master.v rtl/tof_vl53l0x.v
vlog -sv tb/tb_tof_i2c_reg_master.v tb/vl53l0x_sensor_model.sv tb/tb_tof_vl53l0x.sv tb/tb_tof_stale.sv
foreach clock {4000000 50000000} {
    vsim -onfinish stop -gCLK_HZ=$clock work.tb_tof_i2c_reg_master
    run -all
    quit -sim
}
foreach top {tb_tof_vl53l0x tb_tof_stale} {
    vsim -onfinish stop work.$top
    run -all
    quit -sim
}
quit -code 0
