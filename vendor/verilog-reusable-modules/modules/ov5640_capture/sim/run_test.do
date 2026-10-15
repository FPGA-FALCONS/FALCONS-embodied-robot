# ModelSim automated test for the SCCB/I2C bit shifter.
# Run from this directory with: do run_test.do

set script_dir [pwd]
set work_dir   [file join $script_dir work]
set rtl_file   [file normalize [file join $script_dir .. rtl i2c_bit_shift.v]]
set tb_file    [file normalize [file join $script_dir i2c_bit_shift_tb.v]]

if {![file exists $work_dir]} {
    vlib $work_dir
}
vmap work $work_dir

vlog -work work $rtl_file $tb_file
vsim -voptargs=+acc work.i2c_bit_shift_tb
run -all
quit -sim
