# ModelSim automated test for the fixed 1024x600 display timing.
# Run from this directory with: do run_test.do

set script_dir [pwd]
set work_dir   [file join $script_dir work]
set rtl_dir    [file normalize [file join $script_dir .. rtl]]
set driver     [file join $rtl_dir disp_driver.v]
set tb_file    [file normalize [file join $script_dir disp_driver_tb.v]]

if {![file exists $work_dir]} {
    vlib $work_dir
}
vmap work $work_dir

vlog -work work "+incdir+$rtl_dir" $driver $tb_file
vsim -voptargs=+acc work.disp_driver_tb
run -all
quit -sim
