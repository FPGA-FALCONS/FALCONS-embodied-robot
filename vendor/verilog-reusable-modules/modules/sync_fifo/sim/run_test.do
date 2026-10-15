# ModelSim自动测试脚本
# 使用方法：先cd到本文件目录，再执行 do run_test.do

set script_dir [pwd]
set work_dir   [file join $script_dir work]
set rtl_file   [file normalize [file join $script_dir .. rtl sync_fifo.v]]
set tb_file    [file normalize [file join $script_dir sync_fifo_tb.v]]

if {![file exists $work_dir]} {
    vlib $work_dir
}
vmap work $work_dir

vlog -work work $rtl_file $tb_file
vsim -voptargs=+acc work.sync_fifo_tb
run -all
quit -sim
