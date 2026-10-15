# ModelSim自动测试脚本
# 使用方法：先cd到本文件目录，再执行 do run_test.do

set script_dir [pwd]
set work_dir   [file join $script_dir work]
set fifo_rtl   [file normalize [file join $script_dir .. .. sync_fifo rtl sync_fifo.v]]
set uart_rtl   [file normalize [file join $script_dir .. rtl]]
set tb_file    [file normalize [file join $script_dir uart_queue_loopback_tb.v]]

if {![file exists $work_dir]} {
    vlib $work_dir
}
vmap work $work_dir

vlog -work work \
    $fifo_rtl \
    [file join $uart_rtl uart_byte_tx.v] \
    [file join $uart_rtl uart_byte_rx.v] \
    [file join $uart_rtl uart_tx_queue.v] \
    [file join $uart_rtl uart_rx_queue.v] \
    $tb_file

vsim -voptargs=+acc work.uart_queue_loopback_tb
run -all
quit -sim
