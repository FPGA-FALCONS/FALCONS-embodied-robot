set sources {
    ../../../sync_fifo/rtl/sync_fifo.v
    ../../../uart/rtl/uart_byte_tx.v
    ../../../uart/rtl/uart_tx_queue.v
    ../../rtl/spi_master_mode0.v
    ../../rtl/bmi270_cfg_rom.v
    ../../rtl/imu_top.v
    uart_report.v
    imu_uart_top.v
    acg720_imu_uart.cst
    acg720_imu_uart.sdc
}

foreach source $sources {
    add_file $source
}

set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0
set_option -top_module imu_uart_top
set_option -verilog_std v2001
set_option -output_base_name imu_uart_demo
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1
run all
