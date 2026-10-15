// Standalone m10_radar instance used as top, default 50 MHz.
// Change the port names/clock period when embedding in another top module.
create_clock -name radar_clk -period 20.000 [get_ports {clk}]
// radar_rx feeds only u_uart/rx_sync_0, the first synchronizer stage.
set_false_path -from [get_ports {radar_rx}]
set_false_path -from [get_ports {reset_n}]
