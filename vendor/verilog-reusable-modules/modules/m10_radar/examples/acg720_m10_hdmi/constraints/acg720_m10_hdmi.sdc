create_clock -name clk_50m -period 20.000 [get_ports {i_clock}]
// PLL-derived clocks are inferred by Gowin from the PLL primitive.
// Only the first register of each asynchronous input synchronizer is excluded.
// radar_rx has exactly one fanout: u_core/u_radar/u_uart/rx_sync_0.
set_false_path -from [get_ports {i_radar_rx}]
set_false_path -from [get_ports {i_reset_n}]
// Each key feeds only the first stage of the u_core/u_zoom synchronizer.
set_false_path -from [get_ports {i_zoom_in_n i_zoom_out_n}]
