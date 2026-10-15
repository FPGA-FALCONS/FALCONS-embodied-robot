create_clock -name cfg_clk -period 20.000 [get_ports {cfg_clk}]

# Measured configuration: about 52.5 MHz DVP PCLK from the module's 24 MHz oscillator.
create_clock -name camera_pclk -period 19.048 [get_ports {camera_pclk}]
