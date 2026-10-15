create_clock -name clk50M -period 20.000 [get_ports {clk50M}]

# Measured configuration: about 52.5 MHz DVP PCLK from the module's 24 MHz oscillator.
create_clock -name camera_pclk -period 19.048 [get_ports {camera_pclk}]
