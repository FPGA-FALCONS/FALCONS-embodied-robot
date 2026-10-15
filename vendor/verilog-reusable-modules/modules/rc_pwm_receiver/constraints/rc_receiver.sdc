# Verified design clock: 50 MHz = 20 ns. No physical GPIO assignments here.
create_clock -name clk -period 20.000 [get_ports {clk}]
