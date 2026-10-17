# Module timing target only. No board pin assignments are implied.
create_clock -name clk_50m -period 20.000 [get_ports {clk}]
