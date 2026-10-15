# IMU660RA 模块时序约束
# 本模块为单时钟域设计，只需约束系统时钟。
# SCK 是输出给外部器件的时钟，由寄存器翻转产生，不需要额外约束。
create_clock -name i_clock -period 20.000 [get_ports {i_clock}]
