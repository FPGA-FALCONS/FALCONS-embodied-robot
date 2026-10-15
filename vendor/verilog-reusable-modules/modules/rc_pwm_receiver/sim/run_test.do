# Run from this module's sim directory in ModelSim GUI Transcript.
# RTL is Verilog-2001. The self-checking bench requires SystemVerilog ($fatal).
onerror {abort}
if {![file exists work]} {
    vlib work
}
vmap work work
vlog ../rtl/rc_pwm_capture.v ../rtl/rc_receiver.v
vlog -sv rc_receiver_tb.v
vsim -quiet work.rc_receiver_tb
run -all
quit -sim
