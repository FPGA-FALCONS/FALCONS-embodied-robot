# ModelSim GUI entry: do /path/to/sim/run_test.do
# This GUI helper is provided but has not been executed in our verification.
# RTL is Verilog-2001. The benches require SystemVerilog ($fatal).
onerror {abort}
set rc_sim_dir [file dirname [file normalize [info script]]]
set rc_module_dir [file dirname $rc_sim_dir]
set rc_build_dir [file join $rc_module_dir build modelsim]
set rc_work_dir [file join $rc_build_dir work]
file mkdir $rc_build_dir
if {![file exists $rc_work_dir]} {
    vlib $rc_work_dir
}
vmap rc_pwm_work $rc_work_dir
vlog -work rc_pwm_work [file join $rc_module_dir rtl rc_axis_normalize.v] [file join $rc_module_dir rtl rc_pwm_capture.v] [file join $rc_module_dir rtl rc_receiver.v]
foreach rc_bench {rc_receiver_tb rc_axis_normalize_tb rc_receiver_pipeline_tb} {
    transcript file [file join $rc_build_dir ${rc_bench}.log]
    vlog -sv -work rc_pwm_work [file join $rc_sim_dir ${rc_bench}.v]
    vsim -quiet rc_pwm_work.$rc_bench
    onfinish stop
    run -all
    quit -sim
}
transcript file ""
echo "All three GUI simulations returned. Inspect each transcript for its PASS marker."
