# Standalone receiver test. Does not overwrite the production falcons.fs.
# Uses canonical pin assignments; generated test constraints live only under impl/.
set project_root [file dirname [file normalize [info script]]]
cd $project_root
source scripts/vendor_sources.tcl
set files [concat [get_vendor_sources {rc_pwm_receiver uart sync_fifo}] [get_app_sources]]
set absolute_sources {}
foreach f $files { lappend absolute_sources [file normalize $f] }
set output_root [file join $project_root impl rc_led_test]
file mkdir $output_root
set test_cst [file join $output_root rc_led_test.cst]
set input [open [file join $project_root constr falcons.cst] r]
set output [open $test_cst w]
fconfigure $input -encoding utf-8
fconfigure $output -encoding utf-8
puts $output "// Generated from constr/falcons.cst; edit the canonical file, not this one."
set ports {i_clk_50m i_key_reset_n i_key1_n i_rc_pwm o_uart_tx o_stby o_motor_in1 o_motor_in2 o_motor_pwm o_led}
set signal_count 0
while {[gets $input line] >= 0} {
    if {[regexp {^IO_(LOC|PORT)\s+"([^"]+)"} $line unused kind signal]} {
        set base [regsub {\[[0-9]+\]$} $signal ""]
        if {[lsearch -exact $ports $base] >= 0} {
            puts $output $line
            if {$kind eq "LOC"} { incr signal_count }
        }
    }
}
close $input
close $output
if {$signal_count != 31} { error "Expected 31 diagnostic IO assignments, got $signal_count" }
set test_sdc [file join $output_root rc_led_test.sdc]
set output [open $test_sdc w]
fconfigure $output -encoding utf-8
puts $output {create_clock -name i_clk_50m -period 20.000 [get_ports {i_clk_50m}]}
close $output
# Isolate all synthesis and routing state from the production build.
cd $output_root
foreach f $absolute_sources { add_file $f }
add_file $test_cst
add_file $test_sdc
set_device -name GW5AT-138B GW5AT-LV138PG484AC1/I0
set_option -top_module rc_led_test_top
set_option -verilog_std v2001
set_option -output_base_name rc_led_test
set_option -print_all_synthesis_warning 1
set_option -show_all_warn 1
run all
puts "DIAGNOSTIC_BITSTREAM: [file join $output_root impl pnr rc_led_test.fs]"
