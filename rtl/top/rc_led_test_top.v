// Receiver-only board test. The motor outputs are constant zero in ALL states.
// Uses the current production HT10A profile: confirm CH5, select CH7, CH8 unused.
`timescale 1ns/1ps
module rc_led_test_top #(
    parameter integer CLK_HZ=50_000_000, BAUD=115200, REPORT_HZ=10,
    parameter integer KEY_DEBOUNCE_MS=10, RESET_DEBOUNCE_MS=20, EVENT_HOLD_MS=500
)(
    input wire i_clk_50m,i_key_reset_n,i_key1_n,
    input wire [5:0] i_rc_pwm,
    output wire o_uart_tx,o_stby,
    output wire [3:0] o_motor_in1,o_motor_in2,o_motor_pwm,
    output wire [7:0] o_led
);
    wire rst_n,next_view;
    wire signed [11:0] steer,throttle;
    wire [1:0] mode;
    wire confirm,select,menu_up,menu_down,signal_valid,view;
    wire [95:0] chan_us;
    wire [5:0] valid,timeout;
    wire [3:0] event_display;
    reset_sync #(.CLK_FREQ_HZ(CLK_HZ),.DEBOUNCE_MS(RESET_DEBOUNCE_MS)) u_reset(
        .i_clock(i_clk_50m),.i_key_n(i_key_reset_n),.o_reset_n(rst_n));
    app_key #(.CLK_FREQ_HZ(CLK_HZ),.DEBOUNCE_MS(KEY_DEBOUNCE_MS)) u_view_key(
        .i_clock(i_clk_50m),.i_reset_n(rst_n),.i_key_n(i_key1_n),
        .o_pressed(),.o_press_pulse(next_view),.o_release_pulse());
    ht10a_input #(.CLK_FREQ_HZ(CLK_HZ)) u_rc(
        .clk(i_clk_50m),.rst_n(rst_n),.rc_pwm(i_rc_pwm),
        .o_steer(steer),.o_throttle(throttle),.o_mode(mode),
        .o_btn_select(select),.o_btn_confirm(confirm),.o_menu_up(menu_up),.o_menu_down(menu_down),
        .o_signal_valid(signal_valid),.o_chan_us(chan_us),.o_chan_timeout(timeout),.o_chan_valid(valid));
    rc_led_observer #(.CLK_HZ(CLK_HZ),.EVENT_HOLD_MS(EVENT_HOLD_MS)) u_led(
        .clk(i_clk_50m),.rst_n(rst_n),.i_next_view(next_view),.i_signal_valid(signal_valid),
        .i_mode(mode),.i_steer(steer),.i_throttle(throttle),.i_confirm(confirm),.i_select(select),
        .i_menu_up(menu_up),.i_menu_down(menu_down),.o_view(view),.o_led(o_led),.o_events(event_display));
    rc_probe_report #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.REPORT_HZ(REPORT_HZ)) u_report(
        .clk(i_clk_50m),.rst_n(rst_n),.i_view(view),.i_signal_valid(signal_valid),.i_mode(mode),
        .i_steer(steer),.i_throttle(throttle),.i_chan_us(chan_us),.i_valid(valid),.i_timeout(timeout),
        .i_events(event_display),.o_uart_tx(o_uart_tx));
    assign o_stby=1'b0;
    assign o_motor_in1=4'b0;
    assign o_motor_in2=4'b0;
    assign o_motor_pwm=4'b0;
endmodule
