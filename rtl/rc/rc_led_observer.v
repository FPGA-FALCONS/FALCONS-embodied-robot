// Diagnostic display only. No output from this module can authorize motor drive.
// S1 selects stick/status view (0) or latched event view (1).
`timescale 1ns/1ps
module rc_led_observer #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer EVENT_HOLD_MS=500,
    parameter integer STICK_THRESHOLD=100
)(
    input wire clk,rst_n,i_next_view,i_signal_valid,
    input wire [1:0] i_mode,
    input wire signed [11:0] i_steer,i_throttle,
    input wire i_confirm,i_select,i_menu_up,i_menu_down,
    output reg o_view,
    output wire [7:0] o_led,
    output wire [3:0] o_events
);
    localparam integer PHASE_TICKS=(CLK_HZ/10>0) ? CLK_HZ/10 : 1;
    localparam integer HOLD_TICKS=(CLK_HZ/1000)*EVENT_HOLD_MS;
    reg [31:0] phase_count;
    reg [3:0] phase;
    reg [31:0] event_count[0:3];
    wire [3:0] events_now={i_menu_down,i_menu_up,i_select,i_confirm};
    integer n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_view<=0;phase_count<=0;phase<=0;
            for(n=0;n<4;n=n+1) event_count[n]<=0;
        end else begin
            if(i_next_view) o_view<=!o_view;
            if(phase_count==PHASE_TICKS-1) begin
                phase_count<=0;
                if(phase==9) phase<=0;else phase<=phase+1'b1;
            end else phase_count<=phase_count+1'b1;
            for(n=0;n<4;n=n+1) begin
                if(!i_signal_valid) event_count[n]<=0;
                else if(events_now[n]) event_count[n]<=HOLD_TICKS;
                else if(event_count[n]!=0) event_count[n]<=event_count[n]-1'b1;
            end
        end
    end
    genvar g;
    generate for(g=0;g<4;g=g+1) begin: g_events
        assign o_events[g]=(event_count[g]!=0) && i_signal_valid && rst_n;
    end endgenerate
    // Heartbeat also identifies view: slow square wave vs two short flashes/second.
    assign o_led[7]=rst_n && (o_view ? (phase==0 || phase==2) : phase<5);
    assign o_led[6]=rst_n && i_signal_valid;
    assign o_led[5]=rst_n && i_signal_valid && i_mode==1;
    assign o_led[4:0]=!rst_n ? 5'b0 : o_view ? {o_events,1'b1} : {
        i_signal_valid && i_throttle> STICK_THRESHOLD,
        i_signal_valid && i_throttle< -STICK_THRESHOLD,
        i_signal_valid && i_steer> STICK_THRESHOLD,
        i_signal_valid && i_steer< -STICK_THRESHOLD,
        |o_events
    };
endmodule
