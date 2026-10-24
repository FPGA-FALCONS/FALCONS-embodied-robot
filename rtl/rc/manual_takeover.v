// SWC manual always suppresses autonomous commands in the chassis mux.
// Before accepting sticks, brake for at least 200ms AND observe stopped wheels
// in three fresh encoder windows. Both sticks must be centered to hand over.
module manual_takeover #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer HOLD_MS=200,
    parameter integer DEADZONE=100,
    parameter integer STOP_DELTA=2,
    parameter integer STOP_SAMPLES=3
)(
    input wire clk,rst_n,i_manual,i_signal_valid,i_estop,
    input wire signed [11:0] i_throttle,i_steer,
    input wire i_sample_valid,
    input wire signed [31:0] i_delta0,i_delta1,i_delta2,i_delta3,
    output reg o_ready,
    output wire o_brake
);
    localparam integer HOLD_CYCLES=(CLK_HZ/1000)*HOLD_MS;
    reg [31:0] elapsed;
    reg [7:0] quiet;
    wire centered=i_throttle<=DEADZONE && i_throttle>=-DEADZONE &&
                  i_steer<=DEADZONE && i_steer>=-DEADZONE;
    wire stopped=i_delta0<=STOP_DELTA && i_delta0>=-STOP_DELTA &&
                 i_delta1<=STOP_DELTA && i_delta1>=-STOP_DELTA &&
                 i_delta2<=STOP_DELTA && i_delta2>=-STOP_DELTA &&
                 i_delta3<=STOP_DELTA && i_delta3>=-STOP_DELTA;
    assign o_brake=rst_n && i_manual && i_signal_valid && !o_ready;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin elapsed<=0;quiet<=0;o_ready<=0;end
        else if(!i_manual || !i_signal_valid || i_estop) begin elapsed<=0;quiet<=0;o_ready<=0;end
        else if(!o_ready) begin
            if(elapsed<HOLD_CYCLES)elapsed<=elapsed+1'b1;
            if(i_sample_valid)begin
                if(!stopped)quiet<=0;
                else if(quiet<STOP_SAMPLES)quiet<=quiet+1'b1;
                // Fresh stopped feedback is required on the release edge.
                if(stopped && quiet>=STOP_SAMPLES-1 && elapsed>=HOLD_CYCLES && centered)o_ready<=1;
            end
        end
    end
endmodule
