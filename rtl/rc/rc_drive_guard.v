// Manual drive interlock. Centering arms once; pushing throttle then remains allowed.
// Loss, leaving manual, emergency stop or S1 stop disarms; recovery requires center again.
module rc_drive_guard #(parameter integer ARM_DEADZONE=100)(
    input wire clk,rst_n,
    input wire i_manual,i_signal_valid,i_estop,i_stop_pulse,
    input wire signed [11:0] i_throttle,
    output wire o_enable,o_arm_hint
);
    reg armed, stopped;
    wire centered=(i_throttle<=ARM_DEADZONE && i_throttle>=-ARM_DEADZONE);
    assign o_enable=rst_n && armed && !stopped && i_manual && i_signal_valid && !i_estop && !i_stop_pulse;
    assign o_arm_hint=i_manual && i_signal_valid && !i_estop && !stopped && !armed && !centered;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin armed<=0; stopped<=0; end
        else if (!i_manual) begin armed<=0; stopped<=0; end
        else if (i_stop_pulse) begin armed<=0; stopped<=!stopped; end
        else if (!i_signal_valid || i_estop || stopped) armed<=0;
        else if (centered) armed<=1;
    end
endmodule
