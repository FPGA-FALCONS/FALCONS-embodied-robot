// Application-only AI controls. Existing six J6 assignments are unchanged.
module ai_rc_control #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer INSPECTION_ENABLE=0
)(
    input wire clk, rst_n,
    input wire i_enter_confirm, i_ai_page, i_back,
    input wire i_select_prev,i_select_next,i_feature_confirm,
    input wire i_rc_valid, i_manual, i_estop,i_host_alive,
    input wire i_swb_pwm,
    input wire i_inspection_close,
    output reg o_inspection_open,
    output reg [1:0] o_inspection_view,
    output reg o_master,
    output reg o_record,
    output wire o_chat,o_vision,
    output reg [1:0] o_mode,o_selected,
    output wire [15:0] o_flags,
    output wire [7:0] o_controls
);
    wire [15:0] swb_us;
    wire swb_valid,swb_update;
    rc_pwm_capture #(.CLK_HZ(CLK_HZ)) swb(
        .clk(clk),.rst_n(rst_n),.rc_pwm(i_swb_pwm),.pulse_width_us(swb_us),
        .pulse_valid(swb_valid),.pulse_update(swb_update),.timed_out());
    reg swb_candidate,swb_seen,swb_ready,swb_down;
    wire swb_classifiable=swb_us<16'd1200 || swb_us>16'd1800;
    wire swb_next=swb_us>16'd1800; // down/high starts; up/low stops recording.
    reg record_armed,confirm_toggle;
    reg [1:0] mode_d;
    wire active=o_master && i_rc_valid && swb_ready && swb_valid && !i_manual && !i_estop;
    assign o_chat=o_mode==2;
    assign o_vision=active && o_mode==1;
    assign o_flags={11'd0,(swb_ready && swb_valid),i_estop,i_manual,i_rc_valid,o_master};
    // bit5翻转表示一次明确确认，允许暂停后重新启动同一个识别功能。
    assign o_controls={2'd0,confirm_toggle,o_selected,o_record,o_mode};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            o_master<=0;o_mode<=0;o_selected<=0;o_record<=0;
            o_inspection_open<=0;o_inspection_view<=1;
            swb_candidate<=0;swb_seen<=0;swb_ready<=0;swb_down<=0;record_armed<=0;mode_d<=0;confirm_toggle<=0;
        end else begin
            if(!i_host_alive || !i_rc_valid || i_estop) begin o_master<=0;o_inspection_open<=0;end
            else if(i_enter_confirm && !i_manual) begin o_master<=1;o_mode<=0;o_selected<=0;o_inspection_open<=0;end
            else if(!i_ai_page || (i_back && !o_inspection_open)) begin o_master<=0;o_inspection_open<=0;end
            else if(o_master && !i_manual) begin
                if(o_inspection_open)begin
                    if(i_select_prev)o_inspection_view<=o_inspection_view==0?2:o_inspection_view-1'b1;
                    else if(i_select_next)o_inspection_view<=o_inspection_view==2?0:o_inspection_view+1'b1;
                    // A child-task back gesture is consumed by its state
                    // machine on RK. Closing requires a stationary receipt.
                    if(i_inspection_close)o_inspection_open<=0;
                end else begin
                    if(i_select_prev) o_selected<=o_selected==0?(INSPECTION_ENABLE!=0?3:2):o_selected-1'b1;
                    else if(i_select_next) o_selected<=o_selected==(INSPECTION_ENABLE!=0?3:2)?0:o_selected+1'b1;
                    if(i_feature_confirm) begin
                        o_mode<=o_selected;confirm_toggle<=!confirm_toggle;
                        if(INSPECTION_ENABLE!=0 && o_selected==3)begin o_inspection_open<=1;o_inspection_view<=1;end
                    end
                end
            end
            if(!swb_valid) begin swb_seen<=0;swb_ready<=0;end
            else if(swb_update && swb_classifiable) begin
                swb_candidate<=swb_next;swb_seen<=1;
                if(swb_seen && swb_candidate==swb_next) begin swb_down<=swb_next;swb_ready<=1;end
            end
            mode_d<=o_mode;
            if(!active || o_mode!=mode_d) begin o_record<=0;record_armed<=0;end
            else if(!swb_down) begin o_record<=0;record_armed<=1;end
            else if(record_armed) o_record<=1;
        end
    end
endmodule
