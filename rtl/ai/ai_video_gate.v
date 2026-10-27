// Shared display-only gate. Navigation authority remains inspection_permission.
// Register the page/mode decode before the existing high-fanout DDR/UI controls.
module ai_video_gate(
    input wire clk,rst_n,i_ai_page,i_master,i_link,i_manual,i_inspection_open,
    input wire [1:0] i_mode,i_inspection_view,
    output reg o_enabled,
    output reg [1:0] o_view
);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin o_enabled<=0;o_view<=1;end
        else begin
            o_enabled<=i_ai_page && i_master && i_link && !i_manual &&
                       (i_mode==1 || (i_mode==3 && i_inspection_open));
            o_view<=i_mode==1 ? 2'd0 : i_inspection_view;
        end
    end
endmodule
