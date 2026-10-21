// 导航位姿适配：odom 更新后成组寄存毫米位置与回绕航向。
// ready 为定位初始化完成电平；clear 与坐标清零同源。
module nav_pose (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear,
    input  wire               ready,
    input  wire               position_tick,
    input  wire signed [31:0] x_mm,
    input  wire signed [31:0] y_mm,
    input  wire signed [31:0] yaw_cdeg,
    output reg                pose_valid,
    output reg signed [31:0]  pose_x_mm,
    output reg signed [31:0]  pose_y_mm,
    output reg signed [31:0]  pose_yaw_cdeg
);
    reg capture;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            capture       <= 0;
            pose_valid    <= 0;
            pose_x_mm     <= 0;
            pose_y_mm     <= 0;
            pose_yaw_cdeg <= 0;
        end else if (clear || !ready) begin
            capture    <= 0;
            pose_valid <= 0;
        end else begin
            capture <= position_tick;
            if (capture) begin
                pose_x_mm     <= x_mm;
                pose_y_mm     <= y_mm;
                pose_yaw_cdeg <= yaw_cdeg == 32'sd18000 ? -32'sd18000 : yaw_cdeg;
                pose_valid    <= 1;
            end
        end
    end
endmodule
