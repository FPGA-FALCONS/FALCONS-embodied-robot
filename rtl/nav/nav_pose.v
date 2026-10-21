// 导航位姿适配：测速脉冲后一拍 odom 更新位置，再下一拍成组采集。
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
    output reg                pose_tick,
    output reg signed [31:0]  pose_x_mm,
    output reg signed [31:0]  pose_y_mm,
    output reg signed [31:0]  pose_yaw_cdeg
);
    reg [1:0] capture;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            capture       <= 0;
            pose_valid    <= 0;
            pose_tick     <= 0;
            pose_x_mm     <= 0;
            pose_y_mm     <= 0;
            pose_yaw_cdeg <= 0;
        end else if (clear || !ready) begin
            capture    <= 0;
            pose_valid <= 0;
            pose_tick  <= 0;
        end else begin
            capture <= {capture[0], position_tick};
            pose_tick <= capture[1];
            if (capture[1]) begin
                pose_x_mm     <= x_mm;
                pose_y_mm     <= y_mm;
                pose_yaw_cdeg <= yaw_cdeg == 32'sd18000 ? -32'sd18000 : yaw_cdeg;
                pose_valid    <= 1;
            end
        end
    end
endmodule
