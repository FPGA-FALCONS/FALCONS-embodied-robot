// ============================================================================
// 模块：nav_safety
// ----------------------------------------------------------------------------
// 检查定位就绪、地图范围、整圈连续性和报文错误。
// 信任雷达及定位持续及时更新，不计时、不检测静默或冻结。
// sensors_valid 与 brake_clear 同义；raw 车前检查由 nav_drive 独立完成。
// stop_cause：0 正常，1 未就绪或数据错误，3 位置范围越界。
// ============================================================================
module nav_safety #(
    parameter signed [31:0] MAP_HALF = 32'sd6400
) (
    input  wire               reset_n,
    input  wire               pose_valid,
    input  wire signed [31:0] pose_x_mm,
                              pose_y_mm,
    input  wire               scan_valid,
                              stream_error,
    output wire               sensors_valid,
                              brake_clear,
    output wire         [1:0] stop_cause
);
    wire pose_inside = pose_x_mm >= -MAP_HALF && pose_x_mm < MAP_HALF &&
                       pose_y_mm >= -MAP_HALF && pose_y_mm < MAP_HALF;
    assign sensors_valid = reset_n && pose_valid && pose_inside && scan_valid && !stream_error;
    assign brake_clear = sensors_valid;
    assign stop_cause = !reset_n ? 2'd1 : !pose_inside ? 2'd3 : !sensors_valid ? 2'd1 : 2'd0;
endmodule
