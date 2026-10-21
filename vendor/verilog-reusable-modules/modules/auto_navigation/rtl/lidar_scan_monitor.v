// ============================================================================
// 模块：lidar_scan_monitor
// ----------------------------------------------------------------------------
// 职责：
//   - 根据报文角和槽序号检查 M10 整圈连续性，给出 scan_valid 电平与 radar_frame_done 事件。
//   - 只检查被接受的点序列，不缓存点，不检查距离或世界坐标，也不判断观测时效。
//   - stream_error 优先丢弃当前序列，后续从新报文首槽重新建立整圈。
// 接口约定：
//   - 单时钟、异步低有效复位；point_accepted 为本拍点 valid && ready。
//   - point_index 是每报文 0..41 的槽号，包括无效/填充距离槽，不能预先滤掉。
//   - point_angle_cdeg 单位 0.01°，仅 0 号槽用于报文角判定；相邻报文角增量 1500。
//   - radar_frame_done 单拍，scan_valid 持续到错误；断流时效由 nav_safety 单独判断。
// 关键不变量：
//   - 整圈要求 24 报文×42 槽；上一报文全部消费后遇到下一首槽，才判断回绕。
//   - 启动先找到一次合法回绕建立起点，再完整收集 24 报文并再次回绕才能置有效。
// 禁止事项：
//   - 不要丢掉 FFFF 距离槽后再送本模块，否则槽号断裂会永久撤销 scan_valid。
//   - 不要用中间槽插值角回绕作为整圈完成，也不要把 scan_valid 当观测新鲜度。
// ============================================================================
module lidar_scan_monitor (
    input  wire               clk,
    input  wire               reset_n,
    input  wire               point_accepted,
    input  wire        [15:0] point_angle_cdeg,
    input  wire         [5:0] point_index,
    input  wire               stream_error,
    // 完整整圈完成的单拍事件。
    output reg                radar_frame_done,
    output reg                scan_valid
);
    reg packet_sequence_active;
    reg [5:0] expected_index;
    reg [15:0] previous_packet_angle;
    reg circle_started;
    reg [4:0] circle_packet_count;

    // 报文角序列的校验：每一报文递增 15 度（1500 cdeg），到 360 度（36000 cdeg）
    // 回绕。用 17 位加法容纳 36000 附近的进位，回绕时单次减去 36000 即可。
    wire [16:0] packet_angle_sum = {1'b0, previous_packet_angle} + 17'd1500;
    wire [16:0] next_packet_angle = packet_angle_sum >= 17'd36000 ? packet_angle_sum - 17'd36000 :
        packet_angle_sum;
    wire packet_angle_matches = {1'b0, point_angle_cdeg} == next_packet_angle;
    wire packet_angle_wraps = point_angle_cdeg < previous_packet_angle;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            radar_frame_done       <= 1'b0;
            scan_valid             <= 1'b0;
            packet_sequence_active <= 1'b0;
            expected_index         <= 6'd0;
            previous_packet_angle  <= 16'd0;
            circle_started         <= 1'b0;
            circle_packet_count    <= 5'd0;
        end else begin
            radar_frame_done <= 1'b0;
            if (stream_error) begin
                scan_valid             <= 1'b0;
                packet_sequence_active <= 1'b0;
                expected_index         <= 6'd0;
                previous_packet_angle  <= 16'd0;
                circle_started         <= 1'b0;
                circle_packet_count    <= 5'd0;
            end else if (point_accepted) begin
                if (point_angle_cdeg >= 16'd36000) begin
                    scan_valid             <= 1'b0;
                    packet_sequence_active <= 1'b0;
                    expected_index         <= 6'd0;
                    previous_packet_angle  <= 16'd0;
                    circle_started         <= 1'b0;
                    circle_packet_count    <= 5'd0;
                end else if (!packet_sequence_active) begin
                    // 启动/复位/出错可能留下半个报文。只有新的索引 0
                    // 才能重新建立槽序列（slot sequence）。
                    if (point_index == 6'd0) begin
                        packet_sequence_active <= 1'b1;
                        expected_index         <= 6'd1;
                        previous_packet_angle  <= point_angle_cdeg;
                    end
                end else if (point_index != expected_index) begin
                    scan_valid          <= 1'b0;
                    circle_started      <= 1'b0;
                    circle_packet_count <= 5'd0;
                    // 一个意外的报文起始同样是一次全新的播种；它之前那个
                    // 不完整的报文无法确立回绕（wrap）。
                    if (point_index == 6'd0) begin
                        expected_index        <= 6'd1;
                        previous_packet_angle <= point_angle_cdeg;
                    end else begin
                        packet_sequence_active <= 1'b0;
                        expected_index         <= 6'd0;
                        previous_packet_angle  <= 16'd0;
                    end
                end else if (point_index == 6'd0) begin
                    // 走到这里才可能出现整圈边界：上一报文的 42 个槽全部按序消费完
                    // （expected_index 已回到 0），本拍是新的 0 号槽。先要求报文角
                    // 正好等于前一报文角 + 15 度，再要求它发生回绕。
                    expected_index        <= 6'd1;
                    previous_packet_angle <= point_angle_cdeg;
                    if (!packet_angle_matches) begin
                        scan_valid          <= 1'b0;
                        circle_started      <= 1'b0;
                        circle_packet_count <= 5'd0;
                    end else if (packet_angle_wraps) begin
                        // expected_index==0 证明前一个报文的最后一个槽在这个整圈标记
                        // 之前已经完成；circle_packet_count==24 说明覆盖的角度恰好是
                        // 360 度。两者同时成立才承认一个整圈。
                        if (circle_started && circle_packet_count == 5'd24) begin
                            radar_frame_done <= 1'b1;
                            scan_valid       <= 1'b1;
                        end else begin
                            scan_valid <= 1'b0;
                        end
                        circle_started      <= 1'b1;
                        circle_packet_count <= 5'd1;
                    end else if (circle_started) begin
                        circle_packet_count <= circle_packet_count + 1'b1;
                    end
                end else begin
                    expected_index <= point_index == 6'd41 ? 6'd0 : point_index + 1'b1;
                end
            end
        end
    end
endmodule
