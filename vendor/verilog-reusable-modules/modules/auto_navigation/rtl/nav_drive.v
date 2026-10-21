// ============================================================================
// 模块：nav_drive
// ----------------------------------------------------------------------------
// 职责：
//   - 独立读取 raw 图，判断当前轴向直行的车前矩形；不读取 cost。
//   - 默认从前缘向前 3 格，半宽为 ceil(车宽/2)+横向余量；相切格也纳入。
//   - 不限制停稳后的原地转向，不估算底盘制动距离，不输出速度。
// 接口约定：
//   - 单时钟、异步低有效复位；内部按 50 MHz 使用 16 位扫描计数和 17 位结果年龄计数。
//   - 位置用 mm；heading 由路线给出：0 东、1 北、2 西、3 南，不检查实际朝向。
//   - 每 1 ms 启动扫描，扫描超时 1 ms，结果距扫描开始必须小于 2 ms。
//   - mrd_valid/ready 接受格号，mrd_rsp_valid 返回 raw；请求成交后失效也须排空响应。
// 关键不变量：
//   - forward_clear 同时要求方向、检查使能和时效有效；未就绪并不等于已知无障碍。
//   - segment_blocked 只表示有效结果命中 raw/边界，不覆盖过期、无方向和等待结果。
//   - 前探参数相对同一个位姿参考点；默认车宽 180 mm，前缘 150 mm 和余量 20 mm 待整定。
// 禁止事项：
//   - 不要在转向途中沿用旧方向结果，不要把 segment_blocked=0 当成 forward_clear=1。
// ============================================================================
module nav_drive #(
    parameter integer GRID_W              = 256,
                      GRID_H              = 256,
                      CELL_MM             = 100,
    parameter signed  HALF_MM             = 12800,
    parameter integer FORWARD_CHECK_CELLS = 3,
    parameter integer VEHICLE_WIDTH_MM    = 180,
    parameter integer FRONT_OFFSET_MM     = 150,
    parameter integer LATERAL_MARGIN_MM   = 20
) (
    input  wire               clk,
                              reset_n,
    input  wire signed [31:0] pose_x_mm,
                              pose_y_mm,
    input  wire         [1:0] heading,
    input  wire               pose_valid,
                              check_enable,
    output wire               mrd_valid,
    input  wire               mrd_ready,
    output wire        [15:0] mrd_x,
                              mrd_y,
    input  wire               mrd_rsp_valid,
                              mrd_raw,
    output wire               forward_clear,
                              segment_blocked,
                              busy
);
    localparam [15:0] PERIOD  = 16'd50000;
    localparam [16:0] MAX_AGE = 17'd100000;
    localparam integer        HALF_WIDTH = (VEHICLE_WIDTH_MM + 1) / 2 + LATERAL_MARGIN_MM;
    localparam integer        DIST       = FORWARD_CHECK_CELLS * CELL_MM;
    localparam integer        DIV_M      = ((1 << 22) + CELL_MM - 1) / CELL_MM;
    localparam         [ 2:0] IDLE     = 0,
                              GEOMETRY = 1,
                              BOUNDS   = 2,
                              REQUEST  = 3,
                              RESPONSE = 4;
    reg [2:0] state;
    wire check_ok = check_enable && pose_valid;
    reg [1:0] scan_dir, result_dir;
    reg [15:0] scan_count;
    reg [16:0] result_count;
    reg scan_due, scan_bad, result_valid, result_block;
    wire expired = (state != IDLE) && scan_due;
    wire        stale_scan = scan_bad || expired || !check_ok || (heading != scan_dir);
    reg signed [31:0] sx, sy, xmin, xmax, ymin, ymax;
    reg [15:0] xlo, xhi, yhi, cx, cy;
    function [15:0] to_cell;
        input signed [31:0] mm;
        reg [31:0] product;
        begin
            product = (mm + HALF_MM) * DIV_M;
            to_cell = product >> 22;
        end
    endfunction
    assign busy = reset_n && (state != IDLE);
    assign mrd_valid = reset_n && (state == REQUEST) && !stale_scan;
    assign mrd_x = cx;
    assign mrd_y = cy;
    assign forward_clear = reset_n && check_ok && result_valid && heading == result_dir &&
        !result_block && !expired;
    assign segment_blocked = reset_n && check_ok && result_valid && heading == result_dir &&
        result_block;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state        <= IDLE;
            scan_count   <= 0;
            result_count <= 0;
            scan_dir     <= 0;
            result_dir   <= 0;
            scan_due     <= 1;
            scan_bad     <= 0;
            result_valid <= 0;
            result_block <= 0;
            sx           <= 0;
            sy           <= 0;
            xmin         <= 0;
            xmax         <= 0;
            ymin         <= 0;
            ymax         <= 0;
            xlo          <= 0;
            xhi          <= 0;
            yhi          <= 0;
            cx           <= 0;
            cy           <= 0;
        end else begin
            // 到期后保持，直到新扫描启动或新结果返回；不会回绕放行旧结果。
            if (!scan_due) begin
                if (scan_count == PERIOD - 16'd2) scan_due <= 1;
                else scan_count <= scan_count + 1'b1;
            end
            if (result_valid) begin
                if (result_count == MAX_AGE - 17'd1) result_valid <= 0;
                else result_count <= result_count + 1'b1;
            end
            if (!check_ok || heading != scan_dir) begin
                scan_due     <= 1;
                result_valid <= 0;
                scan_bad     <= 1;
            end
            if (expired) begin
                result_valid <= 0;
                scan_bad     <= 1;
            end
            case (state)
                IDLE:
                if (check_ok && (scan_due || heading != scan_dir)) begin
                    sx         <= pose_x_mm;
                    sy         <= pose_y_mm;
                    scan_dir   <= heading;
                    scan_count <= 0;
                    scan_due   <= 0;
                    scan_bad   <= 0;
                    state      <= GEOMETRY;
                end
                GEOMETRY: begin
                    case (scan_dir)
                        0: begin
                            xmin <= sx + FRONT_OFFSET_MM;
                            xmax <= sx + FRONT_OFFSET_MM + DIST;
                            ymin <= sy - HALF_WIDTH;
                            ymax <= sy + HALF_WIDTH;
                        end
                        1: begin
                            ymin <= sy + FRONT_OFFSET_MM;
                            ymax <= sy + FRONT_OFFSET_MM + DIST;
                            xmin <= sx - HALF_WIDTH;
                            xmax <= sx + HALF_WIDTH;
                        end
                        2: begin
                            xmin <= sx - FRONT_OFFSET_MM - DIST;
                            xmax <= sx - FRONT_OFFSET_MM;
                            ymin <= sy - HALF_WIDTH;
                            ymax <= sy + HALF_WIDTH;
                        end
                        3: begin
                            ymin <= sy - FRONT_OFFSET_MM - DIST;
                            ymax <= sy - FRONT_OFFSET_MM;
                            xmin <= sx - HALF_WIDTH;
                            xmax <= sx + HALF_WIDTH;
                        end
                    endcase
                    if (stale_scan) state <= IDLE;
                    else state <= BOUNDS;
                end
                BOUNDS: begin
                    if (stale_scan) state <= IDLE;
                    else if (xmin <= -HALF_MM || ymin <= -HALF_MM || xmax >=
                             GRID_W * CELL_MM - HALF_MM || ymax >= GRID_H * CELL_MM - HALF_MM) begin
                        result_valid <= 1;
                        result_block <= 1;
                        result_dir   <= scan_dir;
                        result_count <= {1'b0, scan_count} + 17'd1;
                        state        <= IDLE;
                    end else begin
                        // 下界减 1 mm：两相邻格与闭区间端点相切时也纳入。
                        xlo   <= to_cell(xmin - 1);
                        cx    <= to_cell(xmin - 1);
                        xhi   <= to_cell(xmax);
                        cy    <= to_cell(ymin - 1);
                        yhi   <= to_cell(ymax);
                        state <= REQUEST;
                    end
                end
                REQUEST: begin
                    if (mrd_valid && mrd_ready) state <= RESPONSE;
                    else if (stale_scan) state <= IDLE;
                end
                RESPONSE:
                if (mrd_rsp_valid) begin
                    // 已被接受的读即使失效也排空；绝不把旧方向响应当成新窗口。
                    if (stale_scan) state <= IDLE;
                    else if (mrd_raw || (cx == xhi && cy == yhi)) begin
                        result_valid <= 1;
                        result_block <= mrd_raw;
                        result_dir   <= scan_dir;
                        result_count <= {1'b0, scan_count} + 17'd1;
                        state        <= IDLE;
                    end else begin
                        if (cx == xhi) begin
                            cx <= xlo;
                            cy <= cy + 16'd1;
                        end else cx <= cx + 16'd1;
                        state <= REQUEST;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
