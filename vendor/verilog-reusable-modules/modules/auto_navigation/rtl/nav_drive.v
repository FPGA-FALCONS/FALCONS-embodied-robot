// ============================================================================
// 模块：nav_drive
// ----------------------------------------------------------------------------
// 职责：
//   - 独立读取 raw 图，判断路线八方向直行的车前矩形；不读取 cost。
//   - 从定位中心点向前探测指定格数，总宽度为车宽；相切格也纳入。
//   - 不限制停稳后的原地转向，不估算底盘制动距离，不输出速度。
// 接口约定：
//   - 单时钟、异步低有效复位；内部按 50 MHz 使用 16 位扫描计数和 17 位结果年龄计数。
//   - 位置用 mm；heading 由路线给出：0 东起逆时针 0..7，不检查实际朝向。
//   - 每 1 ms 启动扫描，扫描超时 1 ms，结果距扫描开始必须小于 2 ms。
//   - mrd_valid/ready 接受格号，mrd_rsp_valid 返回 raw；请求成交后失效也须排空响应。
// 关键不变量：
//   - forward_clear 同时要求方向、检查使能和时效有效；未就绪并不等于已知无障碍。
//   - segment_blocked 只表示有效结果命中 raw/边界，不覆盖过期、无方向和等待结果。
//   - 配置在扫描开始时锁存，UI 默认前探 4 格、车宽 180 mm。
// 禁止事项：
//   - 不要在转向途中沿用旧方向结果，不要把 segment_blocked=0 当成 forward_clear=1。
// ============================================================================
module nav_drive #(
    parameter integer GRID_W              = 256,
                      GRID_H              = 256,
                      CELL_MM             = 50,
    parameter signed  HALF_MM             = 6400
) (
    input  wire               clk,
                              reset_n,

    // 运行时前探配置；每次扫描开始锁存。距离从定位中心点起算。
    input  wire         [7:0] forward_check_cells,
    input  wire        [15:0] vehicle_width_mm,
    input  wire signed [31:0] pose_x_mm,
                              pose_y_mm,
    input  wire         [2:0] heading,
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
    localparam integer DIV_M = ((1 << 22) + CELL_MM - 1) / CELL_MM;
    reg [7:0] scan_cells;
    reg [15:0] scan_width, half_width, distance_mm;
    // Q16 常量投影，尺寸、乘法和边界分拍，运行时不使用开方或除法。
    reg [31:0] diag_half_q16, diag_end_q16, along_q16, across_q16;
    wire signed [31:0] diag_lo = -$signed({16'd0, diag_half_q16[31:16]}) - 1;
    wire signed [31:0] diag_hi = $signed({16'd0, diag_end_q16[31:16]}) + 1;
    wire signed [31:0] along_hi = $signed({16'd0, along_q16[31:16]}) + 1 + CELL_MM;
    wire signed [31:0] across_hi = $signed({16'd0, across_q16[31:16]}) + 1 + CELL_MM;
    localparam [3:0] IDLE=0, GEOMETRY=1, BOUNDS=2, REQUEST=3, RESPONSE=4,
                     CELL_CENTER=5, ROTATE=6, FILTER=7, NEXT_CELL=8, RELATIVE=9, DIMENSIONS=10, PROJECT=11;
    reg [3:0] state;
    reg signed [16:0] cell_dx, cell_dy, along, across;
    reg signed [16:0] cell_world_x, cell_world_y;
    reg cell_inside;
    wire check_ok = check_enable && pose_valid;
    reg [2:0] scan_dir, result_dir;
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
    assign mrd_valid = reset_n && (state == REQUEST) && cell_inside && !stale_scan;
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
            scan_cells   <= 0;
            scan_width   <= 0;
            half_width   <= 0;
            distance_mm  <= 0;
            diag_half_q16 <= 0;
            diag_end_q16 <= 0;
            along_q16    <= 0;
            across_q16   <= 0;
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
            cell_world_x <= 0;
            cell_world_y <= 0;
            cell_dx      <= 0;
            cell_dy      <= 0;
            along        <= 0;
            across       <= 0;
            cell_inside  <= 0;
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
                    scan_cells <= forward_check_cells;
                    scan_width <= vehicle_width_mm;
                    scan_count <= 0;
                    scan_due   <= 0;
                    scan_bad   <= 0;
                    state      <= DIMENSIONS;
                end
                DIMENSIONS: begin
                    half_width  <= ({1'b0, scan_width} + 17'd1) >> 1;
                    distance_mm <= scan_cells * CELL_MM;
                    state       <= PROJECT;
                end
                PROJECT: begin
                    diag_half_q16 <= half_width * 32'd46341;
                    diag_end_q16  <= (half_width + distance_mm) * 32'd46341;
                    along_q16     <= distance_mm * 32'd92682;
                    across_q16    <= half_width * 32'd92682;
                    state         <= GEOMETRY;
                end
                GEOMETRY: begin
                    case (scan_dir)
                        0: begin
                            xmin <= sx;
                            xmax <= sx + distance_mm;
                            ymin <= sy - half_width;
                            ymax <= sy + half_width;
                        end
                        2: begin
                            ymin <= sy;
                            ymax <= sy + distance_mm;
                            xmin <= sx - half_width;
                            xmax <= sx + half_width;
                        end
                        4: begin
                            xmin <= sx - distance_mm;
                            xmax <= sx;
                            ymin <= sy - half_width;
                            ymax <= sy + half_width;
                        end
                        6: begin
                            ymin <= sy - distance_mm;
                            ymax <= sy;
                            xmin <= sx - half_width;
                            xmax <= sx + half_width;
                        end
                        default: begin
                            if (scan_dir==1 || scan_dir==7) begin
                                xmin <= sx+diag_lo;
                                xmax <= sx+diag_hi;
                            end else begin
                                xmin <= sx-diag_hi;
                                xmax <= sx-diag_lo;
                            end
                            if (scan_dir==1 || scan_dir==3) begin
                                ymin <= sy+diag_lo;
                                ymax <= sy+diag_hi;
                            end else begin
                                ymin <= sy-diag_hi;
                                ymax <= sy-diag_lo;
                            end
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
                        state <= CELL_CENTER;
                    end
                end
                CELL_CENTER: begin
                    cell_world_x <= $signed(cx*CELL_MM)+(CELL_MM/2-HALF_MM);
                    cell_world_y <= $signed(cy*CELL_MM)+(CELL_MM/2-HALF_MM);
                    state <= RELATIVE;
                end
                RELATIVE: begin
                    cell_dx <= cell_world_x-sx;
                    cell_dy <= cell_world_y-sy;
                    state <= ROTATE;
                end
                ROTATE: begin
                    case (scan_dir)
                        1: begin along <= cell_dx+cell_dy; across <= cell_dy-cell_dx; end
                        3: begin along <= cell_dy-cell_dx; across <= -cell_dx-cell_dy; end
                        5: begin along <= -cell_dx-cell_dy; across <= cell_dx-cell_dy; end
                        7: begin along <= cell_dx-cell_dy; across <= cell_dx+cell_dy; end
                        default: begin along <= 0; across <= 0; end
                    endcase
                    state <= FILTER;
                end
                FILTER: begin
                    // 格子面积在变换后的两轴各有 CELL_MM 半宽，相切也要检查。
                    cell_inside <= !scan_dir[0] ||
                        (along>=-CELL_MM && along<=along_hi && across>=-across_hi && across<=across_hi);
                    state <= REQUEST;
                end
                REQUEST: begin
                    if (stale_scan) state <= IDLE;
                    else if (!cell_inside) state <= NEXT_CELL;
                    else if (mrd_valid && mrd_ready) state <= RESPONSE;
                end
                RESPONSE: if (mrd_rsp_valid) begin
                    if (stale_scan) state <= IDLE;
                    else if (mrd_raw) begin
                        result_valid <= 1;
                        result_block <= 1;
                        result_dir <= scan_dir;
                        result_count <= {1'b0,scan_count}+17'd1;
                        state <= IDLE;
                    end else state <= NEXT_CELL;
                end
                NEXT_CELL: begin
                    if (stale_scan) state <= IDLE;
                    else if (cx==xhi && cy==yhi) begin
                        result_valid <= 1;
                        result_block <= 0;
                        result_dir <= scan_dir;
                        result_count <= {1'b0,scan_count}+17'd1;
                        state <= IDLE;
                    end else begin
                        if (cx==xhi) begin
                            cx <= xlo;
                            cy <= cy+1'b1;
                        end else cx <= cx+1'b1;
                        state <= CELL_CENTER;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
