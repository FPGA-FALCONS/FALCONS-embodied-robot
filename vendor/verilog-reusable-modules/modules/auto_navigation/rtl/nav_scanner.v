// ============================================================================
// 模块：nav_scanner
// ----------------------------------------------------------------------------
// 职责：
//   - 从采样毫米原点沿 Q16 实际射线用 DDA 逐格行走，途经格写空闲，真实命中端点写占用。
//   - 无回波或图外端点仍清理地图内射线；图外停止，不因遮挡或漏点清除未经过格。
//   - 只生成 raw 写事务，不负责膨胀、采样坐标变换或雷达协议。
// 接口约定：
//   - obs_valid && obs_ready 锁存整份观测；只有空闲时 ready，在途期间载荷由寄存器保存。
//   - origin_* 为 signed mm，ray_* 为 signed Q16 mm 位移，grid_* 为 signed 端点格。
//   - wr_valid 等待 wr_ready，保持格号和值；map_enable=0 时新观测接收即丢弃。
//   - map_dropped 是禁用时接收并丢弃的观测数，16 位回绕，不是 UART 溢出计数。
// 关键不变量：
//   - 按原点格内位置及真实射线求格界，交叉相乘避免除法；恰过格角同时推进两轴。
//   - 端点格只用于终止和命中，不能反推射线方向。
// 禁止事项：
//   - 不要按整圈未出现的障碍清图；不要把无回波端点写成 10 m 障碍。
// ============================================================================
module nav_scanner #(
    parameter integer GRID_W  = 256,
                      GRID_H  = 256,
                      CELL_MM = 50,
    parameter signed  HALF_MM = 6400
) (
    input  wire               clk,
                              reset_n,
    input  wire               obs_valid,
    output wire               obs_ready,
    input  wire signed [15:0] grid_x,
                              grid_y,
    input  wire               grid_valid,
    input  wire signed [31:0] origin_x_mm,
                              origin_y_mm,
    input  wire signed [31:0] ray_dx_q16,
                              ray_dy_q16,
    input  wire               map_enable,
    output wire               wr_valid,
    input  wire               wr_ready,
    output reg                wr_val,
    output reg         [15:0] wr_x,
                              wr_y,
    output wire               busy,
    output reg         [15:0] accepted,
                              dropped,
                              map_dropped
);
    localparam integer GW = $clog2(CELL_MM + 1), PW = 32 + GW;
    localparam [2:0] IDLE     = 0,
                     GAPS     = 1,
                     PRODUCTS = 2,
                     START    = 3,
                     CHECK    = 4,
                     FREE     = 5,
                     HIT      = 6;
    reg [2:0] state;
    reg [15:0] cx, cy, hit_x, hit_y;
    reg [31:0] offset_x, offset_y, dx, dy;
    reg sx_neg, sy_neg, no_obstacle, wr_req;
    reg [GW-1:0] gap_x, gap_y;
    reg [PW-1:0] cross_x, cross_y, step_x, step_y;
    reg signed [PW:0] error, step_both;
    function [15:0] to_cell;
        input signed [31:0] mm;
        reg signed [31:0] shifted;
        reg        [31:0] product;
        begin
            shifted = mm + HALF_MM;
            product = shifted * ((32'd4194304 + CELL_MM - 1) / CELL_MM);
            to_cell = shifted < 0 ? 16'd0 : product[31:22];
        end
    endfunction
    wire [31:0] boundary_x = cx * CELL_MM, boundary_y = cy * CELL_MM;
    wire        move_x = dy == 0 || (dx != 0 && error <= 0);
    wire        move_y = dx == 0 || (dy != 0 && error >= 0);
    wire        taken = map_enable ? wr_ready : 1'b1;
    assign obs_ready = reset_n && state == IDLE;
    assign busy      = reset_n && state != IDLE;
    assign wr_valid  = wr_req && map_enable;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= IDLE;
            cx          <= 0;
            cy          <= 0;
            hit_x       <= 0;
            hit_y       <= 0;
            offset_x    <= 0;
            offset_y    <= 0;
            dx          <= 0;
            dy          <= 0;
            sx_neg      <= 0;
            sy_neg      <= 0;
            no_obstacle <= 0;
            gap_x       <= 0;
            gap_y       <= 0;
            cross_x     <= 0;
            cross_y     <= 0;
            step_x      <= 0;
            step_y      <= 0;
            error       <= 0;
            step_both   <= 0;
            wr_req      <= 0;
            wr_val      <= 0;
            wr_x        <= 0;
            wr_y        <= 0;
            accepted    <= 0;
            dropped     <= 0;
            map_dropped <= 0;
        end else begin
            wr_req <= 0;
            case (state)
                IDLE:
                if (obs_valid && obs_ready) begin
                    hit_x       <= grid_x;
                    hit_y       <= grid_y;
                    no_obstacle <= !grid_valid;
                    cx          <= to_cell(origin_x_mm);
                    cy          <= to_cell(origin_y_mm);
                    offset_x    <= origin_x_mm + HALF_MM;
                    offset_y    <= origin_y_mm + HALF_MM;
                    dx          <= ray_dx_q16 < 0 ? -ray_dx_q16 : ray_dx_q16;
                    dy          <= ray_dy_q16 < 0 ? -ray_dy_q16 : ray_dy_q16;
                    sx_neg      <= ray_dx_q16 < 0;
                    sy_neg      <= ray_dy_q16 < 0;
                    accepted    <= accepted + 1'b1;
                    if (!grid_valid) dropped <= dropped + 1'b1;
                    if (!map_enable) begin
                        map_dropped <= map_dropped + 1'b1;
                        state       <= IDLE;
                    end else state <= GAPS;
                end
                GAPS: begin
                    gap_x <= sx_neg ? offset_x - boundary_x : boundary_x + CELL_MM - offset_x;
                    gap_y <= sy_neg ? offset_y - boundary_y : boundary_y + CELL_MM - offset_y;
                    state <= PRODUCTS;
                end
                PRODUCTS: begin
                    cross_x <= gap_x * dy;
                    cross_y <= gap_y * dx;
                    step_x  <= CELL_MM * dy;
                    step_y  <= CELL_MM * dx;
                    state   <= START;
                end
                START: begin
                    error     <= $signed({1'b0, cross_x}) - $signed({1'b0, cross_y});
                    step_both <= $signed({1'b0, step_x}) - $signed({1'b0, step_y});
                    state     <= CHECK;
                end
                CHECK:
                if (!map_enable || cx >= GRID_W || cy >= GRID_H) state <= IDLE;
                else begin
                    wr_x   <= cx;
                    wr_y   <= cy;
                    wr_req <= 1;
                    wr_val <= cx == hit_x && cy == hit_y && !no_obstacle;
                    state  <= cx == hit_x && cy == hit_y ? HIT : FREE;
                end
                FREE:
                if (taken) begin
                    if (move_x) cx <= sx_neg ? cx - 16'd1 : cx + 16'd1;
                    if (move_y) cy <= sy_neg ? cy - 16'd1 : cy + 16'd1;
                    if (move_x && move_y) error <= error + step_both;
                    else if (move_x) error <= error + $signed({1'b0, step_x});
                    else error <= error - $signed({1'b0, step_y});
                    state <= CHECK;
                end else wr_req <= 1;
                HIT:     if (taken) state <= IDLE;
 else wr_req <= 1;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
