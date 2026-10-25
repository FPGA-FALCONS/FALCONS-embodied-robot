// 八方向规划流水：2/3 Dijkstra → 共线段终点 → 三段窗口居中 → 有限间距航点。
// 中继放在居中之后，避免把长段提前截断而失去两端的转角上下文。
module nav_planner #(
    parameter integer GRID_W = 16, GRID_H = 16, QMAX = 2048, W_MAX = 39,
                      BFS_RELAX = 4, CENTER_EN = 1, L_MIN = 4, PROBE_D = 20, PROBE_STEP = 4,
                      CLEAR_TRIGGER = 8, PATH_INC_MAX = 20
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        req_valid,
    input  wire        allow_start_escape,
    output wire        req_ready,
    input  wire [15:0] start_x, start_y, goal_x, goal_y,
    output wire        mrd_valid,
    input  wire        mrd_ready,
    output wire [15:0] mrd_x, mrd_y,
    input  wire        mrd_rsp_valid, mrd_cost, mrd_raw,
    input  wire [15:0] wp_limit,
    output wire        wp_valid,
    input  wire        wp_ready,
    output reg  [15:0] wp_x, wp_y,
    output reg         wp_last,
    output wire        busy,
    output reg         success, no_path,
    output wire [15:0] path_len
);
    localparam [2:0] IDLE = 0, WAIT_POINT = 1, MEASURE = 2, MAKE_POINT = 3, OUTPUT = 4, DRAIN = 5;
    localparam integer GAP = W_MAX-1;
    reg [2:0] state, run_dir;
    reg first_cell, corner_pending, corner_last, collecting_done, read_owner;
    reg [15:0] corner_x, corner_y;
    reg [15:0] cursor_x, cursor_y, end_x, end_y, remaining, count, limit;
    reg signed [1:0] step_x, step_y;
    reg point_last, chunk_last;
    wire search_ready, search_busy, search_success, search_no_path, start_relaxed;
    wire path_valid, path_ready, path_last;
    wire [15:0] path_x, path_y;
    wire [2:0] path_dir;
    wire b_valid, b_ready, c_valid, c_ready;
    wire [15:0] b_x, b_y, c_x, c_y;
    wire center_ready, center_valid, center_last, center_busy;
    wire [15:0] center_x, center_y;
    wire begin_round = req_valid && req_ready && wp_limit != 0;
    wire cancel_center = state == WAIT_POINT && !search_busy && search_no_path;
    assign req_ready = reset_n && state == IDLE && search_ready && !center_busy;
    assign busy = state != IDLE;
    assign wp_valid = reset_n && state == OUTPUT;
    assign path_ready = state != IDLE && !corner_pending && !collecting_done;
    assign mrd_valid = b_valid || c_valid;
    assign mrd_x = c_valid ? c_x : b_x;
    assign mrd_y = c_valid ? c_y : b_y;
    assign b_ready = mrd_ready && !c_valid;
    assign c_ready = mrd_ready;
    always @(posedge clk) if(mrd_valid && mrd_ready) read_owner <= c_valid;
    nav_dijkstra #(.GRID_W(GRID_W), .GRID_H(GRID_H), .QMAX(QMAX), .START_RELAX(BFS_RELAX)) u_search (
        .clk(clk), .reset_n(reset_n), .req_valid(begin_round), .req_ready(search_ready),
        .allow_start_escape(allow_start_escape),
        .start_x(start_x), .start_y(start_y), .goal_x(goal_x), .goal_y(goal_y),
        .mrd_valid(b_valid), .mrd_ready(b_ready), .mrd_x(b_x), .mrd_y(b_y),
        .mrd_rsp_valid(mrd_rsp_valid && !read_owner), .mrd_raw(mrd_raw), .mrd_cost(mrd_cost),
        .path_valid(path_valid), .path_ready(path_ready), .path_x(path_x), .path_y(path_y),
        .path_dir(path_dir), .path_last(path_last), .busy(search_busy),
        .success(search_success), .no_path(search_no_path), .path_len(path_len), .start_relaxed(start_relaxed)
    );
    nav_center #(.GRID_W(GRID_W), .GRID_H(GRID_H), .CENTER_EN(CENTER_EN),
        .L_MIN(L_MIN), .PROBE_D(PROBE_D), .PROBE_STEP(PROBE_STEP),
        .CLEAR_TRIGGER(CLEAR_TRIGGER), .PATH_INC_MAX(PATH_INC_MAX), .BFS_RELAX(BFS_RELAX)) u_center (
        .clk(clk), .reset_n(reset_n), .start(begin_round), .cancel(cancel_center),
        .start_x(start_x), .start_y(start_y), .start_relaxed(start_relaxed),
        .in_valid(corner_pending), .in_ready(center_ready), .in_x(corner_x), .in_y(corner_y), .in_last(corner_last),
        .out_valid(center_valid), .out_ready(state == WAIT_POINT || state == DRAIN),
        .out_x(center_x), .out_y(center_y), .out_last(center_last),
        .mrd_valid(c_valid), .mrd_ready(c_ready), .mrd_x(c_x), .mrd_y(c_y),
        .mrd_rsp_valid(mrd_rsp_valid && read_owner), .mrd_raw(mrd_raw), .mrd_cost(mrd_cost), .busy(center_busy)
    );
    function [15:0] distance;
        input [15:0] x0, y0, x1, y1;
        reg [15:0] dx, dy;
        begin
            dx = x1 >= x0 ? x1-x0 : x0-x1;
            dy = y1 >= y0 ? y1-y0 : y0-y1;
            distance = dx>dy ? dx : dy;
        end
    endfunction
    function [15:0] increment;
        input signed [1:0] direction;
        begin
            case(direction)
                1:increment = GAP;
                -1:increment = -GAP;
                default:increment = 0;
            endcase
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            first_cell <= 1;
            corner_pending <= 0;
            corner_last <= 0;
            collecting_done <= 0;
            corner_x <= 0;
            corner_y <= 0;
            run_dir <= 0;
        end else if(begin_round) begin
            first_cell <= 1;
            corner_pending <= 0;
            collecting_done <= 0;
        end else begin
            if(corner_pending && center_ready) corner_pending <= 0;
            if(path_valid && path_ready) begin
                if(first_cell) begin
                    first_cell <= 0;
                    run_dir <= path_dir;
                    if(path_last) begin
                        corner_x <= path_x;
                        corner_y <= path_y;
                        corner_last <= 1;
                        corner_pending <= 1;
                        collecting_done <= 1;
                    end
                end else if(path_last || path_dir != run_dir) begin
                    corner_x <= path_x;
                    corner_y <= path_y;
                    corner_last <= path_last;
                    corner_pending <= 1;
                    run_dir <= path_dir;
                    if(path_last) collecting_done <= 1;
                end
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state <= IDLE;
            cursor_x <= 0;
            cursor_y <= 0;
            end_x <= 0;
            end_y <= 0;
            remaining <= 0;
            count <= 0;
            limit <= 0;
            step_x <= 0;
            step_y <= 0;
            point_last <= 0;
            chunk_last <= 0;
            wp_x <= 0;
            wp_y <= 0;
            wp_last <= 0;
            success <= 0;
            no_path <= 0;
        end else case(state)
            IDLE:if(begin_round) begin
                cursor_x <= start_x;
                cursor_y <= start_y;
                count <= 0;
                limit <= wp_limit;
                success <= 0;
                no_path <= 0;
                state <= WAIT_POINT;
            end
            WAIT_POINT:if(center_valid) begin
                end_x <= center_x;
                end_y <= center_y;
                point_last <= center_last;
                state <= MEASURE;
            end else if(!search_busy && search_no_path) begin
                no_path <= 1;
                state <= IDLE;
            end
            MEASURE:begin
                step_x <= end_x == cursor_x ? 0 : end_x>cursor_x ? 2'sd1 : -2'sd1;
                step_y <= end_y == cursor_y ? 0 : end_y>cursor_y ? 2'sd1 : -2'sd1;
                remaining <= distance(cursor_x, cursor_y, end_x, end_y);
                state <= MAKE_POINT;
            end
            MAKE_POINT:begin
                chunk_last <= (remaining <= GAP);
                wp_x <= remaining>GAP ? cursor_x+increment(step_x) : end_x;
                wp_y <= remaining>GAP ? cursor_y+increment(step_y) : end_y;
                wp_last <= (point_last && remaining <= GAP) || count+16'd1 >= limit;
                state <= OUTPUT;
            end
            OUTPUT:if(wp_ready) begin
                count <= count+1'b1;
                cursor_x <= wp_x;
                cursor_y <= wp_y;
                if(wp_last) state <= DRAIN;
                else if(chunk_last) state <= WAIT_POINT;
                else begin
                    remaining <= remaining-GAP;
                    state <= MAKE_POINT;
                end
            end
            DRAIN:if(!search_busy && !center_busy && !corner_pending) begin
                success <= search_success;
                no_path <= search_no_path;
                state <= IDLE;
            end
            default:state <= IDLE;
        endcase
    end
endmodule
