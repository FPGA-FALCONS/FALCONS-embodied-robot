// ============================================================================
// 模块：nav_top
// ----------------------------------------------------------------------------
// 职责：
//   - 协调建图、地图读仲裁、候选验证、活动路线及命令执行器。
//   - 每 100 ms 尝试后台规划；活动路线独立供控制和显示读取，计算不要求停车。
//   - cost 用于规划与完整候选复核，raw 仅由独立前探限制直行。
// 接口约定：
//   - 内部单时钟 50 MHz；23 位局部计数器控制规划和显示刷新。
//   - 位姿用 signed mm / 逆时针 0.01°；goal_x/y 与观测命中坐标用格号。
//   - 观测 valid/ready 成交才更新地图；map_enable=0 接收但丢弃，不能据此认为地图新鲜。
//   - 地图和路线独立输出；display_enable 同时控制两路有效数据和帧边界。
// 关键不变量：
//   - 候选 begin/abort/失败不修改活动 bank；固定起点的全部线段通过后才原子发布。
//   - 换目标立即撤销旧目标路线的运动资格；切换模式不自动清图或重建原点。
//   - 发布状态、RAM 读取、毫米换算、位姿相减分拍；描述符失效时已有动作保持。
// 禁止事项：
//   - 不要将 publish_ok 组合反馈到 descriptor_valid；这会闭合地图判断到执行器的关键路径。
//   - 不要让显示反压建图；不要等待 motion_stopped 才允许开始普通规划。
// ============================================================================
module nav_top #(
    parameter integer        PLAN_INFLATE_CELLS  = 4,
    parameter integer        GRID_W              = 256,
    parameter integer        GRID_H              = 256,
    parameter integer        CELL_MM             = 50,
    parameter signed         HALF_MM             = 6400,
    parameter integer        MAX_WP              = 1024,
    parameter         [22:0] REPLAN_PERIOD       = 23'd5_000_000,  // 50 MHz / 100 ms
    parameter integer        W_MAX               = 39
) (
    input  wire               clk,
    input  wire               reset_n,

    // 运行时前探配置；每次扫描开始锁存。距离从定位中心点起算。
    input  wire         [7:0] forward_check_cells,
    input  wire        [15:0] vehicle_width_mm,

    input  wire               pose_valid,
    input  wire signed [31:0] pose_x_mm,
                              pose_y_mm,

    input  wire               goal_valid,
    input  wire        [15:0] goal_x,
                              goal_y,

    input  wire               plan_enable,
    input  wire               motion_enable,
    input  wire               display_enable,
    input  wire               map_enable,

    input  wire               ctrl_done,
    // 1：即将或正在直行；0：原地转向或等待，不产生平移。
    input  wire               ctrl_linear_active,

    input  wire               obs_valid,
    input  wire               obs_scan_start,
    output wire               obs_ready,
    input  wire signed [15:0] obs_grid_x,
                              obs_grid_y,
    input  wire               obs_grid_valid,
    input  wire signed [31:0] obs_origin_x_mm,
                              obs_origin_y_mm,
    input  wire signed [31:0] obs_ray_dx_q16,
                              obs_ray_dy_q16,
    input  wire        [13:0] obs_range_mm,

    input  wire               scan_valid,
    input  wire               stream_error,

    output wire               cmd_valid,
                              emergency_stop,
    output wire signed [31:0] target_x_mm,
                              target_y_mm,
    // 显示当前任务线型：0 水平、1 正45°、2 竖直、3 负45°，仅 cmd_valid 有效时使用。
    output wire         [1:0] target_line_dir,

    // 正式任务接口：供整车界面读取。
    // 正式任务状态：0关闭/1运行中/2暂停/3已到达/4无路。
    output wire         [3:0] nav_state,

    // 诊断输出：可不连接，不作为任务完成或停车确认依据。
    output wire               st_sensors_valid,
    output wire               st_brake_clear,
    output wire         [1:0] st_stop_cause,
    output wire               st_at_target,
    output wire               st_segment_blocked,
    output wire               st_commit_valid,
    output wire               st_plan_busy,
    output wire        [15:0] st_target_index,
    output wire        [15:0] st_route_length,
    output wire        [15:0] st_route_id,
    output wire        [15:0] st_map_version,
    output wire        [15:0] st_map_dropped,
    output wire        [31:0] st_plan_rounds,
    output wire        [31:0] st_occupied_cells,

    // 地图按 x 递增、再 y 递增扫描；首尾均与有效格同拍。
    output wire               map_disp_valid,
    output wire               map_disp_occupied,
    output wire               map_disp_frame_start,
    output wire               map_disp_frame_done,
    // 路线按航点顺序输出；空路线 start/done 同拍、valid 为 0。
    output wire               route_disp_valid,
    output wire         [7:0] route_disp_x_cell,
    output wire         [7:0] route_disp_y_cell,
    output wire               route_disp_frame_start,
    output wire               route_disp_frame_done
);
    wire publish_ok;
    wire arb_rsp_cost, arb_rsp_raw;
    wire map_wr_valid, map_wr_ready, map_wr_val;
    wire [15:0] map_wr_x, map_wr_y;
    wire map_rd_valid, map_rd_ready;
    wire [15:0] map_rd_x, map_rd_y;
    wire map_rd_rsp_valid, map_rd_raw, map_rd_cost;
    wire map_idle;
    wire scan_wr_valid, scan_wr_ready, scan_wr_val, scan_wr_shadow;
    wire [15:0] scan_wr_x, scan_wr_y;
    wire scanner_ready, circle_ready;
    wire observation_enabled = !obs_scan_start || circle_ready;
    assign obs_ready = scanner_ready && observation_enabled;

    nav_scanner #(
        .GRID_W (GRID_W),
        .GRID_H (GRID_H),
        .CELL_MM(CELL_MM),
        .HALF_MM(HALF_MM)
    ) u_scan (
        .clk        (clk),
        .reset_n    (reset_n),
        .obs_valid  (obs_valid && observation_enabled),
        .obs_ready  (scanner_ready),
        .grid_x     (obs_grid_x),
        .grid_y     (obs_grid_y),
        .grid_valid (obs_grid_valid),
        .origin_x_mm(obs_origin_x_mm),
        .origin_y_mm(obs_origin_y_mm),
        .ray_dx_q16 (obs_ray_dx_q16),
        .ray_dy_q16 (obs_ray_dy_q16),
        .range_mm   (obs_range_mm),
        .map_enable (map_enable),
        .wr_valid   (scan_wr_valid),
        .wr_ready   (scan_wr_ready),
        .wr_val     (scan_wr_val),
        .wr_shadow  (scan_wr_shadow),
        .wr_x       (scan_wr_x),
        .wr_y       (scan_wr_y),
        .busy       (),
        .accepted   (),
        .dropped    (),
        .map_dropped(st_map_dropped)
    );

    nav_scan_accumulator #(
        .GRID_W(GRID_W),
        .GRID_H(GRID_H)
    ) u_scan_accumulator (
        .clk       (clk),
        .reset_n   (reset_n),
        .scan_begin(obs_valid && obs_ready && obs_scan_start),
        .scan_ready(circle_ready),
        .in_valid  (scan_wr_valid),
        .in_ready  (scan_wr_ready),
        .in_value  (scan_wr_val),
        .in_shadow (scan_wr_shadow),
        .in_x      (scan_wr_x),
        .in_y      (scan_wr_y),
        .wr_valid  (map_wr_valid),
        .wr_ready  (map_wr_ready),
        .wr_value  (map_wr_val),
        .wr_x      (map_wr_x),
        .wr_y      (map_wr_y)
    );

    nav_map #(
        .GRID_W (GRID_W),
        .GRID_H (GRID_H),
        .INFLATE(PLAN_INFLATE_CELLS)
    ) u_map (
        .clk           (clk),
        .reset_n       (reset_n),
        .wr_valid      (map_wr_valid),
        .wr_ready      (map_wr_ready),
        .wr_val        (map_wr_val),
        .wr_x          (map_wr_x),
        .wr_y          (map_wr_y),
        .rd_valid      (map_rd_valid),
        .rd_ready      (map_rd_ready),
        .rd_x          (map_rd_x),
        .rd_y          (map_rd_y),
        .rd_rsp_valid  (map_rd_rsp_valid),
        .rd_raw        (map_rd_raw),
        .rd_cost       (map_rd_cost),
        .idle          (map_idle),
        .occupied_cells(st_occupied_cells),
        .map_version   (st_map_version)
    );

    reg [22:0] replan_count;
    reg        replan_due;
    reg  [31:0] round_count;
    reg [15:0] round_goal_x, round_goal_y;
    reg [15:0] active_goal_x, active_goal_y;
    wire active_goal_match = active_goal_x == goal_x && active_goal_y == goal_y;

    wire driving = plan_enable && goal_valid && st_sensors_valid;

    wire load_valid, load_ready, load_last;
    wire segment_load_valid, segment_load_ready, segment_load_last, segments_busy;
    wire [15:0] segment_load_x, segment_load_y;
    wire [2:0] segment_load_dir;
    wire [15:0] load_x, load_y;

    function [15:0] to_cell;
        input signed [31:0] mm;
        reg signed [31:0] shift;
        reg        [31:0] prod;
        begin
            shift = mm + HALF_MM;
            if (shift < 0) to_cell = 16'd0;
            else begin
                prod    = shift * ((32'd4194304 + CELL_MM - 1) / CELL_MM);
                to_cell = prod[31:22];
            end
        end
    endfunction
    localparam [3:0] R_IDLE     = 0,
                     R_BEGIN    = 1,
                     R_REQ      = 2,
                     R_LOAD     = 3,
                     R_FETCH    = 4,
                     R_WAIT     = 5,
                     R_SEG      = 6,
                     R_CHECK    = 7,
                     R_RESPONSE = 8,
                     R_NEXT     = 9,
                     R_PUB      = 10,
                     R_ABORT    = 11;
    reg [3:0] rstate;
    reg commit_valid, candidate_commit, load_inject;
    reg [15:0] plan_x, plan_y;
    reg [2:0] plan_dir;
    wire inject_prefix = candidate_commit && !(plan_x == round_goal_x && plan_y == round_goal_y);
    reg [15:0] candidate_index;
    wire candidate_complete, candidate_valid;
    wire [15:0] candidate_length, candidate_x, candidate_y;
    reg [15:0] vx, vy, entry_x, entry_y;
    reg signed [31:0] start_x_mm, start_y_mm;
    // 保留防立即折返约束，只使用锁定方向和计划点，不读取实际位姿。
    wire [2:0] candidate_direction = candidate_y==plan_y ? (candidate_x>=plan_x ? 3'd0 : 3'd4) :
        candidate_x==plan_x ? (candidate_y>=plan_y ? 3'd2 : 3'd6) :
        candidate_x>plan_x ? (candidate_y>plan_y ? 3'd1 : 3'd7) :
                            (candidate_y>plan_y ? 3'd3 : 3'd5);
    reg [2:0] candidate_dir_q;
    always @(posedge clk) candidate_dir_q <= candidate_direction;
    wire reverse_commit = candidate_commit && candidate_index==1 &&
        (candidate_x!=plan_x || candidate_y!=plan_y) && candidate_dir_q==(plan_dir^3'd4);
    wire line_valid, line_last, line_invalid;
    wire [15:0] line_x, line_y;
    reg [15:0] box_xlo, box_xhi, box_ylo, box_yhi;
    reg local_cost, first_read, validation_dirty;
    reg [15:0] update_x, update_y, seen_version;
    reg [1:0] fetch_wait;
    wire verify_active = rstate >= R_FETCH && rstate <= R_PUB;
    wire update_near =
        $signed({1'b0, update_x}) >= $signed({1'b0, box_xlo}) - PLAN_INFLATE_CELLS &&
        $signed({1'b0, update_x}) <= $signed({1'b0, box_xhi}) + PLAN_INFLATE_CELLS &&
        $signed({1'b0, update_y}) >= $signed({1'b0, box_ylo}) - PLAN_INFLATE_CELLS &&
        $signed({1'b0, update_y}) <= $signed({1'b0, box_yhi}) + PLAN_INFLATE_CELLS;
    wire version_changed = seen_version != st_map_version;
    wire own_read_valid = rstate == R_CHECK && line_valid;
    wire own_read_ready, own_rsp;
    wire bfs_read_valid;
    wire [15:0] bfs_read_x, bfs_read_y;
    wire planner_read = rstate == R_LOAD || rstate == R_REQ;
    function near_entry;
        input [15:0] x, y;
        reg signed [16:0] dx, dy;
        begin
            dx         = $signed({1'b0, x}) - $signed({1'b0, entry_x});
            dy         = $signed({1'b0, y}) - $signed({1'b0, entry_y});
            near_entry = dx >= -PLAN_INFLATE_CELLS && dx <= PLAN_INFLATE_CELLS &&
                         dy >= -PLAN_INFLATE_CELLS && dy <= PLAN_INFLATE_CELLS;
        end
    endfunction
    wire checked_free = !arb_rsp_raw && (!arb_rsp_cost ||
        (!candidate_commit && (first_read || (local_cost && near_entry(line_x, line_y)))));
    wire line_advance = rstate==R_RESPONSE && own_rsp && checked_free && !first_read;
    nav_line_cells u_verify_line (
        .clk(clk), .reset_n(reset_n), .start(rstate==R_SEG), .cancel(rstate==R_ABORT),
        .x0(vx), .y0(vy), .x1(candidate_x), .y1(candidate_y),
        .valid(line_valid), .ready(line_advance), .x(line_x), .y(line_y),
        .last(line_last), .invalid(line_invalid)
    );
    wire pl_req_valid, pl_req_ready;
    wire pl_wp_valid, pl_wp_ready, pl_wp_last;
    wire [15:0] pl_wp_x, pl_wp_y;
    wire pl_busy, pl_success, pl_no_path;
    wire [15:0] pl_path_len;

    wire pl_mrd_valid, pl_mrd_ready;
    wire [15:0] pl_mrd_x, pl_mrd_y;
    wire pl_mrd_rsp_valid, pl_mrd_cost, pl_mrd_raw;

    wire dv_mrd_valid, dv_mrd_ready;
    wire [15:0] dv_mrd_x, dv_mrd_y;
    wire dv_mrd_rsp_valid, dv_mrd_raw;

    wire plan_owns_read = (rstate != R_IDLE);

    reg [15:0] sweep_x, sweep_y;
    reg        disp_active, disp_pending;
    reg [22:0] display_count;
    reg display_started, display_refresh;
    reg        disp_mrd_valid;
    wire       disp_mrd_ready, disp_mrd_rsp_valid, disp_mrd_raw;
    wire [15:0] sweep_addr_x = sweep_x;
    wire [15:0] sweep_addr_y = sweep_y;
    wire disp_first = sweep_x == 0 && sweep_y == 0;
    wire disp_last = sweep_x == GRID_W - 1 && sweep_y == GRID_H - 1;

    wire       arb_pl_en = plan_owns_read;
    wire       arb_dv_en = 1'b1;
    wire       arb_dp_en = display_enable && disp_active;

    wire [2:0] arb_req = {disp_mrd_valid, pl_mrd_valid, dv_mrd_valid};
    wire [2:0] arb_en = {arb_dp_en, arb_pl_en, arb_dv_en};
    wire [2:0] arb_gnt, arb_rsp_valid;
    wire [47:0] arb_req_x = {sweep_addr_x, pl_mrd_x, dv_mrd_x};
    wire [47:0] arb_req_y = {sweep_addr_y, pl_mrd_y, dv_mrd_y};

    wire arb_timeout, arb_busy;
    wire rd_free = !arb_busy;

    nav_map_arb #(
        .TMO(50000)
    ) u_arb (
        .clk          (clk),
        .reset_n      (reset_n),
        .req          (arb_req),
        .req_x        (arb_req_x),
        .req_y        (arb_req_y),
        .gnt          (arb_gnt),
        .rsp_valid    (arb_rsp_valid),
        .rsp_cost     (arb_rsp_cost),
        .rsp_raw      (arb_rsp_raw),
        .enable       (arb_en),
        .map_valid    (map_rd_valid),
        .map_x        (map_rd_x),
        .map_y        (map_rd_y),
        .map_ready    (map_rd_ready),
        .map_rsp_valid(map_rd_rsp_valid),
        .map_cost     (map_rd_cost),
        .map_raw      (map_rd_raw),
        .busy_o       (arb_busy),
        .err_timeout  (arb_timeout)
    );


    assign pl_mrd_valid       = planner_read ? bfs_read_valid : own_read_valid;
    // 首次先读实际接入格，不能把坐标排序后的线段端点当成起点。
    assign pl_mrd_x           = planner_read ? bfs_read_x : (first_read ? entry_x : line_x);
    assign pl_mrd_y           = planner_read ? bfs_read_y : (first_read ? entry_y : line_y);
    assign own_read_ready     = arb_gnt[1];
    assign own_rsp            = arb_rsp_valid[1];
    assign pl_mrd_ready       = arb_gnt[1] && planner_read;
    assign dv_mrd_ready       = arb_gnt[0];
    assign disp_mrd_ready     = arb_gnt[2];

    assign pl_mrd_rsp_valid   = arb_rsp_valid[1] && planner_read;
    assign dv_mrd_rsp_valid   = arb_rsp_valid[0];
    assign disp_mrd_rsp_valid = arb_rsp_valid[2];
    assign pl_mrd_cost        = arb_rsp_cost;
    assign pl_mrd_raw         = arb_rsp_raw;
    assign dv_mrd_raw         = arb_rsp_raw;
    assign disp_mrd_raw       = arb_rsp_raw;

    // 周期比较只更新寄存后的刷新脉冲，不直接驱动地图读口。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            display_count   <= 0;
            display_started <= 0;
            display_refresh <= 0;
        end else begin
            display_refresh <= 0;
            if (!display_enable) display_started <= 0;
            else if (!display_started) begin
                display_count   <= 0;
                display_started <= 1;
            end else if (display_count == REPLAN_PERIOD - 23'd1) begin
                display_count   <= 0;
                display_refresh <= 1;
            end else display_count <= display_count + 1'b1;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sweep_x        <= 0;
            sweep_y        <= 0;
            disp_active    <= 0;
            disp_pending   <= 1;
            disp_mrd_valid <= 0;
        end else if (!display_enable) begin
            sweep_x        <= 0;
            sweep_y        <= 0;
            disp_active    <= 0;
            disp_pending   <= 1;
            disp_mrd_valid <= 0;
        end else begin
            if (display_refresh) disp_pending <= 1;
            // 先排空关闭显示前的在途读，旧响应不能成为新帧的首格。
            if (!disp_active && (disp_pending || display_refresh) && rd_free) begin
                disp_active  <= 1;
                disp_pending <= 0;
                sweep_x      <= 0;
                sweep_y      <= 0;
            end
            if (disp_mrd_valid && disp_mrd_ready) disp_mrd_valid <= 0;
            else if (!disp_mrd_valid && disp_active && rd_free) disp_mrd_valid <= 1;
            if (disp_mrd_rsp_valid && disp_active) begin
                if (disp_last) begin
                    disp_active <= 0;
                    sweep_x     <= 0;
                    sweep_y     <= 0;
                end else if (sweep_x == GRID_W - 1) begin
                    sweep_x <= 0;
                    sweep_y <= sweep_y + 1'b1;
                end else sweep_x <= sweep_x + 1'b1;
            end
        end
    end

    assign pl_req_valid = (rstate == R_REQ);
    assign pl_wp_ready  = (rstate == R_LOAD) && load_ready && !load_inject;

    nav_planner #(
        .GRID_W   (GRID_W),
        .GRID_H   (GRID_H),
        .W_MAX    (W_MAX),
        .BFS_RELAX(PLAN_INFLATE_CELLS)
    ) u_plan (
        .clk          (clk),
        .reset_n      (reset_n),
        .wp_limit     (MAX_WP[15:0] - (inject_prefix ? 16'd1 : 16'd0)),
        .req_valid    (pl_req_valid),
        .req_ready    (pl_req_ready),
        .allow_start_escape(!candidate_commit),
        .start_x      (plan_x),
        .start_y      (plan_y),
        .goal_x       (round_goal_x),
        .goal_y       (round_goal_y),
        .mrd_valid    (bfs_read_valid),
        .mrd_ready    (pl_mrd_ready),
        .mrd_x        (bfs_read_x),
        .mrd_y        (bfs_read_y),
        .mrd_rsp_valid(pl_mrd_rsp_valid),
        .mrd_cost     (pl_mrd_cost),
        .mrd_raw      (pl_mrd_raw),
        .wp_valid     (pl_wp_valid),
        .wp_ready     (pl_wp_ready),
        .wp_x         (pl_wp_x),
        .wp_y         (pl_wp_y),
        .wp_last      (pl_wp_last),
        .busy         (pl_busy),
        .success      (pl_success),
        .no_path      (pl_no_path),
        .path_len     (pl_path_len)
    );

    assign load_valid = (rstate == R_LOAD) && (load_inject ? 1'b1 : pl_wp_valid);
    assign load_x     = load_inject ? plan_x : pl_wp_x;
    assign load_y     = load_inject ? plan_y : pl_wp_y;
    assign load_last  = load_inject ? 1'b0 : pl_wp_last;

    wire active_valid;
    wire [15:0] r_target_x, r_target_y;
    wire [15:0] drive_x, drive_y, drive_index;
    wire final_target;
    reg  [15:0] target_index;
    reg  [ 2:0] target_wait;
    wire [ 2:0] r_target_dir;
    wire advance, motion_stopped, arrived;
    wire [15:0] advance_index;
    wire publish_route = rstate == R_PUB && publish_ok;
    reg route_ready, round_motion, no_path_r;
    wire route_for_goal = active_valid && active_goal_match && route_ready;
    // 发布整拍屏蔽新描述符，已有目标点任务继续保持；不要组合依赖 publish_ok。
    wire descriptor_valid = route_for_goal && target_wait == 0 && rstate != R_PUB && !advance;
    reg signed [31:0] segment_x_mm  /* synthesis syn_dspstyle = "logic" */;
    reg signed [31:0] segment_y_mm  /* synthesis syn_dspstyle = "logic" */;
    // 发布后的描述符刷新期间，旧输出只能确认新旧路线共同保留的锁定段。
    wire [15:0] completed_index = target_x_mm == segment_x_mm && target_y_mm == segment_y_mm ?
        advance_index : target_index;
    always @(posedge clk) begin
        segment_x_mm <= drive_x * CELL_MM - HALF_MM + CELL_MM / 2;
        segment_y_mm <= drive_y * CELL_MM - HALF_MM + CELL_MM / 2;
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            target_index <= 0;
            target_wait  <= 4;
        end else if (publish_route || !active_valid) begin
            target_index <= 0;
            target_wait  <= 4;
        end else if (advance) begin
            if (completed_index + 16'd1 < st_route_length) target_index <= completed_index + 16'd1;
            target_wait <= 4;
        end else if (target_wait != 0) target_wait <= target_wait - 1'b1;
    end

    nav_segments u_segments (
        .clk            (clk),
        .reset_n        (reset_n),
        .start          (rstate == R_REQ && pl_req_ready),
        .cancel         (rstate == R_ABORT),
        .start_x        (plan_x),
        .start_y        (plan_y),
        .start_dir_valid(candidate_commit),
        .start_dir      (plan_dir),
        .in_valid       (load_valid),
        .in_ready       (load_ready),
        .in_x           (load_x),
        .in_y           (load_y),
        .in_last        (load_last),
        .out_valid      (segment_load_valid),
        .out_ready      (segment_load_ready),
        .out_x          (segment_load_x),
        .out_y          (segment_load_y),
        .out_last       (segment_load_last),
        .out_dir        (segment_load_dir),
        .busy           (segments_busy)
    );
    wire at_target, segment_blocked, forward_clear;
    wire route_revoked = !motion_enable || !goal_valid || !st_sensors_valid ||
        (route_ready && !active_goal_match) || (emergency_stop && !motion_stopped) ||
        (advance && completed_index + 16'd1 >= st_route_length);
    always @(posedge clk or negedge reset_n)
        if (!reset_n) route_ready <= 0;
        else if (route_revoked) route_ready <= 0;
        else if (publish_route) route_ready <= round_motion;

    nav_route #(
        .MAX_WP(MAX_WP)
    ) u_route (
        .clk               (clk),
        .reset_n           (reset_n),
        .load_begin        (rstate == R_BEGIN),
        .load_abort        (rstate == R_ABORT),
        .candidate_complete(candidate_complete),
        .candidate_length  (candidate_length),
        .candidate_index   (candidate_index),
        .candidate_x       (candidate_x),
        .candidate_y       (candidate_y),
        .candidate_valid   (candidate_valid),
        .load_valid        (segment_load_valid),
        .load_ready        (segment_load_ready),
        .load_x            (segment_load_x),
        .load_y            (segment_load_y),
        .load_last         (segment_load_last),
        .load_dir          (segment_load_dir),
        .publish           (rstate == R_PUB && publish_ok),
        .active_valid      (active_valid),
        .active_length     (st_route_length),
        .route_id          (st_route_id),
        .target_index      (target_index),
        .target_x          (r_target_x),
        .target_y          (r_target_y),
        .target_dir        (r_target_dir),
        .final_target      (final_target),
        .drive_x           (drive_x),
        .drive_y           (drive_y),
        .drive_index       (drive_index),
        .load_full         (),
        .display_enable         (display_enable),
        .display_refresh        (display_refresh),
        .display_route_valid    (goal_valid && active_goal_match &&
                                 !(no_path_r && !route_for_goal)),
        .route_disp_valid       (route_disp_valid),
        .route_disp_x_cell      (route_disp_x_cell),
        .route_disp_y_cell      (route_disp_y_cell),
        .route_disp_frame_start (route_disp_frame_start),
        .route_disp_frame_done  (route_disp_frame_done)
    );

    assign map_disp_valid = reset_n && display_enable && disp_active && disp_mrd_rsp_valid;
    assign map_disp_occupied = disp_mrd_raw;
    assign map_disp_frame_start = map_disp_valid && disp_first;
    assign map_disp_frame_done = map_disp_valid && disp_last;

    nav_drive #(
        .GRID_W             (GRID_W),
        .GRID_H             (GRID_H),
        .CELL_MM            (CELL_MM),
        .HALF_MM            (HALF_MM)
    ) u_drive (
        .clk            (clk),
        .reset_n        (reset_n),
        .forward_check_cells (forward_check_cells),
        .vehicle_width_mm    (vehicle_width_mm),
        .pose_x_mm      (pose_x_mm),
        .pose_y_mm      (pose_y_mm),
        .pose_valid     (pose_valid),
        .heading        (r_target_dir),
        .check_enable   (motion_enable && st_sensors_valid && active_valid),
        .mrd_valid      (dv_mrd_valid),
        .mrd_ready      (dv_mrd_ready),
        .mrd_x          (dv_mrd_x),
        .mrd_y          (dv_mrd_y),
        .mrd_rsp_valid  (dv_mrd_rsp_valid),
        .mrd_raw        (dv_mrd_raw),
        .segment_blocked(segment_blocked),
        .forward_clear  (forward_clear),
        .busy           ()
    );

    wire [1:0] health_cause;
    nav_safety #(
        .MAP_HALF(HALF_MM)
    ) u_safety (
        .reset_n                (reset_n),
        .pose_valid             (pose_valid),
        .pose_x_mm              (pose_x_mm),
        .pose_y_mm              (pose_y_mm),
        .scan_valid             (scan_valid),
        .stream_error           (stream_error),
        .sensors_valid          (st_sensors_valid),
        .brake_clear            (st_brake_clear),
        .stop_cause             (health_cause)
    );


    assign st_plan_busy       = rstate != R_IDLE;
    assign st_at_target       = at_target;
    assign st_segment_blocked = segment_blocked;
    nav_command u_command (
        .clk            (clk),
        .reset_n        (reset_n),
        .enable         (motion_enable && goal_valid),
        .sensors_valid  (st_sensors_valid),
        .route_valid    (route_for_goal),
        .segment_valid  (descriptor_valid),
        .segment_last   (final_target),
        .segment_final  (final_target && drive_x == goal_x && drive_y == goal_y),
        .segment_dir    (r_target_dir),
        .segment_x_mm   (segment_x_mm),
        .segment_y_mm   (segment_y_mm),
        .segment_index  (drive_index),
        .forward_clear  (forward_clear),
        .ctrl_done      (ctrl_done),
        .ctrl_linear_active     (ctrl_linear_active),
        .cmd_valid      (cmd_valid),
        .target_x_mm    (target_x_mm),
        .target_y_mm    (target_y_mm),
        .target_line_dir  (target_line_dir),
        .emergency_stop (emergency_stop),
        .advance        (advance),
        .advance_index  (advance_index),
        .arrived        (arrived),
        .at_target      (at_target),
        .stopped        (motion_stopped)
    );
    // 任务状态只作整车提示；停车、规划和转向均属于运行中。
    assign nav_state = !reset_n || !goal_valid     ? 4'd0 :
                       !motion_enable              ? 4'd2 :
                       arrived                     ? 4'd3 :
                       no_path_r && !route_for_goal ? 4'd4 : 4'd1;

    assign st_stop_cause=health_cause!=0?health_cause:
        ((cmd_valid&&ctrl_linear_active&&!forward_clear)||(emergency_stop&&segment_blocked))?2'd2:2'd0;
    assign st_commit_valid = commit_valid;
    assign st_target_index = target_index;
    assign st_plan_rounds = round_count;

    wire normal_origin = route_for_goal && motion_enable && !emergency_stop;
    wire normal_complete = cmd_valid && ctrl_done;
    wire round_canceled = !goal_valid || goal_x != round_goal_x || goal_y != round_goal_y ||
        !st_sensors_valid || motion_enable != round_motion || normal_complete ||
        (cmd_valid && ctrl_linear_active && !forward_clear) ||
        (candidate_commit && (!normal_origin || r_target_x != plan_x || r_target_y != plan_y ||
                              r_target_dir != plan_dir));

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rstate           <= R_IDLE;
            replan_count     <= 0;
            replan_due       <= 1;
            round_count      <= 0;
            no_path_r        <= 0;
            round_goal_x     <= 0;
            round_goal_y     <= 0;
            active_goal_x    <= 0;
            active_goal_y    <= 0;
            commit_valid     <= 0;
            candidate_commit <= 0;
            load_inject      <= 0;
            plan_x           <= 0;
            plan_y           <= 0;
            candidate_index  <= 0;
            plan_dir         <= 0;
            round_motion     <= 0;

            vx               <= 0;
            vy               <= 0;


            entry_x          <= 0;
            entry_y          <= 0;
            start_x_mm       <= 0;
            start_y_mm       <= 0;
            box_xlo          <= 0;
            box_xhi          <= 0;
            box_ylo          <= 0;
            box_yhi          <= 0;
            local_cost       <= 0;
            first_read       <= 0;
            validation_dirty <= 0;
            update_x         <= 0;
            update_y         <= 0;
            seen_version     <= 0;
            fetch_wait       <= 0;
        end else begin
            if (!replan_due) begin
                if (replan_count == REPLAN_PERIOD - 23'd2) replan_due <= 1;
                else replan_count <= replan_count + 1'b1;
            end
            if (map_wr_valid && map_wr_ready) begin
                update_x <= map_wr_x;
                update_y <= map_wr_y;
            end
            seen_version <= st_map_version;
            if (verify_active && version_changed && update_near) validation_dirty <= 1;
            if (rstate != R_IDLE && round_canceled) validation_dirty <= 1;
            if (!goal_valid || goal_x != round_goal_x || goal_y != round_goal_y ||
                (route_ready && route_revoked)) no_path_r <= 0;
            if (!goal_valid || goal_x != round_goal_x || goal_y != round_goal_y ||
                motion_enable != round_motion || (route_ready && route_revoked)) replan_due <= 1;
            case (rstate)
                R_IDLE:
                if (driving && !arrived && !normal_complete &&
                    !(route_ready && route_revoked) &&
                    (!motion_enable || motion_stopped || normal_origin) &&
                    (!normal_origin || (target_wait == 0 && MAX_WP > 1)) &&
                    replan_due) begin
                    replan_due <= 0;
                    replan_count <= 0;
                    round_count <= round_count + 1'b1;
                    round_goal_x <= goal_x;
                    round_goal_y <= goal_y;
                    candidate_commit <= normal_origin;
                    round_motion <= motion_enable;
                    validation_dirty <= 0;
                    plan_dir <= r_target_dir;
                    if (normal_origin) begin
                        plan_x <= r_target_x;
                        plan_y <= r_target_y;
                    end else begin
                        start_x_mm <= pose_x_mm;
                        start_y_mm <= pose_y_mm;
                    end
                    load_inject <= 0;
                    rstate <= R_BEGIN;
                end
                R_BEGIN: begin
                    if (!candidate_commit) begin
                        plan_x <= to_cell(start_x_mm);
                        plan_y <= to_cell(start_y_mm);
                    end
                    rstate <= R_REQ;
                end
                R_REQ:
                if (pl_req_ready) begin
                    load_inject <= inject_prefix;
                    rstate      <= R_LOAD;
                end
                R_LOAD: begin
                    // 范围到 R_FETCH 后才使用，初始化不依赖本阶段的成功判断。
                    box_xlo <= plan_x;
                    box_xhi <= plan_x;
                    box_ylo <= plan_y;
                    box_yhi <= plan_y;
                    if (load_inject && load_ready) load_inject <= 0;
                    // 失败不会再有末点，直接取消待发前缀；成功才等合并流排空。
                    if(!pl_busy&&!load_inject&&!load_valid&&(!pl_success||!segments_busy)) begin
                        if (validation_dirty || round_canceled) rstate <= R_ABORT;
                        else if (pl_success && candidate_complete) begin
                            entry_x          <= plan_x;
                            entry_y          <= plan_y;
                            vx               <= plan_x;
                            vy               <= plan_y;
                            candidate_index  <= 0;
                            first_read       <= 1;
                            local_cost       <= 0;
                            rstate           <= R_FETCH;
                        end else begin
                            no_path_r <= pl_no_path;
                            rstate    <= R_ABORT;
                        end
                    end
                end
                R_FETCH: begin
                    fetch_wait <= 2;
                    rstate     <= R_WAIT;
                end
                R_WAIT:  if (fetch_wait != 0) fetch_wait <= fetch_wait - 1'b1;
 else rstate <= R_SEG;
                R_SEG: begin
                    if (!candidate_valid || reverse_commit) rstate <= R_ABORT;
                    else begin
                        if (candidate_x < box_xlo) box_xlo <= candidate_x;
                        if (candidate_x > box_xhi) box_xhi <= candidate_x;
                        if (candidate_y < box_ylo) box_ylo <= candidate_y;
                        if (candidate_y > box_yhi) box_yhi <= candidate_y;
                        rstate <= R_CHECK;
                    end
                end
                R_CHECK: if (line_invalid) rstate <= R_ABORT;
                         else if (own_read_ready) rstate <= R_RESPONSE;
                R_RESPONSE:
                if (own_rsp) begin
                    if (first_read) begin
                        local_cost <= arb_rsp_cost;
                        first_read <= 0;
                    end
                    if (!checked_free) rstate <= R_ABORT;
                    else if (first_read) rstate <= R_CHECK;
                    else if (line_last) rstate <= R_NEXT;
                    else rstate <= R_CHECK;
                end
                R_NEXT: begin
                    vx <= candidate_x;
                    vy <= candidate_y;
                    if (candidate_index + 1'b1 >= candidate_length) rstate <= R_PUB;
                    else begin
                        candidate_index <= candidate_index + 1'b1;
                        rstate          <= R_FETCH;
                    end
                end
                R_PUB: begin
                    if (publish_ok) begin
                        active_goal_x <= round_goal_x;
                        active_goal_y <= round_goal_y;
                        commit_valid  <= candidate_commit;
                        no_path_r     <= 0;
                        rstate        <= R_IDLE;
                    end else if (round_canceled || validation_dirty || (version_changed && update_near))
                        rstate <= R_ABORT;
                end
                R_ABORT: rstate <= R_IDLE;
                default: rstate <= R_IDLE;
            endcase
        end
    end
    wire write_near =
        $signed({1'b0, map_wr_x}) >= $signed({1'b0, box_xlo}) - PLAN_INFLATE_CELLS &&
        $signed({1'b0, map_wr_x}) <= $signed({1'b0, box_xhi}) + PLAN_INFLATE_CELLS &&
        $signed({1'b0, map_wr_y}) >= $signed({1'b0, box_ylo}) - PLAN_INFLATE_CELLS &&
        $signed({1'b0, map_wr_y}) <= $signed({1'b0, box_yhi}) + PLAN_INFLATE_CELLS;
    assign publish_ok = goal_valid && goal_x == round_goal_x && goal_y == round_goal_y &&
        st_sensors_valid && !round_canceled && !validation_dirty &&
        !(version_changed && update_near) && map_idle &&
        !(map_wr_valid && map_wr_ready && write_near);
endmodule
