// 避障导航选点页；原非避障页面不受本模块影响。
// 世界位置为毫米，原点和显示比例沿用原选点页；目标提交为 100 mm 地图格。
// 列表仅设置时编辑，暂停保留当前点；目的地到达后完成撤销握手再提交下一点。
`timescale 1ns/1ps

module nav_obstacle_ui #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer CURSOR_SPEED = 500,     // mm/s @ 满杆
    parameter integer DEADZONE     = 100,     // 摇杆死区（±1000 归一化）
    parameter integer MAX_POINTS   = 16,
    parameter integer X_MIN_MM     = -3400,
    parameter integer X_MAX_MM     = 10183,
    parameter integer Y_MIN_MM     = -4983,
    parameter integer Y_MAX_MM     = 5000,
    parameter integer PX_ORIGIN    = 412,     // 原点像素 x
    parameter integer PY_ORIGIN    = 300,     // 原点像素 y
    parameter integer SCALE_Q16    = 3932     // 0.06 px/mm 的 Q16（= 0.059998）
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire                i_active,      // 1 = 当前停在某个导航页（避障 / 非避障）
    input  wire                i_clear,       // S2 清零（与 odom 同源）
    input  wire signed [31:0]  i_x_mm,        // 车辆当前航位（mm，相对清零点）
    input  wire signed [31:0]  i_y_mm,

    input  wire signed [11:0]  i_steer,       // 摇杆（归一化 ±1000）
    input  wire signed [11:0]  i_throttle,
    input  wire                i_manual,      // 1 = 手动接管档（摇杆此时归底盘，不动准心）
    input  wire                i_confirm,     // 确认键（脉冲，CH5 下拨）
    input  wire                i_back,        // 返回键（脉冲，CH5 上拨）
    input  wire                i_aux_up,      // 辅助菜单向上（脉冲，CH8 上拨）
    input  wire                i_aux_down,    // 辅助菜单向下（脉冲，CH8 下拨）

    input  wire [3:0]          i_nav_state,   // 3=当前目的地到达，4=无路
    input  wire                i_command_idle,
    input  wire                i_pause,
    input  wire                i_ctrl_fault,  // 导航层回告：未到达而终止（堵转/超时/不可达）

    output reg  signed [31:0]  o_cur_x_mm,
    output reg  signed [31:0]  o_cur_y_mm,
    output reg  [2:0]          o_aux_sel,     // 辅助菜单高亮 0..4
    output wire [4:0]          o_pt_count,
    output reg  [1:0]          o_nav_state,   // 0=SETUP 1=RUNNING 2=PAUSED
    output wire                o_nav_ui_active, // 1 = 导航页处于交互态（菜单锁、状态栏显示导航状态）
    output wire                o_nav_run,     // 1 = 导航中
    output wire                o_nav_paused,  // 1 = 导航暂停

    output reg                 o_goal_valid,
    output reg  [7:0]          o_goal_x_cell,
    output reg  [7:0]          o_goal_y_cell,

    output wire [MAX_POINTS*10-1:0] o_pt_px,  // 每个点 10 位
    output wire [MAX_POINTS*10-1:0] o_pt_py,
    output wire [MAX_POINTS-1:0]    o_pt_v,   // 该点有效
    output wire [9:0]               o_car_px, // 车绿点（来自 odom 航位）
    output wire [9:0]               o_car_py,
    output wire [9:0]               o_cur_px, // 准心
    output wire [9:0]               o_cur_py
);

    localparam integer K_SHIFT = (CURSOR_SPEED * 1024 * 1024) / (1000 * 1000);
    localparam integer TICK_END = CLK_HZ / 1000 - 1;      // 50000−1

    localparam [2:0] AUX_SET   = 3'd0,
                     AUX_UNDO  = 3'd1,
                     AUX_START = 3'd2,
                     AUX_CLEAR = 3'd3;

    localparam [1:0] ST_SETUP   = 2'd0,   // 未开跑：可移准心、可编辑点表
                     ST_RUNNING = 2'd1,   // 导航中
                     ST_PAUSED  = 2'd2;   // 导航暂停（返回键暂停，确认键继续）

    wire key_confirm  = i_confirm  & ~i_manual;
    wire key_back     = i_back     & ~i_manual;
    wire key_aux_up   = i_aux_up   & ~i_manual;
    wire key_aux_down = i_aux_down & ~i_manual;

    localparam signed [31:0] XLO = X_MIN_MM, XHI = X_MAX_MM,
                             YLO = Y_MIN_MM, YHI = Y_MAX_MM;

    localparam integer STEER_SIGN    = -1;   // ★ 2026-10-19 上板实测：原 +1 时准心左右反向，取反
    localparam integer THROTTLE_SIGN =  1;

    reg [15:0] tick_cnt;
    wire tick = (tick_cnt == TICK_END);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      tick_cnt <= 16'd0;
        else if (tick)   tick_cnt <= 16'd0;
        else             tick_cnt <= tick_cnt + 16'd1;
    end

    wire signed [11:0] st_eff = (i_steer    >  DEADZONE) ? i_steer    :
                                (i_steer    < -DEADZONE) ? i_steer    : 12'sd0;
    wire signed [11:0] th_eff = (i_throttle >  DEADZONE) ? i_throttle :
                                (i_throttle < -DEADZONE) ? i_throttle : 12'sd0;

    wire signed [31:0] dx_step = (STEER_SIGN    < 0) ? -((st_eff * K_SHIFT) >>> 10)
                                                    :  ((st_eff * K_SHIFT) >>> 10);
    wire signed [31:0] dy_step = (THROTTLE_SIGN < 0) ? -((th_eff * K_SHIFT) >>> 10)
                                                    :  ((th_eff * K_SHIFT) >>> 10);

    reg active_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) active_d <= 1'b0;
        else        active_d <= i_active;
    end
    wire enter = i_active & ~active_d;

    function signed [31:0] clamp_mm;
        input signed [31:0] v;
        input signed [31:0] lo;
        input signed [31:0] hi;
        begin
            if (v > hi)      clamp_mm = hi;
            else if (v < lo) clamp_mm = lo;
            else             clamp_mm = v;
        end
    endfunction

    wire cursor_en = o_nav_ui_active & (o_nav_state == ST_SETUP) & ~i_manual;

    reg signed [31:0] cur_x_q10, cur_y_q10;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (i_clear) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (enter) begin
            cur_x_q10 <= clamp_mm(i_x_mm, XLO, XHI) <<< 10;
            cur_y_q10 <= clamp_mm(i_y_mm, YLO, YHI) <<< 10;
        end else if (tick && cursor_en) begin
            cur_x_q10 <= clamp_mm(cur_x_q10 + dx_step, XLO <<< 10, XHI <<< 10);
            cur_y_q10 <= clamp_mm(cur_y_q10 + dy_step, YLO <<< 10, YHI <<< 10);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_cur_x_mm <= 32'sd0;
            o_cur_y_mm <= 32'sd0;
        end else begin
            o_cur_x_mm <= cur_x_q10 >>> 10;
            o_cur_y_mm <= cur_y_q10 >>> 10;
        end
    end

    reg signed [15:0] pt_x [0:MAX_POINTS-1];
    reg signed [15:0] pt_y [0:MAX_POINTS-1];
    reg        [MAX_POINTS-1:0] pt_v;

    integer j;
    reg [4:0] cnt_sum;
    always @(*) begin
        cnt_sum = 5'd0;
        for (j = 0; j < MAX_POINTS; j = j + 1)
            cnt_sum = cnt_sum + {4'd0, pt_v[j]};
    end

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pt_v <= {MAX_POINTS{1'b0}};
            for (k = 0; k < MAX_POINTS; k = k + 1) begin
                pt_x[k] <= 16'sd0;
                pt_y[k] <= 16'sd0;
            end
        end else if (i_clear || enter) begin
            pt_v <= {MAX_POINTS{1'b0}};
        end else if (o_nav_ui_active && (o_nav_state == ST_SETUP)) begin
            if (key_confirm) begin
                case (o_aux_sel)
                    AUX_SET: begin
                        if (cnt_sum < MAX_POINTS) begin
                            pt_x[cnt_sum[3:0]] <= o_cur_x_mm[15:0];
                            pt_y[cnt_sum[3:0]] <= o_cur_y_mm[15:0];
                            pt_v[cnt_sum[3:0]] <= 1'b1;
                        end
                    end
                    AUX_UNDO: begin
                        if (cnt_sum != 5'd0) pt_v[cnt_sum - 5'd1] <= 1'b0;
                    end
                    AUX_CLEAR: pt_v <= {MAX_POINTS{1'b0}};
                    default: ;   // GAP / CONT 不改点表
                endcase
            end
        end
    end

    assign o_pt_count = cnt_sum;

    wire [2:0] aux_last = AUX_CLEAR;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_aux_sel <= AUX_SET;          // 需求原文："初始默认选中确认路径点"
        end else if (i_clear || enter) begin
            o_aux_sel <= AUX_SET;
        end else if (o_nav_ui_active && (o_nav_state == ST_SETUP)) begin
            if (key_aux_down) o_aux_sel <= (o_aux_sel >= aux_last) ? AUX_SET : (o_aux_sel + 3'd1);
            else if (key_aux_up) o_aux_sel <= (o_aux_sel == AUX_SET) ? aux_last : (o_aux_sel - 3'd1);
        end
    end

    reg ui_active;
    wire reenter = i_active & ~ui_active & key_confirm;   // 手动挡下不重进
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  ui_active <= 1'b0;
        else if (i_clear)            ui_active <= 1'b0;
        else if (!i_active)          ui_active <= 1'b0;
        else if (enter)              ui_active <= 1'b1;
        else if (key_back && (o_nav_state != ST_RUNNING)) ui_active <= 1'b0;
        else if (reenter)            ui_active <= 1'b1;
    end
    assign o_nav_ui_active = ui_active;
    assign o_nav_run       = (o_nav_state == ST_RUNNING);
    assign o_nav_paused    = (o_nav_state == ST_PAUSED);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_nav_state <= ST_SETUP;
        end else if (i_clear || !i_active || enter) begin
            o_nav_state <= ST_SETUP;
        end else if (key_back) begin
            if (o_nav_state == ST_RUNNING) o_nav_state <= ST_PAUSED;
            else                           o_nav_state <= ST_SETUP;
        end else if ((i_manual || i_pause) && (o_nav_state == ST_RUNNING)) begin
            o_nav_state <= ST_PAUSED;
        end else if (i_ctrl_fault) begin
            o_nav_state <= ST_SETUP;
        end else if (key_confirm && o_nav_ui_active) begin
            case (o_nav_state)
                ST_SETUP: begin
                    if (o_aux_sel == AUX_START && cnt_sum != 0)
                        o_nav_state <= ST_RUNNING;
                end
                ST_PAUSED: begin
                    if (!i_manual) o_nav_state <= ST_RUNNING;
                end
                ST_RUNNING: o_nav_state <= ST_PAUSED;
                default: ;
            endcase
        end
    end

    // 只在现有点表内推进索引，不复制任务列表。
    reg [3:0] goal_index;
    reg [1:0] goal_phase;
    reg [15:0] offset_x, offset_y;
    wire signed [15:0] selected_x = pt_x[goal_index];
    wire signed [15:0] selected_y = pt_y[goal_index];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            goal_index   <= 0;
            goal_phase   <= 0;
            o_goal_valid <= 0;
            o_goal_x_cell <= 0;
            o_goal_y_cell <= 0;
            offset_x <= 0;
            offset_y <= 0;
        end else if (i_clear || !ui_active || o_nav_state == ST_SETUP) begin
            goal_index   <= 0;
            goal_phase   <= 0;
            o_goal_valid <= 0;
        end else case (goal_phase)
            0: if (o_nav_state == ST_RUNNING && i_command_idle) begin
                offset_x <= selected_x + 16'd12800;
                offset_y <= selected_y + 16'd12800;
                goal_phase <= 1;
            end
            1: begin
                o_goal_x_cell <= offset_x / 16'd100;
                o_goal_y_cell <= offset_y / 16'd100;
                o_goal_valid <= 1;
                goal_phase <= 2;
            end
            2: if (o_nav_state == ST_RUNNING && i_nav_state == 4'd3 &&
                   {1'b0, goal_index} + 5'd1 < cnt_sum) begin
                o_goal_valid <= 0;
                goal_index <= goal_index + 1'b1;
                goal_phase <= 0;
            end
            default: goal_phase <= 0;
        endcase
    end

    function [9:0] px_of;
        input signed [15:0] xmm;
        begin
            px_of = PX_ORIGIN + (((xmm * SCALE_Q16) >>> 16));
        end
    endfunction

    function [9:0] py_of;
        input signed [15:0] ymm;
        begin
            py_of = PY_ORIGIN - (((ymm * SCALE_Q16) >>> 16));
        end
    endfunction

    reg signed [31:0] ix_d1, iy_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_d1 <= 32'sd0;
            iy_d1 <= 32'sd0;
        end else begin
            ix_d1 <= i_x_mm;
            iy_d1 <= i_y_mm;
        end
    end

    wire signed [31:0] car_px_raw = PX_ORIGIN + (((ix_d1 * SCALE_Q16) >>> 16));
    wire signed [31:0] car_py_raw = PY_ORIGIN - (((iy_d1 * SCALE_Q16) >>> 16));
    wire signed [31:0] car_px_w   = (car_px_raw < 32'sd208)  ? 32'sd208 :
                                    (car_px_raw > 32'sd1023) ? 32'sd1023 : car_px_raw;
    wire signed [31:0] car_py_w   = (car_py_raw < 32'sd0)    ? 32'sd0 :
                                    (car_py_raw > 32'sd599)  ? 32'sd599 : car_py_raw;
    wire signed [31:0] cur_px_w = PX_ORIGIN + (((o_cur_x_mm * SCALE_Q16) >>> 16));
    wire signed [31:0] cur_py_w = PY_ORIGIN - (((o_cur_y_mm * SCALE_Q16) >>> 16));

    reg [9:0] car_px_r, car_py_r, cur_px_r, cur_py_r;
    reg [9:0] pt_px_r [0:MAX_POINTS-1];
    reg [9:0] pt_py_r [0:MAX_POINTS-1];

    integer g2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            car_px_r <= 10'd0; car_py_r <= 10'd0;
            cur_px_r <= 10'd0; cur_py_r <= 10'd0;
            for (g2 = 0; g2 < MAX_POINTS; g2 = g2 + 1) begin
                pt_px_r[g2] <= 10'd0;
                pt_py_r[g2] <= 10'd0;
            end
        end else begin
            car_px_r <= car_px_w[9:0];
            car_py_r <= car_py_w[9:0];
            cur_px_r <= cur_px_w[9:0];
            cur_py_r <= cur_py_w[9:0];
            for (g2 = 0; g2 < MAX_POINTS; g2 = g2 + 1) begin
                pt_px_r[g2] <= px_of(pt_x[g2]);
                pt_py_r[g2] <= py_of(pt_y[g2]);
            end
        end
    end

    assign o_car_px = car_px_r;
    assign o_car_py = car_py_r;
    assign o_cur_px = cur_px_r;
    assign o_cur_py = cur_py_r;

    genvar gi;
    generate
        for (gi = 0; gi < MAX_POINTS; gi = gi + 1) begin : g_pt
            assign o_pt_px[gi*10 +: 10] = pt_px_r[gi];
            assign o_pt_py[gi*10 +: 10] = pt_py_r[gi];
        end
    endgenerate
    assign o_pt_v = pt_v;

endmodule