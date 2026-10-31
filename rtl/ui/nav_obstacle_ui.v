// 避障导航选点页；原非避障页面不受本模块影响。
// 世界位置为毫米，原点像素不变，每格显示 3 px（60 px/m）；目标提交为 50 mm 地图格。
// 仅编辑态改点表；暂停/故障保留当前点。完成只读，返回后开始新任务。
`timescale 1ns/1ps

module nav_obstacle_ui #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer CURSOR_SPEED = 500,     // mm/s @ 满杆
    parameter integer DEADZONE     = 100,     // 摇杆死区（±1000 归一化）
    parameter integer MAX_POINTS   = 16,
    parameter integer X_MIN_MM     = -3400,
    parameter integer X_MAX_MM     = 6375,
    parameter integer Y_MIN_MM     = -4983,
    parameter integer Y_MAX_MM     = 5000,
    parameter integer PX_ORIGIN    = 412,     // 原点像素 x
    parameter integer PY_ORIGIN    = 300,     // 原点像素 y
    parameter integer SCALE_Q16    = 3932     // 0.06 px/mm 的 Q16；选点不越过地图 +6375mm
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire                i_active,      // 1 = 当前显示避障导航页
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
    output reg  [2:0]          o_aux_sel,     // 辅助菜单高亮 0..3
    output wire [4:0]          o_pt_count,
    output wire [2:0]          o_nav_state,   // 编码见 nav_obstacle_states.vh
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

    `include "nav_obstacle_states.vh"
    reg [2:0] ui_state;
    reg [3:0] goal_index;
    localparam [1:0] GOAL_WAIT = 0, GOAL_CONVERT = 1, GOAL_ACTIVE = 2;
    reg [1:0] goal_phase;

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

    // 位移先寄存，再做准心累加与限幅，乘法不进入坐标反馈链。
    reg signed [31:0] dx_step, dy_step;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dx_step <= 0;
            dy_step <= 0;
        end else begin
            dx_step <= (STEER_SIGN < 0) ? -((st_eff * K_SHIFT) >>> 10)
                                       :  ((st_eff * K_SHIFT) >>> 10);
            dy_step <= (THROTTLE_SIGN < 0) ? -((th_eff * K_SHIFT) >>> 10)
                                          :  ((th_eff * K_SHIFT) >>> 10);
        end
    end

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

    // 进入编辑共用初始化，不让同一按键再记录一个点。
    wire enter_edit = i_active && (enter ||
        (ui_state == OBS_BROWSE && key_confirm) ||
        (ui_state == OBS_DONE && key_back));
    wire cancel_task = i_clear || !i_active ||
        (key_back && (ui_state == OBS_EDIT || ui_state == OBS_PAUSE || ui_state == OBS_FAULT));
    wire cursor_en = i_active && ui_state == OBS_EDIT && !i_manual;

    reg signed [31:0] cur_x_q10, cur_y_q10;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (i_clear) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (enter_edit) begin
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
    // 点表始终连续，计数直接寄存；有效点掩码由计数派生，省去逐位求和链。
    reg [4:0] point_count;

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            point_count <= 0;
            for (k = 0; k < MAX_POINTS; k = k + 1) begin
                pt_x[k] <= 16'sd0;
                pt_y[k] <= 16'sd0;
            end
        end else if (cancel_task || enter_edit) begin
            point_count <= 0;
        end else if (i_active && ui_state == OBS_EDIT) begin
            if (key_confirm) begin
                case (o_aux_sel)
                    AUX_SET: begin
                        if (point_count < MAX_POINTS) begin
                            pt_x[point_count[3:0]] <= o_cur_x_mm[15:0];
                            pt_y[point_count[3:0]] <= o_cur_y_mm[15:0];
                            point_count <= point_count + 1'b1;
                        end
                    end
                    AUX_UNDO: begin
                        if (point_count != 5'd0) point_count <= point_count - 1'b1;
                    end
                    AUX_CLEAR: point_count <= 0;
                    default: ;   // 开始不改点表
                endcase
            end
        end
    end

    assign o_pt_count = point_count;

    wire [2:0] aux_last = AUX_CLEAR;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_aux_sel <= AUX_SET;          // 需求原文："初始默认选中确认路径点"
        end else if (cancel_task || enter_edit) begin
            o_aux_sel <= AUX_SET;
        end else if (i_active && ui_state == OBS_EDIT) begin
            if (key_aux_down) o_aux_sel <= (o_aux_sel >= aux_last) ? AUX_SET : (o_aux_sel + 3'd1);
            else if (key_aux_up) o_aux_sel <= (o_aux_sel == AUX_SET) ? aux_last : (o_aux_sel - 3'd1);
        end
    end

    wire goal_arrived = ui_state == OBS_RUN && goal_phase == GOAL_ACTIVE &&
        o_goal_valid && i_nav_state == 4'd3 && !i_ctrl_fault;
    wire final_arrived = goal_arrived && {1'b0, goal_index} + 5'd1 == point_count;

    assign o_nav_ui_active = i_active && ui_state != OBS_BROWSE;
    assign o_nav_run = i_active && ui_state == OBS_RUN;
    assign o_nav_paused = i_active && ui_state == OBS_PAUSE;
    assign o_nav_state = ui_state == OBS_RUN && i_nav_state == 4'd4 ? OBS_NO_PATH : ui_state;

    // 单一行为状态；到达结果留在 UI，不靠保持旧 goal_valid 保存提示。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ui_state <= OBS_BROWSE;
        else if (cancel_task) ui_state <= OBS_BROWSE;
        else if (enter_edit) ui_state <= OBS_EDIT;
        else if (i_ctrl_fault && (ui_state == OBS_RUN || ui_state == OBS_PAUSE))
            ui_state <= OBS_FAULT;
        else if (final_arrived) ui_state <= OBS_DONE;
        else if (ui_state == OBS_RUN && (i_manual || i_pause || key_back || key_confirm))
            ui_state <= OBS_PAUSE;
        else if (key_confirm && !i_pause) begin
            case (ui_state)
                OBS_EDIT: if (o_aux_sel == AUX_START && point_count != 0) ui_state <= OBS_RUN;
                OBS_PAUSE, OBS_FAULT: ui_state <= OBS_RUN;
                default: ;
            endcase
        end
    end

    // WAIT 等旧命令释放，CONVERT 独立一拍做格号换算；暂停/故障不重置索引。
    reg [15:0] offset_x, offset_y;
    wire signed [15:0] selected_x = pt_x[goal_index];
    wire signed [15:0] selected_y = pt_y[goal_index];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            goal_index    <= 0;
            goal_phase    <= GOAL_WAIT;
            o_goal_valid  <= 0;
            o_goal_x_cell <= 0;
            o_goal_y_cell <= 0;
            offset_x      <= 0;
            offset_y      <= 0;
        end else if (cancel_task || enter_edit) begin
            goal_index   <= 0;
            goal_phase   <= GOAL_WAIT;
            o_goal_valid <= 0;
        end else if (final_arrived || ui_state == OBS_DONE) begin
            o_goal_valid <= 0;
            goal_phase   <= GOAL_WAIT;
        end else case (goal_phase)
            GOAL_WAIT: if (ui_state == OBS_RUN && i_command_idle) begin
                offset_x <= selected_x + 16'd6400;
                offset_y <= selected_y + 16'd6400;
                goal_phase <= GOAL_CONVERT;
            end
            GOAL_CONVERT: begin
                o_goal_x_cell <= offset_x / 16'd50;
                o_goal_y_cell <= offset_y / 16'd50;
                o_goal_valid <= 1;
                goal_phase <= GOAL_ACTIVE;
            end
            GOAL_ACTIVE: if (goal_arrived && !final_arrived) begin
                o_goal_valid <= 0;
                goal_index <= goal_index + 1'b1;
                goal_phase <= GOAL_WAIT;
            end
            default: goal_phase <= GOAL_WAIT;
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
    // Split the display-only multiply from the cursor feedback/enable cone.
    // These products update every clock; manual takeover still pauses the
    // state machine and chassis through the original direct safety paths.
    reg signed [31:0] cur_x_scaled /* synthesis syn_preserve=1 */;
    reg signed [31:0] cur_y_scaled /* synthesis syn_preserve=1 */;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cur_x_scaled <= 0; cur_y_scaled <= 0; end
        else begin
            cur_x_scaled <= o_cur_x_mm * SCALE_Q16;
            cur_y_scaled <= o_cur_y_mm * SCALE_Q16;
        end
    end
    wire signed [31:0] cur_px_w = PX_ORIGIN + (cur_x_scaled >>> 16);
    wire signed [31:0] cur_py_w = PY_ORIGIN - (cur_y_scaled >>> 16);

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
            assign o_pt_v[gi] = gi < point_count;
        end
    endgenerate

endmodule
