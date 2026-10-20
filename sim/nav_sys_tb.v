//////////////////////////////////////////////////////////////////////////////////
// 测试台：nav_sys —— 「UI 交互层（nav_free_ctrl）+ 导航控制层（nav_ctrl）」联合
//
// 【为什么要这一台】
//   两台模块各自的 tb 只能证明"自己内部自洽"。真正会上板出事的是**接缝**：
//     · UI 的 `o_nav_run / o_nav_paused / o_nav_mode` 是否真的驱动了控制层
//     · 控制层的 `o_done / o_fault` 是否真的把 UI 打回 SETUP
//     · 暂停时到底有没有把车停住（需求方 m00883 的原话）
//     · 退出交互态后车是否立刻失去指令
//   本 tb 把两模块按 `falcons_top.v` 的接法连起来，用**按键序列**驱动，
//   断言最终落在电机指令上（o_auto_valid / center / diff），而不是只看状态字。
//
// 【节拍换算】nav_free_ctrl 的准心节拍 = CLK_HZ/1000，这里把 CLK_HZ 设成 1000，
//   于是"每 1 个时钟 = 1 ms 虚拟时间"，满杆每拍走 0.5 mm。
//   nav_ctrl 的 i_tick 由 tb 单独发（每 80 拍一次），与准心节拍互不影响。
//
// 【按键语义（CH5/CH8 译码后）】
//   i_confirm = CH5 下拨（脉冲）  i_back = CH5 上拨（脉冲）
//   i_aux_up  = CH8 上拨（脉冲）  i_aux_down = CH8 下拨（脉冲）
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

module nav_sys_tb;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    // ---- 共享输入 ----
    reg                i_active = 1'b0;
    reg                i_is_obs = 1'b0;
    reg                i_manual = 1'b0;
    reg  signed [11:0] i_steer = 12'sd0, i_throttle = 12'sd0;
    reg                i_confirm = 1'b0, i_back = 1'b0;
    reg                i_aux_up = 1'b0, i_aux_down = 1'b0;
    reg  signed [31:0] car_x = 32'sd0, car_y = 32'sd0, car_yaw = 32'sd0, car_v = 32'sd0;
    reg                odo_tick = 1'b0;
    reg  [3:0]         stall = 4'd0;

    // ---- UI 层输出 ----
    wire signed [31:0] cur_x, cur_y;
    wire [2:0]         aux_sel;
    wire [4:0]         pt_count;
    wire [1:0]         ui_state;
    wire               ui_active, ui_run, ui_paused, ui_mode;
    wire [159:0]       pt_px, pt_py;
    wire [255:0]       pt_x_mm, pt_y_mm;
    wire [15:0]        pt_v;
    wire [9:0]         car_px, car_py, cur_px, cur_py;
    wire               ctrl_done, ctrl_fault;   // ★ 必须先声明：u_ui 例化里就要用

    nav_free_ctrl #(
        .CLK_HZ       (1000),      // ★ 每 1 拍 = 1 ms 虚拟时间
        .CURSOR_SPEED (500),
        .DEADZONE     (100),
        .MAX_POINTS   (16)
    ) u_ui (
        .clk        (clk),
        .rst_n      (rst_n),
        .i_active   (i_active),
        .i_is_obs   (i_is_obs),
        .i_clear    (1'b0),
        .i_x_mm     (car_x),
        .i_y_mm     (car_y),
        .i_steer    (i_steer),
        .i_throttle (i_throttle),
        .i_manual   (i_manual),
        .i_confirm  (i_confirm),
        .i_back     (i_back),
        .i_aux_up   (i_aux_up),
        .i_aux_down (i_aux_down),
        .i_ctrl_done  (ctrl_done),
        .i_ctrl_fault (ctrl_fault),
        .o_cur_x_mm (cur_x),
        .o_cur_y_mm (cur_y),
        .o_aux_sel  (aux_sel),
        .o_pt_count (pt_count),
        .o_nav_state     (ui_state),
        .o_nav_ui_active (ui_active),
        .o_nav_run       (ui_run),
        .o_nav_paused    (ui_paused),
        .o_nav_mode      (ui_mode),
        .o_pt_px    (pt_px),
        .o_pt_py    (pt_py),
        .o_pt_v     (pt_v),
        .o_pt_x_mm  (pt_x_mm),
        .o_pt_y_mm  (pt_y_mm),
        .o_car_px   (car_px),
        .o_car_py   (car_py),
        .o_cur_px   (cur_px),
        .o_cur_py   (cur_py)
    );

    // ---- 控制层 ----
    wire               nav_valid;
    wire signed [31:0] nav_center, nav_diff;
    wire [2:0]         nav_state;

    // ★ 2026-10-20：nav_ctrl 的 8 个可调值已从 parameter 改成运行时端口
    //   （参数整定页在线改）。联合台用默认值常量接上；nav_ctrl_tb 里另有
    //   在线改值的 G12 组断言。悬空会让 i_v_cruise 之类变成 0/X，车根本不动。
    localparam signed [31:0] P_V_CRUISE  = 32'sd300;
    localparam signed [31:0] P_D_SLOW    = 32'sd100;
    localparam signed [31:0] P_ALIGN_IN  = 32'sd1000;
    localparam signed [31:0] P_ALIGN_OUT = 32'sd3500;
    localparam signed [31:0] P_MDC       = 32'sd180;
    localparam signed [31:0] P_KP        = 32'sd21;
    localparam signed [31:0] P_KI        = 32'sd4;
    localparam signed [31:0] P_KD        = 32'sd6;
    localparam signed [31:0] P_KSLOW     = 32'sd3072;
localparam signed [31:0] P_I_LIMIT   = 32'sd90;

    nav_ctrl u_nav (
        .clk(clk), .rst_n(rst_n),
        .i_active  (i_active & ui_active),    // 与顶层同一接法
        .i_v_cruise       (P_V_CRUISE),
        .i_d_slow         (P_D_SLOW),
        .i_align_in       (P_ALIGN_IN),
        .i_align_out      (P_ALIGN_OUT),
        .i_max_diff_cruise(P_MDC),
        .i_kp_yaw         (P_KP),
        .i_ki_yaw         (P_KI),
        .i_kd_yaw         (P_KD),
        .i_i_limit        (P_I_LIMIT),
        .i_k_slow_q10     (P_KSLOW),
        .i_run     (ui_run),
        .i_paused  (ui_paused),
        .i_mode    (ui_mode),
        .i_is_obs  (i_is_obs),
        .i_cur_x_mm(cur_x), .i_cur_y_mm(cur_y),
        .i_pt_x_mm (pt_x_mm), .i_pt_y_mm(pt_y_mm), .i_pt_count(pt_count),
        .i_tick    (odo_tick),
        .i_x_mm(car_x), .i_y_mm(car_y), .i_yaw_centi(car_yaw), .i_v_mms(car_v),
        .i_stall(stall),
        .o_auto_valid(nav_valid), .o_center(nav_center), .o_diff(nav_diff),
        .o_done(ctrl_done), .o_fault(ctrl_fault),
        .o_state(nav_state), .o_dbg_ahead(), .o_dbg_left(),
        .o_dbg_psi(), .o_dbg_idx()
    );

    localparam [2:0] S_IDLE = 3'd0, S_ALIGN = 3'd1, S_TRACK = 3'd2,
                     S_HOLD = 3'd3, S_DONE = 3'd4, S_FAULT = 3'd5;
    localparam [1:0] ST_SETUP = 2'd0, ST_RUNNING = 2'd1, ST_PAUSED = 2'd2;

    integer errors = 0, checks = 0;
    integer done_cnt = 0, fault_cnt = 0;
    reg [2:0] st_before;

    // ★ o_done / o_fault 都是**单拍脉冲**，而且 UI 收到后立刻把 i_run 拉低 →
    //   nav_ctrl 下一拍就回 S_IDLE。所以要靠计数抓脉冲，不能事后看状态字。
    always @(posedge clk) begin
        if (rst_n && ctrl_done)  done_cnt  = done_cnt  + 1;
        if (rst_n && ctrl_fault) fault_cnt = fault_cnt + 1;
    end

    task chk_int;
        input [255:0] name;
        input signed [31:0] got;
        input signed [31:0] exp;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s: got %0d, expect %0d", name, got, exp);
            end else $display("ok   %0s: %0d", name, got);
        end
    endtask

    task chk_true;
        input [255:0] name;
        input        cond;
        begin
            checks = checks + 1;
            if (!cond) begin errors = errors + 1; $display("FAIL %0s", name); end
            else $display("ok   %0s", name);
        end
    endtask

    // 单拍按键脉冲（与 rc_input 输出同形态）
    task pulse_confirm; begin
        @(posedge clk); i_confirm = 1'b1;
        @(posedge clk); i_confirm = 1'b0;
        repeat (4) @(posedge clk);
    end endtask

    task pulse_back; begin
        @(posedge clk); i_back = 1'b1;
        @(posedge clk); i_back = 1'b0;
        repeat (4) @(posedge clk);
    end endtask

    task pulse_aux_down; begin
        @(posedge clk); i_aux_down = 1'b1;
        @(posedge clk); i_aux_down = 1'b0;
        repeat (4) @(posedge clk);
    end endtask

    task pulse_aux_up; begin
        @(posedge clk); i_aux_up = 1'b1;
        @(posedge clk); i_aux_up = 1'b0;
        repeat (4) @(posedge clk);
    end endtask

    // 发一个 odom 10 ms 节拍并等流水走完
    task tick1; begin
        @(posedge clk); odo_tick = 1'b1;
        @(posedge clk); odo_tick = 1'b0;
        repeat (80) @(posedge clk);
    end endtask

    // 摇杆推住 n 拍（满杆 → 每拍约 0.5 mm）
    //   ★ 方向：nav_free_ctrl 里 STEER_SIGN = -1（上板实测的极性），
    //     所以 **i_steer 给负值才是向右（+x）**。这里封装成"向右推"，避免每次记方向。
    task push_right;    input integer n; begin i_steer    = -12'sd1000; repeat (n) @(posedge clk); i_steer    = 12'sd0; repeat (4) @(posedge clk); end endtask
    task push_up;       input integer n; begin i_throttle =  12'sd1000; repeat (n) @(posedge clk); i_throttle = 12'sd0; repeat (4) @(posedge clk); end endtask

    initial begin
        $display("=== nav_sys_tb ===");

        // ================================================================
        // A. 进页面 → 记两个点
        // ================================================================
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        i_active = 1'b1;                 // 进入非避障导航页
        repeat (4) @(posedge clk);
        chk_int("a_ui_active", {31'd0, ui_active}, 32'sd1);
        chk_int("a_state_setup", {30'd0, ui_state}, 32'sd0);
        chk_int("a_nav_idle", nav_state, S_IDLE);      // 还没开始，控制层空转

        // 准心右移 200 拍 → +100 mm（每拍约 0.5 mm，累积截断允许 ±2）
        push_right(200);
        chk_true("a_cur_x_pos", (cur_x >= 32'sd98) && (cur_x <= 32'sd102));
        pulse_confirm;                    // AUX_SET：记录第 1 点
        chk_int("a_pt1", {27'd0, pt_count}, 32'sd1);

        // 准心上移 200 拍 → +100 mm（屏幕 y 向上为正）
        push_up(200);
        chk_true("a_cur_y_pos", (cur_y >= 32'sd98) && (cur_y <= 32'sd102));
        pulse_confirm;                    // 记录第 2 点
        chk_int("a_pt2", {27'd0, pt_count}, 32'sd2);

        // 点表 mm 与准心一致（证明 mm 总线接对了，不需要像素反变换）
        chk_true("a_ptx0", ($signed(pt_x_mm[15:0])  >= 32'sd98) && ($signed(pt_x_mm[15:0])  <= 32'sd102));
        chk_int ("a_pty0", $signed(pt_y_mm[15:0]),  32'sd0);
        chk_true("a_ptx1", ($signed(pt_x_mm[31:16]) >= 32'sd98) && ($signed(pt_x_mm[31:16]) <= 32'sd102));
        chk_true("a_pty1", ($signed(pt_y_mm[31:16]) >= 32'sd98) && ($signed(pt_y_mm[31:16]) <= 32'sd102));

        // ================================================================
        // B. 辅助菜单下切到「间断寻点」→ 确认 → RUNNING
        // ================================================================
        chk_int("b_aux_sel0", {29'd0, aux_sel}, 32'sd0);   // 确认路径点
        pulse_aux_down;
        chk_int("b_aux_sel1", {29'd0, aux_sel}, 32'sd1);   // 撤销路径点
        pulse_aux_down;
        chk_int("b_aux_sel2", {29'd0, aux_sel}, 32'sd2);   // 间断寻点
        pulse_confirm;
        chk_int("b_ui_running", {30'd0, ui_state}, 32'sd1);
        chk_int("b_ui_run", {31'd0, ui_run}, 32'sd1);
        chk_int("b_ui_mode_gap", {31'd0, ui_mode}, 32'sd0);

        // 控制层应离开 IDLE 进 ALIGN（目标 = 第 1 点 (100,0)）
        tick1;
        chk_true("b_nav_started", (nav_state == S_ALIGN) || (nav_state == S_TRACK));
        chk_int("b_nav_valid", {31'd0, nav_valid}, 32'sd1);

        // ================================================================
        // C. 暂停：返回键 → PAUSED，车必须停住
        // ================================================================
        tick1;
        pulse_back;
        chk_int("c_ui_paused", {30'd0, ui_state}, 32'sd2);
        chk_int("c_ui_paused_flag", {31'd0, ui_paused}, 32'sd1);
        chk_int("c_ui_run_off", {31'd0, ui_run}, 32'sd0);
        repeat (3) @(posedge clk);
        chk_int("c_nav_valid_still", {31'd0, nav_valid}, 32'sd1);   // 仍 valid（主动制动）
        chk_int("c_nav_center_zero", nav_center, 32'sd0);
        chk_int("c_nav_diff_zero", nav_diff, 32'sd0);

        // 暂停期间再发两拍 odom tick，仍必须零输出
        tick1; tick1;
        chk_int("c_nav_center_zero2", nav_center, 32'sd0);
        chk_int("c_nav_valid_still2", {31'd0, nav_valid}, 32'sd1);

        // ================================================================
        // D. 确认恢复 → RUNNING，车重新动
        // ================================================================
        pulse_confirm;
        chk_int("d_ui_running", {30'd0, ui_state}, 32'sd1);
        tick1; tick1;
        chk_int("d_nav_valid", {31'd0, nav_valid}, 32'sd1);
        chk_true("d_nav_moving", nav_center > 32'sd0);

        // ================================================================
        // E. 到达第 1 点 → HOLD → 切第 2 点 → ALIGN（不报 done）
        // ================================================================
        car_x = $signed(pt_x_mm[15:0]);            // 放到第 1 点的真实坐标
        car_y = $signed(pt_y_mm[15:0]);
        car_v = 32'sd0;
        tick1;
        chk_int("e_hold1", nav_state, S_HOLD);
        repeat (11) tick1;
        chk_int("e_align2", nav_state, S_ALIGN);
        chk_int("e_ui_still_running", {30'd0, ui_state}, 32'sd1);   // 中间点不打断
        chk_int("e_ui_run_still", {31'd0, ui_run}, 32'sd1);

        // ================================================================
        // F. 到达最后一点 → done → UI 自动回 SETUP
        // ================================================================
        // ★ 直接把车放到**点表里真实存的坐标**：光标累积有截断，点可能是 (100,99)
        //   而不是 (100,100)。差 1 mm 在 dx≈0 时会被 atan2 放大成 90°。
        car_x = $signed(pt_x_mm[31:16]);
        car_y = $signed(pt_y_mm[31:16]);
        car_yaw = 32'sd9000;                       // 车头已转向 +y
        tick1;                                     // ALIGN → TRACK（零向量 psi=0）
        tick1;                                     // 到点 → HOLD
        chk_int("f_hold2", nav_state, S_HOLD);
        repeat (11) tick1;
        // done 一拍后：nav_ctrl 回 IDLE（i_run 已被 UI 拉低）、UI 回 SETUP
        chk_int("f_done_pulse", done_cnt, 32'sd1);          // 只报一次
        chk_int("f_nav_idle",   nav_state, S_IDLE);
        chk_int("f_ui_setup",   {30'd0, ui_state}, 32'sd0);  // done → SETUP
        chk_int("f_ui_run_off", {31'd0, ui_run}, 32'sd0);
        repeat (3) @(posedge clk);
        chk_int("f_nav_valid_off", {31'd0, nav_valid}, 32'sd0);    // 车松开

        // ================================================================
        // G. 故障：控制层 fault → UI 回 SETUP
        // ================================================================
        // 重新开一次（点表还在，aux_sel 仍是「间断寻点」）
        pulse_confirm;
        chk_int("g_ui_running", {30'd0, ui_state}, 32'sd1);
        tick1;
        stall = 4'd1;
        tick1;
        chk_int("g_fault_pulse", fault_cnt, 32'sd1);
        chk_int("g_nav_idle", nav_state, S_IDLE);
        chk_int("g_ui_setup_after_fault", {30'd0, ui_state}, 32'sd0);
        chk_int("g_nav_valid_off", {31'd0, nav_valid}, 32'sd0);
        stall = 4'd0;

        // ================================================================
        // H. 退出交互态 → 车立即失去指令；再按确认能立刻重进
        // ================================================================
        pulse_back;
        chk_int("h_ui_exit", {31'd0, ui_active}, 32'sd0);
        repeat (3) @(posedge clk);
        chk_int("h_nav_valid_off", {31'd0, nav_valid}, 32'sd0);
        pulse_confirm;                             // 不用换页，直接能重进
        chk_int("h_ui_reenter", {31'd0, ui_active}, 32'sd1);
        chk_int("h_ui_setup2", {30'd0, ui_state}, 32'sd0);
        // ★ 重进那一拍不许把「间断寻点」当成"开始导航"
        chk_int("h_no_auto_run", {31'd0, ui_run}, 32'sd0);

        // ================================================================
        // K. 运行中切手动挡 → 车停 + 进 PAUSED；回光标挡仍暂停；确认从原任务继续
        //    （需求原话：任务运行时切到手动挡，车停下来且进入手动控制；
        //      返回光标挡时显示任务暂停，此时继续任务的话仍可从刚刚的任务继续）
        // ================================================================
        pulse_confirm;                             // aux_sel 仍是「间断寻点」→ 开跑
        chk_int("k_ui_running", {30'd0, ui_state}, 32'sd1);
        tick1; tick1;
        st_before = nav_state;
        chk_true("k_nav_working", (nav_state == S_ALIGN) || (nav_state == S_TRACK));

        i_manual = 1'b1;                           // 切手动挡
        repeat (4) @(posedge clk);
        chk_int("k_ui_paused",   {30'd0, ui_state}, 32'sd2);   // RUNNING → PAUSED
        chk_int("k_run_off",     {31'd0, ui_run},   32'sd0);
        chk_int("k_paused_flag", {31'd0, ui_paused},32'sd1);
        chk_int("k_nav_valid",   {31'd0, nav_valid},32'sd1);   // 主动制动仍持 valid
        chk_int("k_center_zero", nav_center, 32'sd0);
        chk_int("k_diff_zero",   nav_diff,   32'sd0);
        tick1; tick1;                              // 手动期间来节拍也必须保持 0
        chk_int("k_center_zero2", nav_center, 32'sd0);
        chk_int("k_diff_zero2",   nav_diff,   32'sd0);

        pulse_confirm;                             // 手动挡下确认不许恢复
        chk_int("k_manual_confirm_blocked", {30'd0, ui_state}, 32'sd2);
        pulse_aux_down;                            // 手动挡下 select 也不许改菜单
        chk_int("k_manual_select_blocked", {29'd0, aux_sel}, 32'sd2);
        pulse_back;                                // 手动挡下返回也不许退出交互态
        chk_int("k_manual_back_blocked", {31'd0, ui_active}, 32'sd1);

        i_manual = 1'b0;                           // 回光标挡
        repeat (4) @(posedge clk);
        chk_int("k_cursor_back_still_paused", {30'd0, ui_state}, 32'sd2);  // 显示任务暂停

        pulse_confirm;                             // 此时确认才继续
        chk_int("k_resumed", {31'd0, ui_run}, 32'sd1);
        chk_int("k_state_kept", nav_state, st_before);   // 从刚才的状态继续，不是重开

        // ================================================================
        // I. 操作指令全覆盖：撤销点 / 清除点 / 手动档不动准心
        // ================================================================
        // ★ aux_sel 只在「交互态 + SETUP」下才更新，K 段结束时还在 RUNNING，
        //   必须先退回 SETUP，否则下面的下切全部无效。
        pulse_back;                                // RUNNING → PAUSED
        pulse_back;                                // PAUSED  → SETUP（退出交互态）
        pulse_confirm;                             // 重进，回到 SETUP
        chk_int("i_setup", {30'd0, ui_state}, 32'sd0);
        chk_int("i_ui_active", {31'd0, ui_active}, 32'sd1);
        // 手动档：摇杆归底盘，准心不许动
        begin : manual_lock
            reg signed [31:0] before;
            before = cur_x;
            i_manual = 1'b1;
            push_right(100);
            chk_int("i_manual_cursor_lock", cur_x, before);
            i_manual = 1'b0;
        end
        // ★ aux_sel 不会因"返回退出再重进"而复位（reenter 只解锁菜单，不重开页面），
        //   所以这里必须先摸清当前序号再走。f 段开跑时用的是「间断寻点」= 2。
        chk_int("i_aux_before", {29'd0, aux_sel}, 32'sd2);
        // 下切绕环：2→3→4→0 再 →1（撤销路径点）
        pulse_aux_down; pulse_aux_down; pulse_aux_down;
        chk_int("i_aux_wrap0", {29'd0, aux_sel}, 32'sd0);   // 环回证据
        pulse_aux_down;
        chk_int("i_aux_undo", {29'd0, aux_sel}, 32'sd1);    // 撤销路径点
        pulse_confirm;
        chk_int("i_pt_after_undo", {27'd0, pt_count}, 32'sd1);
        // 切到 CLEAR（1→2→3→4）并清空
        pulse_aux_down; pulse_aux_down; pulse_aux_down;
        chk_int("i_aux_clear", {29'd0, aux_sel}, 32'sd4);
        pulse_confirm;
        chk_int("i_pt_after_clear", {27'd0, pt_count}, 32'sd0);

        // ================================================================
        // J. 避障页：菜单只有 4 项，第 3 项「开始导航」不要求点表
        // ================================================================
        i_active = 1'b0;
        repeat (4) @(posedge clk);
        i_is_obs = 1'b1;
        i_active = 1'b1;
        repeat (4) @(posedge clk);
        chk_int("j_obs_active", {31'd0, ui_active}, 32'sd1);
        pulse_aux_down;                            // → UNDO(1)
        pulse_aux_down;                            // → 开始导航(2)
        chk_int("j_obs_aux", {29'd0, aux_sel}, 32'sd2);
        pulse_confirm;
        chk_int("j_obs_running", {30'd0, ui_state}, 32'sd1);
        tick1;
        chk_int("j_obs_nav_valid", {31'd0, nav_valid}, 32'sd1);

        $display("=== nav_sys_tb: %0d checks, %0d errors ===", checks, errors);
        if (errors != 0) $display("FAIL: %0d / %0d 项未通过", errors, checks);
        else             $display("PASS: nav_sys_tb %0d 项全过", checks);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
