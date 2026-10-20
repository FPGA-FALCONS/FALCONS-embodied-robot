//////////////////////////////////////////////////////////////////////////////////
// 测试台：nav_ctrl —— 全状态 / 全指令 / 全状态切换覆盖
//
// 【为什么这么长】
//   nav_ctrl 是"UI 指令 → 电机指令"的唯一通路，任何一条状态转移漏了都可能上板撞车。
//   本 tb 按**状态转移图的边**组织场景，而不是按功能点：
//     S_IDLE ⇄ S_ALIGN ⇄ S_TRACK → S_HOLD → S_ALIGN / S_DONE
//       ↑         ↖_______ S_FAULT ←（堵转 / 超时）
//       └── i_active=0 或 (i_run=0 且 i_paused=0)
//   每条边至少走一次，且校验输出量（valid/center/diff）与"该边应有的语义"一致。
//
// 【坐标系】odom 相对系：原点 = 清零位置，+x = 那一刻车头，永不重基准。
//   tb 直接写 i_x_mm/i_y_mm/i_yaw_centi 模拟"车已到达某处"，不模拟底盘动力学。
//   本模块对位置的唯一依赖就是这三个输入，所以这样"瞬移"是合法的等价测试。
//
// 【节拍】`i_tick` 在真实系统里是 10 ms；tb 里每 80 拍来一次。
//   一次 tick 内部的流水是 M_A0→M_A4（含 CORDIC 15 拍），与时间尺度无关。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

module nav_ctrl_tb;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    // ================= 主 DUT（默认参数）=================
    reg                i_active = 1'b0;
    reg                i_run = 1'b0;
    reg                i_paused = 1'b0;
    reg                i_mode = 1'b0;
    reg                i_is_obs = 1'b0;
    reg  signed [31:0] i_cur_x = 32'sd0, i_cur_y = 32'sd0;
    reg  [255:0]       i_pt_x = 256'd0, i_pt_y = 256'd0;
    reg  [4:0]         i_pt_count = 5'd0;
    reg                i_tick = 1'b0;
    reg  signed [31:0] i_x = 32'sd0, i_y = 32'sd0, i_yaw = 32'sd0, i_v = 32'sd0;
    reg  [3:0]         i_stall = 4'd0;

    wire               o_auto_valid;
    wire signed [31:0] o_center, o_diff;
    wire               o_done, o_fault;
    wire [2:0]         o_state;
    wire signed [31:0] o_ahead, o_left;
    wire signed [15:0] o_psi;
    wire [4:0]         o_idx;

    // ================= 运行时参数（2026-10-20）=================
    //   nav_ctrl 的这 9 项已从 module parameter 改为端口，由参数整定页
    //   （rtl\ui\tune_ctrl.v）在线修改。tb 里用 reg 驱动：既当默认值，
    //   也用来测「运行中改参数下一个 10 ms 节拍就生效」（G12 组）。
    //   ★ P_KSLOW 是派生量，实际由 tune_ctrl 的迭代除法器算；这里显式给
    //     3072（= 300×1024/100）。改 P_V_CRUISE/P_D_SLOW 时本 tb 不做自动
    //     同步 —— 除 G12 外所有断言的期望值都只对应这一组默认值。
    reg signed [31:0] P_V_CRUISE = 32'sd300;
    reg signed [31:0] P_D_SLOW   = 32'sd100;
    reg signed [31:0] P_ALIGN_IN = 32'sd1000;
    reg signed [31:0] P_ALIGN_OUT= 32'sd3500;
    reg signed [31:0] P_MDC      = 32'sd180;
    reg signed [31:0] P_KP       = 32'sd21;
    reg signed [31:0] P_KI       = 32'sd4;
    reg signed [31:0] P_KD       = 32'sd6;
    reg signed [31:0] P_KSLOW    = 32'sd3072;
    nav_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .i_v_cruise(P_V_CRUISE), .i_d_slow(P_D_SLOW),
        .i_align_in(P_ALIGN_IN), .i_align_out(P_ALIGN_OUT),
        .i_max_diff_cruise(P_MDC),
        .i_kp_yaw(P_KP), .i_ki_yaw(P_KI), .i_kd_yaw(P_KD),
        .i_k_slow_q10(P_KSLOW),
        .i_active(i_active), .i_run(i_run), .i_paused(i_paused),
        .i_mode(i_mode), .i_is_obs(i_is_obs),
        .i_cur_x_mm(i_cur_x), .i_cur_y_mm(i_cur_y),
        .i_pt_x_mm(i_pt_x), .i_pt_y_mm(i_pt_y), .i_pt_count(i_pt_count),
        .i_tick(i_tick),
        .i_x_mm(i_x), .i_y_mm(i_y), .i_yaw_centi(i_yaw), .i_v_mms(i_v),
        .i_stall(i_stall),
        .o_auto_valid(o_auto_valid), .o_center(o_center), .o_diff(o_diff),
        .o_done(o_done), .o_fault(o_fault),
        .o_state(o_state), .o_dbg_ahead(o_ahead), .o_dbg_left(o_left),
        .o_dbg_psi(o_psi), .o_dbg_idx(o_idx)
    );

    // ================= 快超时 DUT（专测 TIMEOUT_TICKS 分支）=================
    //   主 DUT 的 3000 tick 在本 tb 里跑不完，所以另例化一份把超时压到 12 tick。
    //   ★ 只共用输入总线，输出各自观测；不共用 rst_n 以外的任何状态。
    wire               f_valid;
    wire signed [31:0] f_center, f_diff;
    wire               f_done, f_fault;
    wire [2:0]         f_state;

    nav_ctrl #(
        .TIMEOUT_TICKS (12),
        .STOP_SETTLE_TICKS(3)
    ) u_fast (
        .clk(clk), .rst_n(rst_n),
        .i_v_cruise(P_V_CRUISE), .i_d_slow(P_D_SLOW),
        .i_align_in(P_ALIGN_IN), .i_align_out(P_ALIGN_OUT),
        .i_max_diff_cruise(P_MDC),
        .i_kp_yaw(P_KP), .i_ki_yaw(P_KI), .i_kd_yaw(P_KD),
        .i_k_slow_q10(P_KSLOW),
        .i_active(i_active), .i_run(i_run), .i_paused(i_paused),
        .i_mode(i_mode), .i_is_obs(1'b0),
        .i_cur_x_mm(32'sd0), .i_cur_y_mm(32'sd0),
        .i_pt_x_mm(i_pt_x), .i_pt_y_mm(i_pt_y), .i_pt_count(i_pt_count),
        .i_tick(i_tick),
        .i_x_mm(i_x), .i_y_mm(i_y), .i_yaw_centi(i_yaw), .i_v_mms(i_v),
        .i_stall(i_stall),
        .o_auto_valid(f_valid), .o_center(f_center), .o_diff(f_diff),
        .o_done(f_done), .o_fault(f_fault),
        .o_state(f_state), .o_dbg_ahead(), .o_dbg_left(),
        .o_dbg_psi(), .o_dbg_idx()
    );

    localparam [2:0] S_IDLE = 3'd0, S_ALIGN = 3'd1, S_TRACK = 3'd2,
                     S_HOLD = 3'd3, S_DONE = 3'd4, S_FAULT = 3'd5;

    integer errors = 0, checks = 0;
    reg done_seen = 1'b0;
    reg fault_seen = 1'b0;
    integer done_cnt = 0;
    always @(posedge clk) begin
        if (o_done)  begin done_seen  <= 1'b1; done_cnt <= done_cnt + 1; end
        if (o_fault) fault_seen <= 1'b1;
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

    task tick1;
        begin
            @(posedge clk); i_tick = 1'b1;
            @(posedge clk); i_tick = 1'b0;
            repeat (80) @(posedge clk);
        end
    endtask

    task do_reset;
        begin
            rst_n = 1'b0;
            i_active = 1'b0; i_run = 1'b0; i_paused = 1'b0; i_mode = 1'b0;
            i_is_obs = 1'b0; i_pt_x = 256'd0; i_pt_y = 256'd0; i_pt_count = 5'd0;
            i_x = 32'sd0; i_y = 32'sd0; i_yaw = 32'sd0; i_v = 32'sd0; i_stall = 4'd0;
            i_cur_x = 32'sd0; i_cur_y = 32'sd0;
            done_seen = 1'b0; fault_seen = 1'b0; done_cnt = 0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            repeat (3) @(posedge clk);
        end
    endtask

    task set_pt;
        input integer k;
        input integer px;
        input integer py;
        begin
            i_pt_x[k*16 +: 16] = px[15:0];
            i_pt_y[k*16 +: 16] = py[15:0];
        end
    endtask

    // 起动一次单/多点任务（不自动 tick）
    task start_task;
        input integer n_pts;
        input [4:0]   mode;
        input integer cx;
        input integer cy;
        input integer cyaw;
        begin
            i_pt_count = n_pts[4:0];
            i_mode     = mode[0];
            i_x = cx; i_y = cy; i_yaw = cyaw;
            i_active = 1'b1; i_run = 1'b1; i_paused = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    initial begin
        $display("=== nav_ctrl_tb (full) ===");
        // ★ 打印被测参数：避免"改了参数却跑的是旧编译"造成的假通过。
        //   G11 组的断言就是按这几个值写的，值不对必须立刻看见。
        $display("  params: ALIGN_IN=%0d ALIGN_OUT=%0d D_SLOW=%0d MAX_DIFF_CRUISE=%0d K_SLOW_Q10=%0d",
                 P_ALIGN_IN, P_ALIGN_OUT, P_D_SLOW, P_MDC, P_KSLOW);

        // ==================================================================
        // G1 起动与使能：i_active / i_run 的四种组合
        // ==================================================================
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        // (a) active=0, run=0 → IDLE、valid=0
        i_active = 1'b0; i_run = 1'b0;
        repeat (3) @(posedge clk);
        chk_int("g1_idle_state", o_state, S_IDLE);
        chk_int("g1_idle_valid", {31'd0, o_auto_valid}, 32'sd0);
        // (b) active=1, run=0 → 仍 IDLE（没按"开始寻点"）
        i_active = 1'b1;
        repeat (3) @(posedge clk);
        chk_int("g1_active_norun_state", o_state, S_IDLE);
        chk_int("g1_active_norun_valid", {31'd0, o_auto_valid}, 32'sd0);
        // (c) active=1, run=1 → ALIGN
        i_run = 1'b1;
        repeat (3) @(posedge clk);
        chk_int("g1_align_state", o_state, S_ALIGN);
        // (d) 运行中把 run 拉低（等价"退出任务"）→ 回 IDLE、valid=0
        i_run = 1'b0;
        repeat (3) @(posedge clk);
        chk_int("g1_runoff_state", o_state, S_IDLE);
        chk_int("g1_runoff_valid", {31'd0, o_auto_valid}, 32'sd0);
        chk_int("g1_runoff_center", o_center, 32'sd0);

        // ==================================================================
        // G2 ALIGN ⇄ TRACK 迟滞（30° 进 / 35° 退）与符号
        // ==================================================================
        // (a) 目标正右 → psi<0 → diff<0（右转）
        do_reset;
        set_pt(0, 0, -1000); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        chk_int("g2_right_state_align", o_state, S_ALIGN);
        chk_true("g2_right_psi_neg", o_psi < 16'sd0);
        chk_true("g2_right_diff_neg", o_diff < 32'sd0);   // 右转
        chk_int("g2_right_center0", o_center, 32'sd0);    // 原地转，中心速为 0

        // (b) 目标正左 → psi>0 → diff>0（左转）
        do_reset;
        set_pt(0, 0, 1000); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        chk_int("g2_left_state_align", o_state, S_ALIGN);
        chk_true("g2_left_psi_pos", o_psi > 16'sd0);
        chk_true("g2_left_diff_pos", o_diff > 32'sd0);    // 左转

        // (c) 目标正后 → |psi|≈180° → 仍在 ALIGN、diff 满幅 ±612
        do_reset;
        set_pt(0, -1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        chk_int("g2_back_state_align", o_state, S_ALIGN);
        chk_int("g2_back_diff_full", o_diff, 32'sd612);   // psi=+180° → 左转满幅

        // (d) 车头已朝目标（yaw=90° 指向 +y，目标 (0,1000)）→ 直接进 TRACK
        do_reset;
        set_pt(0, 0, 1000); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 9000);
        tick1;
        chk_int("g2_aimed_track", o_state, S_TRACK);
        tick1;                                    // 进 TRACK 的次拍才给巡航速度
        chk_true("g2_aimed_center", o_center > 32'sd800);

        // (e) TRACK 中偏航超过 35° → 回 ALIGN
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;                                    // → TRACK
        chk_int("g2e_track", o_state, S_TRACK);
        i_yaw = 32'sd5000;                        // 车头偏 50° > 35°
        tick1;
        chk_int("g2e_back_align", o_state, S_ALIGN);
        chk_int("g2e_back_center0", o_center, 32'sd0);    // ALIGN 不给前进速度

        // ==================================================================
        // G3 L1 到点链路：TRACK → HOLD →（有下一点）ALIGN /（最后一点）DONE
        // ==================================================================
        // (a) 单点：到点 → HOLD → 停稳 → DONE，done 只一拍
        do_reset;
        set_pt(0, 500, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;                                    // → TRACK
        tick1;                                    // 巡航
        chk_int("g3a_track", o_state, S_TRACK);
        chk_true("g3a_center", o_center > 32'sd800);      // 500mm 远 → 全速
        i_x = 32'sd500; i_v = 32'sd0;
        tick1;
        chk_int("g3a_hold", o_state, S_HOLD);
        chk_int("g3a_hold_center", o_center, 32'sd0);
        chk_int("g3a_hold_diff", o_diff, 32'sd0);
        chk_int("g3a_valid_hold", {31'd0, o_auto_valid}, 32'sd1);  // 停车也要 valid
        repeat (11) tick1;
        chk_int("g3a_done", o_state, S_DONE);
        chk_int("g3a_done_cnt", done_cnt, 1);     // 只报一次
        chk_int("g3a_done_valid", {31'd0, o_auto_valid}, 32'sd0);

        // (b) 两点：第一点到点 → HOLD → ALIGN（不报 done）
        do_reset;
        set_pt(0, 500, 0); set_pt(1, 1000, 0);
        i_pt_count = 5'd2;
        start_task(2, 0, 0, 0, 0);
        tick1; tick1;
        i_x = 32'sd500;
        tick1;
        chk_int("g3b_hold1", o_state, S_HOLD);
        chk_true("g3b_not_done", !done_seen);
        repeat (11) tick1;
        chk_int("g3b_align2", o_state, S_ALIGN);   // 切第二点后重新对准
        chk_true("g3b_still_not_done", !done_seen);
        tick1;                                     // ALIGN → TRACK（第二点正前方）
        tick1;                                     // 巡航；o_idx 打拍后可见
        chk_int("g3b_idx1", {27'd0, o_idx}, 32'sd1);
        chk_int("g3b_track2", o_state, S_TRACK);

        // (c) 两点走完 → done
        i_x = 32'sd1000;
        tick1;                                     // 到点
        chk_int("g3c_hold2", o_state, S_HOLD);
        repeat (11) tick1;
        chk_int("g3c_done", o_state, S_DONE);
        chk_int("g3c_done_cnt", done_cnt, 1);

        // ==================================================================
        // G4 L2 非间断：切点 / 不切三种降级
        // ==================================================================
        // (a) 共线、点距 1000 → 在 90 mm 处切点，不停车
        do_reset;
        set_pt(0, 1000, 0); set_pt(1, 2000, 0);
        i_pt_count = 5'd2;
        start_task(2, 1, 950, 0, 0);
        tick1;                                     // ALIGN → TRACK
        tick1;                                     // 切点
        tick1;                                     // o_idx 打拍后可见
        chk_int("g4a_cut_idx", {27'd0, o_idx}, 32'sd1);
        chk_int("g4a_cut_state", o_state, S_TRACK);         // 不停车
        chk_true("g4a_cut_center", o_center > 32'sd0);

        // (b) 转角 90° → 不切，进 HOLD
        do_reset;
        set_pt(0, 1000, 0); set_pt(1, 1000, 1000);
        i_pt_count = 5'd2;
        start_task(2, 1, 980, 0, 0);
        tick1; tick1;
        chk_int("g4b_nocut_idx", {27'd0, o_idx}, 32'sd0);
        chk_int("g4b_nocut_hold", o_state, S_HOLD);

        // (c) 点距不足 260 mm（1000→1100）→ 不切，进 HOLD
        do_reset;
        set_pt(0, 1000, 0); set_pt(1, 1100, 0);
        i_pt_count = 5'd2;
        start_task(2, 1, 980, 0, 0);
        tick1; tick1;
        chk_int("g4c_short_idx", {27'd0, o_idx}, 32'sd0);
        chk_int("g4c_short_hold", o_state, S_HOLD);

        // (d) 只有一点 → L2 无下一点，按 L1 处理
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 1, 980, 0, 0);
        tick1; tick1;
        chk_int("g4d_single_hold", o_state, S_HOLD);

        // (e) 同一场景把 mode 从 1 切回 0（运行中切模式）：L1 直接到点
        do_reset;
        set_pt(0, 1000, 0); set_pt(1, 2000, 0);
        i_pt_count = 5'd2;
        start_task(2, 1, 950, 0, 0);
        tick1;                                     // → TRACK
        i_mode = 1'b0;                             // 切成间断
        tick1;
        chk_int("g4e_mode_switch_nocut", {27'd0, o_idx}, 32'sd0);
        chk_int("g4e_mode_switch_track", o_state, S_TRACK);

        // ==================================================================
        // G5 暂停 / 恢复：在 ALIGN、TRACK、HOLD、DONE 四态分别暂停
        //    ★ UI 侧暂停时 o_nav_run 也是 0，所以必须给 i_run=0 & i_paused=1
        // ==================================================================
        // (a) TRACK 中暂停 → valid=1、center=diff=0、state 保持
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1; tick1;
        chk_int("g5a_track", o_state, S_TRACK);
        chk_true("g5a_center_on", o_center > 32'sd800);
        i_run = 1'b0; i_paused = 1'b1;
        repeat (3) @(posedge clk);
        chk_int("g5a_pause_valid", {31'd0, o_auto_valid}, 32'sd1);   // 主动制动
        chk_int("g5a_pause_center", o_center, 32'sd0);
        chk_int("g5a_pause_diff", o_diff, 32'sd0);
        chk_int("g5a_pause_state", o_state, S_TRACK);
        // 暂停期间来 tick 也不该动
        tick1;
        chk_int("g5a_pause_tick_center", o_center, 32'sd0);
        // 恢复
        i_paused = 1'b0; i_run = 1'b1;
        tick1; tick1;
        chk_int("g5a_resume_track", o_state, S_TRACK);
        chk_true("g5a_resume_center", o_center > 32'sd800);

        // (b) ALIGN 中暂停 → 也是 center/diff 全 0（不能继续原地转）
        do_reset;
        set_pt(0, 0, 1000); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        chk_int("g5b_align", o_state, S_ALIGN);
        i_run = 1'b0; i_paused = 1'b1;
        repeat (3) @(posedge clk);
        chk_int("g5b_pause_diff", o_diff, 32'sd0);       // ★ 原地转也必须停
        chk_int("g5b_pause_center", o_center, 32'sd0);

        // (c) HOLD 中暂停
        do_reset;
        set_pt(0, 500, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1; tick1;
        i_x = 32'sd500; tick1;
        chk_int("g5c_hold", o_state, S_HOLD);
        i_run = 1'b0; i_paused = 1'b1;
        repeat (3) @(posedge clk);
        chk_int("g5c_pause_valid", {31'd0, o_auto_valid}, 32'sd1);
        chk_int("g5c_pause_center", o_center, 32'sd0);
        // 恢复后仍应完成
        i_paused = 1'b0; i_run = 1'b1;
        repeat (12) tick1;
        chk_int("g5c_resume_done", o_state, S_DONE);

        // (d) 暂停中退出（i_active=0）→ valid 归 0
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1; tick1;
        i_run = 1'b0; i_paused = 1'b1;
        repeat (3) @(posedge clk);
        i_active = 1'b0;
        repeat (3) @(posedge clk);
        chk_int("g5d_exit_valid", {31'd0, o_auto_valid}, 32'sd0);
        chk_int("g5d_exit_state", o_state, S_IDLE);

        // ==================================================================
        // G6 故障：堵转 / 超时
        // ==================================================================
        // (a) 堵转 → FAULT、valid=0、fault 单拍
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        i_stall = 4'd1;
        tick1;
        chk_int("g6a_fault_state", o_state, S_FAULT);
        chk_int("g6a_fault_valid", {31'd0, o_auto_valid}, 32'sd0);
        chk_int("g6a_fault_seen", {31'd0, fault_seen}, 32'sd1);
        // FAULT 后再来 tick 仍保持 FAULT（不自动复活）
        tick1;
        chk_int("g6a_fault_sticky", o_state, S_FAULT);

        // (b) 超时（u_fast：TIMEOUT_TICKS=12）→ FAULT
        do_reset;
        set_pt(0, 5000, 0); i_pt_count = 5'd1;   // 车永远到不了
        start_task(1, 0, 0, 0, 0);
        repeat (15) tick1;
        chk_int("g6b_timeout_fast", f_state, S_FAULT);
        chk_int("g6b_timeout_valid", {31'd0, f_valid}, 32'sd0);

        // ==================================================================
        // G7 避障页：i_is_obs=1 时目标取准心，点表被忽略
        // ==================================================================
        do_reset;
        set_pt(0, 5000, 5000);                   // 点表里有个远点，应被忽略
        i_pt_count = 5'd1;
        i_cur_x = 32'sd800; i_cur_y = 32'sd0;    // 准心在正前方 800
        i_is_obs = 1'b1;
        start_task(1, 0, 0, 0, 0);
        tick1;
        chk_int("g7_obs_track", o_state, S_TRACK);
        chk_int("g7_obs_ahead", o_ahead, 32'sd799);   // 用准心 800，不是 5000
        // 准心移动到车后 → 应回 ALIGN
        i_cur_x = -32'sd800; i_cur_y = 32'sd0;
        tick1;
        chk_int("g7_obs_align", o_state, S_ALIGN);

        // ==================================================================
        // G8 点表为空（i_pt_count=0）：pt_cnt 强制 1，目标 = 原点
        // ==================================================================
        do_reset;
        i_pt_count = 5'd0;                       // 没有任何点
        start_task(0, 0, 0, 0, 0);               // 车就在原点
        tick1;                                    // ALIGN（psi=0）→ TRACK
        tick1;                                    // 到点判定
        chk_int("g8_empty_hold", o_state, S_HOLD);   // 距离 0 → 直接到点
        repeat (11) tick1;
        chk_int("g8_empty_done", o_state, S_DONE);

        // ==================================================================
        // G9 三段全流程（混合模式：L2 切一次、L1 停一次、最后 done）
        // ==================================================================
        do_reset;
        set_pt(0, 1000, 0); set_pt(1, 2000, 0); set_pt(2, 2000, 1000);
        i_pt_count = 5'd3;
        start_task(3, 1, 0, 0, 0);                // 非间断
        tick1; tick1;                             // → TRACK
        i_x = 32'sd950;                           // 进入 90 mm 圈，共线可切
        tick1; tick1;
        chk_int("g9_cut_to_1", {27'd0, o_idx}, 32'sd1);
        chk_int("g9_still_track", o_state, S_TRACK);
        chk_true("g9_not_done", !done_seen);
        // 第二段末端是 90° 转角 → 不切，到点停车
        i_x = 32'sd1980; i_y = 32'sd0;
        tick1;
        chk_int("g9_hold2", o_state, S_HOLD);
        repeat (11) tick1;
        chk_int("g9_align3", o_state, S_ALIGN);
        tick1;                                    // ALIGN → TRACK；o_idx 打拍后可见
        chk_int("g9_idx2", {27'd0, o_idx}, 32'sd2);
        chk_true("g9_not_done2", !done_seen);
        // 第三点：原地转过去再走
        tick1;                                    // 巡航一拍
        i_yaw = 32'sd9000;                        // 车头朝 +y
        tick1; tick1;
        i_y = 32'sd1000;
        tick1;
        chk_int("g9_hold3", o_state, S_HOLD);
        repeat (11) tick1;
        chk_int("g9_done", o_state, S_DONE);
        chk_int("g9_done_cnt", done_cnt, 1);

        // ==================================================================
        // G10 零向量 / 极近目标（防 arctan 抖动）
        // ==================================================================
        // 车正好在点上（dist=0）→ 直接 HOLD，psi 不抖
        do_reset;
        set_pt(0, 300, 400); i_pt_count = 5'd1;
        start_task(1, 0, 300, 400, 0);
        tick1;                                    // ALIGN（零向量 psi=0）→ TRACK
        tick1;                                    // 到点判定
        chk_int("g10_at_point_hold", o_state, S_HOLD);
        chk_int("g10_at_point_psi", o_psi, 16'sd0);      // 零向量 → psi=0

        // ==================================================================
        // G11 三个可调参数的边界（2026-10-20 调参后补）
        //   参数现值：ALIGN_IN_DEG=1000(10°) / D_SLOW_MM=100 / MAX_DIFF_CRUISE=180
        //   ★ 这组是"改坏参数就报错"的守卫：若恢复旧值（3000 / 130 / 180）会失败
        // ==================================================================
        // (a) ALIGN_IN_DEG = 10°：偏 20° 必须留在 ALIGN（旧值 30° 会误进 TRACK）
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        i_yaw = -32'sd2000;                        // 车头偏 -20° → psi = +20°
        tick1;
        chk_int("g11a_20deg_align", o_state, S_ALIGN);
        chk_int("g11a_20deg_center0", o_center, 32'sd0);
        chk_true("g11a_20deg_diff_pos", o_diff > 32'sd0);   // psi>0 → 左转

        // (b) 偏 8° → 必须进 TRACK（证明门限确实在 10°，不是更小）
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        i_yaw = -32'sd800;                         // psi = +8° < 10°
        tick1;
        chk_int("g11b_8deg_track", o_state, S_TRACK);

        // (c) MAX_DIFF_CRUISE = 180：TRACK 中 psi=30° → 差速截到 180（若改回 252 这里会得 252）
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;                                     // psi=0 → 进 TRACK
        chk_int("g11c_pre_track", o_state, S_TRACK);
        i_yaw = -32'sd3000;                        // psi = +30°（<35° 迟滞上限，留 TRACK）
        tick1;
        chk_int("g11c_still_track", o_state, S_TRACK);
        chk_int("g11c_diff_clip", o_diff, 32'sd180);       // 42.94 °/s
        chk_true("g11c_center_cruise", o_center > 32'sd800);

        // (d) 同一 psi=30° 但在 ALIGN 下用 MAX_DIFF=612 → 不会被截到 180
        do_reset;
        set_pt(0, 1000, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        i_yaw = -32'sd3000;                        // 一开始就偏 30° → 留 ALIGN
        tick1;
        chk_int("g11d_align_state", o_state, S_ALIGN);
        chk_true("g11d_align_above_cruise", o_diff > 32'sd180);  // 612 限幅有余量

        // (e) D_SLOW_MM = 100：ahead=110 → 全速 892（旧值 130 会算出 754）
        do_reset;
        set_pt(0, 110, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1;                                     // → TRACK
        tick1;                                     // 巡航
        chk_int("g11e_far_full", o_center, 32'sd892);

        // (f) ahead=95 < 100 → 已进减速段（小于全速但大于 0）
        do_reset;
        set_pt(0, 95, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1; tick1;
        chk_true("g11f_slow_lt_full", o_center < 32'sd892);
        chk_true("g11f_slow_gt_zero", o_center > 32'sd0);

        // (g) 减速斜率 K_SLOW_Q10 = 3072（= 300/100 × 1024 派生）：ahead=50 → 150 mm/s
        //     ★ 理论 446 counts，实测 437。差 2% 来自 sin_lut_rom 的 Q15 量化：
        //       cos(0°) 存的是 32767 而非 32768，使 50 mm 的 ahead 量化为 49 mm
        //       （49×3072=150528 → ×3046>>20 = 437）。故用区间断言守住量级，
        //       不锁死具体值。旧 K_SLOW_Q10=2363 时会得 336，落在区间外。
        do_reset;
        set_pt(0, 50, 0); i_pt_count = 5'd1;
        start_task(1, 0, 0, 0, 0);
        tick1; tick1;
        chk_true("g11g_slope_50mm", (o_center >= 32'sd425) && (o_center <= 32'sd455));

        // ==================================================================
        // G12 运行时改参数立即生效（2026-10-20 参数端口化后新增）
        //   ★ 目的：证明这 9 项参数是**寄存器**而不是编译期常量 —— 若任何一项
        //     还被综合成常量（例如把端口优化回 parameter 默认值），下面每条都会
        //     失败。这是"参数整定页真的改到控制环"的唯一仿真证据。
        //   每条测完立刻恢复默认值（本组在最后，但保持这个习惯）。
        // ==================================================================
        $display("-- G12 运行时改参数 --");
        begin : g12_runtime
            integer d21, d42;

            // (a) 航向 KP 21 → 42：ALIGN 中 psi=20°，比例项应翻倍
            //     用"差值"而不是绝对值断言，避开 i/d 项初值的影响
            do_reset;
            set_pt(0, 1000, 0); i_pt_count = 5'd1;
            start_task(1, 0, 0, 0, 0);
            i_yaw = -32'sd2000;
            tick1;
            d21 = o_diff;
            P_KP = 32'sd42;
            tick1; tick1;
            d42 = o_diff;
            P_KP = 32'sd21;
            chk_true("g12a_kp_changed_diff_grows", d42 > (d21 + 32'sd100));

            // (b) MAX_DIFF_CRUISE 180 → 90：TRACK 中 psi=30°，差速按新限幅截断
            do_reset;
            set_pt(0, 1000, 0); i_pt_count = 5'd1;
            start_task(1, 0, 0, 0, 0);
            tick1;
            i_yaw = -32'sd3000;
            tick1;
            chk_int("g12b_mdc180", o_diff, 32'sd180);
            P_MDC = 32'sd90;
            tick1; tick1;
            chk_int("g12b_mdc90", o_diff, 32'sd90);
            P_MDC = 32'sd180;

            // (c) D_SLOW 100 → 200（同时把 K_SLOW 调到 1024）：ahead=150
            //     默认：150 ≥ 100 → 全速 892
            //     改后：150 < 200 → v = 150 mm/s ≈ 446 counts
            do_reset;
            set_pt(0, 150, 0); i_pt_count = 5'd1;
            start_task(1, 0, 0, 0, 0);
            tick1; tick1;
            chk_int("g12c_dslow100_full", o_center, 32'sd892);
            P_D_SLOW = 32'sd200; P_KSLOW = 32'sd1024;
            tick1; tick1;
            chk_true("g12c_dslow200_slow",
                     (o_center >= 32'sd440) && (o_center <= 32'sd452));
            P_D_SLOW = 32'sd100; P_KSLOW = 32'sd3072;

            // (d) ALIGN_IN 1000 → 2000：同样 psi=15°，改前进 ALIGN、改后进 TRACK
            do_reset;
            set_pt(0, 1000, 0); i_pt_count = 5'd1;
            start_task(1, 0, 0, 0, 0);
            i_yaw = -32'sd1500;                    // psi = +15°
            tick1;
            chk_int("g12d_in1000_align", o_state, S_ALIGN);

            do_reset;
            set_pt(0, 1000, 0); i_pt_count = 5'd1;
            start_task(1, 0, 0, 0, 0);
            P_ALIGN_IN = 32'sd2000;
            i_yaw = -32'sd1500;
            tick1; tick1;
            chk_int("g12d_in2000_track", o_state, S_TRACK);
            P_ALIGN_IN = 32'sd1000;
        end
        $display("=== nav_ctrl_tb: %0d checks, %0d errors ===", checks, errors);
        if (errors != 0) $display("FAIL: %0d / %0d 项未通过", errors, checks);
        else             $display("PASS: nav_ctrl_tb %0d 项全过", checks);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
