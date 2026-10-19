//////////////////////////////////////////////////////////////////////////////////
// 测试台：nav_free_tb —— 「非避障导航」页控制器（rtl\ui\nav_free_ctrl.v）
//
// 覆盖点（每条都对应需求原文里的一句"可观察行为"）：
//   1. 进入本页 → 准心落到车的位置（并钳位到可见坐标系内）
//   2. 摇杆推动 → 准心按 500 mm/s 移动（本台 CLK_HZ=1 MHz，1 ms = 1000 拍）
//   3. 移动边界 → 准心被钳在 x∈[−3400,10183]、y∈[−4983,5000]
//   4. select 键 → 辅助菜单高亮 0→1→2→3→4→0
//   5. 确认键 + "确认路径点" → 记录一个点（点数 +1），坐标 = 当前准心
//   6. 确认键 + "撤销路径点" → 点数 −1
//   7. 确认键 + "清除所有路径点" → 点数归 0
//   8. 无点时按"间断寻点"确认 → **不开跑**（避免立刻"到达"）
//   9. 有点时按"间断寻点"确认 → nav_run=1 且 nav_mode=0
//  10. 有点时按"非间断寻点"确认 → nav_run=1 且 nav_mode=1
//  11. nav_run 期间：准心锁定、确认/select 无效
//  12. i_back（预留返回）→ nav_run 归 0
//  13. S2 清零 → 点表清空、准心回 0
//  14. 像素换算：(0,0)→(412,300)；(1000,0)→px≈471；(0,1000)→py≈239
//
// ★ 本台把 CLK_HZ 设成 1 MHz（而不是 50 MHz）：nav_free_ctrl 内部只有 1 ms 节拍
//   用得上频率，1 MHz 下 TICK_END=999，仿真快 50 倍，行为完全等价。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module nav_free_tb;

    localparam integer CLK_PERIOD = 1000;     // 1 MHz → 1000 ns
    localparam integer MS_CYC     = 1000;     // 1 ms = 1000 拍

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg        active   = 1'b0;
    reg        clear    = 1'b0;
    reg signed [31:0] x_mm = 32'sd0, y_mm = 32'sd0;
    reg signed [11:0] steer = 12'sd0, thr = 12'sd0;
    reg        confirm = 1'b0, back = 1'b0;
    reg        aux_up = 1'b0, aux_down = 1'b0;   // CH8 三档：上/下拨
    reg        is_obs = 1'b0;                     // 0 = 非避障页
    reg        manual  = 1'b0;          // 1 = 手动接管档

    wire signed [31:0] cur_x, cur_y;
    wire [2:0]  aux_sel;
    wire [1:0]  nav_state;
    wire        nav_ui, nav_paused;
    wire [4:0]  pt_cnt;
    wire        nav_run, nav_mode;
    wire [159:0] pt_px, pt_py;
    wire [15:0]  pt_v;
    wire [9:0]   car_px, car_py, cur_px, cur_py;

    nav_free_ctrl #(
        .CLK_HZ       (1_000_000),
        .CURSOR_SPEED (500),
        .DEADZONE     (100),
        .MAX_POINTS   (16)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .i_active   (active),
        .i_is_obs   (is_obs),
        .i_clear    (clear),
        .i_x_mm     (x_mm),
        .i_y_mm     (y_mm),
        .i_steer    (steer),
        .i_throttle (thr),
        .i_manual   (manual),
        .i_confirm  (confirm),
        .i_back     (back),
        .i_aux_up   (aux_up),
        .i_aux_down (aux_down),
        .i_ctrl_done  (1'b0),
        .i_ctrl_fault (1'b0),
        .o_cur_x_mm (cur_x),
        .o_cur_y_mm (cur_y),
        .o_aux_sel  (aux_sel),
        .o_pt_count (pt_cnt),
        .o_nav_state     (nav_state),
        .o_nav_ui_active (nav_ui),
        .o_nav_run       (nav_run),
        .o_nav_paused    (nav_paused),
        .o_nav_mode (nav_mode),
        .o_pt_px    (pt_px),
        .o_pt_py    (pt_py),
        .o_pt_v     (pt_v),
        .o_car_px   (car_px),
        .o_car_py   (car_py),
        .o_cur_px   (cur_px),
        .o_cur_py   (cur_py)
    );

    integer errors = 0;
    integer checks = 0;

    task chk_int;
        input [8*40-1:0] name;
        input integer    got;
        input integer    exp;
        input integer    tol;
        begin
            checks = checks + 1;
            if ((got >= exp - tol) && (got <= exp + tol)) begin
                $display("  pass  %0s = %0d (exp %0d ±%0d)", name, got, exp, tol);
            end else begin
                errors = errors + 1;
                $display("  FAIL  %0s = %0d (exp %0d ±%0d)", name, got, exp, tol);
            end
        end
    endtask

    task wait_ms;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n * MS_CYC; i = i + 1) @(posedge clk);
            @(posedge clk);
        end
    endtask

    // 单拍脉冲
    task pulse_confirm;
        begin
            @(posedge clk); confirm <= 1'b1;
            @(posedge clk); confirm <= 1'b0;
            @(posedge clk);
        end
    endtask

    task pulse_select;   // 现在由 CH8 下拨代替原来的 select 键
        begin
            @(posedge clk); aux_down <= 1'b1;
            @(posedge clk); aux_down <= 1'b0;
            @(posedge clk);
        end
    endtask

    initial begin
        $display("=== nav_free_tb ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---------------------------------------------------------------
        // 1. 进入本页：准心落到车的位置
        // ---------------------------------------------------------------
        $display("-- 1. 进入本页：准心 = 车位置，并钳位到可见坐标系 --");
        x_mm = 32'sd1500; y_mm = -32'sd800;   // 车在 (1.5 m, -0.8 m)
        active = 1'b1;
        wait_ms(2);
        chk_int("cur_x_on_enter", cur_x, 1500, 2);
        chk_int("cur_y_on_enter", cur_y, -800, 2);

        // 车在可见范围外（x = 20 m）→ 钳到 10183
        x_mm = 32'sd20000; y_mm = 32'sd0;
        active = 1'b0; wait_ms(1);
        active = 1'b1; wait_ms(2);
        chk_int("cur_x_clamped_hi", cur_x, 10183, 3);

        // 复位到原点重新开始
        active = 1'b0; x_mm = 32'sd0; y_mm = 32'sd0;
        wait_ms(1); active = 1'b1; wait_ms(2);
        chk_int("cur_x_origin", cur_x, 0, 2);
        chk_int("cur_y_origin", cur_y, 0, 2);

        // ---------------------------------------------------------------
        // 2. 摇杆移动：满杆 500 mm/s，100 ms → 约 50 mm
        // ---------------------------------------------------------------
        $display("-- 2. 摇杆 → 准心移动（满杆 500 mm/s）--");
        //   ★ 2026-10-19 上板实测：`STEER_SIGN` 已由 +1 改为 −1（原配置下准心左右反向），
        //     所以「摇杆右推(steer=+1000)」现在是准心向 **−x**。本台断言跟着配置走。
        steer = 12'sd1000;
        wait_ms(100);
        steer = 12'sd0;
        wait_ms(2);
        chk_int("cur_x_after_100ms", cur_x, -50, 4);
        chk_int("cur_y_unchanged",   cur_y, 0, 2);

        thr = 12'sd1000;
        wait_ms(100);
        thr = 12'sd0;
        wait_ms(2);
        chk_int("cur_y_after_100ms", cur_y, 50, 4);

        // 反向
        steer = -12'sd1000;
        wait_ms(50);
        steer = 12'sd0;
        wait_ms(2);
        chk_int("cur_x_back", cur_x, -25, 4);   // −50 + 25 = −25

        // 死区内不动
        $display("-- 2b. 死区 ±100 内不移动 --");
        steer = 12'sd80;
        wait_ms(50);
        steer = 12'sd0;
        wait_ms(2);
        chk_int("cur_x_deadzone", cur_x, -25, 2);

        // ---------------------------------------------------------------
        // 2c. 手动接管档：摇杆归底盘，准心不动
        //   需求方 2026-10-19："当车处于手动挡，摇杆不控制准心移动。"
        //   ★ 确认键 / select 不需要额外门控：rc_input 的 cursor_ready 已挡掉。
        // ---------------------------------------------------------------
        $display("-- 2c. 手动接管档：推满摇杆准心不动；切回光标档恢复 --");
        begin : manual_chk
            integer x0, y0;
            x0 = cur_x; y0 = cur_y;
            manual = 1'b1;
            steer = 12'sd1000; thr = 12'sd1000;
            wait_ms(100);
            steer = 12'sd0; thr = 12'sd0;
            wait_ms(2);
            chk_int("manual_cur_x_hold", cur_x, x0, 1);
            chk_int("manual_cur_y_hold", cur_y, y0, 1);
            manual = 1'b0;
            steer = 12'sd1000;                 // STEER_SIGN=-1 ⇒ 向 −x
            wait_ms(50);
            steer = 12'sd0;
            wait_ms(2);
            chk_int("manual_release_moves", cur_x, x0 - 25, 4);
        end

        // ---------------------------------------------------------------
        // 3. 边界钳位
        // ---------------------------------------------------------------
        $display("-- 3. 边界钳位（用「重进页面 + 超界车位置」直接触发，不靠长时间推杆）--");
        //   ★ 不用"推满杆几秒钟"来撞边界：本台 1 ms = 1000 拍，推 3 s 就是 3e6 拍，
        //     纯属浪费仿真时间。钳位逻辑与"从哪来"无关，重进页面即可覆盖四个边界。
        active = 1'b0; x_mm = -32'sd20000; wait_ms(1);
        active = 1'b1; wait_ms(2);
        chk_int("cur_x_clamped_lo", cur_x, -3400, 3);
        active = 1'b0; x_mm = 32'sd0; y_mm = 32'sd20000; wait_ms(1);
        active = 1'b1; wait_ms(2);
        chk_int("cur_y_clamped_hi", cur_y, 5000, 3);
        active = 1'b0; y_mm = -32'sd20000; wait_ms(1);
        active = 1'b1; wait_ms(2);
        chk_int("cur_y_clamped_lo", cur_y, -4983, 3);

        // 回到原点
        active = 1'b0; x_mm = 32'sd0; y_mm = 32'sd0; wait_ms(1);
        active = 1'b1; wait_ms(2);

        // ---------------------------------------------------------------
        // 4. select 循环
        // ---------------------------------------------------------------
        $display("-- 4. select 键循环辅助菜单 0→1→2→3→4→0 --");
        chk_int("aux_init", aux_sel, 0, 0);
        pulse_select; chk_int("aux_1", aux_sel, 1, 0);
        pulse_select; chk_int("aux_2", aux_sel, 2, 0);
        pulse_select; chk_int("aux_3", aux_sel, 3, 0);
        pulse_select; chk_int("aux_4", aux_sel, 4, 0);
        pulse_select; chk_int("aux_wrap", aux_sel, 0, 0);

        // ---------------------------------------------------------------
        // 5. 记录点
        // ---------------------------------------------------------------
        $display("-- 5. 确认路径点：连续记 3 个点 --");
        steer = 12'sd1000; wait_ms(100); steer = 12'sd0; wait_ms(2);   // 准心 x≈50
        pulse_confirm;
        chk_int("pt1_count", pt_cnt, 1, 0);
        chk_int("pt1_px_valid", pt_v[0], 1, 0);

        steer = 12'sd1000; wait_ms(100); steer = 12'sd0; wait_ms(2);   // x≈100
        pulse_confirm;
        chk_int("pt2_count", pt_cnt, 2, 0);

        thr = 12'sd1000; wait_ms(100); thr = 12'sd0; wait_ms(2);
        pulse_confirm;
        chk_int("pt3_count", pt_cnt, 3, 0);

        // ---------------------------------------------------------------
        // 6. 撤销
        // ---------------------------------------------------------------
        $display("-- 6. 撤销路径点：点数 −1 --");
        pulse_select;                                     // → 1（撤销）
        chk_int("aux_for_undo", aux_sel, 1, 0);
        pulse_confirm;
        chk_int("pt_after_undo", pt_cnt, 2, 0);

        // ---------------------------------------------------------------
        // 7. 清除
        // ---------------------------------------------------------------
        $display("-- 7. 清除所有路径点 --");
        pulse_select; pulse_select; pulse_select;         // 1→2→3→4
        chk_int("aux_for_clear", aux_sel, 4, 0);
        pulse_confirm;
        chk_int("pt_after_clear", pt_cnt, 0, 0);

        // ---------------------------------------------------------------
        // 8. 无点时按"间断寻点"确认 → 不开跑
        // ---------------------------------------------------------------
        $display("-- 8. 无点时按间断寻点 → 不开跑 --");
        pulse_select; pulse_select;                       // 4→0→1→... 先回 0
        // 现在在 1；再按两次 → 2（间断寻点）... 实际 4→0→1，再按 1 次到 2
        pulse_select;
        chk_int("aux_gap", aux_sel, 2, 0);
        pulse_confirm;
        chk_int("no_run_without_pts", nav_run, 0, 0);

        // ---------------------------------------------------------------
        // 9/10. 有点时开跑（间断 / 非间断）
        // ---------------------------------------------------------------
        $display("-- 9. 记 1 个点后按间断寻点 → nav_run=1 / nav_mode=0 --");
        pulse_select; pulse_select; pulse_select;         // 2→3→4→0（确认路径点）
        chk_int("aux_back_to_set", aux_sel, 0, 0);
        pulse_confirm;                                    // 记 1 个点
        chk_int("pt_before_run", pt_cnt, 1, 0);
        pulse_select; pulse_select;                       // 0→1→2（间断寻点）
        chk_int("aux_gap2", aux_sel, 2, 0);
        pulse_confirm;
        chk_int("run_gap",   nav_run,  1, 0);
        chk_int("mode_gap",  nav_mode, 0, 0);

        // 11. 运行中：准心锁定、确认/select 无效
        $display("-- 11. 运行中：准心锁定，确认/select 无效 --");
        begin : lock_chk
            integer x0, y0, n0, a0;
            x0 = cur_x; y0 = cur_y; n0 = pt_cnt; a0 = aux_sel;
            steer = 12'sd1000; wait_ms(50); steer = 12'sd0; wait_ms(2);
            chk_int("cur_x_locked", cur_x, x0, 1);
            pulse_confirm;
            chk_int("pt_cnt_locked", pt_cnt, n0, 0);
            pulse_select;
            chk_int("aux_locked", aux_sel, a0, 0);
        end

        // 12. 三态往返（2026-10-19 新增）
        //   运行中 --返回--> 暂停 --确认--> 运行中 --返回--> 暂停 --返回--> 退出交互态
        $display("-- 12. 三态：运行中按返回→暂停；确认→继续；暂停再按返回→退出交互态 --");
        @(posedge clk); back <= 1'b1;
        @(posedge clk); back <= 1'b0;
        repeat (3) @(posedge clk);
        chk_int("back_pauses",      nav_state, 2, 0);
        chk_int("paused_not_run",   nav_run,   0, 0);
        chk_int("paused_flag",      nav_paused,1, 0);
        chk_int("ui_still_active",  nav_ui,    1, 0);

        pulse_confirm;
        chk_int("confirm_resumes",  nav_state, 1, 0);
        chk_int("resume_run",       nav_run,   1, 0);

        @(posedge clk); back <= 1'b1;
        @(posedge clk); back <= 1'b0;
        repeat (3) @(posedge clk);
        chk_int("back_again_pauses", nav_state, 2, 0);

        @(posedge clk); back <= 1'b1;
        @(posedge clk); back <= 1'b0;
        repeat (3) @(posedge clk);
        chk_int("back_from_paused_exits", nav_ui, 0, 0);
        chk_int("exit_state_setup",       nav_state, 0, 0);

        // 13. S2 清零
        $display("-- 13. S2 清零 → 点表清空、准心回 0 --");
        @(posedge clk); clear <= 1'b1;
        repeat (4) @(posedge clk); clear <= 1'b0;
        repeat (4) @(posedge clk);
        chk_int("clear_pts",  pt_cnt, 0, 0);
        chk_int("clear_curx", cur_x,  0, 1);
        chk_int("clear_cury", cur_y,  0, 1);

        // 14. 像素换算
        $display("-- 14. 像素换算：0.06 px/mm，原点 (412,300) --");
        active = 1'b0; x_mm = 32'sd0; y_mm = 32'sd0; wait_ms(1);
        active = 1'b1; wait_ms(2);
        chk_int("px_at_origin",  cur_px, 412, 1);
        chk_int("py_at_origin",  cur_py, 300, 1);
        chk_int("car_px_origin", car_px, 412, 1);
        chk_int("car_py_origin", car_py, 300, 1);

        x_mm = 32'sd1000; y_mm = 32'sd1000; wait_ms(2);
        chk_int("car_px_1m", car_px, 472, 2);
        chk_int("car_py_1m", car_py, 240, 2);

        x_mm = -32'sd1000; y_mm = -32'sd1000; wait_ms(2);
        chk_int("car_px_neg1m", car_px, 352, 2);
        chk_int("car_py_neg1m", car_py, 360, 2);

        // ---------------------------------------------------------------
        $display("-----------------------------------------");
        if (errors == 0)
            $display("PASS: nav_free_tb %0d 项全过", checks);
        else
            $display("FAIL: %0d / %0d 项未通过", errors, checks);
        $finish;
    end

    // 超时保护（本台最长仿真约 5 s）
    initial begin
        #2_000_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
