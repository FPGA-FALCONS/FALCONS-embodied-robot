`timescale 1ns/1ps
// ============================================================================
// tune_ctrl_tb.v —— 参数整定页控制器（rtl\ui\tune_ctrl.v）单元测试
//
//  覆盖：复位默认值 / 选行回绕 / 单步调值（必须回中）/ 上下限钳位 /
//        长按连发 / CH5 复位该行 / CH5 上退出 / 手动档禁调 /
//        非激活页与无信号全屏蔽 / 显示值换算 / K_SLOW 除法器重算
//
//  时钟 50 MHz（20 ns）。1 ms = 50000 拍，长按判据 400 ms = 20 000 000 拍。
// ============================================================================
module tune_ctrl_tb;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    reg rst_n   = 1'b0;
    reg i_active = 1'b0;
    reg i_manual = 1'b0;
    reg i_signal = 1'b1;
    reg signed [11:0] i_steer = 12'sd0;
    reg i_confirm = 1'b0, i_back = 1'b0;
reg signed [11:0] i_throttle = 12'sd0;   // CH2：选行（向上 = 行号减 1）

    wire [479:0]       o_param, o_disp;
    wire [3:0]         o_sel;
    wire signed [31:0] o_k_slow_q10;
    wire               o_exit, o_div_busy;

    tune_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .i_active(i_active), .i_manual(i_manual), .i_signal(i_signal),
        .i_steer(i_steer),
        .i_confirm(i_confirm), .i_back(i_back),
        .i_throttle(i_throttle),
        .o_param(o_param), .o_disp(o_disp),
        .o_sel(o_sel), .o_k_slow_q10(o_k_slow_q10),
        .o_exit(o_exit), .o_div_busy(o_div_busy)
    );

    `define P(k) o_param[(k)*32 +: 32]
    `define D(k) o_disp [(k)*32 +: 32]

    integer checks = 0;
    integer errors = 0;

    task chk_int(input [127:0] name, input signed [31:0] got, input signed [31:0] exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %0d, expect %0d", name, got, exp);
            end else begin
                $display("  PASS: %0s", name);
            end
        end
    endtask

    task tick(input integer n);
        begin
            repeat (n) @(posedge clk);
        end
    endtask

    task wait_ms(input integer m);
        begin
            repeat (m * 50000) @(posedge clk);
        end
    endtask

    // 摇杆推一下（物理右 = 加）再回中；中间留有足够拍数让 p[] 更新完成
    //   ★ 极性：`i_steer` 的**正 = 推杆向左**（与 chassis_ctrl 的 man_diff 同号），
    //     所以"物理右推"要给负值。改 RTL 极性时这两个 task 必须同步反号。
    task push_pos;
        begin
            @(posedge clk); #1 i_steer = -12'sd600;
            repeat (5) @(posedge clk);
            #1 i_steer = 12'sd0;
            repeat (5) @(posedge clk);
        end
    endtask

    task push_neg;
        begin
            @(posedge clk); #1 i_steer = 12'sd600;
            repeat (5) @(posedge clk);
            #1 i_steer = 12'sd0;
            repeat (5) @(posedge clk);
        end
    endtask

    // ★ 2026-10-20：选行由 CH8 键改为 CH2 摇杆。
    //   `rc_throttle` 正 = 推杆向上 = 车前进方向 ⇒ 向上 = 行号减 1。
    task pulse_aux_up;              // 选行上移
        begin
            @(posedge clk); #1 i_throttle = 12'sd600;
            repeat (3) @(posedge clk);
            #1 i_throttle = 12'sd0;
            repeat (3) @(posedge clk);
        end
    endtask

    task pulse_aux_down;            // 选行下移
        begin
            @(posedge clk); #1 i_throttle = -12'sd600;
            repeat (3) @(posedge clk);
            #1 i_throttle = 12'sd0;
            repeat (3) @(posedge clk);
        end
    endtask

    task pulse_confirm;
        begin
            @(posedge clk); #1 i_confirm = 1'b1;
            repeat (3) @(posedge clk);
            #1 i_confirm = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    task pulse_back;
        begin
            @(posedge clk); #1 i_back = 1'b1;
            repeat (3) @(posedge clk);
            #1 i_back = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    integer k;
    integer exit_cnt;

    initial begin
        $display("=== tune_ctrl_tb ===");

        // ------------------------------------------------------------------
        // G1 复位默认值
        // ------------------------------------------------------------------
        rst_n = 1'b0;
        tick(20);
        rst_n = 1'b1;
        tick(200);                   // 等首次除法完成

        chk_int("g1_p0_vcruise",  `P(0), 300);
        chk_int("g1_p1_dslow",    `P(1), 100);
        chk_int("g1_p2_align_in", `P(2), 800);
        chk_int("g1_p3_align_out",`P(3), 3500);
        chk_int("g1_p4_mdc",      `P(4), 126);
        chk_int("g1_p5_kp",       `P(5), 40);
        chk_int("g1_p6_ki",       `P(6), 3);
        chk_int("g1_p7_kd",       `P(7), 6);
        // ★ 2026-10-20：积分限幅插到索引 8（航向 Kd 下、速度环 Kp 上），速度环三项后移到 9/10/11
        chk_int("g1_p8_ilimit",   `P(8), 120);   // 积分限幅（需求方指定）
        chk_int("g1_p9_kpspd",    `P(9), 24);    // 2026-10-20 需求方指定（原 0）
        chk_int("g1_p10_kispd",   `P(10), 36);   // 2026-10-20 需求方指定（原 64）
        chk_int("g1_p11_kdspd",   `P(11), 0);
        chk_int("g1_sel0", o_sel, 0);
        chk_int("g1_kslow", o_k_slow_q10, 3072);

        // ------------------------------------------------------------------
        // G2 显示值初值（换算正确性）
        // ------------------------------------------------------------------
        chk_int("g2_d0_vcruise",   `D(0), 300);
        chk_int("g2_d1_dslow",     `D(1), 100);
        chk_int("g2_d2_align_in",  `D(2), 8);      // 800 × 0.01° = 8°
        chk_int("g2_d3_align_out", `D(3), 35);      // 3500 × 0.01° = 35°
        chk_int("g2_d4_mdc",       `D(4), 30);     // 126 cnt → 30.06 → 30 °/s
        chk_int("g2_d5_kp",        `D(5), 40);
        chk_int("g2_d8_ilimit",    `D(8), 120);
        chk_int("g2_d9_kpspd",     `D(9), 24);
        chk_int("g2_d10_kispd",    `D(10), 36);
        chk_int("g2_d11_kdspd",    `D(11), 0);

        // ------------------------------------------------------------------
        // G3 非激活页 / 无信号：一切按键与摇杆都无效
        // ------------------------------------------------------------------
        i_active = 1'b0;
        pulse_aux_down;
        chk_int("g3_inactive_sel", o_sel, 0);
        push_pos;
        chk_int("g3_inactive_p0", `P(0), 300);
        pulse_confirm;
        chk_int("g3_inactive_p0b", `P(0), 300);

        i_active = 1'b1;
        i_signal = 1'b0;
        pulse_aux_down;
        chk_int("g3_nosig_sel", o_sel, 0);
        push_pos;
        chk_int("g3_nosig_p0", `P(0), 300);
        i_signal = 1'b1;

        // ------------------------------------------------------------------
        // G4 选行（CH2 摇杆）：下 0→1…→11→0 回绕；上 0→11
        //   2026-10-20 改：选行由 CH8 键改为 CH2 摇杆，行数 11→12
        // ------------------------------------------------------------------
        pulse_aux_down;
        chk_int("g4_sel1", o_sel, 1);
        for (k = 0; k < 13; k = k + 1) pulse_aux_down;
        chk_int("g4_sel14", o_sel, 14);
        pulse_aux_down;
        chk_int("g4_sel_wrap0", o_sel, 0);
        pulse_aux_up;
        chk_int("g4_sel_wrap14", o_sel, 14);
        pulse_aux_up;
        chk_int("g4_sel13", o_sel, 13);

        // ------------------------------------------------------------------
        // G5 单步调值：必须回中才能再触发（一次 push 只走一步）
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        chk_int("g5_base_sel", o_sel, 0);
        push_pos;
        chk_int("g5_one_step", `P(0), 310);     // 300 + 10
        chk_int("g5_disp",     `D(0), 310);
        push_pos;
        chk_int("g5_two_step", `P(0), 320);
        push_neg;
        chk_int("g5_back",     `P(0), 310);

        // 摇杆停在死区内（|v| ≤ 200）不应触发
        @(posedge clk); #1 i_steer = 12'sd150;
        repeat (10) @(posedge clk);
        #1 i_steer = 12'sd0;
        repeat (5) @(posedge clk);
        chk_int("g5_deadzone", `P(0), 310);

        // ------------------------------------------------------------------
        // G6 上限/下限钳位（选 ALIGN_IN：100..5000，步长 100）
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        for (k = 0; k < 2; k = k + 1) pulse_aux_down;   // o_sel = 2
        chk_int("g6_sel2", o_sel, 2);
        for (k = 0; k < 9; k = k + 1) push_neg;         // 1000 → 100
        chk_int("g6_low", `P(2), 100);
        push_neg;                                        // 已在下限，不再减
        chk_int("g6_low_clamp", `P(2), 100);
        for (k = 0; k < 49; k = k + 1) push_pos;        // 100 → 5000
        chk_int("g6_high", `P(2), 5000);
        push_pos;                                        // 已在上限，不再加
        chk_int("g6_high_clamp", `P(2), 5000);
        chk_int("g6_disp_deg", `D(2), 50);              // 5000 × 0.01° = 50°

        // ------------------------------------------------------------------
        // G7 CH5 下拨 = 当前行恢复默认值
        // ------------------------------------------------------------------
        pulse_confirm;
        chk_int("g7_reset_p2", `P(2), 800);
        chk_int("g7_reset_d2", `D(2), 8);

        // ------------------------------------------------------------------
        // G8 长按连发：400 ms 后每 120 ms 一步
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        chk_int("g8_base", `P(0), 300);
        @(posedge clk); #1 i_steer = -12'sd600;   // 物理右推 = 加
        //   初触发 1 步（300→310）已在推离死区当拍发生，
        //   再等 520 ms 应有 1 次连发（400 ms 到点 + 120 ms 计数满）
        wait_ms(520);
        #1 i_steer = 12'sd0;
        tick(5);
        chk_int("g8_repeat_one", `P(0), 320);

        // ------------------------------------------------------------------
        // G9 CH5 上拨 = 退出页面（o_exit 单拍脉冲）
        // ------------------------------------------------------------------
        exit_cnt = 0;
        @(posedge clk); #1 i_back = 1'b1;
        repeat (10) begin
            @(posedge clk);
            if (o_exit) exit_cnt = exit_cnt + 1;
        end
        #1 i_back = 1'b0;
        repeat (10) begin
            @(posedge clk);
            if (o_exit) exit_cnt = exit_cnt + 1;
        end
        chk_int("g9_exit_pulse", exit_cnt, 1);

        // ------------------------------------------------------------------
        // G10 手动档：CH2/CH4 都归底盘、不选行不改值，确认键也不复位，但能退出
        //   ★ 2026-10-20 行为变更：选行改到 CH2 后，CH2 在手动档是油门，
        //     故选行也受 can_adj 门控（原先 CH8 选行不受手动档限制）。
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        i_manual = 1'b1;
        pulse_aux_down;
        chk_int("g10_sel_blocked", o_sel, 0);   // 手动档下 CH2 不选行
        for (k = 0; k < 5; k = k + 1) push_pos;
        chk_int("g10_no_adjust", `P(1), 100);   // 摇杆归底盘，参数不动
        pulse_confirm;
        chk_int("g10_no_reset", `P(1), 100);    // 确认键也不复位
        exit_cnt = 0;
        @(posedge clk); #1 i_back = 1'b1;
        repeat (8) begin @(posedge clk); if (o_exit) exit_cnt = exit_cnt + 1; end
        #1 i_back = 1'b0;
        repeat (8) begin @(posedge clk); if (o_exit) exit_cnt = exit_cnt + 1; end
        chk_int("g10_exit_ok", exit_cnt, 1);    // 手动档仍能退出
        i_manual = 1'b0;

        // ------------------------------------------------------------------
        // G11 K_SLOW 除法器重算（选 D_SLOW：50..300，步长 5）
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        chk_int("g11_kslow_def", o_k_slow_q10, 3072);       // 300×1024/100
        pulse_aux_down;                                     // o_sel = 1
        chk_int("g11_sel1", o_sel, 1);
        push_pos;                                           // 100 → 105
        tick(100);                                          // 等除法
        chk_int("g11_kslow_105", o_k_slow_q10, 2925);       // 307200/105 = 2925.7 → 2925
        for (k = 0; k < 19; k = k + 1) push_pos;            // 105 → 200
        tick(100);
        chk_int("g11_dslow_200", `P(1), 200);
        chk_int("g11_kslow_200", o_k_slow_q10, 1536);       // 307200/200
        chk_int("g11_disp_dslow", `D(1), 200);

        // ------------------------------------------------------------------
        // G12 MAX_DIFF_CRUISE 显示换算（选 4：30..612，步长 10）
        // ------------------------------------------------------------------
        rst_n = 1'b0; tick(10); rst_n = 1'b1; tick(200);
        for (k = 0; k < 4; k = k + 1) pulse_aux_down;   // o_sel = 4
        chk_int("g12_sel4", o_sel, 4);
        chk_int("g12_mdc_126", `D(4), 30);               // 126×0.33617×2/0.1615 = 30.06 → 30
        push_pos;                                        // 126 → 136
        tick(5);
        chk_int("g12_mdc_136", `P(4), 136);
        chk_int("g12_mdc_136_disp", `D(4), 32);          // 136×0.33617×2/0.1615 = 32.4 → 32

        // 单拍推杆后立即回中，方向必须跟随已产生的事件。
        @(negedge clk); i_steer = -12'sd600;
        @(negedge clk); i_steer = 0;
        tick(5);
        chk_int("g13_short_add", `P(4), 146);
        chk_int("g13_short_disp", `D(4), 35);
        @(negedge clk); i_throttle = 12'sd600;
        @(negedge clk); i_throttle = 0;
        tick(5);
        chk_int("g13_short_up", o_sel, 3);

        // 新增配置：循环选行、调值、恢复默认及上下限。
        rst_n = 0; tick(10); rst_n = 1; tick(200);
        chk_int("front_default", `P(12), 4);
        chk_int("width_default", `P(13), 180);
        for (k = 0; k < 12; k = k + 1) pulse_aux_down;
        chk_int("front_select", o_sel, 12);
        push_pos;
        chk_int("front_add", `P(12), 5);
        chk_int("front_display", `D(12), 5);
        for (k = 0; k < 20; k = k + 1) push_neg;
        chk_int("front_min", `P(12), 1);
        for (k = 0; k < 20; k = k + 1) push_pos;
        chk_int("front_max", `P(12), 16);
        pulse_confirm;
        chk_int("front_restore", `P(12), 4);
        pulse_aux_down;
        push_pos;
        chk_int("width_add", `P(13), 190);
        chk_int("width_display", `D(13), 190);
        for (k = 0; k < 50; k = k + 1) push_neg;
        chk_int("width_min", `P(13), 50);
        for (k = 0; k < 50; k = k + 1) push_pos;
        chk_int("width_max", `P(13), 500);
        pulse_confirm;
        chk_int("width_restore", `P(13), 180);
        i_manual = 1;
        push_pos;
        chk_int("width_manual_lock", `P(13), 180);
        i_manual = 0;
        pulse_aux_down;
        chk_int("window_row", o_sel, 14);
        chk_int("window_default", `P(14), 5);
        chk_int("window_disp", `D(14), 5);
        push_pos;
        chk_int("window_add", `P(14), 6);
        repeat (70) push_neg;
        chk_int("window_min", `P(14), 2);
        repeat (70) push_pos;
        chk_int("window_max", `P(14), 64);
        pulse_confirm;
        chk_int("window_restore", `P(14), 5);

        i_manual = 0;

        // ------------------------------------------------------------------
        $display("");
        // ★ 结论行用 `PASS: <名> <N>` 形式，并且**不打 CHECKS: 行**。
        //   原因：run_sim.ps1 用 `(?m)^\s*#?\s*pass\b`（小写）数实测 pass 行，
        //   而 tb 里逐项行是 `PASS:`（大写）→ passCount 恒为 0；此时它走「兜底」
        //   从结论行取 N 当总数。若同时打 CHECKS: N，就会拿 N 去比 passCount=0 → 判 FAIL。
        if (errors == 0)
            $display("PASS: tune_ctrl_tb %0d checks, 0 errors", checks);
        else
            $display("FAIL: tune_ctrl_tb %0d checks, %0d errors", checks, errors);
        $finish;
    end

    // 超时保护（真实时长上限约 3 s）
    initial begin
        #2000000000;
        $display("TIMEOUT: tune_ctrl_tb");
        $finish;
    end

endmodule
