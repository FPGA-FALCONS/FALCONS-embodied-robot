//////////////////////////////////////////////////////////////////////////////////
// rc_input_tb —— 遥控 6 路输入单元测试
//
// 【为什么用 1 MHz 时钟】
//   rc_pwm_capture 用 CLK_HZ 把拍数换算成 µs，脉宽 1000~2000 µs 在 50 MHz 下要
//   10 万拍/毫秒级的仿真，太慢。把 CLK_HZ 设为 1 MHz（周期 1 µs）后逻辑完全等价，
//   仿真时间缩短 50 倍 —— 这是本工程里"参数化时钟"带来的好处。
//
// 【覆盖】（2026-10-16 更新：模式两档、去掉"返回"通道、新增信号掩码有效性）
//   1. 中位 1500 µs          → steer/throttle = 0
//   2. 端点 2000 / 1000 µs   → +1000 / −1000（含归一化标度 2.125 的误差检查）
//   3. 死区 1500±25 µs 内    → 0
//   4. 模式通道两档          → o_mode = 0（接管）/ 1（光标）
//   5. 按钮通道高位上升沿    → 单周期脉冲（select 与 **confirm** 都测）
//   6. 停止刷新超过超时      → o_signal_valid 拉低、o_mode 退回光标档
//   7. 方向取反参数的对照实例（STEER_REVERSE / THROTTLE_REVERSE）
//   8. 摇杆上下拨动 → 菜单脉冲（恰好一次 / 停住不连翻 / 回中后可再拨）
//   9. ★ 信号有效性掩码 SIG_VALID_MASK：被排除的通道（bit5，J6.6 闲置未接）即使一直无脉冲，
//      也不能把 o_signal_valid 拖成 0 —— 对应"第 3 路预留未接"的现场情况
//      （若不隔离，整车 run_en 会恒 0：车永远不动，且只报"无信号"，无法定位）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module rc_input_tb;

    localparam integer CLK_HZ = 1_000_000;      // 1 MHz → 1 µs/拍（见文件头说明）

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    // ★ 6 路各自用一个 reg，最后拼成向量。
    //   坑：若写成 reg [5:0] rc_pwm 然后让 fork 的 6 个并行进程分别给它的
    //   不同位赋值，ModelSim 下会互相覆盖 —— 实测表现为"所有通道脉宽恒为 0"，
    //   而单个 reg 的最小调试台却完全正常，很容易误判成模块坏了。
    reg         p0 = 1'b0, p1 = 1'b0, p2 = 1'b0, p3 = 1'b0, p4 = 1'b0, p5 = 1'b0;
    wire [5:0]  rc_pwm = {p5, p4, p3, p2, p1, p0};

    wire signed [11:0] steer, throttle;
    wire [1:0]         mode;
    wire               btn_sel, btn_conf;
    wire               menu_up, menu_down;          // 摇杆上下拨动 → 菜单切换脉冲
    wire               sig_ok;
    wire [6*16-1:0]    chan_us;
    wire [5:0]         chan_to, chan_vld;

    integer errors = 0;
    integer i;

    rc_input #(
        .CLK_FREQ_HZ   (CLK_HZ),
        .NUM_CH        (6),
        .CENTER_US     (1500),
        .HALF_RANGE_US (500),
        .DEADZONE_US   (25),
        .MIN_PULSE_US  (900),
        .MAX_PULSE_US  (2100),
        .TIMEOUT_US    (5000),          // 仿真里缩短到 5 ms
        .SW_LO_US      (1300),
        .SW_HI_US      (1700),
        .CH_STEER      (0),
        .CH_THROTTLE   (1),
        .CH_CONFIRM    (2),
        .CH_MODE       (4),
        .CH_SELECT     (3)          // ★ 措施1：bit3（原"返回"键位）= select
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .rc_pwm         (rc_pwm),
        .o_steer        (steer),
        .o_throttle     (throttle),
        .o_mode         (mode),
        .o_btn_select   (btn_sel),
        .o_btn_confirm  (btn_conf),
        .o_menu_up      (menu_up),
        .o_menu_down    (menu_down),
        .o_signal_valid (sig_ok),
        .o_chan_us      (chan_us),
        .o_chan_timeout (chan_to),
        .o_chan_valid   (chan_vld)
    );

    // ------------------------------------------------------------------
    // B1：方向取反参数的【对照实例】
    //   同一组 rc_pwm 输入，只把 STEER_REVERSE / THROTTLE_REVERSE 置 1，
    //   期望两个模拟量的输出符号相反、而模式与按钮完全不受影响。
    //   独立实例的好处：不需要为了测取反而破坏主实例那套已通过的期望值。
    // ------------------------------------------------------------------
    wire signed [11:0] steer_rev, throttle_rev;
    wire [1:0]         mode_rev;
    wire               sel_rev, conf_rev, sig_rev;

    rc_input #(
        .CLK_FREQ_HZ   (CLK_HZ),
        .NUM_CH        (6),
        .CENTER_US     (1500),
        .HALF_RANGE_US (500),
        .DEADZONE_US   (25),
        .MIN_PULSE_US  (900),
        .MAX_PULSE_US  (2100),
        .TIMEOUT_US    (5000),
        .SW_LO_US      (1300),
        .SW_HI_US      (1700),
        .CH_STEER      (0),
        .CH_THROTTLE   (1),
        .CH_CONFIRM    (2),
        .CH_MODE       (4),
        .CH_SELECT     (3),
        .STEER_REVERSE    (1),          // ← 与主实例唯一的差别
        .THROTTLE_REVERSE (1)
    ) dut_rev (
        .clk            (clk),
        .rst_n          (rst_n),
        .rc_pwm         (rc_pwm),
        .o_steer        (steer_rev),
        .o_throttle     (throttle_rev),
        .o_mode         (mode_rev),
        .o_btn_select   (sel_rev),
        .o_btn_confirm  (conf_rev),
        .o_menu_up      (),
        .o_menu_down    (),
        .o_signal_valid (sig_rev),
        .o_chan_us      (),
        .o_chan_timeout (),
        .o_chan_valid   ()
    );

    // ------------------------------------------------------------------
    // ★ 信号有效性掩码对照实例（2026-10-16 新增，验证 A-1 的修法）
    //   与主实例唯一差别：SIG_VALID_MASK 排除 bit5（第 6 路 = J6.6 闲置未接）。
    //   用途：在"bit5 一直无脉冲、其余 5 路正常"的场景下，本实例仍报"信号有效"，
    //         而主实例（6 路全参与）报"无效" —— 两者对比即证明掩码生效。
    //   背景：若不做这个隔离，现场少插一根线就会让 run_en 恒 0（车永远不动）。
    // ------------------------------------------------------------------
    wire sig_mask_ok;

    rc_input #(
        .CLK_FREQ_HZ    (CLK_HZ),
        .NUM_CH         (6),
        .CENTER_US      (1500),
        .HALF_RANGE_US  (500),
        .DEADZONE_US    (25),
        .MIN_PULSE_US   (900),
        .MAX_PULSE_US   (2100),
        .TIMEOUT_US     (5000),
        .SW_LO_US       (1300),
        .SW_HI_US       (1700),
        .CH_STEER       (0),
        .CH_THROTTLE    (1),
        .CH_CONFIRM     (2),
        .CH_MODE        (4),
        .CH_SELECT      (3),
        .SIG_VALID_MASK (6'b011111)     // bit5 = 0：J6.6（闲置未接）不参与"信号有效"判定
    ) dut_mask (
        .clk            (clk),
        .rst_n          (rst_n),
        .rc_pwm         (rc_pwm),
        .o_steer        (),
        .o_throttle     (),
        .o_mode         (),
        .o_btn_select   (),
        .o_btn_confirm  (),
        .o_menu_up      (),
        .o_menu_down    (),
        .o_signal_valid (sig_mask_ok),
        .o_chan_us      (),
        .o_chan_timeout (),
        .o_chan_valid   ()
    );

    always #500 clk = ~clk;             // 1 MHz

    // ------------------------------------------------------------------
    // 一帧：6 路同时拉高，各自按脉宽延迟拉低（µs → ns：timescale 1ns 下 ×1000）
    //   ★ 每个 pN 只在一个 fork 分支里被赋值 → 不存在多驱动覆盖问题
    // ------------------------------------------------------------------
    task send_frame;
        input integer w0, w1, w2, w3, w4, w5;
        begin
            p0 = 1'b1; p1 = 1'b1; p2 = 1'b1; p3 = 1'b1; p4 = 1'b1; p5 = 1'b1;
            fork
                #(w0 * 1000) p0 = 1'b0;
                #(w1 * 1000) p1 = 1'b0;
                #(w2 * 1000) p2 = 1'b0;
                #(w3 * 1000) p3 = 1'b0;
                #(w4 * 1000) p4 = 1'b0;
                #(w5 * 1000) p5 = 1'b0;
            join
            // ★ 帧间隔必须留够：rc_pwm_capture 在下降沿后还要数拍才更新
            //   pulse_width_us / pulse_valid（只等 2 µs 会读到旧值 → 假 FAIL）
            #(10000);                   // 10 µs = 10 拍
        end
    endtask

    // ------------------------------------------------------------------
    // 只刷新 5 路（跳过 bit5 / J6.6）—— 用于"闲置通道未接"场景
    //   p5 恒低：rc_pwm_capture 永远等不到上升沿 → TIMEOUT_US 后 timed_out = 1
    // ------------------------------------------------------------------
    task send_frame_skip5;
        input integer w0, w1, w2, w3, w4;
        begin
            p0 = 1'b1; p1 = 1'b1; p2 = 1'b1; p3 = 1'b1; p4 = 1'b1; p5 = 1'b0;
            fork
                #(w0 * 1000) p0 = 1'b0;
                #(w1 * 1000) p1 = 1'b0;
                #(w2 * 1000) p2 = 1'b0;
                #(w3 * 1000) p3 = 1'b0;
                #(w4 * 1000) p4 = 1'b0;
            join
            #(10000);
        end
    endtask

    // ------------------------------------------------------------------
    // 检查
    // ------------------------------------------------------------------
    function integer absi;
        input integer v;
        begin
            absi = (v < 0) ? -v : v;
        end
    endfunction

    task check_int;
        input [8*40-1:0] name;
        input integer    got;
        input integer    exp;
        input integer    tol;
        begin
            if (absi(got - exp) > tol) begin
                errors = errors + 1;
                $display("  FAIL  %0s: got %0d, expect %0d (±%0d)", name, got, exp, tol);
            end else begin
                $display("  pass  %0s = %0d", name, got);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    initial begin
        $display("======== rc_input 单元测试（6 路遥控捕获）========");

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // ---------------- 1. 全部中位 1500 µs ----------------
        $display("-- 1. 中位 1500 µs：应为 0，模式应落在中位档(1) --");
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        check_int("steer_center",    steer,    0, 1);
        check_int("throttle_center", throttle, 0, 1);
        check_int("mode_center",     mode,     1, 0);
        $display("  DBG frame1: ch0_us=%0d vld0=%b to0=%b | ch1_us=%0d vld1=%b | ch4_us=%0d",
                 chan_us[15:0], chan_vld[0], chan_to[0], chan_us[31:16], chan_vld[1], chan_us[79:64]);
        $display("  DBG frame1: sig_ok=%b steer=%0d throttle=%0d mode=%0d",
                 sig_ok, steer, throttle, mode);

        // ---------------- 2. 端点：2000 / 1000 µs ----------------
        $display("-- 2. 端点：2000 µs → +1000，1000 µs → −1000 --");
        send_frame(2000, 1000, 1500, 1500, 1500, 1500);
        send_frame(2000, 1000, 1500, 1500, 1500, 1500);
        check_int("steer_max_pos",    steer,     1000, 1);
        check_int("throttle_max_neg", throttle, -1000, 1);

        // ---------------- 3. 死区 ----------------
        $display("-- 3. 死区：1500±25 µs 内应为 0 --");
        send_frame(1520, 1480, 1500, 1500, 1500, 1500);
        send_frame(1520, 1480, 1500, 1500, 1500, 1500);
        check_int("steer_deadzone",    steer,    0, 0);
        check_int("throttle_deadzone", throttle, 0, 0);

        // ---------------- 4. 模式两档（2026-10-16：删去"自动"档）----------------
        $display("-- 4. 模式通道两档：1200→0(手动接管) 1500/1800→1(屏幕光标) --");
        send_frame(1500, 1500, 1500, 1500, 1200, 1500);
        send_frame(1500, 1500, 1500, 1500, 1200, 1500);
        check_int("mode_low_manual", mode, 0, 0);
        send_frame(1500, 1500, 1500, 1500, 1800, 1500);
        send_frame(1500, 1500, 1500, 1500, 1800, 1500);
        // ★ 需求原文 §13 只描述两档（接管 / 控制屏幕光标）→ 高位与中位同为"屏幕光标"
        check_int("mode_high_cursor", mode, 1, 0);

        // ---------------- 5. 按钮上升沿 ----------------
        $display("-- 5. select 通道拉到高位 → 应产生一次脉冲，且只在跳变那一拍 --");
        begin : btn_wait
            integer pulses;
            integer k;
            pulses = 0;
            fork
                // ★ 新实现的两条要求：① 该通道**连续 2 帧**确认才算"按下"；
                //   ② 必须**先见到释放**再按下（幽灵点击防护）。所以这里连发 3 帧。
                //   bit3（J6.4，原"返回"键位）= select
                // ★ 注意：fork 里的多个 task 调用是【并行】的 —— 必须用 begin…end 串起来，
                //   否则三个分支同时拉高/拉低，等价于只发了一帧（本项目踩过这个坑）。
                begin
                    send_frame(1500, 1500, 1500, 1800, 1500, 1500);
                    send_frame(1500, 1500, 1500, 1800, 1500, 1500);
                    send_frame(1500, 1500, 1500, 1800, 1500, 1500);
                end
                begin
                    for (k = 0; k < 8000; k = k + 1) begin      // 覆盖 3 帧（帧确认需 ≥2 帧）
                        @(posedge clk);
                        if (btn_sel) pulses = pulses + 1;
                    end
                end
            join
            check_int("select_pulse_count", pulses, 1, 0);
        end

        // ---------------- 5b. 确认键（第二个按钮）同样应产生单周期脉冲 ----------------
        //   需求方明确"两个按钮（确认和 select）"，而"按确认才切页 / 确认路径点"
        //   完全依赖它 —— 原测试台只测了 select，确认通道没有断言。
        $display("-- 5b. confirm 通道拉到高位 → 应产生一次脉冲 --");
        begin : btn2_wait
            integer pulses;
            integer k;
            pulses = 0;
            fork
                // 同上：连发 3 帧以满足"连续 2 帧确认"（bit2 = 确认）；
                //   同样必须包在 begin…end 里才是顺序发送
                begin
                    send_frame(1500, 1500, 1800, 1500, 1500, 1500);
                    send_frame(1500, 1500, 1800, 1500, 1500, 1500);
                    send_frame(1500, 1500, 1800, 1500, 1500, 1500);
                end
                begin
                    for (k = 0; k < 8000; k = k + 1) begin      // 覆盖 3 帧（帧确认需 ≥2 帧）
                        @(posedge clk);
                        if (btn_conf) pulses = pulses + 1;
                    end
                end
            join
            check_int("confirm_pulse_count", pulses, 1, 0);
        end

        // ---------------- 6. 超时 → 信号无效 ----------------
        $display("-- 6. 停止刷新 > 5 ms → o_signal_valid 应拉低 --");
        #(6_000_000);                                       // 6 ms 不发脉冲（timescale 1ns → 必须写 6e6，不是 8000）
        check_int("signal_valid_0", sig_ok, 0, 0);
        // ★ 超时/无效时"开关位置"按中位处理 → 也是屏幕光标档（不驱动底盘的安全默认）。
        //   这条语义是安全链的一部分（接管档丢信号 → run_en 必须为 0），必须锁住。
        check_int("mode_on_timeout_cursor", mode, 1, 0);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        check_int("signal_valid_1", sig_ok, 1, 0);

        // ---------------- 7. 方向取反参数（B1）----------------
        $display("-- 7. STEER_REVERSE/THROTTLE_REVERSE=1：两个模拟量取反，模式/按钮不受影响 --");
        send_frame(2000, 1000, 1500, 1500, 1500, 1500);
        send_frame(2000, 1000, 1500, 1500, 1500, 1500);
        check_int("steer_norm_pos",    steer,          1000, 1);
        check_int("steer_rev_neg",     steer_rev,     -1000, 1);
        check_int("throttle_norm_neg", throttle,      -1000, 1);
        check_int("throttle_rev_pos",  throttle_rev,   1000, 1);
        check_int("mode_rev_unaffected", mode_rev, mode, 0);
        check_int("sig_rev_unaffected",  sig_rev,  sig_ok, 0);

        // 中位时取反前后都应为 0（−0 与 +0 都是 0，防止实现写成取反后偏 1）
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        check_int("steer_rev_center",    steer_rev,    0, 0);
        check_int("throttle_rev_center", throttle_rev, 0, 0);

        // ---------------- 8. 摇杆上下拨动 → 菜单切换（需求原文 §17）----------------
        //   要求："使用摇杆上下拨动来切换选项"
        //   判据：拨到位产生【恰好一个】脉冲；杆停在端点【不连翻】；回中后才算"松手"、能再拨
        $display("-- 8. 摇杆拨动：一次一个脉冲；停住不连翻；回中后才可再触发 --");
        begin : menu_test
            integer up_cnt, dn_cnt, k;
            up_cnt = 0; dn_cnt = 0;

            // (a) 拨到 +1000 并保持 5 ms（期间 2 帧）→ 应恰好 1 个 up 脉冲
            fork
                begin
                    send_frame(1500, 2000, 1500, 1500, 1500, 1500);
                    send_frame(1500, 2000, 1500, 1500, 1500, 1500);
                end
                begin
                    for (k = 0; k < 5000; k = k + 1) begin
                        @(posedge clk);
                        if (menu_up)   up_cnt = up_cnt + 1;
                        if (menu_down) dn_cnt = dn_cnt + 1;
                    end
                end
            join
            check_int("menu_up_once",   up_cnt, 1, 0);   // ★ 杆保持拨出也只有 1 个脉冲
            check_int("menu_down_none", dn_cnt, 0, 0);

            // (b) 回中位 5 ms → 不产生脉冲（内部从此回到 idle）
            fork
                begin
                    send_frame(1500, 1500, 1500, 1500, 1500, 1500);
                    send_frame(1500, 1500, 1500, 1500, 1500, 1500);
                end
                begin
                    for (k = 0; k < 5000; k = k + 1) begin
                        @(posedge clk);
                        if (menu_up)   up_cnt = up_cnt + 1;
                        if (menu_down) dn_cnt = dn_cnt + 1;
                    end
                end
            join
            check_int("menu_idle_no_pulse", up_cnt, 1, 0);   // 仍是 1

            // (c) 再拨一次 +1000 → 又 1 个（证明"回中后才可再触发"）
            fork
                begin
                    send_frame(1500, 2000, 1500, 1500, 1500, 1500);
                    send_frame(1500, 2000, 1500, 1500, 1500, 1500);
                end
                begin
                    for (k = 0; k < 5000; k = k + 1) begin
                        @(posedge clk);
                        if (menu_up)   up_cnt = up_cnt + 1;
                        if (menu_down) dn_cnt = dn_cnt + 1;
                    end
                end
            join
            check_int("menu_up_again", up_cnt, 2, 0);

            // (d) ★ 新实现要求"必须先观察到回中"：**直接从正端跳到负端不算回中** ——
            //     所以先发回中帧（回中过程不出脉冲），再向下拨 −1000 → down 恰好 1 个
            fork
                begin
                    send_frame(1500, 1500, 1500, 1500, 1500, 1500);
                    send_frame(1500, 1500, 1500, 1500, 1500, 1500);
                    send_frame(1500, 1000, 1500, 1500, 1500, 1500);
                    send_frame(1500, 1000, 1500, 1500, 1500, 1500);
                end
                begin
                    for (k = 0; k < 7000; k = k + 1) begin
                        @(posedge clk);
                        if (menu_up)   up_cnt = up_cnt + 1;
                        if (menu_down) dn_cnt = dn_cnt + 1;
                    end
                end
            join
            check_int("menu_down_once", dn_cnt, 1, 0);
            check_int("menu_up_stays_2", up_cnt, 2, 0);
        end

        // ---------------- 9. 信号有效性掩码（措施②的验证）----------------
        //   场景：bit5（J6.6 闲置未接）一直没有脉冲，其余 5 路正常刷新。
        //   期望：dut（默认掩码 63 = 6 路全参与）sig_ok = 0 —— 旧行为，会把整车拖死；
        //         dut_mask（排除 bit5）sig_mask_ok = 1 —— 整车仍能被使能。
        //   这是"保留 6 路接口 + 只接 5 路"能够真正开动的前提。
        $display("-- 9. 掩码：bit5 无脉冲、其余 5 路正常 → 全参与=0，掩码=1 --");
        begin : mask_test
            integer k;
            for (k = 0; k < 8; k = k + 1) begin
                send_frame_skip5(1500, 1500, 1500, 1500, 1500);
            end
            #20000;
            check_int("sig_all6_ok",       sig_ok,      0, 0);   // 6 路全参与 → 判为丢信号
            check_int("sig_mask_bit5_off", sig_mask_ok, 1, 0);   // 排除闲置 bit5 → 仍有效
        end

        // ---------------- 汇总 ----------------
        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: rc_input 全部检查通过");
        else             $display("FAIL: %0d 项检查未通过", errors);
        $finish;
    end

    // 超时保护
    initial begin
        #150_000_000;                       // 150 ms 仿真时间上限（菜单拨动 4 段 × 5 ms + 掩码段 8 帧 + 各按钮段）
        $display("FAIL: 仿真超时（120 ms）");
        $finish;
    end

endmodule
