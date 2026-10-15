//////////////////////////////////////////////////////////////////////////////////
// rc_input_tb —— 遥控 6 路输入单元测试
//
// 【为什么用 1 MHz 时钟】
//   rc_pwm_capture 用 CLK_HZ 把拍数换算成 µs，脉宽 1000~2000 µs 在 50 MHz 下要
//   10 万拍/毫秒级的仿真，太慢。把 CLK_HZ 设为 1 MHz（周期 1 µs）后逻辑完全等价，
//   仿真时间缩短 50 倍 —— 这是本工程里"参数化时钟"带来的好处。
//
// 【覆盖】
//   1. 中位 1500 µs          → steer/throttle = 0
//   2. 端点 2000 / 1000 µs   → +1000 / −1000（含归一化标度 2.125 的误差检查）
//   3. 死区 1500±25 µs 内    → 0
//   4. 模式通道三档          → o_mode = 0/1/2
//   5. 按钮通道高位上升沿    → 单周期脉冲
//   6. 停止刷新超过超时      → o_signal_valid 拉低
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
    wire               btn_sel, btn_conf, btn_back;
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
        .CH_BACK       (3),
        .CH_MODE       (4),
        .CH_SELECT     (5)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .rc_pwm         (rc_pwm),
        .o_steer        (steer),
        .o_throttle     (throttle),
        .o_mode         (mode),
        .o_btn_select   (btn_sel),
        .o_btn_confirm  (btn_conf),
        .o_btn_back     (btn_back),
        .o_signal_valid (sig_ok),
        .o_chan_us      (chan_us),
        .o_chan_timeout (chan_to),
        .o_chan_valid   (chan_vld)
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

        // ---------------- 4. 模式三档 ----------------
        $display("-- 4. 模式通道三档：1200→0(手动) 1500→1(光标) 1800→2(自动) --");
        send_frame(1500, 1500, 1500, 1500, 1200, 1500);
        send_frame(1500, 1500, 1500, 1500, 1200, 1500);
        check_int("mode_low_manual", mode, 0, 0);
        send_frame(1500, 1500, 1500, 1500, 1800, 1500);
        send_frame(1500, 1500, 1500, 1500, 1800, 1500);
        check_int("mode_high_auto", mode, 2, 0);

        // ---------------- 5. 按钮上升沿 ----------------
        $display("-- 5. select 通道拉到高位 → 应产生一次脉冲，且只在跳变那一拍 --");
        begin : btn_wait
            integer pulses;
            integer k;
            pulses = 0;
            fork
                send_frame(1500, 1500, 1500, 1500, 1500, 1800);   // select 拨到高位
                begin
                    for (k = 0; k < 6000; k = k + 1) begin
                        @(posedge clk);
                        if (btn_sel) pulses = pulses + 1;
                    end
                end
            join
            check_int("select_pulse_count", pulses, 1, 0);
        end

        // ---------------- 6. 超时 → 信号无效 ----------------
        $display("-- 6. 停止刷新 > 5 ms → o_signal_valid 应拉低 --");
        #(6_000_000);                                       // 6 ms 不发脉冲（timescale 1ns → 必须写 6e6，不是 8000）
        check_int("signal_valid_0", sig_ok, 0, 0);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        send_frame(1500, 1500, 1500, 1500, 1500, 1500);
        check_int("signal_valid_1", sig_ok, 1, 0);

        // ---------------- 汇总 ----------------
        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: rc_input 全部检查通过");
        else             $display("FAIL: %0d 项检查未通过", errors);
        $finish;
    end

    // 超时保护
    initial begin
        #50_000_000;                        // 50 ms 仿真时间上限
        $display("FAIL: 仿真超时（50 ms）");
        $finish;
    end

endmodule
