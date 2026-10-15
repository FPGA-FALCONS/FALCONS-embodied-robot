//////////////////////////////////////////////////////////////////////////////////
// motor_top_tb —— 四路集成测试平台
//
// 这是【集成测试】：重点覆盖跨模块的契约，而不是单个模块的内部逻辑。
// 单元测试各自通过、接起来不通，正是这个工程踩过的坑（见 gyro_tilt_comp
// 的 o_data_valid / o_acc_valid 事件），所以集成测试必须单独写。
//
// 覆盖：
//   1) ★ 复位/上电期间的安全态（STBY=0、IN1=IN2=0、PWM=0）—— 防"上电窜车"
//   2) STBY 门控：任一路使能才拉起
//   3) 四路方向/占空比互相独立
//   4) 单路刹车不影响其他路
//   5) 四路编码器独立计数
//   6) 四路测速窗口同步
//
// 参数取小值：CLK=1MHz，PWM 周期 16 拍，滤波 4 拍，窗口 1ms
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module motor_top_tb;

    localparam integer CLK_FREQ_HZ = 1_000_000;
    localparam integer PWM_BITS    = 4;
    localparam integer FILTER_BITS = 2;
    localparam integer WINDOW_MS   = 1;
    localparam integer CLK_PERIOD  = 1000;
    localparam integer PWM_PERIOD  = (1 << PWM_BITS);   // 16
    localparam integer SETTLE      = 8;

    reg  clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg                    reset_n;
    reg  [3:0]             enable, dir, brake;
    reg  [4*PWM_BITS-1:0]  duty_in;

    wire                   stby;
    wire [3:0]             in1, in2, pwm;

    reg  [3:0]             enc_a, enc_b;
    reg                    count_clear;
    reg                    stall_clear;

    wire [4*32-1:0]        count_out;
    wire [4*32-1:0]        delta_out;
    wire                   delta_valid;
    wire [3:0]             enc_dir;
    wire [3:0]             stall_out;

    // 取出各路计数（signed）
    wire signed [31:0] cnt0 = count_out[31:0];
    wire signed [31:0] cnt1 = count_out[63:32];
    wire signed [31:0] cnt2 = count_out[95:64];
    wire signed [31:0] cnt3 = count_out[127:96];

    integer errors = 0;
    integer n;
    integer high_cnt;
    integer i;

    //---------------------------------------------------------------
    motor_top #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_BITS    (PWM_BITS),
        .FILTER_BITS (FILTER_BITS),
        .WINDOW_MS   (WINDOW_MS),
        .INV_DIR_VEC     (4'b0000),
        // 仿真加速：堵转保护的时间参数按比例缩小；duty 阈值按 PWM_BITS=4 缩放
        // （真实工程里 DUTY_TH=200 对应 PWM_BITS=11 的 10%）
        .STALL_STARTUP_WINDOWS (3),
        .STALL_WINDOWS         (4),
        .STALL_DUTY_TH         (4),
        .STALL_MOVE_TH         (300)
    ) dut (
        .i_clock       (clk),
        .i_reset_n     (reset_n),
        .i_enable      (enable),
        .i_dir         (dir),
        .i_brake       (brake),
        .i_duty        (duty_in),
        .i_stall_clear (stall_clear),
        .o_stby        (stby),
        .o_in1         (in1),
        .o_in2         (in2),
        .o_pwm         (pwm),
        .o_stall       (stall_out),
        .i_enc_a       (enc_a),
        .i_enc_b       (enc_b),
        .i_count_clear (count_clear),
        .o_count       (count_out),
        .o_delta       (delta_out),
        .o_delta_valid (delta_valid),
        .o_enc_dir     (enc_dir)
    );

    //---------------------------------------------------------------
    task checki;
        input [8*52-1:0] label;
        input integer    got;
        input integer    exp;
        begin
            if (got == exp) begin
                $display("OK  : %0s = %0d", label, got);
            end else begin
                $display("FAIL: %0s 期望 %0d，实测 %0d", label, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    // 设置第 k 路编码器电平并等待滤波确认
    task set_enc;
        input integer k;
        input         a, b;
        begin
            enc_a[k] = a;
            enc_b[k] = b;
            repeat (SETTLE) @(posedge clk);
        end
    endtask

    // 第 k 路正转一步（4 个计数）
    task enc_step_fwd;
        input integer k;
        begin
            set_enc(k, 1'b1, 1'b0);
            set_enc(k, 1'b1, 1'b1);
            set_enc(k, 1'b0, 1'b1);
            set_enc(k, 1'b0, 1'b0);
        end
    endtask

    // 第 k 路反转一步（4 个计数）
    task enc_step_rev;
        input integer k;
        begin
            set_enc(k, 1'b0, 1'b1);
            set_enc(k, 1'b1, 1'b1);
            set_enc(k, 1'b1, 1'b0);
            set_enc(k, 1'b0, 1'b0);
        end
    endtask

    // 统计第 k 路 PWM 在 cycles 拍内的高电平拍数
    task meas_high;
        input  integer k;
        input  integer cycles;
        output integer high_cnt_o;
        integer j;
        begin
            high_cnt_o = 0;
            for (j = 0; j < cycles; j = j + 1) begin
                @(posedge clk);
                #1;
                if (pwm[k]) high_cnt_o = high_cnt_o + 1;
            end
        end
    endtask

    task wait_valid;
        begin
            @(posedge clk); #1;
            while (delta_valid !== 1'b1) begin
                @(posedge clk); #1;
            end
        end
    endtask

    //---------------------------------------------------------------
    initial begin
        $display("");
        $display("======== motor_top_tb ========");

        reset_n     = 1'b0;
        enable      = 4'b0000;
        dir         = 4'b0000;
        brake       = 4'b0000;
        duty_in     = {(4*PWM_BITS){1'b0}};
        enc_a       = 4'b0000;
        enc_b       = 4'b0000;
        count_clear = 1'b0;
        stall_clear = 1'b0;

        //---- 用例 1：★ 复位/上电期间的安全态 ----
        $display("--- 用例1：★ 复位期间必须安全（防上电窜车）---");
        repeat (8) @(posedge clk);
        #1;
        checki("复位时 STBY", stby, 0);
        checki("复位时 o_in1", in1, 0);
        checki("复位时 o_in2", in2, 0);
        checki("复位时 o_pwm", pwm, 0);

        reset_n = 1'b1;
        repeat (SETTLE) @(posedge clk);
        #1;

        //---- 用例 2：STBY 门控 ----
        $display("--- 用例2：STBY 门控（任一路使能才拉起）---");
        enable = 4'b0000;
        repeat (4) @(posedge clk); #1;
        checki("全路未使能时 STBY", stby, 0);

        enable = 4'b0001;                 // 只使能 A 路
        repeat (4) @(posedge clk); #1;
        checki("仅 A 路使能时 STBY", stby, 1);

        enable = 4'b1000;                 // 只使能 D 路
        repeat (4) @(posedge clk); #1;
        checki("仅 D 路使能时 STBY", stby, 1);

        enable = 4'b0000;
        repeat (4) @(posedge clk); #1;
        checki("再次全部关闭后 STBY", stby, 0);

        //---- 用例 3：四路方向互相独立 ----
        $display("--- 用例3：四路方向独立 ---");
        enable = 4'b1111;
        duty_in = {4'd8, 4'd8, 4'd8, 4'd8};
        dir  = 4'b0000;                   // 全正转
        brake = 4'b0000;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("全正转 o_in1", in1, 4'b1111);
        checki("全正转 o_in2", in2, 4'b0000);

        dir = 4'b0101;                    // bit0=A 反转、bit2=C 反转（B、D 保持正转）
        repeat (PWM_PERIOD) @(posedge clk); #1;
        // 正转路 IN1=1/IN2=0，反转路 IN1=0/IN2=1
        checki("混合方向 o_in1 (B,D 正转)", in1, 4'b1010);
        checki("混合方向 o_in2 (A,C 反转)", in2, 4'b0101);

        //---- 用例 4：四路占空比互相独立 ----
        $display("--- 用例4：四路占空比独立（测 256 拍）---");
        dir = 4'b0000;
        // A=8(50%)  B=4(25%)  C=12(75%)  D=1(6.25%)
        duty_in = {4'd1, 4'd12, 4'd4, 4'd8};
        repeat (PWM_PERIOD) @(posedge clk);

        meas_high(0, 256, high_cnt);
        checki("A 路 duty=8 高电平拍数", high_cnt, 128);
        meas_high(1, 256, high_cnt);
        checki("B 路 duty=4 高电平拍数", high_cnt, 64);
        meas_high(2, 256, high_cnt);
        checki("C 路 duty=12 高电平拍数", high_cnt, 192);
        meas_high(3, 256, high_cnt);
        checki("D 路 duty=1 高电平拍数", high_cnt, 16);

        //---- 用例 5：单路刹车不影响其他路 ----
        $display("--- 用例5：单路刹车不影响其他路 ---");
        enable = 4'b1110;                 // A 路关闭
        brake  = 4'b0001;                 // A 路刹车
        duty_in = {4'd8, 4'd8, 4'd8, 4'd8};
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("A 路刹车时 o_in1[0]", in1[0], 1);
        checki("A 路刹车时 o_in2[0]", in2[0], 1);
        checki("B 路不受影响 o_in1[1]", in1[1], 1);
        checki("B 路不受影响 o_in2[1]", in2[1], 0);
        checki("STBY 仍为使能", stby, 1);

        //---- 用例 6：四路编码器独立计数 ----
        $display("--- 用例6：四路编码器独立计数 ---");
        enable = 4'b0000;                 // 关掉电机，只测编码器
        repeat (SETTLE) @(posedge clk);

        // 先清零
        count_clear = 1'b1;
        @(posedge clk);
        count_clear = 1'b0;
        repeat (SETTLE) @(posedge clk);

        n = cnt0; checki("清零后 cnt0", n, 0);
        n = cnt3; checki("清零后 cnt3", n, 0);

        enc_step_fwd(0);                  // 只动 A 路：+4
        enc_step_fwd(0);
        n = cnt0; checki("A 路正转 2 步后 cnt0", n, 8);
        n = cnt1; checki("A 路动作时 cnt1 应不变", n, 0);
        n = cnt2; checki("A 路动作时 cnt2 应不变", n, 0);
        n = cnt3; checki("A 路动作时 cnt3 应不变", n, 0);
        checki("A 路方向位", enc_dir[0], 1);

        enc_step_rev(3);                  // 只动 D 路：-4
        n = cnt3; checki("D 路反转 1 步后 cnt3", n, -4);
        n = cnt0; checki("D 路动作时 cnt0 应不变", n, 8);
        checki("D 路方向位", enc_dir[3], 0);

        enc_step_fwd(2);                  // 只动 C 路：+4
        n = cnt2; checki("C 路正转 1 步后 cnt2", n, 4);
        n = cnt1; checki("C 路动作时 cnt1 应不变", n, 0);

        //---- 用例 7：四路测速窗口同步 ----
        $display("--- 用例7：四路测速窗口同步出数 ---");
        wait_valid;
        $display("      （A 路 8 个计数、C 路 4 个、D 路 -4 个已在上一个窗口内累积）");
        checki("valid 有效时 A 路增量非零", (delta_out[31:0] != 32'sd0), 1);

        //---- 用例 8：★ 集成中的堵转保护 ----
        //   四路全部使能、占空比 50%（高于 STALL_DUTY_TH），但【不给编码器任何
        //   脉冲】→ 净位移始终为 0 → 应被判堵转并自动切断该路使能。
        //   这是本用例的价值：单元测试各自通过、接起来是否真能保护电机。
        $display("--- 用例8：★ 集成中的堵转保护（四路同时静止）---");
        enable  = 4'b1111;
        brake   = 4'b0000;
        duty_in = {4'd8, 4'd8, 4'd8, 4'd8};
        // 触发需要 (STARTUP+1+STALL) ≈ 8 个测速窗口，每窗口 1000 拍 → 等 12000 拍
        repeat (12000) @(posedge clk);
        #1;
        checki("四路堵转标志", stall_out, 4'b1111);
        checki("堵转后 o_in1（使能被切断→滑行，应全 0）", in1, 4'b0000);
        checki("堵转后 o_in2（应全 0）", in2, 4'b0000);

        // 清除锁存后应立刻恢复驱动
        stall_clear = 1'b1;
        @(posedge clk);
        stall_clear = 1'b0;
        repeat (8) @(posedge clk);
        #1;
        checki("清除后堵转标志", stall_out, 4'b0000);
        checki("清除后 A 路恢复驱动 o_in1[0]", in1[0], 1);

        //---- 汇总 ----
        $display("");
        if (errors == 0) begin
            $display("PASS: motor_top 四路集成测试全部通过");
        end else begin
            $display("FAIL: motor_top 测试共 %0d 处错误", errors);
        end
        $display("");
        $finish;
    end

endmodule
