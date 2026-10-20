//////////////////////////////////////////////////////////////////////////////////
// 模块：tune_ctrl —— 参数整定页面控制器（11 项运行时参数 + 按键/摇杆交互）
//
// 【它在整条链路里的位置】
//   参数值原来写死在 `rtl\nav\nav_ctrl.v` 与 `rtl\top\falcons_top.v` 的
//   `localparam` 里，改一次要"改代码 → 重综合 → 烧录"（一轮 8 分钟以上）。
//   本模块把这些值搬进寄存器，由遥控器在线改，**页面上改完立刻生效**。
//
//       tune_ctrl ──p0..p7──> nav_ctrl   （巡航线速度/减速距离/对准角/航向 PID）
//            │    ──p8..p10─> chassis_ctrl 的 4 路 speed_pid（速度环 PID）
//            │    ──k_slow_q10──> nav_ctrl  （派生量，本模块用迭代除法器算）
//            └────disp0..10──> num_text    （页面显示值，已做单位换算）
//
// 【11 项参数与默认值】
//   0 V_CRUISE        300   50..600   步 10   mm/s      巡航线速度
//   1 D_SLOW          100   50..300   步 5    mm        开始减速的纵向距离
//   2 ALIGN_IN       1000  100..5000  步 100  0.01°     ALIGN→TRACK 门限（显示×100）
//   3 ALIGN_OUT      3500 1000..9000  步 100  0.01°     TRACK→ALIGN 门限（显示×100）
//   4 MAX_DIFF_CRUISE 180   30..612   步 10   cnt/10ms  行进中差速限幅（显示为 °/s）
//   5 KP_YAW           21    0..200   步 1     Q8       航向 PID 比例
//   6 KI_YAW            4    0..50    步 1     Q8       航向 PID 积分
//   7 KD_YAW            6    0..50    步 1     Q8       航向 PID 微分
//   8 KP_SPD            0    0..255   步 1     Q8       速度环比例（出厂 0）
//   9 KI_SPD           64    0..255   步 1     Q8       速度环积分（出厂 64）
//  10 KD_SPD            0    0..255   步 1     Q8       速度环微分（出厂 0）
//
//   ★ 默认值表 `p_def()` 必须与 `rtl\nav\nav_ctrl.v` 的参数默认、以及
//     `rtl\top\falcons_top.v` 里 `KP_INIT/KI_INIT/KD_INIT` **保持一致**。
//     综合后 nav_ctrl 的 parameter 默认已经不再参与逻辑（全部被端口覆盖），
//     那两处只剩"文档价值"，但改默认值时仍要三处同改，否则复位值与本模块不符。
//
// 【交互模型】
//   CH8 上/下  → 移动选中行（0..10 循环回绕）
//   CH4 摇杆左右 → 调整当前行数值
//       · 死区 ±200，越过才动；**必须回中才能再触发一次**（防误触连跳）
//       · 按住同一方向超过 400 ms 后，每 120 ms 再走一步（长按快调）
//   CH5 下（confirm） → 当前行恢复默认值
//   CH5 上（back）    → 退出参数页（回菜单）
//   ★ 手动档（`i_manual`）时摇杆归底盘用，本页**只显示不可调**（选行和退出仍可用，
//     因为选行用 CH8、退出用 CH5，都不占用摇杆）。
//
// 【为什么显示值要单独算、还要锁存】
//   页面要显示"人能读懂的量"：ALIGN_IN 内部是 0.01° 的 1000，要显示成 10；
//   MAX_DIFF_CRUISE 内部是 counts/10ms 的 180，要显示成 43 °/s。
//   换算含乘法（×328、×7813），组合直连会在 sys 域拉出长路径
//   （本工程对组合乘法有明确禁忌，见 nav_ctrl 里 M_V1/M_V2/M_V3 的拆拍记录），
//   所以只在**参数变化那一刻**用 2 个乘法器算一次并锁存。
//
// 【K_SLOW_Q10 为什么在这里算】
//   它是 `V_CRUISE×1024/D_SLOW` 的派生量，参数写死时是常量除法（综合器折叠），
//   运行时可调后就是真除法。放在本模块用多周期 `div_iter` 算好再送 nav_ctrl，
//   好处是 nav_ctrl 内部不出现除法器，且除法频率极低（只在改 V_CRUISE/D_SLOW 时）。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module tune_ctrl (
    input  wire               clk,
    input  wire               rst_n,

    // ---- 页面与权限 ----
    input  wire               i_active,     // 1 = 当前显示的是参数整定页
    input  wire               i_manual,     // 1 = 手动档，摇杆归底盘 → 禁调
    input  wire               i_signal,     // 1 = 遥控信号有效

    // ---- 遥控输入（rc_input 已消抖，均为单拍脉冲或归一化量）----
    input  wire signed [11:0] i_steer,      // CH4 左右，-1000..+1000
    input  wire               i_confirm,    // CH5 下
    input  wire               i_back,       // CH5 上
    input  wire               i_aux_up,     // CH8 上
    input  wire               i_aux_down,   // CH8 下

    // ---- 参数原值（11×32 位扁平，供 nav_ctrl / chassis_ctrl 使用）----
    output wire [351:0]       o_param,

    // ---- 显示值（11×32 位扁平，已按单位换算，直接接 num_text）----
    output wire [351:0]       o_disp,

    // ---- 页面状态 ----
    output reg  [3:0]         o_sel,        // 当前选中行 0..10
    output reg  signed [31:0] o_k_slow_q10, // 派生减速斜率（Q10）
    output reg                o_exit,       // 退出页面脉冲
    output wire               o_div_busy    // 除法器忙（调试用）
);

    // ------------------------------------------------------------------
    // 参数表：默认值 / 下限 / 上限 / 步长
    //   用 function 而不是数组常量，因为需要可综合且不改动既有代码风格。
    // ------------------------------------------------------------------
    function [31:0] p_def(input [3:0] i);
        case (i)
            4'd0:  p_def = 32'd300;
            4'd1:  p_def = 32'd100;
            4'd2:  p_def = 32'd1000;
            4'd3:  p_def = 32'd3500;
            4'd4:  p_def = 32'd180;
            4'd5:  p_def = 32'd21;
            4'd6:  p_def = 32'd4;
            4'd7:  p_def = 32'd6;
            4'd8:  p_def = 32'd0;
            4'd9:  p_def = 32'd64;
            4'd10: p_def = 32'd0;
            default: p_def = 32'd0;
        endcase
    endfunction

    function [31:0] p_min(input [3:0] i);
        case (i)
            4'd0:  p_min = 32'd50;
            4'd1:  p_min = 32'd50;
            4'd2:  p_min = 32'd100;
            4'd3:  p_min = 32'd1000;
            4'd4:  p_min = 32'd30;
            4'd5:  p_min = 32'd0;
            4'd6:  p_min = 32'd0;
            4'd7:  p_min = 32'd0;
            4'd8:  p_min = 32'd0;
            4'd9:  p_min = 32'd0;
            4'd10: p_min = 32'd0;
            default: p_min = 32'd0;
        endcase
    endfunction

    function [31:0] p_max(input [3:0] i);
        case (i)
            4'd0:  p_max = 32'd600;
            4'd1:  p_max = 32'd300;
            4'd2:  p_max = 32'd5000;
            4'd3:  p_max = 32'd9000;
            4'd4:  p_max = 32'd612;
            4'd5:  p_max = 32'd200;
            4'd6:  p_max = 32'd50;
            4'd7:  p_max = 32'd50;
            4'd8:  p_max = 32'd255;
            4'd9:  p_max = 32'd255;
            4'd10: p_max = 32'd255;
            default: p_max = 32'd0;
        endcase
    endfunction

    function [31:0] p_step(input [3:0] i);
        case (i)
            4'd0:  p_step = 32'd10;
            4'd1:  p_step = 32'd5;
            4'd2:  p_step = 32'd100;
            4'd3:  p_step = 32'd100;
            4'd4:  p_step = 32'd10;
            4'd5:  p_step = 32'd1;
            4'd6:  p_step = 32'd1;
            4'd7:  p_step = 32'd1;
            4'd8:  p_step = 32'd1;
            4'd9:  p_step = 32'd1;
            4'd10: p_step = 32'd1;
            default: p_step = 32'd0;
        endcase
    endfunction

    // 显示值初值（= 默认值换算后的结果，写死可省复位分支里的乘法）
    function [31:0] d_def(input [3:0] i);
        case (i)
            4'd0:  d_def = 32'd300;   // 300 mm/s
            4'd1:  d_def = 32'd100;   // 100 mm
            4'd2:  d_def = 32'd10;    // 1000 × 0.01° = 10°
            4'd3:  d_def = 32'd35;    // 3500 × 0.01° = 35°
            4'd4:  d_def = 32'd43;    // 180 cnt → 42.94 °/s → 43
            4'd5:  d_def = 32'd21;
            4'd6:  d_def = 32'd4;
            4'd7:  d_def = 32'd6;
            4'd8:  d_def = 32'd0;
            4'd9:  d_def = 32'd64;
            4'd10: d_def = 32'd0;
            default: d_def = 32'd0;
        endcase
    endfunction

    // ------------------------------------------------------------------
    // 参数寄存器
    // ------------------------------------------------------------------
    reg [31:0] p [0:10];
    reg [31:0] d [0:10];
    integer    i;
    integer    j;

    // ------------------------------------------------------------------
    // 1 ms 时基（连发计时用）
    // ------------------------------------------------------------------
    localparam integer MS_DIV = 50000;      // 50 MHz / 1000
    reg [15:0] ms_cnt;
    reg        ms_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt  <= 16'd0;
            ms_tick <= 1'b0;
        end else if (ms_cnt == (MS_DIV[15:0] - 16'd1)) begin
            ms_cnt  <= 16'd0;
            ms_tick <= 1'b1;
        end else begin
            ms_cnt  <= ms_cnt + 16'd1;
            ms_tick <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // 按键边沿检测（rc_input 送来的是单拍脉冲，但这里统一按上升沿取，
    //   这样即使将来改成电平输出也不会连发）
    // ------------------------------------------------------------------
    reg aux_up_d, aux_down_d, confirm_d, back_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aux_up_d   <= 1'b0;
            aux_down_d <= 1'b0;
            confirm_d  <= 1'b0;
            back_d     <= 1'b0;
        end else begin
            aux_up_d   <= i_aux_up;
            aux_down_d <= i_aux_down;
            confirm_d  <= i_confirm;
            back_d     <= i_back;
        end
    end

    wire aux_up_ev   = i_aux_up   & ~aux_up_d;
    wire aux_down_ev = i_aux_down & ~aux_down_d;
    wire confirm_ev  = i_confirm  & ~confirm_d;
    wire back_ev     = i_back     & ~back_d;

    // 页面激活才响应；手动档禁用调值（选行/退出不受影响）
    wire ui_on   = i_active & i_signal;
    wire can_adj = ui_on & ~i_manual;

    // ------------------------------------------------------------------
    // 摇杆方向与连发
    //   dir = 2'd1 右推（加）、2'd2 左推（减）、2'd0 回中
    //   ★ 必须用**方向编码**而不是 1 位 dir_p：回中时 dir_r 要归 0，
    //     这样"回中后再往任一方向推"都能被识别为一次新触发。
    //     原实现用 1 位 dir_r 与 dir_p 比较，左推时 dir_p=0、回中态 dir_r=0
    //     两者相等 → 永远不触发，变成"只能加不能减"（tune_ctrl_tb 抓出）。
    // ------------------------------------------------------------------
    localparam signed [11:0] STEER_DEADZONE = 12'sd200;

    wire signed [11:0] st = i_steer;
    wire        dir_p = (st >  STEER_DEADZONE);
    wire        dir_n = (st < -STEER_DEADZONE);
    wire        dir_nz = dir_p | dir_n;
    wire [1:0]  dir_cur = dir_p ? 2'd1 : (dir_n ? 2'd2 : 2'd0);

    reg [1:0]  dir_r;       // 上一拍的方向编码（0 回中 / 1 右 / 2 左）
    reg [15:0] hold_ms;     // 同向按住毫秒数
    reg [7:0]  rep_cnt;     // 连发分频计数（0..119）
    reg        step_pulse;  // 单步骤触发

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_r      <= 2'd0;
            hold_ms    <= 16'd0;
            rep_cnt    <= 8'd0;
            step_pulse <= 1'b0;
        end else begin
            step_pulse <= 1'b0;

            if (!can_adj || !dir_nz) begin
                // 回中或禁调：清计时、方向归零（下次推出必触发一步）
                dir_r   <= 2'd0;
                hold_ms <= 16'd0;
                rep_cnt <= 8'd0;
            end else begin
                if (dir_cur != dir_r) begin
                    // 换向或刚推离死区：先走一步，然后进入长按计时
                    dir_r      <= dir_cur;
                    hold_ms    <= 16'd0;
                    rep_cnt    <= 8'd0;
                    step_pulse <= 1'b1;
                end else begin
                    if (ms_tick && hold_ms != 16'hFFFF)
                        hold_ms <= hold_ms + 16'd1;

                    if (hold_ms >= 16'd400) begin
                        // 长按连发：每 120 ms 一步
                        if (ms_tick) begin
                            if (rep_cnt == 8'd119) begin
                                rep_cnt    <= 8'd0;
                                step_pulse <= 1'b1;
                            end else begin
                                rep_cnt <= rep_cnt + 8'd1;
                            end
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 选行（CH8）
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_sel <= 4'd0;
        end else if (ui_on) begin
            if (aux_up_ev) begin
                if (o_sel == 4'd0) o_sel <= 4'd10;
                else               o_sel <= o_sel - 4'd1;
            end else if (aux_down_ev) begin
                if (o_sel == 4'd10) o_sel <= 4'd0;
                else                o_sel <= o_sel + 4'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // 参数更新：单步调值 / 恢复默认
    // ------------------------------------------------------------------
    reg        edit_ev;         // 本拍有参数被改（触发显示值重算与 K_SLOW 重算）
    reg [31:0] edit_val;        // 改后的值（供显示换算）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 11; i = i + 1) p[i] <= p_def(i[3:0]);
            edit_ev  <= 1'b0;
            edit_val <= 32'd0;
        end else begin
            edit_ev <= 1'b0;

            // ★ 用 can_adj 而不是 ui_on：手动档（i_manual=1）时摇杆归底盘、
            //   确认键也不该顺手把参数复位掉（顶层注释承诺"只显示不改"）。
            //   退出页面仍走 ui_on（见下方 o_exit），否则手动档下退不出去。
            if (can_adj && (step_pulse || confirm_ev)) begin
                for (i = 0; i < 11; i = i + 1) begin
                    if (o_sel == i[3:0]) begin
                        if (confirm_ev) begin
                            p[i]     <= p_def(o_sel);
                            edit_val <= p_def(o_sel);
                            edit_ev  <= 1'b1;
                        end else if (step_pulse) begin
                            if (dir_p) begin
                                // 加：不越过上限
                                if (p[i] > (p_max(o_sel) - p_step(o_sel)))
                                    p[i] <= p_max(o_sel);
                                else
                                    p[i] <= p[i] + p_step(o_sel);
                                edit_val <= (p[i] > (p_max(o_sel) - p_step(o_sel)))
                                          ? p_max(o_sel) : (p[i] + p_step(o_sel));
                            end else begin
                                // 减：不越过下限
                                if (p[i] < (p_min(o_sel) + p_step(o_sel)))
                                    p[i] <= p_min(o_sel);
                                else
                                    p[i] <= p[i] - p_step(o_sel);
                                edit_val <= (p[i] < (p_min(o_sel) + p_step(o_sel)))
                                          ? p_min(o_sel) : (p[i] - p_step(o_sel));
                            end
                            edit_ev <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 显示值换算：只在 edit_ev 那拍算一次并锁存
    //   ALIGN_IN / ALIGN_OUT：内部 0.01° → 度，用 ×328>>15（0.0100098，足够准）
    //     1000 → 10、3500 → 35、9000 → 90
    //   MAX_DIFF_CRUISE：counts/10ms → °/s，用 (×7813+16384)>>15
    //     180 → 43、252 → 60、612 → 146
    //   其余直显
    //   ★ 两个乘法器就够，因为同一拍只改一行（o_sel 唯一）
    // ------------------------------------------------------------------
    wire [31:0] mul_deg = edit_val * 32'd328;
    wire [31:0] mul_ws  = edit_val * 32'd7813;

    wire [31:0] disp_new = (o_sel == 4'd2 || o_sel == 4'd3)
                         ? (mul_deg >> 15)
                         : (o_sel == 4'd4)
                         ? ((mul_ws + 32'd16384) >> 15)
                         : edit_val;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (j = 0; j < 11; j = j + 1) d[j] <= d_def(j[3:0]);
        end else if (edit_ev) begin
            for (j = 0; j < 11; j = j + 1) begin
                if (o_sel == j[3:0]) d[j] <= disp_new;
            end
        end
    end

    // ------------------------------------------------------------------
    // K_SLOW_Q10 = (V_CRUISE × 1024) / D_SLOW
    //   参数变更后启动一次迭代除法；结果锁存供 nav_ctrl 用。
    //   ★ 除数下限 50 由 p_min 保证，不会除零；div_iter 里另有兜底。
    // ------------------------------------------------------------------
    reg  [23:0] div_a;
    reg  [23:0] div_b;
    reg         div_start;
    wire [23:0] div_q;
    wire [23:0] div_r;
    wire        div_valid;

    div_iter #(.WIDTH(24)) u_div (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_start (div_start),
        .i_a     (div_a),
        .i_b     (div_b),
        .o_q     (div_q),
        .o_r     (div_r),
        .o_valid (div_valid),
        .o_busy  (o_div_busy)
    );

    // edit_ev 与 p[] 更新同拍，p[] 的新值下一拍才可见，
    // 所以把重算请求再延 2 拍（edit_d[1] = edit_ev 之后第 2 拍）。
    reg [1:0] edit_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) edit_d <= 2'd0;
        else        edit_d <= {edit_d[0], edit_ev};
    end
    wire rediv = edit_d[1];

    reg started;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_start    <= 1'b0;
            div_a        <= 24'd0;
            div_b        <= 24'd0;
            started      <= 1'b0;
            o_k_slow_q10 <= 32'sd3072;     // 默认 300×1024/100 = 3072
        end else begin
            div_start <= 1'b0;

            if (!started) begin
                // 复位后的首次启动：p[] 已是默认值
                div_a     <= p[0][23:0] << 10;   // V_CRUISE × 1024（≤ 614400，24 位够）
                div_b     <= p[1][23:0];         // D_SLOW
                div_start <= 1'b1;
                started   <= 1'b1;
            end else if (rediv) begin
                // 参数 0/1 变更后重算（其余参数变更不会走到这里也没坏处，
                // 多算一次而已，除法器 24 拍即完成）
                div_a     <= p[0][23:0] << 10;
                div_b     <= p[1][23:0];
                div_start <= 1'b1;
            end

            if (div_valid) o_k_slow_q10 <= {8'd0, div_q[23:0]};
        end
    end

    // ------------------------------------------------------------------
    // 退出页面
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) o_exit <= 1'b0;
        else        o_exit <= ui_on & back_ev;
    end

    // ------------------------------------------------------------------
    // 输出打包
    // ------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < 11; gi = gi + 1) begin : g_pack
            assign o_param[gi*32 +: 32] = p[gi];
            assign o_disp [gi*32 +: 32] = d[gi];
        end
    endgenerate

endmodule
