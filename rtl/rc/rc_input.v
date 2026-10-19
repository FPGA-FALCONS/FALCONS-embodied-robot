// Application RC adapter. Channel indices refer to FPGA input slots, NOT receiver CH numbers.
// Board-specific HT-10A wiring/calibration lives in ht10a_input and falcons_top.
// Vendor rc_pwm_capture is reused unchanged.
//
// ── 2026-10-16 合并裁定（需求方）────────────────────────────────────────────
//  ① 没有"返回"按钮：J6.4/bit3 改作 **select**（CH_SELECT=3），J6.6/bit5 **闲置**。
//     因此删除 CH_BACK / BACK_ACTIVE_LOW / o_btn_back 与整条 back 事件链。
//  ② "信号有效"用**通道掩码** SIG_VALID_MASK（位 i = 通道 i 是否参与判定）：
//     原实现要求"六路全部有效"（all_good = &valid && !|expired），而 bit5 闲置未接时
//     它的 timed_out 恒 1 → o_signal_valid 恒 0 → 整车永不能动，且只报"无信号"无法定位。
//     现在只对**参与判定的通道**做要求（本车 = bit0~bit4，排除 bit5）。
//  ③ 遥控**事件**（按钮 / 菜单拨动）的有效性与档位门控统一在本模块的 cursor_ready 里实现，
//     顶层不再重复门控（单一真源）—— 改这里等于改全局，顶层请勿再加同类条件。
// ───────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps
module rc_input #(
    parameter integer CLK_FREQ_HZ=50_000_000, NUM_CH=6,
    parameter integer CENTER_US=1500, HALF_RANGE_US=500, DEADZONE_US=25,
    parameter integer STEER_CENTER_US=CENTER_US, THROTTLE_CENTER_US=CENTER_US,
    parameter integer STEER_HALF_RANGE_US=HALF_RANGE_US, THROTTLE_HALF_RANGE_US=HALF_RANGE_US,
    parameter integer STEER_DEADZONE_US=DEADZONE_US, THROTTLE_DEADZONE_US=DEADZONE_US,
    parameter integer MIN_PULSE_US=900, MAX_PULSE_US=2100, TIMEOUT_US=100_000,
    parameter integer SW_LO_US=1300, SW_HI_US=1700, SW_HYST_US=30, SW_CONFIRM_FRAMES=2,
    parameter integer STEER_REVERSE=0, THROTTLE_REVERSE=0,
    parameter integer MODE_REVERSE=0, SELECT_ACTIVE_LOW=0, CONFIRM_ACTIVE_LOW=0,
    parameter integer MENU_STEP_TH=400, MENU_IDLE_TH=150, MENU_REVERSE=0,
    // 通道映射：零起始的 **J6 槽位索引**，不是接收机通道号
    //   bit0 转向 / bit1 油门 / bit2 确认 / **bit3 select**（原"返回"键位）/ bit4 模式 / bit5 闲置
    parameter integer CH_STEER=0, CH_THROTTLE=1, CH_CONFIRM=2, CH_MODE=4, CH_SELECT=3,
    // ★ 2026-10-19：CH_AUX = 辅助菜单上下切的三档开关（槽位 5 = J6.6 = CH8）
    parameter integer CH_AUX=5, AUX_SWAP=0,
    // 参与"信号有效"判定的通道掩码（位 i = 通道 i）。本车 = 6'b011111（排除闲置的 bit5）。
    //   ★ 2026-10-16 修正：原写成 6'b101111 —— 那是**排除 bit4（模式通道）**，
    //     与本意的"排除闲置的 bit5"刚好错一位；按 6'b101111 接线上板时，
    //     模式通道不参与判定、而未接的 bit5 反而参与，等于回到"未接通道拖死整车"的老问题。
    //   ★ 默认值口径：这里按**本车五路**配置（bit0~bit4 参与）—— 板级层
    //     （rtl\rc\ht10a_input.v，其默认值同样是 6'b011111）会显式覆盖；
    //     给这个默认值的目的是：万一有人**直接例化 rc_input**（绕过板级层），
    //     也不会退回"六路全有效"而与现场五路的实际情况不符。
    // 注意：integer 参数不能位选（vlog-13069），下面用 wire 截断取低位。
    parameter integer SIG_VALID_MASK=6'b011111
)(
    input wire clk, rst_n,
    input wire [NUM_CH-1:0] rc_pwm,
    output reg signed [11:0] o_steer, o_throttle,
    output reg [1:0] o_mode,
    output reg o_btn_select, o_btn_confirm,
    output reg o_menu_up, o_menu_down,
    // ★ 2026-10-19：确认键所在通道改为**三档开关**（下=确认、上=返回），
    //   另加一路三档开关做辅助菜单上下切。两路都只产出**单拍脉冲**（沿触发）。
    output reg o_back,
    output reg o_aux_up, o_aux_down,
    output wire o_signal_valid,
    output wire [NUM_CH*16-1:0] o_chan_us,
    output wire [NUM_CH-1:0] o_chan_timeout, o_chan_valid
);
    wire [15:0] us [0:NUM_CH-1];
    wire [NUM_CH-1:0] updated, valid, expired;
    wire [NUM_CH-1:0] sig_mask    = SIG_VALID_MASK;       // integer → 向量（不能直接位选）
    wire [NUM_CH-1:0] ch_healthy  = valid & ~expired;     // 每路是否"有合法且新鲜的输入"
    // ★ 只要求【参与判定】的通道全部健康：掩码外（预留未接）的通道不影响结论。
    //   &(ch_healthy | ~sig_mask)：掩码内的位必须健康，掩码外的位恒通过。
    wire all_good = rst_n && (&(ch_healthy | ~sig_mask));
    genvar g;
    generate for (g=0; g<NUM_CH; g=g+1) begin: g_cap
        rc_pwm_capture #(.CLK_HZ(CLK_FREQ_HZ), .MIN_PULSE_US(MIN_PULSE_US),
            .MAX_PULSE_US(MAX_PULSE_US), .TIMEOUT_US(TIMEOUT_US)) u_cap (
            .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[g]), .pulse_width_us(us[g]),
            .pulse_valid(valid[g]), .timed_out(expired[g]), .pulse_update(updated[g]));
        assign o_chan_us[g*16 +: 16] = us[g];
    end endgenerate
    assign o_chan_valid=valid;
    assign o_chan_timeout=expired;

    // Calibration is real: each stick has its own center, travel and deadzone.
    // Offset, constant-Q10 multiply and clipping occupy separate registers.
    localparam integer SS=(STEER_HALF_RANGE_US>STEER_DEADZONE_US) ? STEER_HALF_RANGE_US-STEER_DEADZONE_US : 1;
    localparam integer TS=(THROTTLE_HALF_RANGE_US>THROTTLE_DEADZONE_US) ? THROTTLE_HALF_RANGE_US-THROTTLE_DEADZONE_US : 1;
    localparam signed [31:0] SK=(1024000+SS-1)/SS, TK=(1024000+TS-1)/TS;
    function signed [31:0] offset_us;
        input [15:0] width;
        input integer center, deadzone, span;
        integer delta, mag;
        begin
            delta=$signed({1'b0,width})-center;
            mag=(delta<0) ? -delta : delta;
            if (mag<=deadzone) mag=0;
            else mag=mag-deadzone;
            if (mag>span) mag=span;
            offset_us=(delta<0) ? -mag : mag;
        end
    endfunction
    function signed [11:0] clip_cmd;
        input signed [31:0] product;
        reg signed [31:0] value;
        begin
            value=(product<0) ? -((-product) >>> 10) : (product >>> 10);
            if (value>1000) clip_cmd=12'sd1000;
            else if (value< -1000) clip_cmd=-12'sd1000;
            else clip_cmd=value[11:0];
        end
    endfunction
    reg signed [31:0] steer_delta, throttle_delta, steer_product, throttle_product;
    reg signed [11:0] throttle_unreversed;
    reg [3:0] command_ready;
    // Prevent a temporary zero from the arithmetic pipeline authorizing startup.
    assign o_signal_valid=all_good && command_ready[3];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            steer_delta<=0; throttle_delta<=0; steer_product<=0; throttle_product<=0;
            o_steer<=0; o_throttle<=0; throttle_unreversed<=0; command_ready<=0;
        end else if (!all_good) begin
            steer_delta<=0; throttle_delta<=0; steer_product<=0; throttle_product<=0;
            o_steer<=0; o_throttle<=0; throttle_unreversed<=0; command_ready<=0;
        end else begin
            command_ready<={command_ready[2:0],1'b1};
            steer_delta<=offset_us(us[CH_STEER],STEER_CENTER_US,STEER_DEADZONE_US,SS);
            throttle_delta<=offset_us(us[CH_THROTTLE],THROTTLE_CENTER_US,THROTTLE_DEADZONE_US,TS);
            steer_product<=steer_delta*SK;
            throttle_product<=throttle_delta*TK;
            o_steer<=command_ready[2] ? (STEER_REVERSE ? -clip_cmd(steer_product) : clip_cmd(steer_product)) : 12'sd0;
            o_throttle<=command_ready[2] ? (THROTTLE_REVERSE ? -clip_cmd(throttle_product) : clip_cmd(throttle_product)) : 12'sd0;
            throttle_unreversed<=command_ready[2] ? clip_cmd(throttle_product) : 12'sd0;
        end
    end

    // Schmitt thresholds + consecutive completed frames, not consecutive clocks.
    // Stable positions: 0=low, 1=middle/invalid, 2=high. MODE uses low vs others.
    function [1:0] classify;
        input [15:0] width;
        input [1:0] previous;
        begin
            if ((previous==0 && width<SW_LO_US+SW_HYST_US) || width<SW_LO_US-SW_HYST_US) classify=0;
            else if ((previous==2 && width>SW_HI_US-SW_HYST_US) || width>SW_HI_US+SW_HYST_US) classify=2;
            else classify=1;
        end
    endfunction
    reg [1:0] position [0:NUM_CH-1];
    reg [1:0] candidate [0:NUM_CH-1];
    reg [7:0] frames [0:NUM_CH-1];
    reg [NUM_CH-1:0] switch_ready;
    generate for (g=0; g<NUM_CH; g=g+1) begin: g_switch
        wire [1:0] next_position=classify(us[g],position[g]);
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                position[g]<=1; candidate[g]<=1; frames[g]<=0; switch_ready[g]<=0;
            end else if (!all_good) begin
                position[g]<=1; candidate[g]<=1; frames[g]<=0; switch_ready[g]<=0;
            end else if (updated[g]) begin
                if (next_position!=candidate[g]) begin
                    candidate[g]<=next_position;
                    frames[g]<=1;
                    if (SW_CONFIRM_FRAMES==1) begin position[g]<=next_position; switch_ready[g]<=1; end
                end else if (frames[g]<SW_CONFIRM_FRAMES) begin
                    frames[g]<=frames[g]+1'b1;
                    if (frames[g]>=SW_CONFIRM_FRAMES-1) begin position[g]<=next_position; switch_ready[g]<=1; end
                end
            end
        end
    end endgenerate
    wire select_down=SELECT_ACTIVE_LOW ? position[CH_SELECT]==0 : position[CH_SELECT]==2;
    // ★ 2026-10-19：CH_CONFIRM 由"按钮"升级为**三档开关**：
    //   CONFIRM_ACTIVE_LOW=0（默认）→ 高电平(=2)侧为「确认」，低电平(=0)侧为「返回」
    //   CONFIRM_ACTIVE_LOW=1         → 反过来
    //   中位(=1)两侧都不成立，不产生任何事件。
    //   ★ 物理上"哪个方向是下拨"由开关极性决定，所以在 ht10a_input / falcons_top
    //     的例化处用 CONFIRM_ACTIVE_LOW 标定，不要改这里的逻辑。
    wire confirm_hi = (position[CH_CONFIRM]==2);
    wire confirm_lo = (position[CH_CONFIRM]==0);
    wire confirm_yes  = CONFIRM_ACTIVE_LOW ? confirm_lo : confirm_hi;
    wire confirm_back = CONFIRM_ACTIVE_LOW ? confirm_hi : confirm_lo;
    // 辅助菜单三档：上=up、下=down；AUX_SWAP=1 时交换
    wire aux_hi = (position[CH_AUX]==2);
    wire aux_lo = (position[CH_AUX]==0);
    wire aux_is_up   = AUX_SWAP ? aux_lo : aux_hi;
    wire aux_is_down = AUX_SWAP ? aux_hi : aux_lo;
    reg select_armed, confirm_armed, back_armed, auxu_armed, auxd_armed;
    reg menu_armed;
    wire signed [11:0] menu_value=MENU_REVERSE ? -throttle_unreversed : throttle_unreversed;
    wire mode_manual=MODE_REVERSE ? (position[CH_MODE]==2) : (position[CH_MODE]==0);
    wire cursor_ready=o_signal_valid && switch_ready[CH_MODE] && !mode_manual && o_mode==1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_mode<=1; o_btn_select<=0; o_btn_confirm<=0;
            o_menu_up<=0; o_menu_down<=0;
            select_armed<=0; confirm_armed<=0; menu_armed<=0;
            back_armed<=0; auxu_armed<=0; auxd_armed<=0;
            o_back<=0; o_aux_up<=0; o_aux_down<=0;
        end else begin
            o_btn_select<=0; o_btn_confirm<=0;
            o_menu_up<=0; o_menu_down<=0;
            o_back<=0; o_aux_up<=0; o_aux_down<=0;
            if (!o_signal_valid || !switch_ready[CH_MODE]) o_mode<=1;
            else if (MODE_REVERSE) o_mode<=(position[CH_MODE]==2) ? 0 : 1;
            else o_mode<=(position[CH_MODE]==0) ? 0 : 1;
            if (!cursor_ready) begin
                select_armed<=0; confirm_armed<=0; menu_armed<=0;
                back_armed<=0; auxu_armed<=0; auxd_armed<=0;
            end else begin
                // Require a verified release after reset/loss/mode entry; held buttons never create phantom clicks.
                // select 现在接在原"返回"键位（J6.4 / bit3）上；bit5 闲置、无事件
                if (switch_ready[CH_SELECT]) begin
                    if (!select_down) select_armed<=1;
                    else if (select_armed) begin o_btn_select<=1; select_armed<=0; end
                end
                // 确认 / 返回：同一路三档的两侧各一个沿触发
                if (switch_ready[CH_CONFIRM]) begin
                    if (!confirm_yes) confirm_armed<=1;
                    else if (confirm_armed && confirm_yes) begin o_btn_confirm<=1; confirm_armed<=0; end
                    if (!confirm_back) back_armed<=1;
                    else if (back_armed && confirm_back) begin o_back<=1; back_armed<=0; end
                end
                // 辅助菜单：上/下各一个沿触发
                if (switch_ready[CH_AUX]) begin
                    if (!aux_is_up)   auxu_armed<=1;
                    else if (auxu_armed && aux_is_up) begin o_aux_up<=1; auxu_armed<=0; end
                    if (!aux_is_down) auxd_armed<=1;
                    else if (auxd_armed && aux_is_down) begin o_aux_down<=1; auxd_armed<=0; end
                end
                // A sampled center is required; jumping directly between endpoints is NOT a release.
                if (menu_value>=-MENU_IDLE_TH && menu_value<=MENU_IDLE_TH) menu_armed<=1;
                else if (menu_armed && menu_value>MENU_STEP_TH) begin o_menu_up<=1; menu_armed<=0; end
                else if (menu_armed && menu_value< -MENU_STEP_TH) begin o_menu_down<=1; menu_armed<=0; end
            end
        end
    end
endmodule
