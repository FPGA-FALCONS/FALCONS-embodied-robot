// Board/application profile. Receiver CH numbers are wired to J6 slots below.
//   J6.1 / bit0 <- CH4 左摇杆左右 = steering
//   J6.2 / bit1 <- CH2 右摇杆上下 = throttle
//   J6.3 / bit2 <- CH5 / SWA    = confirm
//   J6.4 / bit3 <- CH7 / SWC    = **select**   （★ 2026-10-16：无"返回"按钮，这一路改作 select）
//   J6.5 / bit4 <- CH6 / SWB    = mode（两档拨杆）
//   J6.6 / bit5 <- CH8 / SWD    = **闲置**（无角色，且已排除出"信号有效"判定）
// CH1/CH3 joystick axes are deliberately NOT wired to button inputs.
//
// ★ 参数口径：下面传给 rc_input 的 CH_* 是**零起始的 J6 槽位索引**，不是接收机通道号。
//   "CH4→J6.1" 这种对应关系写在本注释与 doc/HT10A接线与验收.md 里，代码里只有槽位号。
module ht10a_input #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer STEER_CENTER_US=1500, THROTTLE_CENTER_US=1500,
    parameter integer STEER_HALF_RANGE_US=500, THROTTLE_HALF_RANGE_US=500,
    parameter integer STEER_DEADZONE_US=25, THROTTLE_DEADZONE_US=25,
    parameter integer STEER_REVERSE=0, THROTTLE_REVERSE=0, MENU_REVERSE=0,
    parameter integer MODE_REVERSE=0, SELECT_ACTIVE_LOW=0, CONFIRM_ACTIVE_LOW=0,
    parameter integer SW_LO_US=1300, SW_HI_US=1700, SW_HYST_US=30, SW_CONFIRM_FRAMES=2,
    parameter integer MIN_PULSE_US=900, MAX_PULSE_US=2100, TIMEOUT_US=100_000,
    parameter integer MENU_STEP_TH=400, MENU_IDLE_TH=150,
    // 参与"信号有效"判定的通道掩码：6'b011111 = 排除闲置的 bit5（见 rc_input 头注释②）
    //   ★ 位序提醒：6'b011111 的 MSB 是 bit5=0（排除）、其余 bit4..bit0=1（参与）。
    parameter integer SIG_VALID_MASK=6'b011111
)(
    input wire clk,rst_n,
    input wire [5:0] rc_pwm,
    output wire signed [11:0] o_steer,o_throttle,
    output wire [1:0] o_mode,
    output wire o_btn_select,o_btn_confirm,o_menu_up,o_menu_down,o_signal_valid,
    output wire [95:0] o_chan_us,
    output wire [5:0] o_chan_timeout,o_chan_valid
);
    rc_input #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .NUM_CH(6),
        // 槽位索引：bit3 = select（原"返回"键位），bit5 闲置不进映射
        .CH_STEER(0),.CH_THROTTLE(1),.CH_CONFIRM(2),.CH_SELECT(3),.CH_MODE(4),
        .SIG_VALID_MASK(SIG_VALID_MASK),
        .STEER_CENTER_US(STEER_CENTER_US),.THROTTLE_CENTER_US(THROTTLE_CENTER_US),
        .STEER_HALF_RANGE_US(STEER_HALF_RANGE_US),.THROTTLE_HALF_RANGE_US(THROTTLE_HALF_RANGE_US),
        .STEER_DEADZONE_US(STEER_DEADZONE_US),.THROTTLE_DEADZONE_US(THROTTLE_DEADZONE_US),
        .STEER_REVERSE(STEER_REVERSE),.THROTTLE_REVERSE(THROTTLE_REVERSE),.MENU_REVERSE(MENU_REVERSE),
        .MODE_REVERSE(MODE_REVERSE),.SELECT_ACTIVE_LOW(SELECT_ACTIVE_LOW),
        .CONFIRM_ACTIVE_LOW(CONFIRM_ACTIVE_LOW),
        .SW_LO_US(SW_LO_US),.SW_HI_US(SW_HI_US),.SW_HYST_US(SW_HYST_US),.SW_CONFIRM_FRAMES(SW_CONFIRM_FRAMES),
        .MIN_PULSE_US(MIN_PULSE_US),.MAX_PULSE_US(MAX_PULSE_US),.TIMEOUT_US(TIMEOUT_US),
        .MENU_STEP_TH(MENU_STEP_TH),.MENU_IDLE_TH(MENU_IDLE_TH)
    ) u_input (
        .clk(clk),.rst_n(rst_n),.rc_pwm(rc_pwm),.o_steer(o_steer),.o_throttle(o_throttle),.o_mode(o_mode),
        .o_btn_select(o_btn_select),.o_btn_confirm(o_btn_confirm),
        .o_menu_up(o_menu_up),.o_menu_down(o_menu_down),.o_signal_valid(o_signal_valid),
        .o_chan_us(o_chan_us),.o_chan_timeout(o_chan_timeout),.o_chan_valid(o_chan_valid)
    );
endmodule
