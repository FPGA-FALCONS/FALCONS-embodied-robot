// Board/application profile. Receiver CH numbers are wired to J6 slots below.
//   J6.1 / bit0 <- CH4 左摇杆左右 = steering
//   J6.2 / bit1 <- CH2 右摇杆上下 = throttle
//   J6.3 / bit2 <- CH5 / SWA    = **三档**：下=确认、上=返回（2026-10-19 需求变更）
//   J6.4 / bit3 <- CH7 / SWC    = **两档模式**：上=手动挡、下=光标档（2026-10-19 起接管模式）
//   J6.5 / bit4 <- CH6 / SWB    = AI录音：下拨开始、上拨停止（2026-10-24）
//   J6.6 / bit5 <- CH8 / SWD    = **三档 select**：上/下选择页面内子菜单
// CH1/CH3 joystick axes are deliberately NOT wired to button inputs.
//
// ★ 参数口径：下面传给 rc_input 的 CH_* 是**零起始的 J6 槽位索引**，不是接收机通道号。
//   "CH4→J6.1" 这种对应关系写在本注释与 doc/HT10A接线与验收.md 里，代码里只有槽位号。
module ht10a_input #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer STEER_CENTER_US=1500, THROTTLE_CENTER_US=1500,
    parameter integer STEER_HALF_RANGE_US=500, THROTTLE_HALF_RANGE_US=500,
    parameter integer STEER_DEADZONE_US=25, THROTTLE_DEADZONE_US=25,
    // 通用默认方向不代表整车；顶层u_rc显式覆盖STEER_REVERSE=1、THROTTLE_REVERSE=0。
    // 轮位/电机方向单独由底盘配置维护，不能用四路全反代替遥控轴方向修正。
    parameter integer STEER_REVERSE=0, THROTTLE_REVERSE=0, MENU_REVERSE=0,
    parameter integer MODE_REVERSE=0, SELECT_ACTIVE_LOW=0, CONFIRM_ACTIVE_LOW=0,
    parameter integer AUX_SWAP=0,
    parameter integer SW_LO_US=1300, SW_HI_US=1700, SW_HYST_US=30, SW_CONFIRM_FRAMES=2,
    parameter integer MIN_PULSE_US=900, MAX_PULSE_US=2100, TIMEOUT_US=100_000,
    parameter integer MENU_STEP_TH=400, MENU_IDLE_TH=150,
    // 通用默认011111供独立配置/测试；本车六路均使用，顶层显式覆盖111111。
    parameter integer SIG_VALID_MASK=6'b011111
)(
    input wire clk,rst_n,
    input wire [5:0] rc_pwm,
    output wire signed [11:0] o_steer,o_throttle,
    output wire [1:0] o_mode,
    output wire o_btn_select,o_btn_confirm,o_menu_up,o_menu_down,o_signal_valid,
    output wire o_back, o_aux_up, o_aux_down,
    output wire [95:0] o_chan_us,
    output wire [5:0] o_chan_timeout,o_chan_valid
);
    rc_input #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .NUM_CH(6),
        // 零起始J6槽位索引，映射见文件头。o_btn_select保留端口来自CH6，
        // 整车录音直接读取CH6；用户所称select为CH8的o_aux_up/o_aux_down。
        .CH_STEER(0),.CH_THROTTLE(1),.CH_CONFIRM(2),.CH_SELECT(4),.CH_MODE(3),.CH_AUX(5),
        .AUX_SWAP(AUX_SWAP),
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
        .o_back(o_back),.o_aux_up(o_aux_up),.o_aux_down(o_aux_down),
        .o_chan_us(o_chan_us),.o_chan_timeout(o_chan_timeout),.o_chan_valid(o_chan_valid)
    );
endmodule
