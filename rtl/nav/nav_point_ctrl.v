// 避障导航目标点执行器。原 nav_ctrl 保留供无雷达调试。
// 位置 signed mm，yaw 为 [-18000,18000) 的逆时针 0.01°；i_tick 为 10 ms。
// 正常：请求↑→完成↑→请求↓→完成↓；急停先清旧完成，再确认本次停稳。
// 保留原位置投影 / CORDIC / PID / 减速算法，控制计算分拍，停车抢占不等 i_tick。
module nav_point_ctrl #(
    parameter integer ARRIVE_R_MM       = 25,
    parameter integer STOP_V_EPS        = 50,
    parameter integer STOP_W_EPS        = 300,
    parameter integer STOP_SETTLE_TICKS = 10,
    parameter integer TIMEOUT_TICKS     = 30000,
    parameter integer MAX_DIFF          = 612,
    parameter integer OM_CLIP           = 3000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               i_cmd_valid,
    input  wire               i_emergency_stop,
    input  wire signed [31:0] i_target_x_mm,
    input  wire signed [31:0] i_target_y_mm,
    input  wire               i_tick,
    input  wire signed [31:0] i_x_mm,
    input  wire signed [31:0] i_y_mm,
    input  wire signed [31:0] i_yaw_centi,
    input  wire signed [31:0] i_v_mms,
    input  wire        [3:0]  i_stall,
    input  wire signed [31:0] i_v_cruise,
    input  wire signed [31:0] i_d_slow,
    input  wire signed [31:0] i_align_in,
    input  wire signed [31:0] i_align_out,
    input  wire signed [31:0] i_max_diff_cruise,
    input  wire signed [31:0] i_kp_yaw,
    input  wire signed [31:0] i_ki_yaw,
    input  wire signed [31:0] i_kd_yaw,
    input  wire signed [31:0] i_i_limit,
    input  wire signed [31:0] i_k_slow_q10,
    output wire               o_auto_valid,
    output reg signed [31:0]  o_center,
    output reg signed [31:0]  o_diff,
    output reg                o_done,
    output reg                o_linear_active,
    output reg                o_fault
);
    localparam [2:0] IDLE=0, ALIGN=1, TRACK=2, STOP=3, DONE=4, FAULT=5, ESTOP=6;
    localparam [3:0] M_IDLE=0, M_INDEX=1, M_PROJECT=2, M_VECTOR=3,
                     M_SLOW=4, M_SPEED=5, M_COUNTS=6, M_ANGLE=7, M_PID=8, M_APPLY=9;
    localparam [31:0] R2 = ARRIVE_R_MM * ARRIVE_R_MM;
    reg [2:0] state;
    reg [3:0] phase;
    reg estop_seen, reorient;
    reg [15:0] settle_count, elapsed_count;
    assign o_auto_valid = i_emergency_stop || (i_cmd_valid && state != FAULT);

    // 航向增量与停稳反馈不依赖是否有目标，急停期间也持续更新。
    reg signed [31:0] yaw_previous, dyaw_latched, omega;
    reg omega_pending;
    wire signed [31:0] dyaw_raw = i_yaw_centi - yaw_previous;
    wire signed [31:0] dyaw = dyaw_raw > 18000 ? dyaw_raw - 36000 :
                              dyaw_raw < -18000 ? dyaw_raw + 36000 : dyaw_raw;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_previous <= 0;
            dyaw_latched <= 0;
            omega <= 0;
            omega_pending <= 0;
        end else begin
            omega_pending <= i_tick;
            if (i_tick) begin
                yaw_previous <= i_yaw_centi;
                dyaw_latched <= dyaw;
            end
            if (omega_pending) omega <= dyaw_latched * 32'sd100;
        end
    end
    wire still = i_v_mms <= STOP_V_EPS && i_v_mms >= -STOP_V_EPS &&
                 omega <= STOP_W_EPS && omega >= -STOP_W_EPS;

    reg signed [31:0] dx, dy, yaw_q, index_product;
    wire [7:0] sin_index = index_product[7:0];
    wire [7:0] cos_index = sin_index + 8'd64;
    wire signed [15:0] sin_value, cos_value;
    sin_lut_rom u_sin(.i_idx(sin_index),.o_val(sin_value));
    sin_lut_rom u_cos(.i_idx(cos_index),.o_val(cos_value));
    reg signed [31:0] dx_cos, dy_sin, dx_sin, dy_cos;
    reg signed [31:0] ahead_q8, left_q8;
    wire signed [31:0] ahead_mm = ahead_q8 >>> 8;
    wire signed [31:0] left_mm  = left_q8 >>> 8;
    reg [31:0] ahead_square, left_square, distance_square;
    reg signed [31:0] slow_speed, speed_q10, speed_counts;
    wire signed [31:0] cruise_q10 = i_v_cruise <<< 10;
    wire signed [31:0] limited_speed = speed_q10 > cruise_q10 ? cruise_q10 : speed_q10;
    wire signed [47:0] count_product = limited_speed * 16'sd3046;
    wire signed [15:0] ahead16 = ahead_q8[23:8];
    wire signed [15:0] slow_gain = i_k_slow_q10[15:0];

    reg angle_start;
    wire angle_valid, angle_zero;
    wire signed [15:0] angle;
    reg signed [15:0] psi;
    wire signed [31:0] psi_abs = psi < 0 ? -$signed({{16{psi[15]}},psi}) : $signed({{16{psi[15]}},psi});
    cordic_atan2 u_angle(.clk(clk),.rst_n(rst_n),.i_start(angle_start),
        .i_x(ahead_q8),.i_y(left_q8),.o_angle(angle),.o_valid(angle_valid),.o_zero(angle_zero));

    // 参数与积分限幅保留寄存边界，避免乘加、限幅和状态选择串成一拍。
    reg signed [31:0] integral, integral_sum, integral_limit, limited_integral, pid;
    wire signed [15:0] kp = i_kp_yaw[15:0], ki = i_ki_yaw[15:0], kd = i_kd_yaw[15:0];
    wire signed [31:0] omega_limited = omega > OM_CLIP ? OM_CLIP : omega < -OM_CLIP ? -OM_CLIP : omega;
    wire signed [15:0] omega16 = omega_limited[15:0];
    wire signed [31:0] p_term = (psi * kp) >>> 8;
    wire signed [31:0] i_increment = (angle * ki) >>> 8;
    wire signed [31:0] d_term = (omega16 * kd) >>> 8;
    wire signed [31:0] integral_clamped = integral_sum > integral_limit ? integral_limit :
                                          integral_sum < -integral_limit ? -integral_limit : integral_sum;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integral_limit <= 120;
            integral_sum <= 0;
        end else begin
            integral_limit <= i_i_limit;
            integral_sum <= integral + i_increment;
        end
    end
    function signed [31:0] clamp;
        input signed [31:0] value, limit;
        begin
            clamp = value > limit ? limit : value < -limit ? -limit : value;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            phase <= M_IDLE;
            estop_seen <= 0;
            reorient <= 0;
            settle_count <= 0;
            elapsed_count <= 0;
            o_center <= 0;
            o_diff <= 0;
            o_done <= 0;
            o_linear_active <= 0;
            o_fault <= 0;
            dx <= 0;
            dy <= 0;
            yaw_q <= 0;
            index_product <= 0;
            dx_cos <= 0;
            dy_sin <= 0;
            dx_sin <= 0;
            dy_cos <= 0;
            ahead_q8 <= 0;
            left_q8 <= 0;
            ahead_square <= 0;
            left_square <= 0;
            distance_square <= 0;
            slow_speed <= 0;
            speed_q10 <= 0;
            speed_counts <= 0;
            angle_start <= 0;
            psi <= 0;
            integral <= 0;
            limited_integral <= 0;
            pid <= 0;
        end else begin
            angle_start <= 0;
            o_fault <= 0;
            estop_seen <= i_emergency_stop;
            if (i_emergency_stop) begin
                // 抢占旧计算；一次新急停必须先输出低完成，之后才确认停稳。
                phase <= M_IDLE;
                state <= ESTOP;
                integral <= 0;
                o_center <= 0;
                o_diff <= 0;
                elapsed_count <= 0;
                if (!estop_seen) begin o_done <= 0; settle_count <= 0; end
                else if (i_tick && !o_done) begin
                    if (!still) settle_count <= 0;
                    else if (settle_count >= STOP_SETTLE_TICKS-1) begin
                        o_done <= 1;
                        o_linear_active <= 0;
                    end else settle_count <= settle_count + 1'b1;
                end
            end else if (!i_cmd_valid || state == ESTOP) begin
                state <= IDLE;
                phase <= M_IDLE;
                integral <= 0;
                o_center <= 0;
                o_diff <= 0;
                o_done <= 0;
                o_linear_active <= 0;
                settle_count <= 0;
                elapsed_count <= 0;
            end else begin
                if (state == IDLE) state <= ALIGN;
                case (phase)
                    M_IDLE: if (i_tick && state != DONE && state != FAULT) begin
                        dx <= i_target_x_mm - i_x_mm;
                        dy <= i_target_y_mm - i_y_mm;
                        yaw_q <= i_yaw_centi;
                        phase <= M_INDEX;
                    end
                    M_INDEX: begin index_product <= (yaw_q * 32'sd7457) >>> 20; phase <= M_PROJECT; end
                    M_PROJECT: begin
                        dx_cos <= dx * cos_value;
                        dy_sin <= dy * sin_value;
                        dx_sin <= dx * sin_value;
                        dy_cos <= dy * cos_value;
                        phase <= M_VECTOR;
                    end
                    M_VECTOR: begin
                        ahead_q8 <= (dx_cos + dy_sin) >>> 7;
                        left_q8 <= (dy_cos - dx_sin) >>> 7;
                        phase <= M_SLOW;
                    end
                    M_SLOW: begin
                        slow_speed <= ahead16 * slow_gain;
                        ahead_square <= ahead_mm * ahead_mm;
                        left_square <= left_mm * left_mm;
                        phase <= M_SPEED;
                    end
                    M_SPEED: begin
                        speed_q10 <= ahead_mm >= i_d_slow ? cruise_q10 : ahead_mm > 0 ? slow_speed : 0;
                        distance_square <= ahead_square + left_square;
                        phase <= M_COUNTS;
                    end
                    M_COUNTS: begin speed_counts <= count_product >>> 20; angle_start <= 1; phase <= M_ANGLE; end
                    M_ANGLE: if (angle_valid) begin psi <= angle; limited_integral <= integral_clamped; phase <= M_PID; end
                    M_PID: begin
                        pid <= p_term + limited_integral - d_term;
                        if (state == ALIGN || state == TRACK) integral <= limited_integral;
                        phase <= M_APPLY;
                    end
                    M_APPLY: begin
                        phase <= M_IDLE;
                        elapsed_count <= elapsed_count + 1'b1;
                        case (state)
                            ALIGN: begin
                                o_center <= 0;
                                o_diff <= clamp(pid, MAX_DIFF);
                                if (psi_abs <= i_align_in) begin
                                    state <= TRACK;
                                    integral <= 0;
                                    o_linear_active <= 1;
                                end
                            end
                            TRACK: begin
                                if (distance_square <= R2 || psi_abs > i_align_out) begin
                                    state <= STOP;
                                    settle_count <= 0;
                                    reorient <= distance_square > R2;
                                    o_center <= 0;
                                    o_diff <= 0;
                                end else begin
                                    o_center <= speed_counts;
                                    o_diff <= clamp(pid, i_max_diff_cruise);
                                end
                            end
                            STOP: begin
                                o_center <= 0;
                                o_diff <= 0;
                                // 同向目标延长时，不把旧终点的停车确认当作新终点到达。
                                if (!reorient && distance_square > R2) state <= TRACK;
                                else if (!still) settle_count <= 0;
                                else if (settle_count >= STOP_SETTLE_TICKS-1) begin
                                    o_linear_active <= 0;
                                    integral <= 0;
                                    if (reorient) state <= ALIGN;
                                    else begin state <= DONE; o_done <= 1; end
                                end else settle_count <= settle_count + 1'b1;
                            end
                            default: begin o_center <= 0; o_diff <= 0; end
                        endcase
                        if (i_stall != 0 || elapsed_count >= TIMEOUT_TICKS) begin
                            state <= FAULT;
                            o_fault <= 1;
                            o_center <= 0;
                            o_diff <= 0;
                        end
                    end
                    default: phase <= M_IDLE;
                endcase
            end
        end
    end
endmodule
