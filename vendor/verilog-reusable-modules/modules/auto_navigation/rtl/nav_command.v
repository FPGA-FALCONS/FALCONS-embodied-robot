// ============================================================================
// 模块：nav_command
// ----------------------------------------------------------------------------
// 职责：
//   - 向外部控制器发目标点任务及四相电平请求，转向和直行由控制器完成。
//   - 数据无效、撤权、直行前探未放行或新方向不兼容时抢占停车。
//   - 输出当前方向最远已验证终点；不生成 v/ω，不重复判断控制器停稳。
// 接口约定：
//   - 单时钟、异步低有效复位；目标位置用 signed mm。
//   - segment_dir：0 东起逆时针 0..7；segment_valid 覆盖坐标和元数据对齐。
//   - 正常 cmd_valid↑→ctrl_done↑→cmd_valid↓→ctrl_done↓；done 是保持电平。
//   - 急停先等 done=0 再等新的 done=1；有可执行动作才撤急停，再等 done=0。
//   - 同方向纵向终点允许实时更新；ctrl_linear_active 表示即将或正在平移。
// 关键不变量：
//   - 正常任务不在未完成时换方向；emergency_stop 抢占优先于同拍正常完成。
//   - descriptor 短暂无效只暂停目标更新，不能撤销正在执行的动作。
//   - 两轴均输出计划目标；推进只接受该次控制器完成，不复核实际位置或角度。
//   - arrived 只对真正最终目标成立，前缀末点停稳后等待路线延长。
// 禁止事项：
//   - 不要将旧 done 高电平直接当成抢占停车完成，不要用 ctrl_done 的单拍脉冲代替握手。
//   - 不要将 stopped 当物理速度反馈，它是本模块已完成停止握手后的内部状态。
// ============================================================================
module nav_command (
    input  wire               clk,
                              reset_n,
    input  wire               enable,
                              sensors_valid,
                              route_valid,
    input  wire               segment_valid,
                              segment_last,
                              segment_final,
    // 0 东、1 东北、2 北、3 西北、4 西、5 西南、6 南、7 东南
    input  wire         [2:0] segment_dir,
    input  wire signed [31:0] segment_x_mm,
                              segment_y_mm,
    input  wire        [15:0] segment_index,
    input  wire               forward_clear,
                              ctrl_linear_active,
                              ctrl_done,
    output wire               cmd_valid,
    output reg  signed [31:0] target_x_mm,
                              target_y_mm,
    // 显示当前任务线型：0 水平、1 正45°、2 竖直、3 负45°，仅 cmd_valid 有效时使用。
    output wire         [1:0] target_line_dir,
    output wire               emergency_stop,
    output reg                advance,
    output reg         [15:0] advance_index,
    output reg                arrived,
                              at_target,
    output wire               stopped
);
    // STOP_LOW/STOP_DONE 强制本次急停低→高确认；HOLD 等待可执行动作。
    // RELEASE 完成请求撤销后的 done 回零；SELECT 才解释下一份对齐描述符。
    localparam [3:0] IDLE         = 0,
                     STOP_LOW     = 1,
                     STOP_DONE    = 2,
                     HOLD         = 3,
                     RELEASE      = 4,
                     SELECT       = 5,
                     MOVE         = 6,
                     WAIT_SEGMENT = 7,
                     FINISHED     = 8,
                     MOVE_RELEASE = 9;
    reg [3:0] state;
    reg [2:0] command_dir;
    reg signed [31:0] sx_q, sy_q;
    reg [15:0] index_q, command_index;
    reg [2:0] dir_q;
    reg last_q, final_q, command_last, command_final;
    reg segment_valid_q;
    // 描述符寄存后再执行，路线 RAM 和毫米换算不叠进控制寄存器使能。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sx_q            <= 0;
            sy_q            <= 0;
            index_q         <= 0;
            dir_q           <= 0;
            last_q          <= 0;
            final_q         <= 0;
            segment_valid_q <= 0;
        end else begin
            sx_q            <= segment_x_mm;
            sy_q            <= segment_y_mm;
            index_q         <= segment_index;
            dir_q           <= segment_dir;
            last_q          <= segment_last;
            final_q         <= segment_final;
            segment_valid_q <= segment_valid;
        end
    end
    wire usable = segment_valid && segment_valid_q && route_valid;
    wire active = state == MOVE;
    wire data_fault = !enable || !sensors_valid || !route_valid;
    wire incompatible = usable && dir_q != command_dir;
    assign cmd_valid = reset_n && active;
    assign target_line_dir = command_dir[1:0];
    assign emergency_stop = reset_n && (state == STOP_LOW || state == STOP_DONE || state == HOLD);
    assign stopped = state == HOLD || state == RELEASE || state == SELECT ||
        state == WAIT_SEGMENT || state == FINISHED || state == MOVE_RELEASE;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state           <= IDLE;
            command_dir     <= 0;
            command_index   <= 0;
            command_last    <= 0;
            command_final   <= 0;
            target_x_mm     <= 0;
            target_y_mm     <= 0;
            advance         <= 0;
            advance_index   <= 0;
            at_target       <= 0;
            arrived         <= 0;
        end else begin
            advance <= 0;
            at_target <= 0;
            if (!enable) arrived <= 0;
            // 停车抢占早于正常完成，旧完成不用于本次停车确认。
            if (active && (data_fault || incompatible || (ctrl_linear_active && !forward_clear)))
                state <= STOP_LOW;
            else
                case (state)
                    IDLE: if (enable) begin
                        state <= STOP_LOW;
                    end
                    STOP_LOW: if (!ctrl_done) state <= STOP_DONE;
                    STOP_DONE: if (ctrl_done) state <= HOLD;
                    HOLD:
                    if (!enable || (sensors_valid && usable))
                        state <= RELEASE;
                    RELEASE: if (!ctrl_done) state <= enable ? SELECT : IDLE;
                    SELECT: begin
                        if (!enable) state <= IDLE;
                        else if (!sensors_valid || !route_valid) begin
                            state <= HOLD;
                        end
                        else if (usable && !ctrl_done) begin
                            command_dir   <= dir_q;
                            target_x_mm   <= sx_q;
                            target_y_mm   <= sy_q;
                            command_index <= index_q;
                            command_last  <= last_q;
                            command_final <= final_q;
                            state         <= MOVE;
                        end
                    end
                    MOVE: begin
                        if (ctrl_done) begin
                            at_target <= 1;
                            advance_index <= command_index;
                            advance <= !command_final;
                            state <= MOVE_RELEASE;
                        end
                        else if (usable && dir_q == command_dir) begin
                            target_x_mm   <= sx_q;
                            target_y_mm   <= sy_q;
                            command_index <= index_q;
                            command_last  <= last_q;
                            command_final <= final_q;
                        end
                    end
                    MOVE_RELEASE: if (!ctrl_done) begin
                        if (!enable) state <= IDLE;
                        else if (command_final) begin
                            arrived <= 1;
                            state <= FINISHED;
                        end else state <= command_last ? WAIT_SEGMENT : SELECT;
                    end
                    WAIT_SEGMENT:
                    if (!enable) state <= IDLE;
                    else if (!sensors_valid || !route_valid) begin
                        state <= HOLD;
                    end else if (usable) begin
                        if (sx_q != target_x_mm || sy_q != target_y_mm) state <= SELECT;
                        else if (!last_q) begin
                            advance <= 1;
                            advance_index <= index_q;
                            state <= SELECT;
                        end
                    end
                    FINISHED:
                    if (!enable) state <= IDLE;
                    else if (!route_valid) begin
                        arrived <= 0;
                        state   <= STOP_LOW;
                    end
                    default: state <= IDLE;
                endcase
        end
    end
endmodule
