// ============================================================================
// 模块：nav_segments
// ----------------------------------------------------------------------------
// 职责：
//   - 流式合并候选路线的同向点，保留拐角、折返和本轮前缀末点。
//   - 一个待发点和一个前看点；输出方向与坐标一起进入路线 RAM。
//   - 不读地图、不校验可通行性；非轴向输入由候选完整验证拒绝。
// 接口约定：
//   - 单时钟、异步低有效复位；start 锁存实际起格及承诺方向，cancel 清空未发候选。
//   - in/out 各为 valid/ready 流，坐标用格号，last 是本轮末点，不保证为最终目标。
//   - out_dir：0 东、1 北、2 西、3 南；背压时输出保持。
// 关键不变量：
//   - 承诺拐点与当前位置同格也不能删除其未完成方向；start_dir_valid 保留这个信息。
//   - 候选内部仅合并同轴同向点；锁定前缀单独输出，不能与后续段合并。
// 禁止事项：
//   - 不要在规划前置检查尚未结束时锁存 start；应与 BFS 请求真正接受同沿。
// ============================================================================
module nav_segments (
    input  wire               clk,
                              reset_n,
                              start,
                              cancel,
    input  wire        [15:0] start_x,
                              start_y,
    input  wire               start_dir_valid,
    input  wire         [1:0] start_dir,
    input  wire               in_valid,
    output wire               in_ready,
    input  wire        [15:0] in_x,
                              in_y,
    input  wire               in_last,
    output wire               out_valid,
    input  wire               out_ready,
    output wire        [15:0] out_x,
                              out_y,
    output wire               out_last,
    output wire         [1:0] out_dir,
    output wire               busy
);
    localparam [1:0] EMPTY   = 0,
                     COLLECT = 1,
                     EMIT    = 2;
    reg [1:0] state;
    reg [15:0] ax, ay, px, py, nx, ny;
    reg last_q, next_last;
    reg prefix_q;
    reg [1:0] prefix_dir;
    wire same_cell = ax == px && ay == py;
    wire duplicate = in_x == px && in_y == py;
    wire same_direction = (ay == py && in_y == py &&
                           ((px > ax && in_x >= px) || (px < ax && in_x <= px))) ||
        (ax == px && in_x == px && ((py > ay && in_y >= py) || (py < ay && in_y <= py)));
    // 已进入承诺拐点格，不代表连续位姿已到中心；仅同方向时允许合并。
    wire zero_prefix = same_cell && !prefix_q && (in_x == px || in_y == py);
    assign in_ready = reset_n && !start && !cancel && state != EMIT;
    assign out_valid = reset_n && !start && !cancel && state == EMIT;
    assign out_x = px;
    assign out_y = py;
    assign out_last = last_q;
    assign out_dir = prefix_q ? prefix_dir :
        ay == py ? (px >= ax ? 2'd0 : 2'd2) : (py >= ay ? 2'd1 : 2'd3);
    assign busy = state != EMPTY;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= EMPTY;
            ax         <= 0;
            ay         <= 0;
            px         <= 0;
            py         <= 0;
            nx         <= 0;
            ny         <= 0;
            last_q     <= 0;
            next_last  <= 0;
            prefix_q   <= 0;
            prefix_dir <= 0;
        end else if (start || cancel) begin
            state      <= EMPTY;
            ax         <= start_x;
            ay         <= start_y;
            prefix_q   <= start && !cancel && start_dir_valid;
            prefix_dir <= start_dir;
        end else
            case (state)
                EMPTY:
                if (in_valid && in_ready) begin
                    px     <= in_x;
                    py     <= in_y;
                    last_q <= in_last;
                    state  <= in_last ? EMIT : COLLECT;
                end
                COLLECT:
                if (in_valid && in_ready) begin
                    if (prefix_q && !duplicate) begin
                        nx        <= in_x;
                        ny        <= in_y;
                        next_last <= in_last;
                        last_q    <= 0;
                        state     <= EMIT;
                    end else if (duplicate || same_direction || zero_prefix) begin
                        px     <= in_x;
                        py     <= in_y;
                        last_q <= in_last;
                        if (in_last) state <= EMIT;
                    end else begin
                        nx        <= in_x;
                        ny        <= in_y;
                        next_last <= in_last;
                        last_q    <= 0;
                        state     <= EMIT;
                    end
                end
                EMIT:
                if (out_ready) begin
                    prefix_q <= 0;
                    if (last_q) state <= EMPTY;
                    else begin
                        ax     <= px;
                        ay     <= py;
                        px     <= nx;
                        py     <= ny;
                        last_q <= next_last;
                        state  <= next_last ? EMIT : COLLECT;
                    end
                end
                default: state <= EMPTY;
            endcase
    end
endmodule
