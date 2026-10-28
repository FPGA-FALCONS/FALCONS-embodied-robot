// ============================================================================
// 模块：nav_segments
// ----------------------------------------------------------------------------
// 职责：
//   - 保留候选路线的全部短段边界，逐点登记方向。
//   - 单项寄存缓冲；输出方向与坐标一起进入路线 RAM。
//   - 不读地图、不校验可通行性；非八方向输入由候选完整验证拒绝。
// 接口约定：
//   - 单时钟、异步低有效复位；start 锁存本轮起格及承诺方向，cancel 清空未发候选。
//   - in/out 各为 valid/ready 流，坐标用格号，last 是本轮末点，不保证为最终目标。
//   - out_dir：0 东起逆时针 0..7；背压时输出保持。
// 关键不变量：
//   - 承诺拐点与当前位置同格也不能删除其未完成方向；start_dir_valid 保留这个信息。
//   - 同向中继点也必须保留；最远同向目标只由路线读出端选择。
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
    input  wire         [2:0] start_dir,
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
    output wire         [2:0] out_dir,
    output wire               busy
);
    reg valid_q, prefix_q;
    reg [15:0] ax, ay, px, py;
    reg [2:0] prefix_dir, dir_q;
    reg last_q;

    function [2:0] direction;
        input [15:0] x0, y0, x1, y1;
        begin
            direction = y0 == y1 ? (x1 >= x0 ? 3'd0 : 3'd4) :
                        x0 == x1 ? (y1 >= y0 ? 3'd2 : 3'd6) :
                        x1 > x0 ? (y1 > y0 ? 3'd1 : 3'd7) :
                                  (y1 > y0 ? 3'd3 : 3'd5);
        end
    endfunction

    assign in_ready  = reset_n && !start && !cancel && (!valid_q || out_ready);
    assign out_valid = reset_n && !start && !cancel && valid_q;
    assign out_x     = px;
    assign out_y     = py;
    assign out_last  = last_q;
    assign out_dir   = dir_q;
    assign busy      = valid_q;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            valid_q    <= 0;
            prefix_q   <= 0;
            ax         <= 0;
            ay         <= 0;
            px         <= 0;
            py         <= 0;
            last_q     <= 0;
            prefix_dir <= 0;
            dir_q      <= 0;
        end else if (start || cancel) begin
            valid_q    <= 0;
            ax         <= start_x;
            ay         <= start_y;
            prefix_q   <= start && !cancel && start_dir_valid;
            prefix_dir <= start_dir;
        end else if (in_ready) begin
            valid_q <= in_valid;
            if (in_valid) begin
                px       <= in_x;
                py       <= in_y;
                last_q   <= in_last;
                dir_q    <= prefix_q ? prefix_dir : direction(ax, ay, in_x, in_y);
                ax       <= in_x;
                ay       <= in_y;
                prefix_q <= 0;
            end
        end
    end
endmodule
