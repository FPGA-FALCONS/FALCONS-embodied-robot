// ============================================================================
// 模块：nav_map
// ----------------------------------------------------------------------------
// 职责：
//   - 保存 raw 原始占用和 cost 两张一位图，未知格初始化为可通行，图外读取为阻挡。
//   - raw 置位时写入 ±INFLATE 方形 cost；raw 清除时逐格重算邻域 OR，保留重叠膨胀。
//   - 同格同值写只探测不重算；occupied_cells 统计当前 raw=1 的格数。
// 接口约定：
//   - 单时钟、异步低有效复位；复位释放后逐拍清 N=GRID_W*GRID_H 格，期间读写不 ready。
//   - wr_valid && wr_ready 接受一笔写；wr_val=1 占用、0 清空，越界写不修改地图。
//   - rd_valid && rd_ready 接受地址，一拍后 rd_rsp_valid；raw/cost 同步返回。
//   - idle 只表示可接受事务；完整更新期间背压新读写，map_version 在实际改变事务结束后加一。
// 关键不变量：
//   - 只在完整事务边界对外开放读口；读写可同沿接受，此时读到该写入前的地图。
//   - raw 清除必须重算周围所有受影响 cost，不能直接抹掉其他障碍的膨胀。
// 禁止事项：
//   - 不要将 cost 当急停判据；不要将 map_version 当雷达报文数。
// ============================================================================
module nav_map #(
    parameter integer GRID_W  = 256,
                      GRID_H  = 256,
                      INFLATE = 2
) (
    input  wire               clk,
                              reset_n,
    input  wire               wr_valid,
    output wire               wr_ready,
    input  wire               wr_val,
    input  wire        [15:0] wr_x,
                              wr_y,
    input  wire               rd_valid,
    output wire               rd_ready,
    input  wire        [15:0] rd_x,
                              rd_y,
    output reg                rd_rsp_valid,
                              rd_raw,
                              rd_cost,
    output wire               idle,
    output reg         [31:0] occupied_cells,
    output wire        [15:0] map_version
);
    localparam integer N  = GRID_W * GRID_H,
                       IW = 2 * INFLATE + 1;
    localparam integer CW = IW < 2 ? 1 : $clog2(IW);
    localparam [2:0] INIT       = 0,
                     IDLE       = 1,
                     PROBE      = 2,
                     APPLY      = 3,
                     TARGET     = 4,
                     NEIGHBOR   = 5,
                     TAKE       = 6,
                     WRITE_COST = 7;
    reg [2:0] state;
    reg raw[0:N-1], cost[0:N-1];
    reg [15:0] init_addr, wx, wy, version;
    reg value_q, aux_raw, any_raw;
    reg [CW-1:0] ti, tj, ni, nj;
    function [15:0] addr_of;
        input [15:0] x, y;
        reg     [31:0] acc;
        integer        b;
        begin
            acc = 0;
            for (b = 0; b < 17; b = b + 1) if ((GRID_W >> b) & 1) acc = acc + ({16'd0, y} << b);
            addr_of = acc[15:0] + x;
        end
    endfunction
    wire signed [16:0] target_x = $signed({1'b0, wx}) + $signed({1'b0, ti}) - INFLATE;
    wire signed [16:0] target_y = $signed({1'b0, wy}) + $signed({1'b0, tj}) - INFLATE;
    wire target_inside = target_x >= 0 && target_x < GRID_W && target_y >= 0 && target_y < GRID_H;
    wire signed [16:0] near_x = target_x + $signed({1'b0, ni}) - INFLATE;
    wire signed [16:0] near_y = target_y + $signed({1'b0, nj}) - INFLATE;
    wire near_inside = near_x >= 0 && near_x < GRID_W && near_y >= 0 && near_y < GRID_H;
    wire accept_write = wr_valid && wr_ready && wr_x < GRID_W && wr_y < GRID_H;
    wire [15:0] target_addr = addr_of(target_x[15:0], target_y[15:0]);
    // 写探针与清障重算复用同一 raw 辅助读口，不生成多余 RAM 副本。
    wire aux_read = accept_write || (state == NEIGHBOR);
    wire [15:0] aux_addr = state == NEIGHBOR ?
        addr_of(near_x[15:0], near_y[15:0]) : addr_of(wr_x, wr_y);
    always @(posedge clk)
        if (aux_read)
            aux_raw <= (state == NEIGHBOR && !near_inside) ? 1'b0 : raw[aux_addr];
    wire raw_we = reset_n && (state == INIT || state == APPLY);
    wire [15:0] raw_wa = state == INIT ? init_addr : addr_of(wx, wy);
    wire raw_wd = state == INIT ? 1'b0 : value_q;
    wire cost_we = reset_n &&
        ((state == INIT) || (state == WRITE_COST) || (state == TARGET && value_q && target_inside));
    wire [15:0] cost_wa = state == INIT ? init_addr : target_addr;
    wire cost_wd = state == INIT ? 1'b0 : (value_q ? 1'b1 : any_raw);
    always @(posedge clk) begin
        if (raw_we) raw[raw_wa] <= raw_wd;
        if (cost_we) cost[cost_wa] <= cost_wd;
    end
    assign idle        = reset_n && state == IDLE;
    assign wr_ready    = idle;
    assign rd_ready    = idle;
    assign map_version = version;
    always @(posedge clk) begin
        rd_rsp_valid <= rd_valid && rd_ready;
        if (rd_valid && rd_ready) begin
            if (rd_x >= GRID_W || rd_y >= GRID_H) begin
                rd_raw  <= 1;
                rd_cost <= 1;
            end else begin
                rd_raw  <= raw[addr_of(rd_x, rd_y)];
                rd_cost <= cost[addr_of(rd_x, rd_y)];
            end
        end
    end
    task next_target;
        begin
            state <= TARGET;
            if (tj == IW - 1) begin
                tj <= 0;
                if (ti == IW - 1) begin
                    version <= version + 16'd1;
                    state   <= IDLE;
                end else ti <= ti + 1'b1;
            end else tj <= tj + 1'b1;
        end
    endtask
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state          <= INIT;
            init_addr      <= 0;
            wx             <= 0;
            wy             <= 0;
            value_q        <= 0;
            version        <= 0;
            occupied_cells <= 0;
            ti             <= 0;
            tj             <= 0;
            ni             <= 0;
            nj             <= 0;
            any_raw        <= 0;
        end else
            case (state)
                INIT:
                if (init_addr == N - 1) state <= IDLE;
                else init_addr <= init_addr + 16'd1;
                IDLE:
                if (accept_write) begin
                    wx      <= wr_x;
                    wy      <= wr_y;
                    value_q <= wr_val;
                    state   <= PROBE;
                end
                PROBE:
                if (aux_raw == value_q) state <= IDLE;
                else begin
                    occupied_cells <= value_q ? occupied_cells + 32'd1 : occupied_cells - 32'd1;
                    ti             <= 0;
                    tj             <= 0;
                    state          <= APPLY;
                end
                APPLY: state <= TARGET;
                TARGET:
                if (!target_inside || value_q) next_target;
                else begin
                    ni      <= 0;
                    nj      <= 0;
                    any_raw <= 0;
                    state   <= NEIGHBOR;
                end
                NEIGHBOR: state <= TAKE;
                TAKE: begin
                    if (aux_raw) begin
                        any_raw <= 1;
                        state   <= WRITE_COST;
                    end else if (ni == IW - 1 && nj == IW - 1) state <= WRITE_COST;
                    else begin
                        if (nj == IW - 1) begin
                            nj <= 0;
                            ni <= ni + 1'b1;
                        end else nj <= nj + 1'b1;
                        state <= NEIGHBOR;
                    end
                end
                WRITE_COST: next_target;
                default: state <= INIT;
            endcase
    end
endmodule
