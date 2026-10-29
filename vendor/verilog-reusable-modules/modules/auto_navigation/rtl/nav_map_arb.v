// ============================================================================
// 模块：nav_map_arb
// ----------------------------------------------------------------------------
// 职责：
//   - 将三个客户端复用至单地图读口；0 raw 前探、1 规划/验证、2 地图显示。
//   - 同拍请求按低编号优先，先寄存选择/地址，再授权；只允许一个未响应读事务。
//   - 响应超时锁存故障并禁止新授权，避免迟到响应串到其他客户端。
// 接口约定：
//   - 单时钟、异步低有效复位；每组 req/enable 各一位，坐标总线按客户端号每 16 位打包。
//   - gnt 在 map_valid && map_ready 的接受拍产生；rsp_valid 仅发给已锁存 owner。
//   - map_rsp_valid 是返回事件，数据不能按未成交请求的当前地址解释。
// 关键不变量：
//   - 接受后锁定 owner 和地址直到响应排空；TMO 只计已接受请求等待响应的周期。
//   - err_timeout 保持到复位；前探按有限窗口和轮询节拍让出读口。
// 禁止事项：
//   - 不要在响应之前切 owner，也不要用显示请求反压雷达观测。
// ============================================================================
module nav_map_arb #(
    parameter integer TMO = 50000
) (
    input  wire               clk,
                              reset_n,
    input  wire         [2:0] req,
                              enable,
    input  wire        [47:0] req_x,
                              req_y,
    output wire         [2:0] gnt,
                              rsp_valid,
    output wire               rsp_cost,
                              rsp_raw,
    output wire               map_valid,
    output wire        [15:0] map_x,
                              map_y,
    input  wire               map_ready,
                              map_rsp_valid,
                              map_cost,
                              map_raw,
    output wire               busy_o,
    output reg                err_timeout
);
    reg busy, armed;
    reg [1:0] owner, pick, selected;
    reg pick_valid;
    reg [15:0] address_x, address_y;
    reg [31:0] wait_cycles;
    always @* begin
        pick       = 0;
        pick_valid = 0;
        if (req[0] && enable[0]) begin
            pick       = 0;
            pick_valid = 1;
        end else if (req[1] && enable[1]) begin
            pick       = 1;
            pick_valid = 1;
        end else if (req[2] && enable[2]) begin
            pick       = 2;
            pick_valid = 1;
        end
    end
    // One registered selection stage isolates clients from the combinational
    // priority chain. A withdrawn/changed request is cancelled before grant.
    wire [2:0] live;
    genvar client;
    generate for(client=0;client<3;client=client+1)begin:g_live
        assign live[client] = armed && selected==client && req[client] && enable[client] &&
            req_x[client*16+:16]==address_x && req_y[client*16+:16]==address_y;
    end endgenerate
    wire selected_live=|live;
    assign map_valid = reset_n && !busy && !err_timeout && selected_live;
    assign map_x     = address_x;
    assign map_y     = address_y;
    // Each grant depends only on that client's current request. Sharing a
    // priority-mux valid here would recreate the long inter-client path.
    assign gnt = live & {3{reset_n && !busy && !err_timeout && map_ready}};
    assign rsp_valid = reset_n && busy && map_rsp_valid ? (3'b001 << owner) : 3'b000;
    assign rsp_raw   = map_raw;
    assign rsp_cost  = map_cost;
    assign busy_o    = busy; // Only accepted reads block an atomic layer commit.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy        <= 0;
            armed       <= 0;
            selected    <= 0;
            owner       <= 0;
            address_x   <= 0;
            address_y   <= 0;
            wait_cycles <= 0;
            err_timeout <= 0;
        end else if (!busy) begin
            if (armed) begin
                if (!selected_live) armed <= 0;
                else if (map_ready && !err_timeout) begin
                    busy <= 1;
                    armed <= 0;
                    owner <= selected;
                    wait_cycles <= 0;
                end
            end else if (pick_valid && !err_timeout) begin
                armed <= 1;
                selected <= pick;
                address_x <= req_x[pick*16+:16];
                address_y <= req_y[pick*16+:16];
            end
        end else if (map_rsp_valid) begin
            busy        <= 0;
            wait_cycles <= 0;
        end else if (!err_timeout) begin
            if (wait_cycles == TMO - 1) err_timeout <= 1;
            else wait_cycles <= wait_cycles + 1'b1;
        end
    end
endmodule
