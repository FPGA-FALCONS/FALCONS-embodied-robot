// 同一圈内命中优先：命中立即写地图，清空请求先记下，换圈后再处理。
// 两组 hit/free 位图交替接收和清理；未被射线经过的历史障碍保持不变。
// 清理上一圈时仍检查本圈 hit，避免旧清空请求删除刚发现的新障碍。
module nav_scan_accumulator #(
    parameter integer GRID_W = 256,
    parameter integer GRID_H = 256
) (
    input  wire        clk,
    input  wire        reset_n,

    // 在第一条新圈射线开始前握手；上一条射线必须已经输出完毕。
    input  wire        scan_begin,
    output wire        scan_ready,

    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_value,
    input  wire [15:0] in_x,
    input  wire [15:0] in_y,

    output wire        wr_valid,
    input  wire        wr_ready,
    output wire        wr_value,
    output wire [15:0] wr_x,
    output wire [15:0] wr_y
);
    localparam integer N = GRID_W * GRID_H;
    localparam integer AW = N < 2 ? 1 : $clog2(N);
    localparam [2:0] INIT = 0, IDLE = 1, READ = 2,
                     CHECK = 3, CLEAR = 4, CLEAN = 5;

    reg [2:0] state;
    reg active_bank;
    reg [AW-1:0] address;
    reg [15:0] merge_x, merge_y;
    reg hit0 [0:N-1], free0 [0:N-1];
    reg hit1 [0:N-1], free1 [0:N-1];
    reg hit0_q, free0_q, hit1_q, free1_q;

    wire [AW-1:0] input_address = in_y * GRID_W + in_x;
    wire incoming_hit = state != INIT && in_valid && in_value;
    wire accepted = in_valid && in_ready;
    wire current_hit = active_bank ? hit1_q : hit0_q;
    wire previous_hit = active_bank ? hit0_q : hit1_q;
    wire previous_free = active_bank ? free0_q : free1_q;
    wire clear_request = state == CLEAR && !current_hit && !incoming_hit;

    assign scan_ready = reset_n && state == IDLE;
    assign in_ready = reset_n && state != INIT && (!in_value || wr_ready);
    assign wr_valid = reset_n && (incoming_hit || clear_request);
    assign wr_value = incoming_hit;
    assign wr_x = incoming_hit ? in_x : merge_x;
    assign wr_y = incoming_hit ? in_y : merge_y;

    // 四个位图各只有一个同步读口、一个写口，避免复制整张地图。
    always @(posedge clk) begin
        hit0_q <= hit0[address];
        free0_q <= free0[address];
        hit1_q <= hit1[address];
        free1_q <= free1[address];
        if (reset_n) begin
            if (state == INIT) begin
                hit0[address] <= 1'b0;
                free0[address] <= 1'b0;
            end else if (!active_bank && accepted) begin
                if (in_value) hit0[input_address] <= 1'b1;
                else free0[input_address] <= 1'b1;
            end else if (active_bank && state == CLEAN) begin
                hit0[address] <= 1'b0;
                free0[address] <= 1'b0;
            end

            if (state == INIT) begin
                hit1[address] <= 1'b0;
                free1[address] <= 1'b0;
            end else if (active_bank && accepted) begin
                if (in_value) hit1[input_address] <= 1'b1;
                else free1[input_address] <= 1'b1;
            end else if (!active_bank && state == CLEAN) begin
                hit1[address] <= 1'b0;
                free1[address] <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= INIT;
            active_bank <= 1'b0;
            address <= 0;
            merge_x <= 0;
            merge_y <= 0;
        end else begin
            case (state)
                INIT: begin
                    if (address == N - 1) begin
                        address <= 0;
                        state <= IDLE;
                    end else address <= address + 1'b1;
                end
                IDLE: begin
                    if (scan_begin) begin
                        active_bank <= !active_bank;
                        address <= 0;
                        merge_x <= 0;
                        merge_y <= 0;
                        state <= READ;
                    end
                end
                READ: state <= CHECK;
                CHECK: begin
                    if (previous_free && !previous_hit && !current_hit)
                        state <= CLEAR;
                    else state <= CLEAN;
                end
                CLEAR: begin
                    // 等待地图期间持续重读本圈命中位。命中写优先，且 nav_map
                    // 接受命中后经过 PROBE，恢复 ready 时已能读到最新命中位。
                    if (current_hit || (clear_request && wr_ready)) state <= CLEAN;
                end
                CLEAN: begin
                    if (address == N - 1) state <= IDLE;
                    else begin
                        address <= address + 1'b1;
                        if (merge_x == GRID_W - 1) begin
                            merge_x <= 0;
                            merge_y <= merge_y + 1'b1;
                        end else merge_x <= merge_x + 1'b1;
                        state <= READ;
                    end
                end
                default: state <= INIT;
            endcase
        end
    end
endmodule
