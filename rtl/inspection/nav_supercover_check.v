`timescale 1ns/1ps
// Standalone any-angle segment checker. NOT connected to production nav_top.
// Coordinates are cell centers; blocked cells are closed unit squares in an
// ALREADY inflated configuration-space map. Corner touches check both side cells.
// Map contract: one outstanding request, response strictly after acceptance;
// map_revision changes on ANY map write, and cannot wrap during a transaction.
// After cancel/stale, drain an accepted read before returning a result. Memory
// must eventually respond; there is no fabricated clear result on a timeout.
// Result stays immutable under backpressure. Consumer MUST compare its revision
// with the current map at acceptance AND before use. Reset requires memory flush.
// Addition/comparison only: no divider, multiplier or path RAM.
module nav_supercover_check #(
    parameter MAP_WIDTH = 256,
    parameter MAP_HEIGHT = 256
) (
    input wire clk, reset_n, cancel,
    input wire start_valid,
    output wire start_ready,
    input wire [15:0] start_x, start_y, end_x, end_y,
    input wire [31:0] map_revision,
    output wire map_req_valid,
    input wire map_req_ready,
    output wire [7:0] map_x, map_y,
    input wire map_resp_valid, map_blocked,
    output wire result_valid,
    input wire result_ready,
    // 0 clear, 1 blocked, 2 stale map, 3 invalid endpoint, 4 cancelled.
    output reg [2:0] result_code,
    output reg [31:0] result_revision,
    output reg [10:0] checked_cells,
    output wire busy
);
    localparam IDLE=0, ISSUE=1, WAIT_READ=2, STEP=3, DRAIN=4, DONE=5;
    reg [2:0] state;
    reg [7:0] cx, cy, gx, gy, qx, qy;
    reg positive_x, positive_y;
    reg [1:0] phase; // 0 main cell, 1 x-side of corner, 2 y-side.
    reg signed [18:0] error_q, twice_dx, twice_dy;
    wire [15:0] dx = end_x >= start_x ? end_x-start_x : start_x-end_x;
    wire [15:0] dy = end_y >= start_y ? end_y-start_y : start_y-end_y;
    wire [7:0] next_x = positive_x ? cx+8'd1 : cx-8'd1;
    wire [7:0] next_y = positive_y ? cy+8'd1 : cy-8'd1;
    wire active = state==ISSUE || state==WAIT_READ || state==STEP;
    assign busy = state != IDLE;
    assign start_ready = reset_n && !cancel && state==IDLE;
    assign map_req_valid = reset_n && !cancel && state==ISSUE &&
                           map_revision==result_revision;
    assign map_x = qx;
    assign map_y = qy;
    assign result_valid = reset_n && state==DONE;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; cx<=0; cy<=0; gx<=0; gy<=0; qx<=0; qy<=0;
            positive_x<=0; positive_y<=0; phase<=0;
            error_q<=0; twice_dx<=0; twice_dy<=0;
            result_code<=0; result_revision<=0; checked_cells<=0;
        end else if (active && (cancel || map_revision!=result_revision)) begin
            result_code <= cancel ? 3'd4 : 3'd2;
            // ISSUE suppresses a new request combinationally on abort.
            state <= state==WAIT_READ && !map_resp_valid ? DRAIN : DONE;
        end else case (state)
            IDLE: if (start_valid && start_ready) begin
                result_revision<=map_revision; checked_cells<=0;
                result_code<=0; phase<=0;
                if (MAP_WIDTH<1 || MAP_WIDTH>256 || MAP_HEIGHT<1 || MAP_HEIGHT>256 ||
                    start_x>=MAP_WIDTH || end_x>=MAP_WIDTH ||
                    start_y>=MAP_HEIGHT || end_y>=MAP_HEIGHT) begin
                    result_code<=3'd3; state<=DONE;
                end else begin
                    cx<=start_x[7:0]; cy<=start_y[7:0];
                    qx<=start_x[7:0]; qy<=start_y[7:0];
                    gx<=end_x[7:0]; gy<=end_y[7:0];
                    positive_x<=end_x>start_x; positive_y<=end_y>start_y;
                    error_q <= $signed({3'b0,dy})-$signed({3'b0,dx});
                    twice_dx <= $signed({2'b0,dx,1'b0});
                    twice_dy <= $signed({2'b0,dy,1'b0});
                    state<=ISSUE;
                end
            end
            ISSUE: if (map_req_valid && map_req_ready) begin
                checked_cells<=checked_cells+11'd1; state<=WAIT_READ;
            end
            WAIT_READ: if (map_resp_valid) begin
                if (map_blocked) begin result_code<=3'd1; state<=DONE; end
                else state<=STEP;
            end
            STEP: begin
                if (phase==1) begin
                    qx<=cx; qy<=next_y; phase<=2; state<=ISSUE;
                end else if (phase==2) begin
                    cx<=next_x; cy<=next_y; qx<=next_x; qy<=next_y;
                    error_q<=error_q+twice_dy-twice_dx; phase<=0; state<=ISSUE;
                end else if (cx==gx && cy==gy) begin
                    state<=DONE;
                end else if (error_q<0) begin
                    cx<=next_x; qx<=next_x; qy<=cy;
                    error_q<=error_q+twice_dy; state<=ISSUE;
                end else if (error_q>0) begin
                    cy<=next_y; qx<=cx; qy<=next_y;
                    error_q<=error_q-twice_dx; state<=ISSUE;
                end else begin
                    qx<=next_x; qy<=cy; phase<=1; state<=ISSUE;
                end
            end
            DRAIN: if (map_resp_valid) state<=DONE;
            DONE: if (result_ready) state<=IDLE;
            default: begin state<=IDLE; result_code<=3'd4; end
        endcase
    end
endmodule
