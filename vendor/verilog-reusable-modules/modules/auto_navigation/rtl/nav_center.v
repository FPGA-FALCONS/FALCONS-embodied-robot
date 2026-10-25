// 八方向三段窗口居中。A 为已输出点；B/C 待定，D 提供后一段方向。
// 只平移 B-C 的支撑线，以前后支撑线求新交点；不移动 A、D。
// 探针累计整段最小间隙，候选只有通过几何、预算和三段完整复核才采用。
module nav_center #(
    parameter integer GRID_W = 256, GRID_H = 256, CENTER_EN = 1,
                      L_MIN = 4, PROBE_D = 20, PROBE_STEP = 4,
                      CLEAR_TRIGGER = 8, PATH_INC_MAX = 20, BFS_RELAX = 4
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        start,
    input  wire        cancel,
    input  wire [15:0] start_x, start_y,
    input  wire        start_relaxed,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [15:0] in_x, in_y,
    input  wire        in_last,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [15:0] out_x, out_y,
    output wire        out_last,
    output reg         mrd_valid,
    input  wire        mrd_ready,
    output reg  [15:0] mrd_x, mrd_y,
    input  wire        mrd_rsp_valid, mrd_raw, mrd_cost,
    output wire        busy
);
    localparam [5:0] IDLE = 0, WAIT_B = 1, WAIT_C = 2, WAIT_D = 3, DIRECTIONS = 4,
    ELIGIBLE = 5, BASE = 6, PROBE_POINT = 7, PROBE_QUERY = 8, PROBE_TAKE = 9,
    RD_REQUEST = 10, RD_RESPONSE = 11, CANDIDATE = 12, SHIFT = 13, CONSTANTS = 14,
    INTERSECT = 15, GEOMETRY = 16, STEPS = 17, WEIGHTS = 18, SUMS = 19, DELTA = 20,
    TRIAL = 21, BUDGET = 22, LINE_BEGIN = 23, LINE_QUERY = 24, LINE_TAKE = 25,
    ACCEPT = 26, EMIT_B = 27, FLUSH_B = 28, FINAL = 29, PROBE_FLANK = 30, SUM_TAIL = 31;
    localparam integer DIAG_D = PROBE_D*181/256;
    localparam integer DIAG_STEP = PROBE_STEP*181/256>0 ? PROBE_STEP*181/256 : 1;
    localparam integer DIAG_MIN = (L_MIN*181+255)/256;
    localparam integer DIAG_TRIGGER = (CLEAR_TRIGGER*181+255)/256;
    reg [5:0] state, read_return;
    reg [15:0] ax, ay, bx, by, cx, cy, dx, dy, origin_x, origin_y;
    reg d_last;
    reg [2:0] prev_dir, mid_dir, next_dir;
    reg [15:0] run_len, probe_p;
    reg [4:0] radius, d_plus, d_minus;
    reg probe_side, outside;
    reg [1:0] probe_phase, line_index;
    reg signed [1:0] ux, uy, nx, ny;
    reg signed [17:0] base_x, base_y, probe_x, probe_y, shift_x, shift_y;
    reg signed [5:0] offset;
    reg signed [17:0] c_prev, c_mid, c_next;
    reg [36:0] intersection_b, intersection_c;
    reg [15:0] cand_bx, cand_by, cand_cx, cand_cy;
    reg [15:0] old0, old1, old2, new0, new1, new2;
    reg [17:0] old_sum, new_sum;
    reg signed [18:0] delta_length;
    reg signed [23:0] cumulative, trial_length;
    reg raw_q, cost_q;
    wire [4:0] depth = mid_dir[0] ? DIAG_D : PROBE_D;
    wire [15:0] probe_step = mid_dir[0] ? DIAG_STEP : PROBE_STEP;
    wire [15:0] minimum = mid_dir[0] ? DIAG_MIN : L_MIN;
    wire [4:0] trigger = mid_dir[0] ? DIAG_TRIGGER : CLEAR_TRIGGER;
    function [2:0] direction;
        input [15:0] x0, y0, x1, y1;
        begin
            direction = y0 == y1 ? (x1 >= x0 ? 3'd0 : 3'd4) :
            x0 == x1 ? (y1 >= y0 ? 3'd2 : 3'd6) :
            x1>x0 ? (y1>y0 ? 3'd1 : 3'd7) : (y1>y0 ? 3'd3 : 3'd5);
        end
    endfunction
    function signed [1:0] vx;
        input [2:0] dir;
        begin
            case(dir) 0, 1, 7:vx = 1;
                3, 4, 5:vx = -1;
                default:vx = 0;
            endcase
        end
    endfunction
    function signed [1:0] vy;
        input [2:0] dir;
        begin
            case(dir) 1, 2, 3:vy = 1;
                5, 6, 7:vy = -1;
                default:vy = 0;
            endcase
        end
    endfunction
    function signed [17:0] step;
        input [15:0] magnitude;
        input signed [1:0] sign;
        begin
            case(sign)
                1: step = $signed({2'b0, magnitude});
                -1: step = -$signed({2'b0, magnitude});
                default: step = 0;
            endcase
        end
    endfunction
    function [15:0] distance;
        input [15:0] x0, y0, x1, y1;
        reg [15:0] lx, ly;
        begin
            lx = x1 >= x0 ? x1-x0 : x0-x1;
            ly = y1 >= y0 ? y1-y0 : y0-y1;
            distance = lx>ly ? lx : ly;
        end
    endfunction
    function [15:0] weighted;
        input [15:0] length;
        input diagonal;
        begin
            weighted = diagonal ? (length<<4)-(length<<1) :
            (length<<3)+(length<<1);
        end
    endfunction
    function signed [17:0] constant_of;
        input [1:0] family;
        input signed [17:0] x, y;
        begin
            case(family)
                0:constant_of = y;
                1:constant_of = y-x;
                2:constant_of = x;
                3:constant_of = x+y;
            endcase
        end
    endfunction
    function [36:0] intersection;
        input [1:0] family0, family1;
        input signed [17:0] c0, c1;
        reg [1:0] a, b;
        reg signed [17:0] ca, cb, x, y;
        reg ok;
        begin
            a = family0<family1 ? family0 : family1;
            b = family0<family1 ? family1 : family0;
            ca = family0<family1 ? c0 : c1;
            cb = family0<family1 ? c1 : c0;
            x = 0;
            y = 0;
            ok = 1;
            case({a, b})
                4'b0001:begin
                    x = ca-cb;
                    y = ca;
                end
                4'b0010:begin
                    x = cb;
                    y = ca;
                end
                4'b0011:begin
                    x = cb-ca;
                    y = ca;
                end
                4'b0110:begin
                    x = cb;
                    y = cb+ca;
                end
                4'b0111:begin
                    x = (cb-ca)>>>1;
                    y = (cb+ca)>>>1;
                    ok = ca[0] == cb[0];
                end
                4'b1011:begin
                    x = ca;
                    y = cb-ca;
                end
                default:ok = 0;
            endcase
            intersection = {ok, x, y};
        end
    endfunction
    function forward;
        input signed [17:0] x0, y0, x1, y1;
        input [2:0] dir;
        begin
            forward = (vx(dir) == 0 ? x1 == x0 : vx(dir)>0 ? x1>x0 : x1<x0) &&
            (vy(dir) == 0 ? y1 == y0 : vy(dir)>0 ? y1>y0 : y1<y0);
        end
    endfunction
    function allowed;
        input [15:0] x, y;
        input raw, cost;
        reg signed [16:0] ex, ey;
        begin
            ex = $signed({1'b0, x})-$signed({1'b0, origin_x});
            ey = $signed({1'b0, y})-$signed({1'b0, origin_y});
            allowed = !raw && (!cost || (start_relaxed && ex >= -BFS_RELAX && ex <= BFS_RELAX &&
            ey >= -BFS_RELAX && ey <= BFS_RELAX));
        end
    endfunction
    wire signed [17:0] ibx = intersection_b[35:18], iby = intersection_b[17:0];
    wire signed [17:0] icx = intersection_c[35:18], icy = intersection_c[17:0];
    wire geometry_ok = intersection_b[36] && intersection_c[36] &&
    ibx >= 0 && ibx<GRID_W && iby >= 0 && iby<GRID_H &&
    icx >= 0 && icx<GRID_W && icy >= 0 && icy<GRID_H &&
    forward($signed({2'b0, ax}), $signed({2'b0, ay}), ibx, iby, prev_dir) &&
    forward(ibx, iby, icx, icy, mid_dir) &&
    forward(icx, icy, $signed({2'b0, dx}), $signed({2'b0, dy}), next_dir);
    wire probe_hit = outside || raw_q || cost_q;
    wire checked_free = allowed(mrd_x, mrd_y, raw_q, cost_q);
    wire line_valid, line_last, line_invalid;
    wire [15:0] line_x, line_y;
    wire [15:0] line_x0 = line_index == 0 ? ax : line_index == 1 ? cand_bx : cand_cx;
    wire [15:0] line_y0 = line_index == 0 ? ay : line_index == 1 ? cand_by : cand_cy;
    wire [15:0] line_x1 = line_index == 0 ? cand_bx : line_index == 1 ? cand_cx : dx;
    wire [15:0] line_y1 = line_index == 0 ? cand_by : line_index == 1 ? cand_cy : dy;
    nav_line_cells u_line (
        .clk(clk), .reset_n(reset_n), .start(state == LINE_BEGIN), .cancel(cancel),
        .x0(line_x0), .y0(line_y0), .x1(line_x1), .y1(line_y1),
        .valid(line_valid), .ready(state == LINE_TAKE && checked_free),
        .x(line_x), .y(line_y), .last(line_last), .invalid(line_invalid)
    );
    assign busy = state != IDLE;
    assign in_ready = reset_n && !cancel && (state == WAIT_B || state == WAIT_C || state == WAIT_D);
    assign out_valid = reset_n && !cancel && (state == EMIT_B || state == FLUSH_B || state == FINAL);
    assign out_x = bx;
    assign out_y = by;
    assign out_last = state == FINAL;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state <= IDLE;
            read_return <= IDLE;
            ax <= 0;
            ay <= 0;
            bx <= 0;
            by <= 0;
            cx <= 0;
            cy <= 0;
            dx <= 0;
            dy <= 0;
            origin_x <= 0;
            origin_y <= 0;
            d_last <= 0;
            prev_dir <= 0;
            mid_dir <= 0;
            next_dir <= 0;
            run_len <= 0;
            probe_p <= 0;
            radius <= 0;
            d_plus <= 0;
            d_minus <= 0;
            probe_side <= 0;
            probe_phase <= 0;
            outside <= 0;
            line_index <= 0;
            ux <= 0;
            uy <= 0;
            nx <= 0;
            ny <= 0;
            base_x <= 0;
            base_y <= 0;
            probe_x <= 0;
            probe_y <= 0;
            shift_x <= 0;
            shift_y <= 0;
            offset <= 0;
            c_prev <= 0;
            c_mid <= 0;
            c_next <= 0;
            intersection_b <= 0;
            intersection_c <= 0;
            cand_bx <= 0;
            cand_by <= 0;
            cand_cx <= 0;
            cand_cy <= 0;
            old0 <= 0;
            old1 <= 0;
            old2 <= 0;
            new0 <= 0;
            new1 <= 0;
            new2 <= 0;
            old_sum <= 0;
            new_sum <= 0;
            delta_length <= 0;
            cumulative <= 0;
            trial_length <= 0;
            mrd_valid <= 0;
            mrd_x <= 0;
            mrd_y <= 0;
            raw_q <= 0;
            cost_q <= 0;
        end else if(cancel) begin
            state <= IDLE;
            mrd_valid <= 0;
        end else if(start) begin
            ax <= start_x;
            ay <= start_y;
            origin_x <= start_x;
            origin_y <= start_y;
            cumulative <= 0;
            mrd_valid <= 0;
            state <= WAIT_B;
        end else case(state)
            WAIT_B:if(in_valid) begin
                bx <= in_x;
                by <= in_y;
                state <= in_last ? FINAL : WAIT_C;
            end
            WAIT_C:if(in_valid) begin
                cx <= in_x;
                cy <= in_y;
                state <= in_last ? FLUSH_B : WAIT_D;
            end
            WAIT_D:if(in_valid) begin
                dx <= in_x;
                dy <= in_y;
                d_last <= in_last;
                state <= DIRECTIONS;
            end
            DIRECTIONS:begin
                prev_dir <= direction(ax, ay, bx, by);
                mid_dir <= direction(bx, by, cx, cy);
                next_dir <= direction(cx, cy, dx, dy);
                ux <= vx(direction(bx, by, cx, cy));
                uy <= vy(direction(bx, by, cx, cy));
                nx <= -vy(direction(bx, by, cx, cy));
                ny <= vx(direction(bx, by, cx, cy));
                run_len <= distance(bx, by, cx, cy);
                state <= ELIGIBLE;
            end
            ELIGIBLE:if(!CENTER_EN || run_len<minimum || prev_dir[1:0] == mid_dir[1:0] ||
            next_dir[1:0] == mid_dir[1:0]) state <= EMIT_B;
            else begin
                probe_p <= 0;
                probe_side <= 0;
                probe_phase <= 0;
                radius <= 1;
                d_plus <= depth+1'b1;
                d_minus <= depth+1'b1;
                state <= BASE;
            end
            BASE:begin
                base_x <= $signed({2'b0, bx})+step(probe_p, ux);
                base_y <= $signed({2'b0, by})+step(probe_p, uy);
                state <= PROBE_POINT;
            end
            PROBE_POINT:begin
                probe_x <= base_x+(probe_side ? -step({11'b0, radius}, nx) : step({11'b0, radius}, nx));
                probe_y <= base_y+(probe_side ? -step({11'b0, radius}, ny) : step({11'b0, radius}, ny));
                state <= PROBE_FLANK;
            end
            PROBE_FLANK:begin
                if(probe_phase == 2) probe_x <= probe_x-(probe_side ? -step(16'd1, nx) : step(16'd1, nx));
                if(probe_phase == 1) probe_y <= probe_y-(probe_side ? -step(16'd1, ny) : step(16'd1, ny));
                state <= PROBE_QUERY;
            end
            PROBE_QUERY:begin
                outside <= probe_x<0 || probe_y<0 || probe_x >= GRID_W || probe_y >= GRID_H;
                if(probe_x<0 || probe_y<0 || probe_x >= GRID_W || probe_y >= GRID_H) state <= PROBE_TAKE;
                else begin
                    mrd_x <= probe_x[15:0];
                    mrd_y <= probe_y[15:0];
                    mrd_valid <= 1;
                    read_return <= PROBE_TAKE;
                    state <= RD_REQUEST;
                end
            end
            RD_REQUEST:if(mrd_ready) begin
                mrd_valid <= 0;
                state <= RD_RESPONSE;
            end
            RD_RESPONSE:if(mrd_rsp_valid) begin
                raw_q <= mrd_raw;
                cost_q <= mrd_cost;
                state <= read_return;
            end
            PROBE_TAKE:begin
                if(probe_hit) begin
                    if(!probe_side && radius<d_plus) d_plus <= radius;
                    if(probe_side && radius<d_minus) d_minus <= radius;
                end
                if(!probe_hit && mid_dir[0] && probe_phase<2) begin
                    probe_phase <= probe_phase+1'b1;
                    state <= PROBE_POINT;
                end else begin
                    probe_phase <= 0;
                    if(!probe_hit && radius<depth) begin
                        radius <= radius+1'b1;
                        state <= PROBE_POINT;
                    end
                    else if(!probe_side) begin
                        probe_side <= 1;
                        radius <= 1;
                        state <= PROBE_POINT;
                    end
                    else if(probe_p == run_len) state <= CANDIDATE;
                    else begin
                        probe_p <= probe_p+probe_step<run_len ? probe_p+probe_step : run_len;
                        probe_side <= 0;
                        radius <= 1;
                        state <= BASE;
                    end
                end
            end
            CANDIDATE:begin
                if((d_plus<d_minus ? d_plus : d_minus) >= trigger || d_plus == d_minus ||
                (d_plus>d_minus ? d_plus-d_minus : d_minus-d_plus)<2) state <= EMIT_B;
                else begin
                    offset <= d_plus>d_minus ? $signed({1'b0, (d_plus-d_minus)})>>>1 :
                    -$signed({1'b0, ((d_minus-d_plus)>>1)});
                    state <= SHIFT;
                end
            end
            SHIFT:begin
                shift_x <= $signed({2'b0, bx})+(nx == 0 ? 18'sd0 : nx>0 ? offset : -offset);
                shift_y <= $signed({2'b0, by})+(ny == 0 ? 18'sd0 : ny>0 ? offset : -offset);
                state <= CONSTANTS;
            end
            CONSTANTS:begin
                c_prev <= constant_of(prev_dir[1:0], $signed({2'b0, ax}), $signed({2'b0, ay}));
                c_mid <= constant_of(mid_dir[1:0], shift_x, shift_y);
                c_next <= constant_of(next_dir[1:0], $signed({2'b0, dx}), $signed({2'b0, dy}));
                state <= INTERSECT;
            end
            INTERSECT:begin
                intersection_b <= intersection(prev_dir[1:0], mid_dir[1:0], c_prev, c_mid);
                intersection_c <= intersection(mid_dir[1:0], next_dir[1:0], c_mid, c_next);
                state <= GEOMETRY;
            end
            GEOMETRY:if(!geometry_ok) state <= EMIT_B;
            else begin
                cand_bx <= ibx[15:0];
                cand_by <= iby[15:0];
                cand_cx <= icx[15:0];
                cand_cy <= icy[15:0];
                state <= STEPS;
            end
            STEPS:begin
                old0 <= distance(ax, ay, bx, by);
                old1 <= distance(bx, by, cx, cy);
                old2 <= distance(cx, cy, dx, dy);
                new0 <= distance(ax, ay, cand_bx, cand_by);
                new1 <= distance(cand_bx, cand_by, cand_cx, cand_cy);
                new2 <= distance(cand_cx, cand_cy, dx, dy);
                state <= WEIGHTS;
            end
            WEIGHTS:begin
                old0 <= weighted(old0, prev_dir[0]);
                old1 <= weighted(old1, mid_dir[0]);
                old2 <= weighted(old2, next_dir[0]);
                new0 <= weighted(new0, prev_dir[0]);
                new1 <= weighted(new1, mid_dir[0]);
                new2 <= weighted(new2, next_dir[0]);
                state <= SUMS;
            end
            SUMS:begin
                old_sum <= {2'b0, old0}+{2'b0, old1};
                new_sum <= {2'b0, new0}+{2'b0, new1};
                state <= SUM_TAIL;
            end
            SUM_TAIL:begin
                old_sum <= old_sum+{2'b0, old2};
                new_sum <= new_sum+{2'b0, new2};
                state <= DELTA;
            end
            DELTA:begin
                delta_length <= $signed({1'b0, new_sum})-$signed({1'b0, old_sum});
                state <= TRIAL;
            end
            TRIAL:begin
                trial_length <= cumulative+delta_length;
                state <= BUDGET;
            end
            BUDGET:if(trial_length>PATH_INC_MAX*10) state <= EMIT_B;
            else begin
                line_index <= 0;
                state <= LINE_BEGIN;
            end
            LINE_BEGIN:state <= LINE_QUERY;
            LINE_QUERY:if(line_invalid) state <= EMIT_B;
            else if(line_valid) begin
                mrd_x <= line_x;
                mrd_y <= line_y;
                mrd_valid <= 1;
                read_return <= LINE_TAKE;
                state <= RD_REQUEST;
            end
            LINE_TAKE:if(!checked_free) state <= EMIT_B;
            else if(line_last) begin
                if(line_index == 2) state <= ACCEPT;
                else begin
                    line_index <= line_index+1'b1;
                    state <= LINE_BEGIN;
                end
            end else state <= LINE_QUERY;
            ACCEPT:begin
                bx <= cand_bx;
                by <= cand_by;
                cx <= cand_cx;
                cy <= cand_cy;
                cumulative <= trial_length;
                state <= EMIT_B;
            end
            EMIT_B:if(out_ready) begin
                ax <= bx;
                ay <= by;
                bx <= cx;
                by <= cy;
                cx <= dx;
                cy <= dy;
                state <= d_last ? FLUSH_B : WAIT_D;
            end
            FLUSH_B:if(out_ready) begin
                bx <= cx;
                by <= cy;
                state <= FINAL;
            end
            FINAL:if(out_ready) state <= IDLE;
            default:state <= IDLE;
        endcase
    end
endmodule
