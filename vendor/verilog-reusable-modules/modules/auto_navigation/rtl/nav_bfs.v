// 八邻域等权反向 BFS。方向从东开始逆时针编号，0..7，每次扩展优先直行。
// 周围八格先顺序读入，斜向同时检查两侧格；seen、父方向和 FIFO 分拍访问。
// 输出从起点到目标的逐格路径（包含两端），背压时保持；本模块不做居中。
module nav_bfs #(
    parameter integer GRID_W = 256,
                      GRID_H = 256,
                      QMAX = 8192,
                      BFS_RELAX = 4
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        req_valid,
    output wire        req_ready,
    input  wire        allow_start_escape,
    input  wire [15:0] start_x, start_y, goal_x, goal_y,
    output reg         mrd_valid,
    input  wire        mrd_ready,
    output reg  [15:0] mrd_x, mrd_y,
    input  wire        mrd_rsp_valid,
    input  wire        mrd_raw, mrd_cost,
    output wire        path_valid,
    input  wire        path_ready,
    output wire [15:0] path_x, path_y,
    output reg   [2:0] path_dir,
    output wire        path_last,
    output wire        busy,
    output reg         success, no_path,
    output reg  [15:0] path_len
);
    localparam integer N = GRID_W * GRID_H;
    localparam [4:0] IDLE = 0, START_READ = 1, GOAL_READ = 2, READ_WAIT = 3,
                     READ_TAKE = 4, CLEAR = 5, SEED = 6, POP = 7, CACHE = 8,
                     CACHE_NEXT = 9, EXPAND = 10, SEEN_TAKE = 11, MARK = 12,
                     NEXT = 13, WALK_READ = 14, WALK_TAKE = 15, EMIT = 16;
    reg [4:0] state, read_return;
    reg seen [0:N-1];
    reg [2:0] parent [0:N-1];
    reg [18:0] queue [0:QMAX-1];
    reg [15:0] sx, sy, start_cell, goal_cell, clear_cell;
    reg [15:0] head, tail, current, walk_cell, next_cell;
    reg [2:0] heading, cache_dir, slot, next_dir;
    reg [7:0] free_cells;
    reg seen_q, raw_q, cost_q, escape_q, start_blocked;

    function signed [1:0] dx;
        input [2:0] dir;
        begin
            case (dir)
                0,1,7: dx = 1;
                3,4,5: dx = -1;
                default: dx = 0;
            endcase
        end
    endfunction
    function signed [1:0] dy;
        input [2:0] dir;
        begin
            case (dir)
                1,2,3: dy = 1;
                5,6,7: dy = -1;
                default: dy = 0;
            endcase
        end
    endfunction
    function [2:0] ordered_dir;
        input [2:0] dir, position;
        begin
            case (position)
                0: ordered_dir = dir;
                1: ordered_dir = dir+3'd1;
                2: ordered_dir = dir-3'd1;
                3: ordered_dir = dir+3'd2;
                4: ordered_dir = dir-3'd2;
                5: ordered_dir = dir+3'd3;
                6: ordered_dir = dir-3'd3;
                default: ordered_dir = dir^3'd4;
            endcase
        end
    endfunction
    function [16:0] neighbour;
        input [15:0] cell_index;
        input [2:0] dir;
        reg signed [16:0] x, y;
        begin
            x = $signed({1'b0,cell_index%GRID_W})+dx(dir);
            y = $signed({1'b0,cell_index/GRID_W})+dy(dir);
            neighbour = x >= 0 && y >= 0 && x<GRID_W && y<GRID_H ?
                        y*GRID_W+x : 17'h10000;
        end
    endfunction
    function allowed;
        input [15:0] x,y;
        input raw,cost;
        reg signed [16:0] ex,ey;
        begin
            ex = $signed({1'b0,x})-$signed({1'b0,sx});
            ey = $signed({1'b0,y})-$signed({1'b0,sy});
            allowed = !raw && (!cost || (start_blocked && ex >= -BFS_RELAX && ex <= BFS_RELAX && ey >= -BFS_RELAX && ey <= BFS_RELAX));
        end
    endfunction
    wire [2:0] expand_dir = ordered_dir(heading,slot);
    wire [2:0] flank_left = expand_dir+3'd1, flank_right = expand_dir-3'd1;
    wire edge_free = free_cells[expand_dir] && (!expand_dir[0] || (free_cells[flank_left] && free_cells[flank_right]));
    wire [16:0] cache_cell = neighbour(current,cache_dir);
    wire [16:0] expand_cell = neighbour(current,expand_dir);
    wire [16:0] walk_next = neighbour(walk_cell,path_dir);
    wire mark = state == MARK && !seen_q;
    wire [15:0] queue_mask = QMAX-1;
    assign req_ready = reset_n && state == IDLE;
    assign busy = state != IDLE;
    assign path_valid = reset_n && state == EMIT;
    assign path_x = walk_cell%GRID_W;
    assign path_y = walk_cell/GRID_W;
    assign path_last = walk_cell == goal_cell;

    always @(posedge clk) begin
        if (reset_n) begin
            if (state == CLEAR) seen[clear_cell] <= 0;
            else if (state == SEED) seen[goal_cell] <= 1;
            else if (mark) seen[next_cell] <= 1;
            if (mark) parent[next_cell] <= next_dir^3'd4;
            if (state == SEED) queue[0] <= {3'd0,goal_cell};
            else if (mark && next_cell != start_cell && tail-head<QMAX)
                queue[tail&queue_mask] <= {next_dir,next_cell};
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            read_return <= IDLE;
            mrd_valid <= 0;
            mrd_x <= 0;
            mrd_y <= 0;
            sx <= 0;
            sy <= 0;
            start_cell <= 0;
            goal_cell <= 0;
            clear_cell <= 0;
            head <= 0;
            tail <= 0;
            current <= 0;
            walk_cell <= 0;
            next_cell <= 0;
            heading <= 0;
            cache_dir <= 0;
            slot <= 0;
            next_dir <= 0;
            path_dir <= 0;
            free_cells <= 0;
            seen_q <= 0;
            raw_q <= 0;
            cost_q <= 0;
            escape_q <= 0;
            start_blocked <= 0;
            success <= 0;
            no_path <= 0;
            path_len <= 0;
        end else case (state)
            IDLE: if (req_valid) begin
                success <= 0;
                no_path <= 0;
                path_len <= 0;
                sx <= start_x;
                sy <= start_y;
                start_cell <= start_y*GRID_W+start_x;
                goal_cell <= goal_y*GRID_W+goal_x;
                escape_q <= allow_start_escape;
                start_blocked <= 0;
                if (start_x >= GRID_W || start_y >= GRID_H || goal_x >= GRID_W || goal_y >= GRID_H)
                    no_path <= 1;
                else begin
                    mrd_x <= start_x;
                    mrd_y <= start_y;
                    mrd_valid <= 1;
                    read_return <= START_READ;
                    state <= READ_WAIT;
                end
            end
            READ_WAIT: if (mrd_valid && mrd_ready) begin
                mrd_valid <= 0;
                state <= READ_TAKE;
            end
            READ_TAKE: if (mrd_rsp_valid) begin
                raw_q <= mrd_raw;
                cost_q <= mrd_cost;
                state <= read_return;
            end
            START_READ: begin
                start_blocked <= cost_q && escape_q;
                if (raw_q || (cost_q && !escape_q)) begin no_path <= 1;
                state <= IDLE;
                end
                else begin
                    mrd_x <= goal_cell%GRID_W;
                    mrd_y <= goal_cell/GRID_W;
                    mrd_valid <= 1;
                    read_return <= GOAL_READ;
                    state <= READ_WAIT;
                end
            end
            GOAL_READ: if (!allowed(mrd_x,mrd_y,raw_q,cost_q)) begin
                no_path <= 1;
                state <= IDLE;
            end else begin clear_cell <= 0;
            state <= CLEAR;
            end
            CLEAR: if (clear_cell == N-1) state <= SEED;
                   else clear_cell <= clear_cell+1'b1;
            SEED: begin head <= 0;
            tail <= 1;
            state <= POP;
            end
            POP: if (start_cell == goal_cell) begin
                walk_cell <= start_cell;
                state <= WALK_READ;
            end else if (head == tail) begin no_path <= 1;
            state <= IDLE;
            end
            else begin
                {heading,current} <= queue[head&queue_mask];
                head <= head+1'b1;
                cache_dir <= 0;
                free_cells <= 0;
                state <= CACHE;
            end
            CACHE: if (cache_cell[16]) state <= CACHE_NEXT;
            else begin
                mrd_x <= cache_cell%GRID_W;
                mrd_y <= cache_cell/GRID_W;
                mrd_valid <= 1;
                read_return <= CACHE_NEXT;
                state <= READ_WAIT;
            end
            CACHE_NEXT: begin
                if (!cache_cell[16]) free_cells[cache_dir] <= allowed(mrd_x,mrd_y,raw_q,cost_q);
                if (cache_dir == 7) begin slot <= 0;
                state <= EXPAND;
                end
                else begin cache_dir <= cache_dir+1'b1;
                state <= CACHE;
                end
            end
            EXPAND: if (!edge_free) state <= NEXT;
            else begin
                next_cell <= expand_cell[15:0];
                next_dir <= expand_dir;
                state <= SEEN_TAKE;
            end
            SEEN_TAKE: begin seen_q <= seen[next_cell];
            state <= MARK;
            end
            MARK: if (seen_q) state <= NEXT;
            else if (next_cell == start_cell) begin walk_cell <= start_cell;
            state <= WALK_READ;
            end
            else if (tail-head >= QMAX) begin no_path <= 1;
            state <= IDLE;
            end
            else begin tail <= tail+1'b1;
            state <= NEXT;
            end
            NEXT: if (slot == 7) state <= POP;
                  else begin slot <= slot+1'b1;
                  state <= EXPAND;
                  end
            WALK_READ: begin path_dir <= path_last ? 3'd0 : parent[walk_cell];
            state <= WALK_TAKE;
            end
            WALK_TAKE: state <= EMIT;
            EMIT: if (path_ready) begin
                if (path_last) begin success <= 1;
                state <= IDLE;
                end
                else begin walk_cell <= walk_next[15:0];
                path_len <= path_len+1'b1;
                state <= WALK_READ;
                end
            end
            default: state <= IDLE;
        endcase
    end
endmodule
