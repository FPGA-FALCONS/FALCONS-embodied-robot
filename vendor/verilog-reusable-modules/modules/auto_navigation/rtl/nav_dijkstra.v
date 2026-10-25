// 八方向反向 Dijkstra：直步 2、斜步 3，四个循环桶内按 FIFO 排序。
// 等代价优先累计转弯少，再保留先发现的父方向；单格单父，不保证全局最少转弯。
// 距离、父方向、条目格号和链指针均为同步 RAM，单写口；访问和比较分拍。
// 输出包含起终点，path_len 仍为格步数；局部接出和斜向两侧格规则沿用规划器。
module nav_dijkstra #(
    parameter integer GRID_W      = 256,
                      GRID_H      = 256,
                      QMAX        = 8192,
                      START_RELAX = 4
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
    output reg  [15:0] path_len,
    output wire        start_relaxed
);
    localparam integer N = GRID_W * GRID_H;
    localparam integer AW = $clog2(QMAX);
    localparam integer INIT_N = N > QMAX ? N : QMAX;
    // 65535 条边、每边最多 3，18 位可覆盖整个地图；全 1 表示未发现。
    localparam [17:0] INF = 18'h3ffff;
    localparam [4:0] IDLE = 0, START_READ = 1, GOAL_READ = 2,
                     READ_WAIT = 3, READ_TAKE = 4, CLEAR = 5, SEED = 6,
                     PICK = 7, POP_READ = 8, POP_TAKE = 9,
                     DIST_READ = 10, DIST_TAKE = 11, CACHE = 12,
                     CACHE_NEXT = 13, EXPAND = 14, RELAX_READ = 15,
                     RELAX = 16, ALLOC_READ = 17, ENQUEUE = 18,
                     NEXT = 19, WALK_READ = 20, WALK_TAKE = 21, EMIT = 22,
                     RELAX_TAKE = 23, TIE_MARK = 24;

    reg [4:0] state, read_return;
    reg [17:0] distance [0:N-1];
    reg [15:0] turns [0:N-1];
    reg [2:0] parent [0:N-1];
    reg [15:0] entry_cell [0:QMAX-1];
    reg [AW-1:0] link [0:QMAX-1];
    reg [AW-1:0] bucket_head [0:3], bucket_tail [0:3];
    reg [3:0] bucket_empty;
    reg [AW-1:0] free_head, pop_slot, alloc_slot;
    reg [AW:0] free_count;
    reg [17:0] current_cost, trial_cost, distance_q;
    reg [15:0] current_turns, trial_turns, turns_q;
    reg shorter, equal_cost, fewer_turns;
    reg [AW-1:0] link_q;
    reg [15:0] cell_q;
    reg [2:0] parent_q;
    reg [15:0] sx, sy, start_cell, goal_cell, clear_cell;
    reg [15:0] current, walk_cell, next_cell;
    reg [2:0] heading, cache_dir, slot, next_dir, seed_heading;
    reg [7:0] free_cells;
    reg raw_q, cost_q, escape_q, start_blocked;
    integer i;

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
                1: ordered_dir = dir + 3'd1;
                2: ordered_dir = dir - 3'd1;
                3: ordered_dir = dir + 3'd2;
                4: ordered_dir = dir - 3'd2;
                5: ordered_dir = dir + 3'd3;
                6: ordered_dir = dir - 3'd3;
                default: ordered_dir = dir ^ 3'd4;
            endcase
        end
    endfunction
    function [16:0] neighbour;
        input [15:0] cell_index;
        input [2:0] dir;
        reg signed [16:0] x, y;
        begin
            x = $signed({1'b0,cell_index % GRID_W}) + dx(dir);
            y = $signed({1'b0,cell_index / GRID_W}) + dy(dir);
            neighbour = x >= 0 && y >= 0 && x < GRID_W && y < GRID_H ?
                        y * GRID_W + x : 17'h10000;
        end
    endfunction
    function allowed;
        input [15:0] x, y;
        input raw, cost;
        reg signed [16:0] ex, ey;
        begin
            ex = $signed({1'b0,x}) - $signed({1'b0,sx});
            ey = $signed({1'b0,y}) - $signed({1'b0,sy});
            allowed = !raw && (!cost || (start_blocked &&
                      ex >= -START_RELAX && ex <= START_RELAX &&
                      ey >= -START_RELAX && ey <= START_RELAX));
        end
    endfunction
    wire [2:0] expand_dir = ordered_dir(heading,slot);
    wire [2:0] flank_left = expand_dir + 3'd1;
    wire [2:0] flank_right = expand_dir - 3'd1;
    wire edge_free = free_cells[expand_dir] &&
                     (!expand_dir[0] || (free_cells[flank_left] && free_cells[flank_right]));
    wire [16:0] cache_cell = neighbour(current,cache_dir);
    wire [16:0] expand_cell = neighbour(current,expand_dir);
    wire [16:0] walk_next = neighbour(walk_cell,path_dir);
    wire clear_en = reset_n && state == CLEAR;
    wire seed_en = reset_n && state == SEED;
    wire push_en = reset_n && state == ENQUEUE;
    wire tie_en = reset_n && state == TIE_MARK;
    wire pop_en = reset_n && state == POP_TAKE;
    wire [1:0] push_bucket = trial_cost[1:0];
    wire [1:0] pop_bucket = current_cost[1:0];

    // 各 RAM 统一写口；读口寄存器不复位，使用前由对应读状态装入。
    wire dist_we = (clear_en && clear_cell < N) || seed_en || push_en || tie_en;
    wire [15:0] dist_wa = clear_en ? clear_cell : (seed_en ? goal_cell : next_cell);
    wire [17:0] dist_wd = clear_en ? INF : (seed_en ? 18'd0 : trial_cost);
    wire [15:0] dist_ra = state == RELAX_READ ? next_cell : current;
    wire [15:0] parent_ra = state == WALK_READ ? walk_cell : current;
    wire cell_we = seed_en || push_en;
    wire [AW-1:0] cell_wa = seed_en ? {AW{1'b0}} : alloc_slot;
    wire [15:0] cell_wd = seed_en ? goal_cell : next_cell;
    wire link_we = (clear_en && clear_cell < QMAX) || pop_en ||
                   (push_en && !bucket_empty[push_bucket]);
    wire [AW-1:0] link_wa = clear_en ? clear_cell[AW-1:0] :
                                (pop_en ? pop_slot : bucket_tail[push_bucket]);
    wire [AW-1:0] link_wd = clear_en ? clear_cell[AW-1:0] + 1'b1 :
                                (pop_en ? free_head : alloc_slot);
    wire [AW-1:0] link_ra = state == POP_READ ? pop_slot : free_head;
    always @(posedge clk) begin
        if (dist_we) distance[dist_wa] <= dist_wd;
        if (dist_we) turns[dist_wa] <= clear_en || seed_en ? 16'd0 : trial_turns;
        if (push_en || tie_en) parent[next_cell] <= next_dir ^ 3'd4;
        if (cell_we) entry_cell[cell_wa] <= cell_wd;
        if (link_we) link[link_wa] <= link_wd;
        if (state == DIST_READ || state == RELAX_READ) distance_q <= distance[dist_ra];
        if (state == DIST_READ || state == RELAX_READ) turns_q <= turns[dist_ra];
        if (state == DIST_READ || state == WALK_READ) parent_q <= parent[parent_ra];
        if (state == POP_READ || state == ALLOC_READ) link_q <= link[link_ra];
        if (state == POP_READ) cell_q <= entry_cell[pop_slot];
    end

    assign start_relaxed = start_blocked;
    assign req_ready = reset_n && state == IDLE;
    assign busy = state != IDLE;
    assign path_valid = reset_n && state == EMIT;
    assign path_x = walk_cell % GRID_W;
    assign path_y = walk_cell / GRID_W;
    assign path_last = walk_cell == goal_cell;

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
            current <= 0;
            walk_cell <= 0;
            next_cell <= 0;
            seed_heading <= 0;
            heading <= 0;
            cache_dir <= 0;
            slot <= 0;
            next_dir <= 0;
            path_dir <= 0;
            free_cells <= 0;
            raw_q <= 0;
            cost_q <= 0;
            escape_q <= 0;
            start_blocked <= 0;
            success <= 0;
            no_path <= 0;
            path_len <= 0;
            current_cost <= 0;
            trial_cost <= 0;
            current_turns <= 0;
            trial_turns <= 0;
            shorter <= 0;
            equal_cost <= 0;
            fewer_turns <= 0;
            free_head <= 0;
            free_count <= 0;
            pop_slot <= 0;
            alloc_slot <= 0;
            bucket_empty <= 4'b1111;
            for (i = 0; i < 4; i = i + 1) begin
                bucket_head[i] <= 0;
                bucket_tail[i] <= 0;
            end
        end else case (state)
            IDLE: if (req_valid) begin
                success <= 0;
                no_path <= 0;
                path_len <= 0;
                seed_heading <= start_y == goal_y ? (start_x >= goal_x ? 3'd0 : 3'd4) :
                                start_x == goal_x ? (start_y >= goal_y ? 3'd2 : 3'd6) :
                                start_x > goal_x ? (start_y > goal_y ? 3'd1 : 3'd7) :
                                                  (start_y > goal_y ? 3'd3 : 3'd5);
                sx <= start_x;
                sy <= start_y;
                start_cell <= start_y * GRID_W + start_x;
                goal_cell <= goal_y * GRID_W + goal_x;
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
                if (raw_q || (cost_q && !escape_q)) begin
                    no_path <= 1;
                    state <= IDLE;
                end else begin
                    mrd_x <= goal_cell % GRID_W;
                    mrd_y <= goal_cell / GRID_W;
                    mrd_valid <= 1;
                    read_return <= GOAL_READ;
                    state <= READ_WAIT;
                end
            end
            GOAL_READ: if (!allowed(mrd_x,mrd_y,raw_q,cost_q)) begin
                no_path <= 1;
                state <= IDLE;
            end else begin
                clear_cell <= 0;
                state <= CLEAR;
            end
            CLEAR: if (clear_cell == INIT_N-1) state <= SEED;
                   else clear_cell <= clear_cell + 1'b1;
            SEED: begin
                bucket_head[0] <= 0;
                bucket_tail[0] <= 0;
                bucket_empty <= 4'b1110;
                free_head <= 1;
                free_count <= QMAX-1;
                current_cost <= 0;
                state <= PICK;
            end
            PICK: if (&bucket_empty) begin
                no_path <= 1;
                state <= IDLE;
            end else if (bucket_empty[pop_bucket]) current_cost <= current_cost + 1'b1;
            else begin
                pop_slot <= bucket_head[pop_bucket];
                state <= POP_READ;
            end
            POP_READ: state <= POP_TAKE;
            POP_TAKE: begin
                current <= cell_q;
                if (bucket_head[pop_bucket] == bucket_tail[pop_bucket]) bucket_empty[pop_bucket] <= 1;
                else bucket_head[pop_bucket] <= link_q;
                free_head <= pop_slot;
                free_count <= free_count + 1'b1;
                state <= DIST_READ;
            end
            DIST_READ: state <= DIST_TAKE;
            DIST_TAKE: if (distance_q != current_cost) state <= PICK;
            // 只有从最小代价桶取出起点，才确定最终父链；旧条目直接跳过。
            else if (current == start_cell) begin
                walk_cell <= start_cell;
                state <= WALK_READ;
            end else begin
                current_turns <= turns_q;
                heading <= current == goal_cell ? seed_heading : parent_q ^ 3'd4;
                cache_dir <= 0;
                free_cells <= 0;
                state <= CACHE;
            end
            CACHE: if (cache_cell[16]) state <= CACHE_NEXT;
            else begin
                mrd_x <= cache_cell % GRID_W;
                mrd_y <= cache_cell / GRID_W;
                mrd_valid <= 1;
                read_return <= CACHE_NEXT;
                state <= READ_WAIT;
            end
            CACHE_NEXT: begin
                if (!cache_cell[16]) free_cells[cache_dir] <= allowed(mrd_x,mrd_y,raw_q,cost_q);
                if (cache_dir == 7) begin
                    slot <= 0;
                    state <= EXPAND;
                end else begin
                    cache_dir <= cache_dir + 1'b1;
                    state <= CACHE;
                end
            end
            EXPAND: if (!edge_free) state <= NEXT;
            else begin
                next_cell <= expand_cell[15:0];
                next_dir <= expand_dir;
                trial_cost <= current_cost + (expand_dir[0] ? 18'd3 : 18'd2);
                trial_turns <= current_turns + {15'd0,current != goal_cell && expand_dir != heading};
                state <= RELAX_READ;
            end
            RELAX_READ: state <= RELAX;
            RELAX: begin
                shorter <= trial_cost < distance_q;
                equal_cost <= trial_cost == distance_q;
                fewer_turns <= trial_turns < turns_q;
                state <= RELAX_TAKE;
            end
            RELAX_TAKE: if (shorter) begin
                if (free_count == 0) begin
                    no_path <= 1;
                    state <= IDLE;
                end else state <= ALLOC_READ;
            end else state <= equal_cost && fewer_turns ? TIE_MARK : NEXT;
            // 所有边代价均为正：等价父链更新先于该节点出桶，不需要重复入桶。
            TIE_MARK: state <= NEXT;
            ALLOC_READ: begin
                alloc_slot <= free_head;
                state <= ENQUEUE;
            end
            ENQUEUE: begin
                if (bucket_empty[push_bucket]) bucket_head[push_bucket] <= alloc_slot;
                bucket_tail[push_bucket] <= alloc_slot;
                bucket_empty[push_bucket] <= 0;
                free_head <= link_q;
                free_count <= free_count - 1'b1;
                state <= NEXT;
            end
            NEXT: if (slot == 7) state <= PICK;
                  else begin
                      slot <= slot + 1'b1;
                      state <= EXPAND;
                  end
            WALK_READ: state <= WALK_TAKE;
            WALK_TAKE: begin
                path_dir <= path_last ? 3'd0 : parent_q;
                state <= EMIT;
            end
            EMIT: if (path_ready) begin
                if (path_last) begin
                    success <= 1;
                    state <= IDLE;
                end else begin
                    walk_cell <= walk_next[15:0];
                    path_len <= path_len + 1'b1;
                    state <= WALK_READ;
                end
            end
            default: state <= IDLE;
        endcase
    end
endmodule
