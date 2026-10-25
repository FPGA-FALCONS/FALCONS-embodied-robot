// 八邻域 BFS 的流式分段入口；起点是隐含段起点，输出拐点和有限间距中继。
// 坐标和方向来自同步父方向读口，背压只停路径输出，不占用读图口。
module nav_planner #(
    parameter integer GRID_W = 16,
                      GRID_H = 16,
                      QMAX = 8192,
                      W_MAX = 39,
                      BFS_RELAX = 4
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        req_valid,
    input  wire        allow_start_escape,
    output wire        req_ready,
    input  wire [15:0] start_x, start_y, goal_x, goal_y,
    output wire        mrd_valid,
    input  wire        mrd_ready,
    output wire [15:0] mrd_x, mrd_y,
    input  wire        mrd_rsp_valid, mrd_cost, mrd_raw,
    input  wire [15:0] wp_limit,
    output wire        wp_valid,
    input  wire        wp_ready,
    output reg  [15:0] wp_x, wp_y,
    output reg         wp_last,
    output wire        busy,
    output reg         success, no_path,
    output wire [15:0] path_len
);
    localparam [2:0] IDLE=0, FIRST=1, COLLECT=2, OUTPUT=3, DRAIN=4;
    reg [2:0] state, run_dir;
    reg [15:0] run_len, count, limit;
    wire bfs_ready, bfs_busy, bfs_success, bfs_no_path;
    wire path_valid, path_ready, path_last;
    wire [15:0] path_x, path_y;
    wire [2:0] path_dir;
    assign req_ready = reset_n && state==IDLE && bfs_ready;
    assign busy = state!=IDLE;
    assign wp_valid = reset_n && state==OUTPUT;
    assign path_ready = state==FIRST || state==COLLECT || state==DRAIN;
    nav_bfs #(.GRID_W(GRID_W), .GRID_H(GRID_H), .QMAX(QMAX), .BFS_RELAX(BFS_RELAX)) u_bfs (
        .clk(clk), .reset_n(reset_n),
        .req_valid(req_valid && req_ready && wp_limit!=0), .req_ready(bfs_ready),
        .allow_start_escape(allow_start_escape),
        .start_x(start_x), .start_y(start_y), .goal_x(goal_x), .goal_y(goal_y),
        .mrd_valid(mrd_valid), .mrd_ready(mrd_ready), .mrd_x(mrd_x), .mrd_y(mrd_y),
        .mrd_rsp_valid(mrd_rsp_valid), .mrd_raw(mrd_raw), .mrd_cost(mrd_cost),
        .path_valid(path_valid), .path_ready(path_ready),
        .path_x(path_x), .path_y(path_y), .path_dir(path_dir), .path_last(path_last),
        .busy(bfs_busy), .success(bfs_success), .no_path(bfs_no_path), .path_len(path_len)
    );
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            run_dir <= 0;
            run_len <= 0;
            count <= 0;
            limit <= 0;
            wp_x <= 0;
            wp_y <= 0;
            wp_last <= 0;
            success <= 0;
            no_path <= 0;
        end else case (state)
            IDLE: if (req_valid && wp_limit!=0) begin
                success <= 0;
                no_path <= 0;
                count <= 0;
                limit <= wp_limit;
                state <= FIRST;
            end
            FIRST: if (path_valid) begin
                run_dir <= path_dir;
                run_len <= 0;
                if (path_last) begin
                    wp_x <= path_x;
                    wp_y <= path_y;
                    wp_last <= 1;
                    state <= OUTPUT;
                end else state <= COLLECT;
            end else if (!bfs_busy && bfs_no_path) begin
                no_path <= 1;
                state <= IDLE;
            end
            COLLECT: if (path_valid) begin
                if (path_last || path_dir!=run_dir || run_len+16'd1>=W_MAX-1) begin
                    wp_x <= path_x;
                    wp_y <= path_y;
                    wp_last <= path_last || count+16'd1>=limit;
                    run_dir <= path_dir;
                    run_len <= 0;
                    state <= OUTPUT;
                end else run_len <= run_len+1'b1;
            end
            OUTPUT: if (wp_ready) begin
                count <= count+1'b1;
                state <= wp_last ? DRAIN : COLLECT;
            end
            DRAIN: if (!bfs_busy) begin
                success <= bfs_success;
                no_path <= bfs_no_path;
                state <= IDLE;
            end
            default: state <= IDLE;
        endcase
    end
endmodule
