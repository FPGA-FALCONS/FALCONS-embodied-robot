// ============================================================================
// 模块：nav_route
// ----------------------------------------------------------------------------
// 职责：
//   - 双 bank 保存活动路线和候选路线；候选完整后经外部几何验证才能 publish。
//   - 活动跟踪、显示及候选复核使用独立同步读路径。
//   - 只管理存储和原子切换，不决定目标归属、障碍安全或动作完成。
// 接口约定：
//   - 单时钟、异步低有效复位；load_begin/load_abort 只清候选的计数与完整标志。
//   - load_valid && load_ready 写一个格坐标和方向；load_last 标记本轮候选完整。
//   - publish 单拍且候选完整才翻 bank；route_id 16 位回绕，复位清零。
//   - target_index/candidate_index 为同步读；显示航点和首尾标志同步输出。
//   - target_x/y 保留当前段；drive_x/y/index 为最远控制目标，final_target 属于这个输出目标。
// 关键不变量：
//   - 活动路线在候选失败时保持；只有 publish 更新 active_length/route_id。
//   - X 字高两位存方向，低 16 位是格号；锁定段保留，仅输出时跨同向边界取最远点。
// 禁止事项：
//   - 不要把 final_target 当整车最终目标，它只表示活动前缀末点。
//   - 显示帧遇发布或失效会中止，下次 start 丢弃未完成帧；完成帧只含同一活动路线。
// ============================================================================
module nav_route #(
    parameter integer MAX_WP = 64
) (
    input  wire               clk,
                              reset_n,
    input  wire               load_begin,
                              load_abort,
    input  wire               load_valid,
                              load_last,
    output wire               load_ready,
                              load_full,
    input  wire        [15:0] load_x,
                              load_y,
    input  wire         [1:0] load_dir,
    input  wire               publish,
    output reg                active_valid,
    output reg         [15:0] active_length,
                              route_id,
    input  wire        [15:0] target_index,
    output reg         [15:0] target_x,
                              target_y,
    output reg          [1:0] target_dir,
    output wire               final_target,
    output wire        [15:0] drive_x,
                              drive_y,
                              drive_index,
    output wire               candidate_complete,
    output wire        [15:0] candidate_length,
    input  wire        [15:0] candidate_index,
    output reg         [15:0] candidate_x,
                              candidate_y,
    output reg                candidate_valid,
    input  wire               display_enable,
    input  wire               display_refresh,
    input  wire               display_route_valid,
    output wire               route_disp_valid,
    output reg          [7:0] route_disp_x_cell,
    output reg          [7:0] route_disp_y_cell,
    output wire               route_disp_frame_start,
    output wire               route_disp_frame_done
);
    // X 字扩为 18 位，方向占用原 BSRAM 字的两位余量，共用已有读口。
    reg [17:0] wp_x[0:1][0:MAX_WP-1];
    reg [15:0] wp_y[0:1][0:MAX_WP-1];
    reg bank, done;
    reg [15:0] count, first_x, first_y;
    reg [1:0] publish_hold, first_dir;
    // 后续路线已经合并同向点，只有锁定首段与其后第一段可能同向。
    reg [15:0] first_drive_x, first_drive_y, active_drive_x, active_drive_y;
    reg first_joined, active_joined;
    wire [15:0] read_index = target_index < active_length ? target_index : active_length - 1'b1;
    assign drive_x = target_index == 0 && active_joined ? active_drive_x : target_x;
    assign drive_y = target_index == 0 && active_joined ? active_drive_y : target_y;
    assign drive_index = target_index == 0 && active_joined ? 16'd1 : read_index;
    assign final_target = active_valid && drive_index + 16'd1 >= active_length;
    assign load_full          = count >= MAX_WP;
    assign load_ready         = reset_n && !done && !load_full && !load_begin && !load_abort;
    assign candidate_complete = done;
    assign candidate_length   = count;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            bank          <= 0;
            done          <= 0;
            count         <= 0;
            first_x       <= 0;
            first_y       <= 0;
            publish_hold  <= 0;
            active_valid  <= 0;
            active_length <= 0;
            route_id      <= 0;
            target_x      <= 0;
            target_y      <= 0;
            target_dir    <= 0;
            first_dir     <= 0;
            first_drive_x <= 0;
            first_drive_y <= 0;
            active_drive_x <= 0;
            active_drive_y <= 0;
            first_joined  <= 0;
            active_joined <= 0;
        end else begin
            if (load_begin || load_abort) begin
                count <= 0;
                done  <= 0;
            end else if (load_valid && load_ready) begin
                wp_x[!bank][count] <= {load_dir, load_x};
                wp_y[!bank][count] <= load_y;
                if (count == 0) begin
                    first_x   <= load_x;
                    first_y   <= load_y;
                    first_dir <= load_dir;
                    first_drive_x <= load_x;
                    first_drive_y <= load_y;
                    first_joined <= 0;
                end else if (count == 1 && load_dir == first_dir) begin
                    first_drive_x <= load_x;
                    first_drive_y <= load_y;
                    first_joined <= 1;
                end
                count <= count + 1'b1;
                if (load_last) done <= 1;
            end
            if (publish && done && !load_abort && !load_begin) begin
                active_drive_x <= first_drive_x;
                active_drive_y <= first_drive_y;
                active_joined  <= first_joined;
                bank          <= !bank;
                active_length <= count;
                active_valid  <= 1;
                route_id      <= route_id + 1'b1;
                target_x      <= first_x;
                target_y      <= first_y;
                target_dir    <= first_dir;
                count         <= 0;
                done          <= 0;
                publish_hold  <= 2;
            end else if (publish_hold != 0) publish_hold <= publish_hold - 1'b1;
            else if (active_valid) begin
                {target_dir, target_x} <= wp_x[bank][read_index];
                target_y               <= wp_y[bank][read_index];
            end
        end
    end
    always @(posedge clk) begin
        candidate_valid <= reset_n && done && candidate_index < count;
        if (done && candidate_index < count) begin
            candidate_x <= wp_x[!bank][candidate_index][15:0];
            candidate_y <= wp_y[!bank][candidate_index];
        end
    end

    // 活动 bank 和长度只在发布时变化。帧中检测 bank 改变即重发，
    // 用已有 bank 位保证一致性，无需复制路线或再保存一份版本编号。
    localparam integer DISP_INDEX_BITS = MAX_WP > 1 ? $clog2(MAX_WP) : 1;
    reg [DISP_INDEX_BITS-1:0] display_index;
    reg display_active, display_pending, display_bank, display_visible_q;
    reg response_valid, response_start, response_done, response_bank;
    wire display_visible = active_valid && display_route_valid;
    wire display_begin = !display_active &&
        (display_pending || display_refresh || !display_visible_q);
    wire display_read = reset_n && display_enable && display_visible &&
        (display_begin || (display_active && display_bank == bank));
    wire [DISP_INDEX_BITS-1:0] display_address = display_begin ? 0 : display_index;
    wire display_last = {1'b0, display_address} + 1'b1 >= active_length;
    wire display_empty = display_enable && !display_visible &&
        (display_pending || display_refresh || display_visible_q || display_active);
    wire response_current = reset_n && display_enable &&
        (!response_valid || response_bank == bank);

    assign route_disp_valid       = response_current && response_valid;
    assign route_disp_frame_start = response_current && response_start;
    assign route_disp_frame_done  = response_current && response_done;

    always @(posedge clk) begin
        if (display_read) begin
            route_disp_x_cell <= wp_x[bank][display_address][7:0];
            route_disp_y_cell <= wp_y[bank][display_address][7:0];
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            display_index     <= 0;
            display_active    <= 0;
            display_pending   <= 1;
            display_bank      <= 0;
            display_visible_q <= 0;
            response_valid    <= 0;
            response_start    <= 0;
            response_done     <= 0;
            response_bank     <= 0;
        end else begin
            response_valid <= display_read;
            response_start <= (display_read && display_begin) || display_empty;
            response_done  <= (display_read && display_last) || display_empty;
            response_bank  <= bank;
            display_visible_q <= display_visible;
            if (!display_enable) begin
                display_active    <= 0;
                display_pending   <= 1;
                display_visible_q <= 0;
            end else begin
                if (display_refresh) display_pending <= 1;
                if (display_read) begin
                    display_active <= !display_last;
                    display_index  <= display_address + 1'b1;
                    display_bank   <= bank;
                    if (display_begin) display_pending <= 0;
                end else if (display_empty) begin
                    display_active  <= 0;
                    display_pending <= 0;
                end else if (display_bank != bank) begin
                    display_active  <= 0;
                    display_pending <= 1;
                    display_bank    <= bank;
                end
            end
        end
    end
endmodule
