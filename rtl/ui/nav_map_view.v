// 地图和路线独立双缓冲。系统域写后台，像素域在帧边界换图。
// 显示不反压导航；忙时跳过一整帧，下一 start 重新接收。
// 地图完整扫描覆盖后台；路线接完后清后台，补画当前任务轴线并连接航点。
module nav_map_view (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        map_valid,
    input  wire        map_occupied,
    input  wire        map_start,
    input  wire        map_done,
    input  wire        route_valid,
    input  wire [7:0]  route_x,
    input  wire [7:0]  route_y,
    input  wire        route_start,
    input  wire        route_done,

    input  wire               cmd_valid,
    input  wire               target_axis_y, // 0 水平、1 竖直；忽略横向偏差。
    input  wire signed [31:0] pose_x_mm,
    input  wire signed [31:0] pose_y_mm,
    input  wire signed [31:0] target_x_mm,
    input  wire signed [31:0] target_y_mm,

    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        frame_start,
    input  wire [11:0] pixel_x,
    input  wire [11:0] pixel_y,
    output wire        map_pixel,
    output wire        route_pixel
);
    reg map_memory [0:131071];
    reg route_memory [0:131071];
    reg [15:0] points [0:1023];
    reg map_request, route_request;
    reg map_front, route_front, map_show, route_show;
    reg map_ack1, map_ack2, route_ack1, route_ack2;
    wire map_available = map_ack2 == map_request;
    wire route_available = route_ack2 == route_request;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            map_ack1 <= 0;
            map_ack2 <= 0;
            route_ack1 <= 0;
            route_ack2 <= 0;
        end else begin
            map_ack1 <= map_front;
            map_ack2 <= map_ack1;
            route_ack1 <= route_front;
            route_ack2 <= route_ack1;
        end
    end

    reg map_receiving;
    reg [15:0] map_address;
    wire map_accept = map_start ? map_available : map_receiving;
    wire [15:0] map_write_address = map_start ? 16'd0 : map_address;
    always @(posedge clk) begin
        if (rst_n && map_valid && map_accept)
            map_memory[{!map_request, map_write_address}] <= map_occupied;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            map_request <= 0;
            map_receiving <= 0;
            map_address <= 0;
        end else begin
            if (map_start) map_receiving <= map_available;
            if (map_valid && map_accept) begin
                map_address <= map_write_address + 1'b1;
                if (map_done) begin
                    map_request <= !map_request;
                    map_receiving <= 0;
                end
            end
        end
    end

    localparam [2:0] RECEIVE = 0,
                     CLEAR   = 1,
                     FETCH   = 2,
                     TAKE    = 3,
                     PLOT    = 4;
    reg [2:0] route_state;
    reg route_receiving;
    reg [10:0] point_count, point_length, point_index;
    reg [15:0] clear_address, point_q;
    reg [7:0] draw_x, draw_y, end_x, end_y;
    reg current_segment, current_axis_y;
    reg [13:0] pose_offset_x, pose_offset_y, target_offset_x, target_offset_y;
    wire route_accept = route_state == RECEIVE && route_available &&
                        (route_start || route_receiving);
    wire [9:0] point_write_address = route_start ? 10'd0 : point_count[9:0];
    wire [10:0] received_length = route_start ? {10'd0, route_valid} :
                                  point_count + {10'd0, route_valid};
    wire route_write = route_state == CLEAR || route_state == PLOT;
    wire [15:0] route_write_address = route_state == CLEAR ? clear_address : {draw_y, draw_x};
    always @(posedge clk) begin
        if (rst_n && route_valid && route_accept)
            points[point_write_address] <= {route_y, route_x};
        if (route_state == FETCH) point_q <= points[point_index[9:0]];
        if (rst_n && route_write)
            route_memory[{!route_request, route_write_address}] <= route_state == PLOT;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            route_request <= 0;
            route_state <= RECEIVE;
            route_receiving <= 0;
            point_count <= 0;
            point_length <= 0;
            point_index <= 0;
            clear_address <= 0;
            draw_x <= 0;
            draw_y <= 0;
            end_x <= 0;
            end_y <= 0;
            current_segment <= 0;
            current_axis_y <= 0;
            pose_offset_x <= 0;
            pose_offset_y <= 0;
            target_offset_x <= 0;
            target_offset_y <= 0;
        end else begin
            if (route_start) route_receiving <= route_state == RECEIVE && route_available;
            case (route_state)
                RECEIVE: if (route_accept) begin
                    point_count <= received_length;
                    if (route_done) begin
                        // 与目标和方向同拍取样；先平移到地图坐标，清图时再换格号。
                        current_segment <= cmd_valid;
                        current_axis_y <= target_axis_y;
                        pose_offset_x <= pose_x_mm + 32'sd6400;
                        pose_offset_y <= pose_y_mm + 32'sd6400;
                        target_offset_x <= target_x_mm + 32'sd6400;
                        target_offset_y <= target_y_mm + 32'sd6400;
                        point_length <= received_length;
                        point_index <= 0;
                        clear_address <= 0;
                        route_receiving <= 0;
                        route_state <= CLEAR;
                    end
                end
                CLEAR: begin
                    if (clear_address == 0) begin
                        draw_x <= (current_axis_y ? target_offset_x : pose_offset_x) / 14'd50;
                        draw_y <= (current_axis_y ? pose_offset_y : target_offset_y) / 14'd50;
                        end_x <= target_offset_x / 14'd50;
                        end_y <= target_offset_y / 14'd50;
                    end
                    clear_address <= clear_address + 1'b1;
                    if (&clear_address) begin
                        if (current_segment) route_state <= PLOT;
                        else if (point_length == 0) begin
                            route_request <= !route_request;
                            route_state <= RECEIVE;
                        end else route_state <= FETCH;
                    end
                end
                FETCH: route_state <= TAKE;
                TAKE: begin
                    if (point_index == 0) begin
                        draw_x <= point_q[7:0];
                        draw_y <= point_q[15:8];
                    end
                    end_x <= point_q[7:0];
                    end_y <= point_q[15:8];
                    route_state <= PLOT;
                end
                PLOT: begin
                    if (draw_x == end_x && draw_y == end_y) begin
                        current_segment <= 0;
                        if (current_segment ? point_length == 0 : point_index + 1'b1 >= point_length) begin
                            route_request <= !route_request;
                            route_state <= RECEIVE;
                        end else begin
                            if (!current_segment) point_index <= point_index + 1'b1;
                            route_state <= FETCH;
                        end
                    end else if (draw_x != end_x)
                        draw_x <= draw_x < end_x ? draw_x + 1'b1 : draw_x - 1'b1;
                    else draw_y <= draw_y < end_y ? draw_y + 1'b1 : draw_y - 1'b1;
                end
                default: route_state <= RECEIVE;
            endcase
        end
    end

    reg map_req1, map_req2, route_req1, route_req2;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            map_req1 <= 0;
            map_req2 <= 0;
            route_req1 <= 0;
            route_req2 <= 0;
            map_front <= 0;
            route_front <= 0;
            map_show <= 0;
            route_show <= 0;
        end else begin
            map_req1 <= map_request;
            map_req2 <= map_req1;
            route_req1 <= route_request;
            route_req2 <= route_req1;
            if (frame_start) begin
                if (map_req2 != map_front) begin
                    map_front <= map_req2;
                    map_show <= 1;
                end
                if (route_req2 != route_front) begin
                    route_front <= route_req2;
                    route_show <= 1;
                end
            end
        end
    end
    // 原选点页每格约 6 像素。水平提前一像素，补偿块 RAM 同步读一拍。
    wire [11:0] grid_x = (pixel_x + 12'd357) / 12'd6;
    wire [11:0] grid_y = (12'd1068 - pixel_y) / 12'd6;
    wire [15:0] read_address = {grid_y[7:0], grid_x[7:0]};
    reg map_bit, route_bit;
    always @(posedge pixel_clk) begin
        map_bit <= map_memory[{map_front, read_address}];
        route_bit <= route_memory[{route_front, read_address}];
    end
    wire content = pixel_x >= 208 && pixel_x < 1024 && pixel_y < 600;
    assign map_pixel = map_show && content && map_bit;
    assign route_pixel = route_show && content && route_bit;
endmodule
