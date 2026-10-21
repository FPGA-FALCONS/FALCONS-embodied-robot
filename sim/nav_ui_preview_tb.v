`timescale 1ns/1ps
// 读取实际 ui_render / nav_map_view 输出，示例地图仅用于展示页面布局。
// 不跑车辆运动；运行 / 暂停等输入为页面状态快照，不作为流程自检。
module nav_ui_preview_tb;
    reg clk = 0, pixel_clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    always #11 pixel_clk = ~pixel_clk;
    reg [11:0] px = 0, py = 0;
    reg frame_start = 0;
    reg [2:0] page = 2, aux = 0;
    reg [1:0] ui_mode = 1;
    reg running = 0, paused = 0, editing = 0, estop = 0, signal_ok = 1;
    reg [3:0] planner = 0;
    reg [9:0] car_x = 415, car_y = 297, cursor_x = 745, cursor_y = 165;
    reg [159:0] pt_x = 0, pt_y = 0;
    reg [15:0] pt_v = 0;
    reg map_valid = 0, occupied = 0, map_start = 0, map_done = 0;
    reg route_valid = 0, route_start = 0, route_done = 0;
    reg [7:0] route_x = 0, route_y = 0;
    wire map_pixel, route_pixel;
    wire [23:0] rgb;
    wire [5:0] text_address;
    reg [7:0] text_memory [0:63];
    reg [7:0] text_q;
    always @(posedge pixel_clk) text_q <= text_memory[text_address];
    reg [11:0] scan_px = 0, scan_py = 0;
    always @(posedge pixel_clk) begin
        scan_px <= px;
        scan_py <= py;
    end
    nav_map_view view (
        .clk(clk), .rst_n(rst_n), .map_valid(map_valid), .map_occupied(occupied),
        .map_start(map_start), .map_done(map_done),
        .route_valid(route_valid), .route_x(route_x), .route_y(route_y),
        .route_start(route_start), .route_done(route_done),
        .pixel_clk(pixel_clk), .pixel_rst_n(rst_n), .frame_start(frame_start),
        .pixel_x(scan_px), .pixel_y(scan_py), .map_pixel(map_pixel), .route_pixel(route_pixel)
    );
    ui_render render (
        .i_pixel_clk(pixel_clk), .i_reset_n(rst_n), .i_pixel_x(scan_px), .i_pixel_y(scan_py),
        .i_pixel_req(1'b1), .i_video_active(1'b1), .o_pixel_rgb888(rgb),
        .i_page(page), .i_ui_mode(ui_mode), .i_estop(estop), .i_signal_ok(signal_ok),
        .i_arm_hint(1'b0), .i_menu_sel(page), .i_map_pixel(map_pixel),
        .i_route_pixel(route_pixel), .i_planner_state(planner),
        .i_nav_run(running), .i_nav_paused(paused), .i_menu_dim(editing),
        .i_aux_sel(aux), .i_car_px(car_x), .i_car_py(car_y),
        .i_cur_px(cursor_x), .i_cur_py(cursor_y), .i_pt_px(pt_x), .i_pt_py(pt_y), .i_pt_v(pt_v),
        .o_dyn_raddr(text_address), .i_dyn_rdata(text_q),
        .i_imu_fail(1'b0), .i_tune_sel(4'd0), .i_tune_active(1'b0)
    );
    integer n, gx, gy, row, col, fd;
    reg [4095:0] output_dir;
    reg [4095:0] filename;
    task swap_frame;
        begin
            repeat (6) @(negedge pixel_clk);
            frame_start = 1;
            @(negedge pixel_clk);
            frame_start = 0;
            repeat (6) @(negedge pixel_clk);
        end
    endtask
    task map_image;
        input closed;
        begin
            for (n = 0; n < 65536; n = n + 1) begin
                @(negedge clk);
                gx = n % 256; gy = n / 256;
                map_valid = 1; map_start = n == 0; map_done = n == 65535;
                occupied = ((gx == 122 || gx == 208) && gy >= 97 && gy <= 163) ||
                           ((gy == 97 || gy == 163) && gx >= 122 && gx <= 208) ||
                           (gx >= 154 && gx <= 157 && gy >= 110 && gy <= 142) ||
                           (gx >= 173 && gx <= 194 && gy >= 130 && gy <= 133) ||
                           (closed && gx == 163 && gy >= 98 && gy <= 162);
            end
            @(negedge clk); map_valid = 0; map_start = 0; map_done = 0;
            swap_frame();
        end
    endtask
    task waypoint;
        input [7:0] x, y;
        input first, last;
        begin
            @(negedge clk);
            route_x = x; route_y = y;
            route_start = first; route_done = last; route_valid = 1;
        end
    endtask
    task route_image;
        input empty;
        reg previous;
        begin
            previous = view.route_request;
            if (empty) begin
                @(negedge clk);
                route_start = 1; route_done = 1; route_valid = 0;
            end else begin
                waypoint(128,128,1,0);
                waypoint(143,128,0,0);
                waypoint(143,150,0,0);
                waypoint(183,150,0,1);
            end
            @(negedge clk);
            route_start = 0; route_done = 0; route_valid = 0;
            wait (view.route_request != previous);
            swap_frame();
        end
    endtask
    task capture;
        input [255:0] name;
        begin
            $sformat(filename, "%0s/%0s.ppm", output_dir, name);
            fd = $fopen(filename, "w");
            if (!fd) begin $display("FAIL: cannot open preview output"); $finish; end
            $fwrite(fd, "P3\n1024 600\n255\n");
            for (row = 0; row < 600; row = row + 1) begin
                // 水平消隐的准备拍，保留真实扫描的逐像素流水关系。
                @(negedge pixel_clk); px = 4094; py = row;
                repeat (5) @(negedge pixel_clk);
                px = 4095;
                @(negedge pixel_clk);
                for (col = 0; col < 1024; col = col + 1) begin
                    px = col;
                    @(posedge pixel_clk); #1;
                    $fwrite(fd, "%0d %0d %0d\n", rgb[23:16], rgb[15:8], rgb[7:0]);
                    @(negedge pixel_clk);
                end
            end
            $fclose(fd);
            $display("RENDER: %0s", name);
        end
    endtask
    initial begin
        if (!$value$plusargs("OUT=%s", output_dir)) output_dir = "logs/nav_ui_preview";
        for (n = 0; n < 64; n = n + 1) text_memory[n] = 0;
        // 准心坐标 2.775 m / 1.125 m，对应格中心 (183,150)。
        text_memory[47] = 18; text_memory[48] = 23; text_memory[49] = 23;
        text_memory[52] = 17; text_memory[53] = 17; text_memory[54] = 18;
        repeat (6) @(negedge clk); rst_n = 1;
        map_image(0);
        capture("00_radar_map");
        page = 3;
        capture("01_navigation_idle");
        editing = 1;
        capture("02_empty_selection");
        pt_x[9:0] = 505; pt_y[9:0] = 165;
        pt_x[19:10] = 745; pt_y[19:10] = 165; pt_v = 3;
        capture("03_destinations");
        aux = 1; capture("04_undo_selected");
        aux = 3; capture("05_clear_selected");
        aux = 2; capture("06_ready_start");
        route_image(0); running = 1; planner = 1; car_x = 505; car_y = 231;
        capture("07_running");
        running = 0; paused = 1; planner = 2;
        capture("08_paused");
        running = 1; paused = 0; planner = 3; car_x = 745; car_y = 165;
        capture("09_arrived");
        route_image(1); map_image(1); planner = 4; car_x = 505; car_y = 231;
        capture("10_no_path");
        running = 0; paused = 1; planner = 2; estop = 1;
        capture("11_emergency_stop");
        estop = 0; ui_mode = 0;
        capture("12_manual_takeover");
        ui_mode = 1; signal_ok = 0;
        capture("13_no_signal");
        $display("PASS: nav_ui_preview_tb 14 frames");
        $finish;
    end
endmodule
