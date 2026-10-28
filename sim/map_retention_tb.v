`timescale 1ns/1ps
// 清图取证：真实 UART -> M10 解析 -> 极坐标 -> 射线 -> 整圈累计 -> raw/cost。
// 不替换任何生产模块；只省略规划、控制及显示。输入由 map_retention.py 生成。
module map_retention_tb;
    reg clk = 0, reset_n = 0, rx = 1;
    always #10 clk = ~clk;
    reg signed [31:0] px = 0, py = 0, yaw = 0;
    wire pv, pr, pe, ue, overflow;
    wire [15:0] angle, distance;
    wire [5:0] point_index;
    wire [96:0] context;
    m10_radar radar (
        .clk(clk), .reset_n(reset_n), .radar_rx(rx),
        .capture_context({1'b1, px, py, yaw}), .point_context(context),
        .point_valid(pv), .point_ready(pr), .point_angle_cdeg(angle),
        .point_distance_mm(distance), .point_index(point_index),
        .packet_error(pe), .packet_overflow(overflow), .rx_error(ue),
        .packet_valid(), .packet_error_reason(), .packet_speed(),
        .rx_data(), .rx_valid(), .data_active(), .error_sticky(), .overflow_sticky(),
        .packet_count(), .rx_byte_count(), .uart_error_count(), .bad_packet_count(),
        .last_rx_byte(), .last_error_reason()
    );
    reg packet_seen = 0;
    reg [15:0] previous_angle = 0;
    wire begin_point = packet_seen && point_index == 0 && angle < previous_angle;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin packet_seen <= 0; previous_angle <= 0; end
        else if (pv && pr && point_index == 0) begin
            packet_seen <= 1;
            previous_angle <= angle;
        end
    end
    wire gv, gr, gs, gh;
    wire signed [15:0] gx, gy;
    wire signed [31:0] ox, oy, dx, dy;
    wire [13:0] range_mm;
    nav_polar polar (
        .clk(clk), .reset_n(reset_n), .in_valid(pv), .in_ready(pr),
        .in_scan_start(begin_point), .in_angle_cdeg(angle), .in_distance_mm(distance),
        .in_pose_valid(context[96]), .in_pose_x_mm(context[95:64]),
        .in_pose_y_mm(context[63:32]), .in_pose_yaw_cdeg(context[31:0]),
        .out_valid(gv), .out_ready(gr), .out_scan_start(gs), .out_hit(gh),
        .out_hit_x(gx), .out_hit_y(gy), .out_origin_x_mm(ox), .out_origin_y_mm(oy),
        .out_ray_dx_q16(dx), .out_ray_dy_q16(dy), .out_range_mm(range_mm), .out_rejected()
    );
    wire sr, cr, sv, si, value, shadow;
    wire [15:0] sx, sy;
    wire enabled = !gs || cr;
    assign gr = sr && enabled;
    nav_scanner scanner (
        .clk(clk), .reset_n(reset_n), .obs_valid(gv && enabled), .obs_ready(sr),
        .grid_x(gx), .grid_y(gy), .grid_valid(gh),
        .origin_x_mm(ox), .origin_y_mm(oy), .ray_dx_q16(dx), .ray_dy_q16(dy),
        .range_mm(range_mm),
        .map_enable(1'b1), .wr_valid(sv), .wr_ready(si), .wr_val(value),
        .wr_shadow(shadow),
        .wr_x(sx), .wr_y(sy), .busy(), .accepted(), .dropped(), .map_dropped()
    );
    wire wv, wr, wvalue, idle;
    wire [15:0] wx, wy;
    wire circle_begin = gv && gr && gs;
    nav_scan_accumulator accumulator (
        .clk(clk), .reset_n(reset_n), .scan_begin(circle_begin), .scan_ready(cr),
        .in_valid(sv), .in_ready(si), .in_value(value), .in_shadow(shadow), .in_x(sx), .in_y(sy),
        .wr_valid(wv), .wr_ready(wr), .wr_value(wvalue), .wr_x(wx), .wr_y(wy)
    );
    nav_map map (
        .clk(clk), .reset_n(reset_n), .wr_valid(wv), .wr_ready(wr),
        .wr_val(wvalue), .wr_x(wx), .wr_y(wy), .rd_valid(1'b0),
        .rd_x(16'd0), .rd_y(16'd0), .rd_ready(), .rd_rsp_valid(),
        .rd_raw(), .rd_cost(), .idle(idle), .occupied_cells(), .map_version()
    );

    integer cycles = 0, errors = 0, checks = 0, stream_errors = 0;
    integer circle = 0, sample_angle = 0, sample_distance = 0;
    integer polar_angle = 0, polar_distance = 0;
    integer hit_epoch [0:65535];
    integer free_epoch [0:131071];
    integer free_angle [0:131071];
    integer free_distance [0:131071];
    integer clear_count = 0, protected_clears = 0, addr, a, f, evidence;
    integer commands, status, op, snap, arg_x, arg_y, expect_value;
    integer packet_angle, n, slot, k, data, initialized = 0, begin_cycles;
    integer packets = 0, points = 0, zeros = 0, rays = 0, shadow_cells = 0;
    reg [15:0] ranges [0:41];
    reg [4095:0] input_path, output_dir, file_path;
    integer snapshot_file;
    reg continuous_input;

    always @(posedge clk) begin
        cycles = cycles + 1;
        if (reset_n) begin
            if (circle_begin) circle = circle + 1;
            if (pe || ue || overflow) begin
                stream_errors = stream_errors + 1;
                $display("STREAM_ERROR: cycle=%0d snapshot=%0d circle=%0d overflow=%b packet=%b uart=%b",
                         cycles, snap, circle, overflow, pe, ue);
            end
            if (pv && pr) points = points + 1;
            if (gv && gr) begin
                rays = rays + 1;
                sample_angle = polar_angle;
                sample_distance = polar_distance;
                if (sample_distance == 0) zeros = zeros + 1;
            end
            if (pv && pr && distance <= 10000) begin
                polar_angle = angle;
                polar_distance = distance;
            end
            if (sv && si) begin
                a = sy * 256 + sx;
                if (value || shadow) begin
                    hit_epoch[a] = circle;
                    if (shadow) shadow_cells = shadow_cells + 1;
                end
                else begin
                    f = (circle % 2) * 65536 + a;
                    free_epoch[f] = circle;
                    free_angle[f] = sample_angle;
                    free_distance[f] = sample_distance;
                end
            end
            if (wv && wr && !wvalue && map.raw[wy * 256 + wx]) begin
                a = wy * 256 + wx;
                f = ((circle - 1) % 2) * 65536 + a;
                clear_count = clear_count + 1;
                if (hit_epoch[a] >= circle - 1) begin
                    protected_clears = protected_clears + 1;
                    errors = errors + 1;
                    $display("FAIL: protected hit cleared circle=%0d hit_epoch=%0d cell=(%0d,%0d)",
                             circle, hit_epoch[a], wx, wy);
                end
                $fdisplay(evidence, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d", cycles,
                          circle, wx, wy, hit_epoch[a], free_epoch[f],
                          free_angle[f], free_distance[f], snap);
            end
        end
    end

    task settled;
        begin
            repeat (4) @(negedge clk);
            while (radar.u_parser.output_active || radar.u_parser.bank_pending != 0 ||
                   polar.state != 0 || !sr || !cr || !idle) @(negedge clk);
        end
    endtask
    task send_byte;
        input [7:0] b;
        begin
            rx = 0;
            repeat (109) @(negedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k];
                repeat (109) @(negedge clk);
            end
            rx = 1;
            repeat (109) @(negedge clk);
        end
    endtask
    task packet;
        begin
            send_byte(8'ha5); send_byte(8'h5a);
            send_byte(packet_angle >> 8); send_byte(packet_angle);
            send_byte(0); send_byte(60);
            for (slot = 0; slot < 42; slot = slot + 1) begin
                send_byte(ranges[slot] >> 8); send_byte(ranges[slot]);
            end
            repeat (10) send_byte(0);
            send_byte(8'hfa); send_byte(8'hfb);
            // 默认隔离清图功能；CONTINUOUS 检查连续 UART 输入下的队列和圈边界。
            repeat (10) @(negedge clk);
            if (!continuous_input) settled;
            packets = packets + 1;
        end
    endtask
    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin errors = errors + 1; $display("FAIL: %0s", message); end
            else $display("pass %0s", message);
        end
    endtask
    initial begin
        if (!$value$plusargs("INPUT=%s", input_path) ||
            !$value$plusargs("OUT=%s", output_dir)) $fatal(1, "INPUT and OUT required");
        $sformat(file_path, "%0s/clear_events.csv", output_dir);
        evidence = $fopen(file_path, "w");
        $fdisplay(evidence, "cycle,circle,x,y,hit_epoch,free_epoch,angle,distance,prior_snapshot");
        commands = $fopen(input_path, "r");
        if (!commands || !evidence) $fatal(1, "cannot open experiment files");
        snap = -1;
        continuous_input = $test$plusargs("CONTINUOUS");
        while (!$feof(commands)) begin
            status = $fscanf(commands, "%d", op);
            if (status == 1) case (op)
                0: begin
                    @(negedge clk); reset_n = 0; rx = 1;
                    repeat (4) @(negedge clk);
                    circle = 0;
                    for (addr = 0; addr < 65536; addr = addr + 1) begin
                        hit_epoch[addr] = -100;
                        free_epoch[addr] = -100;
                        free_epoch[addr + 65536] = -100;
                    end
                    reset_n = 1;
                    settled;
                    initialized = initialized + 1;
                end
                1: begin
                    status = $fscanf(commands, "%d %d %d %d", px, py, yaw, packet_angle);
                    if (status != 4) $fatal(1, "bad packet header");
                    for (n = 0; n < 42; n = n + 1) begin
                        status = $fscanf(commands, "%d", data);
                        if (status != 1) $fatal(1, "bad packet range");
                        ranges[n] = data;
                    end
                    packet;
                end
                2: begin
                    status = $fscanf(commands, "%d", snap);
                    settled;
                    $sformat(file_path, "%0s/map_%0d.txt", output_dir, snap);
                    snapshot_file = $fopen(file_path, "w");
                    for (addr = 0; addr < 65536; addr = addr + 1)
                        if (map.raw[addr]) $fdisplay(snapshot_file, "%0d %0d", addr % 256, addr / 256);
                    $fclose(snapshot_file);
                    $display("SNAPSHOT: id=%0d cycle=%0d circle=%0d raw=%0d clears=%0d",
                             snap, cycles, circle, map.occupied_cells, clear_count);
                end
                3: begin
                    status = $fscanf(commands, "%d %d %d", arg_x, arg_y, expect_value);
                    settled;
                    $display("CASE: cell=(%0d,%0d) expected_raw=%0d", arg_x, arg_y, expect_value);
                    check(map.raw[arg_y * 256 + arg_x] === (expect_value != 0),
                          "raw cell agrees with expected retention result");
                end
                default: $fatal(1, "bad command");
            endcase
        end
        check(stream_errors == 0, "UART parsing and packet queue report no errors");
        check(points == packets * 42, "every UART packet delivers all 42 slots exactly once");
        check(protected_clears == 0, "no old clear deletes a hit from its own or the new circle");
        $fclose(evidence); $fclose(commands);
        $display("CHECKS: %0d", checks);
        $display("METRICS: cycles=%0d packets=%0d points=%0d rays=%0d zero_rays=%0d shadow_cells=%0d actual_clears=%0d protected_clears=%0d resets=%0d errors=%0d",
                 cycles, packets, points, rays, zeros, shadow_cells, clear_count, protected_clears, initialized, errors);
        if (!errors) $display("PASS: map_retention_tb");
        $finish;
    end
    initial begin
        #(64'd10000000000);
        $display("FAIL: map_retention_tb timeout");
        $finish;
    end
endmodule
