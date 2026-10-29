`timescale 1ns/1ps
`include "disp_parameter_cfg.vh"

// 用真实扫描时序驱动 UI，锁定优化前六页完整帧的逐像素结果。
// 坐标在时钟沿后随扫描计数器推进，输出由显示驱动在下个沿采样。
module ui_pipeline_tb;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg [2:0] page = 0;
    wire [11:0] px, py;
    wire req, de;
    wire [23:0] rgb;
    wire [7:0] red, green, blue;
    wire [6:0] raddr;
    reg [7:0] text_q;
    reg [7:0] text_mem [0:127];
    reg [31:0] expected [0:5];
    reg [31:0] hash;
    integer p, n, cycles = 0, errors = 0;
    localparam integer FRAME_CYCLES = `HDMI_1024X600_H_TOTAL * `HDMI_1024X600_V_TOTAL;

    disp_driver scan (
        .ClkDisp(clk), .Rst_n(rst_n), .Data(rgb), .DataReq(req),
        .H_Addr(px), .V_Addr(py), .Disp_DE(de),
        .Disp_Red(red), .Disp_Green(green), .Disp_Blue(blue)
    );

    always @(posedge clk) text_q <= text_mem[raddr];
    // Preserve the original six-menu pixel baseline. The production seven-item
    // layout and AI content are rendered/checked by preview_ai_ui.py.
    ui_render #(.MENU_ITEMS(6),.MENU_Y0(72)) dut (
        .i_pixel_clk(clk), .i_reset_n(rst_n), .i_pixel_x(px), .i_pixel_y(py),
        .i_pixel_req(req), .i_video_active(de), .o_pixel_rgb888(rgb),
        .i_page(page), .i_ui_mode(2'd1), .i_estop(page == 4),
        .i_signal_ok(page != 5), .i_arm_hint(1'b0), .i_menu_sel(page),
        .i_map_pixel((px[6:3] == 4'd5) && (py[6:3] == 4'd6)),
        .i_route_pixel((px[5:2] == 4'd7) || (py[5:2] == 4'd4)),
        .i_obstacle_state(3'd4), .i_nav_run(1'b0), .i_nav_paused(page == 4),
        .i_menu_dim(page == 3 || page == 4), .i_aux_sel(3'd2),
        .i_car_px(10'd415), .i_car_py(10'd297),
        .i_cur_px(10'd745), .i_cur_py(10'd165),
        .i_pt_px({140'd0, 10'd610, 10'd520}),
        .i_pt_py({140'd0, 10'd210, 10'd370}), .i_pt_v(16'd3),
        .o_dyn_raddr(raddr), .i_dyn_rdata(text_q), .i_imu_fail(1'b1),
        .i_tune_sel(4'd8), .i_tune_active(1'b1)
    );

    initial begin
        // 优化前原版 ui_render + disp_driver 的完整扫描输出指纹。
        expected[0] = 32'h1b1c15f3;
        expected[1] = 32'h8c6a9a7c;
        expected[2] = 32'h71923fef;
        expected[3] = 32'h877039e8; // 避障完成态禁编辑并提示返回；其他页保持原指纹。
        expected[4] = 32'h1eaec97b;
        expected[5] = 32'he6e73667; // 参数页十六行、30px 行高；前五页指纹保持不变。
        for (n = 0; n < 128; n = n + 1) text_mem[n] = 8'd16 + n % 10;
        for (p = 0; p < 6; p = p + 1) begin
            @(negedge clk); rst_n = 0; page = p;
            repeat (4) @(negedge clk);
            rst_n = 1;
            hash = 32'h811C9DC5;
            for (n = 0; n < FRAME_CYCLES; n = n + 1) begin
                @(posedge clk); #1;
                hash = (hash ^ {red, green, blue}) * 32'd16777619;
            end
            cycles = cycles + FRAME_CYCLES;
            $display("FRAME: page=%0d hash=%08x expected=%08x", p, hash, expected[p]);
            if (hash !== expected[p]) begin
                errors = errors + 1;
                $display("FAIL: page %0d pixel stream differs", p);
            end
        end
        $display("CYCLES: %0d", cycles);
        if (errors == 0) $display("PASS: ui_pipeline_tb 6 checks");
        else $display("FAIL: ui_pipeline_tb %0d frame errors", errors);
        $finish;
    end
endmodule
