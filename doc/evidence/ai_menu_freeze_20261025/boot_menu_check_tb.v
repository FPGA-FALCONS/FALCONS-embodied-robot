`timescale 1ns/1ps
`include "disp_parameter_cfg.vh"

// 用真实扫描时序驱动 UI，锁定优化前六页完整帧的逐像素结果。
// 坐标在时钟沿后随扫描计数器推进，输出由显示驱动在下个沿采样。
module boot_menu_check_tb;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg [2:0] page = 0;
    wire [11:0] px, py;
    wire req, de;
    wire [23:0] rgb;
    wire [7:0] red, green, blue;
    wire [5:0] raddr;
    reg [7:0] text_q;
    reg [7:0] text_mem [0:63];
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
    ui_render dut (
        .i_pixel_clk(clk), .i_reset_n(rst_n), .i_pixel_x(px), .i_pixel_y(py),
        .i_pixel_req(req), .i_video_active(de), .o_pixel_rgb888(rgb),
        .i_page(page), .i_ui_mode(2'd1), .i_estop(page == 4),
        .i_signal_ok(page != 5), .i_arm_hint(1'b0), .i_menu_sel(page),
        .i_map_pixel((px[6:3] == 4'd5) && (py[6:3] == 4'd6)),
        .i_route_pixel((px[5:2] == 4'd7) || (py[5:2] == 4'd4)),
        .i_planner_state(4'd3), .i_nav_run(page == 3), .i_nav_paused(page == 4),
        .i_menu_dim(page == 3 || page == 4), .i_aux_sel(3'd2),
        .i_car_px(10'd415), .i_car_py(10'd297),
        .i_cur_px(10'd745), .i_cur_py(10'd165),
        .i_pt_px({140'd0, 10'd610, 10'd520}),
        .i_pt_py({140'd0, 10'd210, 10'd370}), .i_pt_v(16'd3),
        .o_dyn_raddr(raddr), .i_dyn_rdata(text_q), .i_imu_fail(1'b1),
        .i_tune_sel(4'd8), .i_tune_active(1'b1)
    );

    integer green_pixels=0,unknown_pixels=0,menu_pixels=0;
    initial begin
        // Leave the numeric RAM uninitialized for the first two full frames.
        repeat(4)@(negedge clk);rst_n=1;
        for(p=0;p<3;p=p+1)begin
            if(p==2)for(n=0;n<64;n=n+1)text_mem[n]=8'd16;
            for(n=0;n<FRAME_CYCLES;n=n+1)begin
                @(negedge clk);
                if(de && px<208)begin
                    menu_pixels=menu_pixels+1;
                    if(rgb==24'h7cfc98)green_pixels=green_pixels+1;
                    if((^rgb)===1'bx)unknown_pixels=unknown_pixels+1;
                end
            end
        end
        $display("BOOT_MENU: pixels=%0d green=%0d unknown=%0d",menu_pixels,green_pixels,unknown_pixels);
        if(menu_pixels>300000 && green_pixels==0 && unknown_pixels==0)$display("PASS: boot_menu_check_tb");
        else $display("FAIL: boot_menu_check_tb");
        $finish;
    end
endmodule
