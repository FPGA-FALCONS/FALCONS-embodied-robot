`timescale 1ns/1ps
// 对照原串行覆盖顺序，穷举图层重叠和文字 / 背景配色组合。
module ui_layers_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg [4:0] mode = 0;
    reg [12:0] hits = 1;
    wire [23:0] rgb;
    reg [23:0] expected;
    integer m, h, errors = 0, cycles = 0;

    ui_render dut (
        .i_pixel_clk(clk), .i_reset_n(1'b1), .i_pixel_x(12'd0), .i_pixel_y(12'd0),
        .i_pixel_req(1'b1), .i_video_active(1'b1), .o_pixel_rgb888(rgb),
        .i_page(3'd0), .i_ui_mode(mode[1:0]), .i_nav_run(mode[0]), .i_nav_paused(mode[1])
    );

    initial begin
        force dut.layer_hit = hits;
        force dut.nav_ui = mode[2];
        force dut.tune_ui = mode[3];
        force dut.in_menu_bar = mode[3];
        force dut.in_status_bar = mode[4];
        force dut.menu_sel_band = mode[2];
        force dut.lg_imu_fail_d1 = mode[0];
        force dut.sm_src_d1 = mode[1];
        force dut.lg_aux_dim_d1 = 0;
        for (m = 0; m < 32; m = m + 1) begin
            mode = m;
            for (h = 0; h < 4096; h = h + 1) begin
                @(negedge clk);
                hits = {h[11:0], 1'b1};
                #1;
                if (mode[4])
                    expected = mode[2] && mode[1] ? 24'h994D00 :
                        mode[2] && mode[0] ? 24'h006600 :
                        mode[1:0] == 0 ? 24'h660000 : 24'h202020;
                else if (mode[3]) expected = 24'h0B1019;
                else expected = 0;
                if (hits[1]) expected = 24'h2E3440;
                if (hits[2]) expected = 24'h1D4ED8;
                if (hits[3]) expected = 24'h3B82F6;
                if (hits[4]) expected = 24'hE87946;
                if (hits[5]) expected = 24'h101828;
                if (hits[6]) expected = 24'h1D4ED8;
                if (hits[7]) expected = 24'h5A6470;
                if (hits[8]) expected = 24'hFFFFFF;
                if (hits[9]) expected = 24'h00C000;
                if (hits[10]) begin
                    if (mode[0]) expected = 24'hFF4040;
                    else if (mode[4]) expected = 24'hFFFFFF;
                    else if (mode[3] && mode[2]) expected = 24'h8A7A2E;
                    else if (mode[3]) expected = 24'h5A6170;
                    else expected = 24'hFFFFFF;
                end
                if (hits[11]) expected = mode[1] ? 24'hFFFFFF : 24'h7CFC98;
                if (hits[12]) expected = 24'hFFFFFF;
                cycles = cycles + 1;
                if (rgb !== expected) begin
                    errors = errors + 1;
                    if (errors < 8) $display("FAIL: mode=%0d hits=%h rgb=%h expected=%h", mode, hits, rgb, expected);
                end
            end
        end
        $display("CYCLES: %0d", cycles);
        if (errors == 0) $display("PASS: ui_layers_tb 1 checks");
        else $display("FAIL: ui_layers_tb %0d errors", errors);
        $finish;
    end
endmodule
