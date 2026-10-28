`timescale 1ns/1ps
module ai_edge_probe_tb;
    reg clk=0,pclk=0,rst=0,rx_valid=0;
    reg [7:0] rx_data=0;
    reg [11:0] px=0,py=0;
    reg [1:0] selected=0;
    reg ai_active=0;
    reg inspection=0;
    integer task_preview=0;
    always #10 clk=~clk;always #10 pclk=~pclk;
    wire [23:0] base,rgb;
    wire glyph,connected,local_region,highlight;
    wire [6:0] dyn_addr;
    reg [7:0] packet_bytes[0:119999];reg [31:0] packet_counts[0:4];
    reg [2047:0] output_dir;reg [4095:0] filename;
    integer fd,mode,i,row,col,root_count;
    ui_render ui(.i_pixel_clk(pclk),.i_reset_n(rst),.i_pixel_x(px),.i_pixel_y(py),
        .i_pixel_req(1'b1),.i_video_active(1'b1),.o_pixel_rgb888(base),
        .i_page(3'd6),.i_menu_sel(3'd6),.i_ui_mode(2'd1),.i_estop(1'b0),
        .i_signal_ok(1'b1),.i_arm_hint(1'b0),.i_map_pixel(1'b0),.i_route_pixel(1'b0),
        .i_obstacle_state(3'd0),.i_nav_run(1'b0),.i_nav_paused(1'b0),.i_menu_dim(ai_active),
        .i_aux_sel(3'd0),.i_car_px(10'd0),.i_car_py(10'd0),.i_cur_px(10'd0),.i_cur_py(10'd0),
        .i_pt_px(160'd0),.i_pt_py(160'd0),.i_pt_v(16'd0),.o_dyn_raddr(dyn_addr),
        .i_dyn_rdata(8'd0),.i_imu_fail(1'b0),.i_tune_sel(4'd0),.i_tune_active(1'b0));
    ai_display_link panel(.clk(clk),.rst_n(rst),.rx_data(rx_data),.rx_valid(rx_valid),
        .pixel_clk(pclk),.pixel_rst_n(rst),.pixel_x(px),.pixel_y(py),.selected(selected),.i_inspection(inspection),
        .o_glyph(glyph),.o_connected(connected),.o_local_region(local_region),.o_highlight(highlight));
    assign rgb=px>=208 && (local_region || (connected && py>=120 && py<540))
        ? (glyph?(highlight?24'hffe066:24'he8f0f8):(highlight?24'h1d4ed8:24'h101820)):base;
    integer checked=0,errors=0;
    initial begin
        #100;rst=1;#200;
        for(mode=0;mode<7;mode=mode+1)begin
            selected=mode<4?mode:mode-4;inspection=mode>=4;ai_active=1;
            for(row=64;row<108;row=row+1)begin
                @(negedge pclk);px=4094;py=row;repeat(5)@(negedge pclk);px=4095;@(negedge pclk);
                for(col=0;col<1024;col=col+1)begin
                    px=col;@(posedge pclk);#1;
                    if(col>=208 && col<224)begin
                        checked=checked+1;
                        if(rgb!==24'h101820)begin errors=errors+1;if(errors<5)$display("FAIL: AI edge mode=%0d x=%0d y=%0d rgb=%h",mode,col,row,rgb);end
                    end
                    @(negedge pclk);
                end
            end
            for(row=520;row<532;row=row+1)begin
                @(negedge pclk);px=4094;py=row;repeat(5)@(negedge pclk);px=4095;@(negedge pclk);
                for(col=0;col<1024;col=col+1)begin
                    px=col;@(posedge pclk);#1;
                    if(col>=4 && col<20)begin
                        checked=checked+1;
                        if(rgb!==24'h0b1019)begin errors=errors+1;if(errors<5)$display("FAIL: menu edge rgb=%h",rgb);end
                    end
                    @(negedge pclk);
                end
            end
        end
        if(errors==0)$display("PASS: ai_edge_probe %0d expected background pixels",checked);
        else $display("FAIL: ai_edge_probe %0d errors out of %0d",errors,checked);
        $finish;
    end
endmodule
