`timescale 1ns/1ps
module obstacle_display_tb;
    `include "nav_obstacle_states.vh"
    `include "ui_text_index.vh"
    reg clk=0, rst_n=0, active=0;
    always #10 clk=~clk;
    reg [11:0] px=1100, py=0, sx=0, sy=0;
    reg [2:0] state=OBS_EDIT, page=3, aux=0;
    reg [1:0] mode=1;
    wire [23:0] rgb;
    integer checks=0, errors=0, cycles=0, x, y, hits;
    always @(posedge clk) begin sx<=px; sy<=py; cycles<=cycles+1; end
    ui_render dut (
        .i_pixel_clk(clk), .i_reset_n(rst_n), .i_pixel_x(sx), .i_pixel_y(sy),
        .i_pixel_req(active), .i_video_active(active), .o_pixel_rgb888(rgb),
        .i_page(page), .i_ui_mode(mode), .i_estop(1'b0), .i_signal_ok(1'b1),
        .i_arm_hint(1'b0), .i_menu_sel(3'd3), .i_map_pixel(1'b0), .i_route_pixel(1'b0),
        .i_obstacle_state(state), .i_nav_run(state==OBS_RUN || state==OBS_NO_PATH),
        .i_nav_paused(state==OBS_PAUSE), .i_menu_dim(state!=OBS_BROWSE),
        .i_aux_sel(aux), .i_car_px(10'd412), .i_car_py(10'd300),
        .i_cur_px(10'd500), .i_cur_py(10'd200), .i_pt_px(160'd0), .i_pt_py(160'd0), .i_pt_v(16'd0),
        .o_dyn_raddr(), .i_dyn_rdata(8'd0), .i_imu_fail(1'b0),
        .i_tune_sel(4'd0), .i_tune_active(1'b0)
    );
    task check;
        input condition;
        input [511:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: %0s",message); end
            else $display("pass %0s",message);
        end
    endtask
    task pixels;
        input integer x0,x1,y0,y1;
        input [23:0] color;
        begin
            hits=0;
            for(y=y0;y<=y1;y=y+1) begin
                @(negedge clk); active=0; px=1100; py=y;
                repeat(5) @(negedge clk);
                for(x=x0-4;x<=x1;x=x+1) begin
                    px=x; active=1;
                    @(posedge clk); #1;
                    if(x>=x0 && rgb==color) hits=hits+1;
                    @(negedge clk);
                end
            end
            active=0;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        pixels(840,1007,426,449,24'hFFFFFF); check(hits>0,"editing enables add-point label");
        pixels(832,1023,420,455,24'h1D4ED8); check(hits>0,"editing shows selected action highlight");
        pixels(490,510,190,210,24'hFFFFFF); check(hits>0,"editing shows crosshair");
        state=OBS_RUN; aux=2;
        pixels(840,1007,426,449,24'h5A6170); check(hits>0,"running dims unavailable editing action");
        pixels(840,1007,426,449,24'hFFFFFF); check(hits==0,"disabled action has no white glyph pixels");
        check(dut.nav_aux_text(2)=={UI_TXT_AUX_PAUSE_NAV_ADDR,UI_TXT_AUX_PAUSE_NAV_LEN},"running action says pause");
        state=OBS_PAUSE; #1;
        check(dut.nav_aux_text(2)=={UI_TXT_AUX_RESUME_NAV_ADDR,UI_TXT_AUX_RESUME_NAV_LEN},"paused action says continue");
        state=OBS_FAULT; #1;
        check(dut.nav_aux_text(2)=={UI_TXT_AUX_RETRY_NAV_ADDR,UI_TXT_AUX_RETRY_NAV_LEN},"fault action says retry");
        check(dut.status_text=={UI_TXT_ST_CTRL_FAULT_ADDR,UI_TXT_ST_CTRL_FAULT_LEN},"fault remains visible");
        state=OBS_DONE; #1;
        check(dut.status_text=={UI_TXT_ST_ARRIVED_ADDR,UI_TXT_ST_ARRIVED_LEN},"completed task reports arrived without live core goal");
        check(dut.nav_aux_text(2)=={UI_TXT_AUX_RETURN_EDIT_ADDR,UI_TXT_AUX_RETURN_EDIT_LEN},"completed page instructs back-to-edit");
        pixels(832,1023,492,527,24'h1D4ED8); check(hits==0,"completion hides action highlight");
        pixels(490,510,190,210,24'hFFFFFF); check(hits==0,"completion hides crosshair");
        state=OBS_NO_PATH; #1;
        check(dut.status_text=={UI_TXT_ST_NO_PATH_ADDR,UI_TXT_ST_NO_PATH_LEN},"no-path waiting remains visible");
        mode=0; #1;
        check(dut.status_text=={UI_TXT_ST_MANUAL_ADDR,UI_TXT_ST_MANUAL_LEN},"manual ownership overrides paused or no-path text");
        pixels(840,1007,498,521,24'h5A6170); check(hits>0,"manual mode dims unavailable continue action");
        pixels(832,1023,492,527,24'h1D4ED8); check(hits==0,"manual mode hides interaction highlight");
        mode=1; state=OBS_BROWSE;
        pixels(832,1023,492,527,24'h1D4ED8); check(hits==0,"browser has no navigation action highlight");
        state=OBS_EDIT; aux=0;
        pixels(840,1007,426,449,24'hFFFFFF); check(hits>0,"returning to editor restores enabled labels");
        page=4; state=OBS_BROWSE;
        pixels(840,1007,426,449,24'hFFFFFF); check(hits>0,"non-obstacle navigation labels remain enabled");
        $display("CHECKS: %0d",checks);
        $display("METRICS: obstacle_display_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: obstacle_display_tb");
        $finish;
    end
    initial begin #50000000; $display("FAIL: obstacle display timeout"); $finish; end
endmodule
