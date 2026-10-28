`timescale 1ns/1ps
module visual_navigation_tb;
    reg clk=0,rst=0,ai_enable=1,begin_layer=0,commit_layer=0,word_valid=0;
    reg [12:0] word_index=0;reg [31:0] word=0;
    reg [31:0] fixture[0:4096];
    wire word_ready,layer_valid,layer_committed,layer_error;
    wire [15:0] layer_version,route_id,route_length;
    wire route_valid,route_start,route_done;
    wire [7:0] route_x,route_y;
    reg monitor=0,detour=0,route_complete=0,path_invalid=0,have_point=0;
    integer checks=0,errors=0,n,wait_cycles,old_route_id,previous_x,previous_y,dx,dy,steps,k,x,y;
    always #10 clk=~clk;
    nav_top #(.AI_LAYER_ENABLE(1),.GRID_W(16),.GRID_H(16),.HALF_MM(400),
              .MAX_WP(32),.PLAN_INFLATE_CELLS(0),.REPLAN_PERIOD(50000)) dut(
        .clk(clk),.reset_n(rst),.w_max(8'd3),.forward_check_cells(8'd2),.vehicle_width_mm(16'd50),
        .ai_enable(ai_enable),.ai_begin(begin_layer),.ai_abort(1'b0),.ai_commit(commit_layer),
        .ai_word_valid(word_valid),.ai_stopped(1'b1),.ai_epoch(32'd1),.ai_begin_epoch(32'd1),
        .ai_expected_crc(fixture[0]),.ai_word(word),.ai_begin_version(16'd1),.ai_word_index(word_index),
        .ai_word_ready(word_ready),.ai_layer_valid(layer_valid),.ai_layer_version(layer_version),
        .ai_committed(layer_committed),.ai_error(layer_error),.ai_error_code(),
        .pose_valid(1'b1),.pose_x_mm(-32'sd275),.pose_y_mm(32'sd25),
        .goal_valid(1'b1),.goal_x(16'd14),.goal_y(16'd8),.plan_enable(1'b1),.motion_enable(1'b0),
        .display_enable(1'b1),.map_enable(1'b1),.ctrl_done(1'b0),.ctrl_linear_active(1'b0),
        .obs_scan_start(1'b0),.obs_valid(1'b0),.obs_grid_x(16'sd0),.obs_grid_y(16'sd0),.obs_grid_valid(1'b0),
        .obs_origin_x_mm(32'sd0),.obs_origin_y_mm(32'sd0),.obs_ray_dx_q16(32'sd0),.obs_ray_dy_q16(32'sd0),
        .scan_valid(1'b1),.stream_error(1'b0),.st_route_length(route_length),.st_route_id(route_id),
        .route_disp_valid(route_valid),.route_disp_x_cell(route_x),.route_disp_y_cell(route_y),
        .route_disp_frame_start(route_start),.route_disp_frame_done(route_done));
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    always @(posedge clk) if(monitor && route_id>old_route_id) begin
        if(route_start) begin have_point=0;detour=0;path_invalid=0;end
        if(route_valid) begin
            if(route_y!=8)detour=1;
            if(have_point) begin
                dx=$signed({1'b0,route_x})-previous_x;dy=$signed({1'b0,route_y})-previous_y;
                steps=(dx<0?-dx:dx)>(dy<0?-dy:dy)?(dx<0?-dx:dx):(dy<0?-dy:dy);
                for(k=0;k<=32;k=k+1)if(k<=steps)begin
                    x=previous_x+(dx>0?k:dx<0?-k:0);y=previous_y+(dy>0?k:dy<0?-k:0);
                    if(x==8 && y>=4 && y<=12)path_invalid=1;
                end
            end
            previous_x=route_x;previous_y=route_y;have_point=1;
        end
        if(route_done)route_complete=1;
    end
    initial begin
        $readmemh("vectors/inspection_wall_words.hex",fixture);
        repeat(4)@(negedge clk);rst=1;repeat(5)@(negedge clk);
        @(negedge clk);begin_layer=1;@(negedge clk);begin_layer=0;
        for(n=0;n<4096;n=n+1)begin
            wait(word_ready);@(negedge clk);word_index=n;word=fixture[n+1];word_valid=1;
            @(negedge clk);word_valid=0;
        end
        wait(!dut.visual_blocked);@(negedge clk);commit_layer=1;@(negedge clk);commit_layer=0;
        wait(layer_committed);@(negedge clk);
        check(layer_valid && layer_version==1 && !layer_error,"full Python layer accepted by real navigation");
        old_route_id=route_id;monitor=1;wait_cycles=0;
        while(!route_complete && wait_cycles<1000000)begin @(negedge clk);wait_cycles=wait_cycles+1;end
        check(route_complete && detour,"Dijkstra routes around camera-only obstacle with zero radar hits");
        check(!path_invalid && route_length>2,"published segments avoid every blocked wall cell");
        monitor=0;ai_enable=0;old_route_id=route_id;route_complete=0;detour=0;
        @(negedge clk);monitor=1;wait_cycles=0;
        while(!route_complete && wait_cycles<1000000)begin @(negedge clk);wait_cycles=wait_cycles+1;end
        check(route_complete && !detour,"leaving AI restores original obstacle-free navigation map");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: visual_navigation %0d checks",checks);else $display("FAIL: visual_navigation %0d",errors);
        $finish;
    end
    initial begin #100000000;$display("FAIL: visual_navigation timeout");$finish;end
endmodule
