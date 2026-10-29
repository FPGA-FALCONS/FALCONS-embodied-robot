`timescale 1ns/1ps
module inspection_highspeed_tb;
    reg clk=0,fclk=0,rst=0;
    reg [15:0] vectors[0:135],reply_words[0:39];
    reg [15:0] map_words[0:4119],route_words[0:27];
    reg route_valid=0;wire route_release,route_read;wire [6:0] route_address;reg [15:0] route_data;
    always @(posedge clk)begin
        if(route_read)case(route_address)0:route_data<=3;1:route_data<=16'h8080;2:route_data<=16'h808a;default:route_data<=16'h8a8a;endcase
        if(route_release)route_valid<=0;
    end
    reg map_valid=0;
    wire map_release,map_read;
    wire [11:0] map_address;
    reg [15:0] map_data;
    always @(posedge clk)begin
        if(map_read)map_data<={4'd0,map_address}^16'h5a5a;
        if(map_release)map_valid<=0;
    end
    wire [15:0] fd;
    wire [1:0] address;
    wire rd_n,wr_n,oe_n,end_n,cs_n;
    wire [31:0] diagnostics;
    integer out_index=0,in_index=0,checks=0,errors=0,flushes=0;
    always #10 clk=~clk;
    always #11 fclk=~fclk;
    assign fd=!oe_n && out_index<80?vectors[out_index%40]:16'bz;
    inspection_highspeed #(.FIFO_DEPTH(16)) dut(
        .clk(clk),.rst_n(rst),.fx2_clk(fclk),.fx2_fd(fd),.fx2_flagb(out_index<80),.fx2_flagc(1'b1),
        .fx2_address(address),.fx2_slrd_n(rd_n),.fx2_slwr_n(wr_n),.fx2_sloe_n(oe_n),.fx2_pktend_n(end_n),.fx2_slcs_n(cs_n),
        .i_display_enabled(1'b0),.i_session(32'd0),.i_task(32'd0),.i_epoch(32'd1),.i_context(32'd0),.i_view(2'd0),
        .o_frame_begin(),.o_frame_commit(),.o_frame_abort(),.o_frame_sequence(),.i_frame_ready(1'b0),
        .o_frame_data(),.o_frame_valid(),.i_frame_word_ready(1'b0),.o_control_valid(),.o_control_words(),
        .i_route_valid(route_valid),.o_route_release(route_release),.i_route_session(32'd1),.i_route_task(32'd2),
        .i_route_epoch(32'd3),.i_route_context(32'd7),.i_route_sequence(32'h00070006),.i_route_crc(32'heb2b2be0),.i_route_view(2'd1),
        .i_route_words(7'd4),.o_route_read(route_read),.o_route_address(route_address),.i_route_data(route_data),
        .i_map_valid(map_valid),.o_map_release(map_release),.i_map_session(32'd1),.i_map_task(32'd2),
        .i_map_epoch(32'd3),.i_map_context(32'd7),.i_map_sequence(32'h00050006),.i_map_crc(32'he23affd8),
        .i_map_view(2'd1),.o_map_read(map_read),.o_map_address(map_address),.i_map_data(map_data),
        .i_control_ready(1'b1),.i_control_read(1'b0),.i_control_address(8'd0),.o_control_data(),.o_diagnostics(diagnostics));
    always @(posedge fclk)begin
        if(!rd_n)out_index<=out_index+1;
        if(!wr_n)begin
            if((in_index<80 && fd!==reply_words[in_index%40]) || (in_index>=80 && in_index<4200 && fd!==map_words[in_index-80]) || (in_index>=4200 && (in_index>=4228 || fd!==route_words[in_index-4200])))begin errors=errors+1;$display("FAIL: physical diagnostic reply word %0d value %h",in_index,fd);end
            in_index<=in_index+1;
        end
        if(!end_n)flushes<=flushes+1;
        if(!oe_n && !wr_n)begin errors=errors+1;$display("FAIL: bus contention");end
    end
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        $readmemh("vectors/inspection_stream.hex",vectors);
        $readmemh("vectors/inspection_diagnostic_reply.hex",reply_words);
        $readmemh("vectors/inspection_map_stream.hex",map_words);
        $readmemh("vectors/inspection_route_stream.hex",route_words);
        #100;rst=1;
        wait(in_index==80);repeat(30)@(negedge fclk);
        check(out_index==80 && in_index==80 && !errors,"back-to-back host containers survive PHY and both asynchronous FIFOs");
        check(diagnostics==2 && flushes==2,"exactly two diagnostics and short packet flushes without AI or motor authority");
        map_valid=1;wait(in_index==4200);repeat(30)@(negedge fclk);
        check(!map_valid && !errors && diagnostics==2 && flushes==3,"full 8192-byte map matches host CRC/header and releases held source only after TX");
        route_valid=1;wait(in_index==4228);repeat(30)@(negedge fclk);
        check(!route_valid && !errors && flushes==4,"navigation prefix matches host container and releases only its own source");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_highspeed %0d checks",checks);else $display("FAIL: inspection_highspeed %0d",errors);
        $finish;
    end
    initial begin #5000000;$display("FAIL: inspection_highspeed timeout");$finish;end
endmodule
