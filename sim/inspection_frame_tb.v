`timescale 1ns/1ps
module inspection_frame_tb;
    reg sclk=0,dclk=0,pclk=0,rst=0;
    always #10 sclk=~sclk;
    always #7 dclk=~dclk;
    always #11 pclk=~pclk;
    reg begin_frame=0,commit_frame=0,abort_frame=0,pv=0;
    reg [15:0] pixel=0;
    reg [31:0] context=4,sequence=11;
    reg [31:0] session=1;
    reg [1:0] view=0;
    reg show=1;
    wire frame_ready,pixel_ready,token_valid,token_ready,dt_valid,dt_ready;
    wire [17:0] token,dt;
    reg response_valid=0;
    wire bank_ready,done_toggle,write_valid,write_ready,publish_toggle,publish_bank,publish_done,writer_error;
    wire [27:0] write_address,read_address;
    wire [127:0] write_data,response_data;
    wire [175:0] metadata;
    reg [1:0] ready_sync=0,done_sync=0;
    always @(posedge sclk)begin ready_sync<={ready_sync[0],bank_ready};done_sync<={done_sync[0],done_toggle};end
    inspection_frame_tokens sender(.clk(sclk),.rst_n(rst),.i_bank_ready(ready_sync[1]),.i_done_toggle(done_sync[1]),
        .i_begin(begin_frame),.i_commit(commit_frame),.i_abort(abort_frame),.i_session(session),.i_task(32'd2),
        .i_epoch(32'd3),.i_context(context),.i_sequence(sequence),.i_view(view),.o_frame_ready(frame_ready),
        .i_pixel(pixel),.i_pixel_valid(pv),.o_pixel_ready(pixel_ready),.o_token(token),.o_token_valid(token_valid),.i_token_ready(token_ready));
    stream_async_fifo #(.WIDTH(18),.DEPTH(16)) tokens(.wclk(sclk),.wrst_n(rst),.i_valid(token_valid),.i_data(token),.o_ready(token_ready),
        .rclk(dclk),.rrst_n(rst),.o_valid(dt_valid),.o_data(dt),.i_ready(dt_ready));
    inspection_frame_writer #(.PIXELS(16)) writer(.clk(dclk),.rst_n(rst),.i_initialized(1'b1),
        .i_token(dt),.i_token_valid(dt_valid),.o_token_ready(dt_ready),.o_bank_ready(bank_ready),.o_done_toggle(done_toggle),
        .o_write_valid(write_valid),.o_write_address(write_address),.o_write_data(write_data),.i_write_ready(write_ready),
        .o_publish_toggle(publish_toggle),.o_publish_bank(publish_bank),.o_publish_metadata(metadata),.i_publish_done(publish_done),.o_error(writer_error));
    reg [11:0] x=0,y=0;
    always @(posedge pclk)if(rst)begin
        if(x==19)begin x<=0;if(y==7)y<=0;else y<=y+1'b1;end else x<=x+1'b1;
    end
    wire tick,publish_ack,read_valid,read_ready,fifo_write,fifo_reset,fault,empty,fdvalid,fdread,display_valid,underflow;
    wire [127:0] fifo_data,fddata;
    wire [31:0] free;
    wire [15:0] display_pixel;
    inspection_frame_reader #(.BLOCKS(2)) reader(.clk(dclk),.rst_n(rst),.i_enable(1'b1),.i_pixel_tick(tick),.i_pixel_publish_ack(publish_ack),
        .i_publish_toggle(publish_toggle),.i_publish_bank(publish_bank),.o_publish_done(publish_done),
        .o_read_valid(read_valid),.o_read_address(read_address),.i_read_ready(read_ready),.i_response_valid(response_valid),
        .i_response_data(response_data),.i_fifo_free(free),.o_fifo_write(fifo_write),.o_fifo_data(fifo_data),.o_fifo_reset_n(fifo_reset),.o_fault(fault));
    acg_async_fifo #(.WIDTH(128),.DEPTH(8)) display_fifo(.wr_clk(dclk),.wr_rst_n(fifo_reset && rst),
        .wr_en(fifo_write),.wr_data(fifo_data),.wr_full(),.wr_free(free),.rd_clk(pclk),.rd_rst_n(fifo_reset && rst),
        .rd_en(fdread),.rd_data(fddata),.rd_valid(fdvalid),.rd_empty(empty));
    inspection_frame_pixels #(.X0(4),.Y0(2),.WIDTH(8),.HEIGHT(2)) pixels(.clk(pclk),.rst_n(rst),
        .i_frame_sync(x==0 && y==0),.i_pixel_req(1'b1),.i_x(x),.i_y(y),.i_show(show),
        .i_session(session),.i_task(32'd2),.i_epoch(32'd3),.i_context(context),.i_view(view),
        .i_publish_toggle(publish_toggle),.i_publish_metadata(metadata),.o_tick(tick),.o_publish_ack(publish_ack),
        .i_fifo_reset_n(fifo_reset),.i_fifo_empty(empty),.i_fifo_valid(fdvalid),.i_fifo_data(fddata),.o_fifo_read(fdread),
        .o_pixel(display_pixel),.o_pixel_valid(display_valid),.o_underflow(underflow));
    reg [127:0] bank0[0:1],bank1[0:1];
    integer cycle=0,qhead=0,qtail=0,errors=0,checks=0,shown1=0,shown2=0,pubs=0,writes=0,before_shown=0;
    reg [27:0] readq[0:1023];
    reg [127:0] response=0;
    assign response_data=response;
    assign write_ready=cycle%7!=0 && cycle%7!=1;
    assign read_ready=cycle%5!=0;
    always @(negedge dclk)begin
        cycle=cycle+1;response_valid=0;
        if(qhead<qtail && cycle%4==0)begin
            response_valid=1;
            if(readq[qhead]>=28'h180000)response=bank1[(readq[qhead]-28'h180000)/8];
            else response=bank0[(readq[qhead]-28'h100000)/8];
            qhead=qhead+1;
        end
    end
    always @(posedge dclk)if(rst)begin
        if(write_valid && write_ready)begin
            writes=writes+1;
            if(write_address>=28'h180000)bank1[(write_address-28'h180000)/8]=write_data;
            else bank0[(write_address-28'h100000)/8]=write_data;
        end
        if(read_valid && read_ready)begin readq[qtail]=read_address;qtail=qtail+1;end
        if(publish_done)begin
            pubs=pubs+1;
            if(reader.outstanding!=0)begin errors=errors+1;$display("FAIL: old bank released with outstanding reads");end
        end
    end
    always @(posedge pclk)if(rst && display_valid)begin
        if(display_pixel[15:12]==1)shown1=shown1+1;
        else if(display_pixel[15:12]==2)shown2=shown2+1;
        else begin errors=errors+1;$display("FAIL: dirty/aborted display pixel %h",display_pixel);end
        if(display_pixel[11:0]!==((y-2)*8+x-4))begin
            errors=errors+1;$display("FAIL: pixel scan alignment xy=%0d,%0d data=%h",x,y,display_pixel);
        end
    end
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send_frame;input [15:0] pattern;input integer count;input abort;
        integer k;
        begin
            wait(frame_ready);@(negedge sclk);begin_frame=1;@(negedge sclk);begin_frame=0;
            for(k=0;k<count;k=k+1)begin
                pixel=pattern+k;pv=1;
                @(posedge sclk);while(!pixel_ready)@(posedge sclk);
                @(negedge sclk);pv=0;
            end
            if(abort)abort_frame=1;else commit_frame=1;
            @(negedge sclk);abort_frame=0;commit_frame=0;
            wait(frame_ready);@(negedge sclk);
        end
    endtask
    initial begin
        #100;rst=1;
        send_frame(16'h1000,16,0);
        wait(shown1>=32);
        check(pubs==1 && !fault && !errors,"first complete bank publishes only at frame boundary and scans in exact pixel order");
        sequence=12;send_frame(16'h3000,8,1);
        repeat(250)@(negedge pclk);
        check(pubs==1 && shown1>=48 && !errors,"aborted back bank cannot replace or damage the front image");
        before_shown=shown1;context=5;view=2;
        repeat(330)@(negedge pclk);
        check(shown1>=before_shown+16 && pubs==1,"view and confirmation context changes retain the complete front picture");
        show=0;repeat(4)@(negedge pclk);before_shown=shown1;
        repeat(170)@(negedge pclk);
        check(shown1==before_shown,"manual or AI exit hides retained picture immediately");
        show=1;session=7;repeat(4)@(negedge pclk);before_shown=shown1;
        repeat(170)@(negedge pclk);
        check(shown1==before_shown,"new session cannot display retained old-session picture");
        session=1;sequence=13;send_frame(16'h2000,16,0);
        wait(shown2>=32);
        check(pubs==2 && !fault && !errors,"new context swaps banks after drain and starts each display frame from pixel zero");
        sequence=14;send_frame(16'h3000,8,0);
        repeat(200)@(negedge pclk);
        check(pubs==2 && !errors,"short committed payload is rejected even after real DDR writes");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_frame %0d checks",checks);else $display("FAIL: inspection_frame %0d",errors);
        $finish;
    end
    initial begin #200000;$display("FAIL: inspection_frame timeout");$finish;end
endmodule
