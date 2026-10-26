`timescale 1ns/1ps
module inspection_stream_tb;
    reg clk=0,rst=0,valid=0,message_ready=0,read_word=0,tx_start=0;
    reg [15:0] data=0,vectors[0:135];
    reg [7:0] read_address=0;
    reg [31:0] context=7;
    wire ready,begin_frame,commit_frame,abort_frame,pixel_valid,message_valid,error;
    wire [15:0] pixel,message_data;
    wire [7:0] kind;wire [8:0] message_words;wire [31:0] message_crc,sequence;
    integer cycles=0,checks=0,errors=0,begins=0,commits=0,aborts=0,pixels=0,i,n,tx_count=0;
    wire pixel_ready=cycles%11!=4;
    always #10 clk=~clk;
    always @(posedge clk)begin
        cycles=cycles+1;
        if(begin_frame)begins=begins+1;
        if(commit_frame)commits=commits+1;
        if(abort_frame)aborts=aborts+1;
        if(pixel_valid && pixel_ready)pixels=pixels+1;
    end
    inspection_stream_rx #(.CLK_HZ(1000)) receiver(
        .clk(clk),.rst_n(rst),.i_data(data),.i_valid(valid),.o_ready(ready),
        .i_display_enabled(1'b1),.i_session(32'd1),.i_task(32'd2),.i_epoch(32'd3),.i_context(context),.i_view(2'd1),
        .o_frame_begin(begin_frame),.o_frame_commit(commit_frame),.o_frame_abort(abort_frame),.o_sequence(sequence),
        .i_frame_ready(1'b1),.o_frame_data(pixel),.o_frame_valid(pixel_valid),.i_frame_word_ready(pixel_ready),
        .o_message_valid(message_valid),.i_message_ready(message_ready),.o_message_kind(kind),.o_message_words(message_words),
        .o_message_crc(message_crc),.i_message_read(read_word),.i_message_address(read_address),.o_message_data(message_data),.o_error(error));
    wire tx_busy,tx_read,tx_valid,tx_last;
    wire [11:0] tx_address;wire [15:0] tx_data;
    reg [15:0] tx_payload;
    wire tx_ready=cycles%7!=3 && cycles%7!=4;
    always @(posedge clk)begin
        if(tx_read)tx_payload<=vectors[24+tx_address];
        if(tx_valid && tx_ready)begin
            if(tx_data!==vectors[tx_count] || tx_last!==(tx_count==39))begin errors=errors+1;$display("FAIL: TX differs from host CRC vector at %0d",tx_count);end
            tx_count=tx_count+1;
        end
    end
    inspection_stream_tx transmitter(.clk(clk),.rst_n(rst),.i_start(tx_start),.i_kind(8'd4),
        .i_session(32'd1),.i_task(32'd2),.i_epoch(32'd3),.i_context(32'd7),.i_sequence(32'd42),.i_bytes(32'd32),
        .i_crc({vectors[21],vectors[20]}),.i_width(16'd0),.i_height(16'd0),.i_stride(16'd0),.i_view(16'd0),
        .o_busy(tx_busy),.o_payload_read(tx_read),.o_payload_address(tx_address),.i_payload_data(tx_payload),
        .o_data(tx_data),.o_valid(tx_valid),.o_last(tx_last),.i_ready(tx_ready));
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send_word;input [15:0] value;
        begin wait(ready);@(negedge clk);data=value;valid=1;@(negedge clk);valid=0;end
    endtask
    task release_message;
        begin @(negedge clk);message_ready=1;@(negedge clk);message_ready=0;repeat(3)@(negedge clk);end
    endtask
    task display_payload;input integer corrupt,change_view;
        begin
            for(n=40;n<64;n=n+1)send_word(vectors[n]);
            for(n=0;n<342720;n=n+1)begin
                send_word(((((2*n+1)%256)<<8)|((2*n)%256))^(n==corrupt?16'd1:16'd0));
                if(n==change_view)context=8;
            end
            repeat(10)@(negedge clk);
        end
    endtask
    initial begin
        $readmemh("vectors/inspection_stream.hex",vectors);
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        send_word(16'h9999);send_word(16'h4941);send_word(16'h1234);
        for(i=0;i<10;i=i+1)send_word(vectors[i]);
        repeat(510)@(negedge clk);check(!message_valid && ready,"incomplete header times out without exposing partial payload");
        for(i=0;i<40;i=i+1)send_word(vectors[i]);
        repeat(10)@(negedge clk);
        check(message_valid && kind==4 && message_words==16 && sequence==42,"host diagnostic accepted only after complete CRC");
        read_address=5;read_word=1;repeat(2)@(negedge clk);read_word=0;
        check(message_data==vectors[29] && !ready,"message RAM has synchronous read and holds under backpressure");
        release_message();
        for(i=0;i<40;i=i+1)send_word(vectors[i]^(i==30?16'd1:16'd0));
        repeat(10)@(negedge clk);check(!message_valid,"bad body CRC cannot publish a control or diagnostic");
        for(i=64;i<136;i=i+1)send_word(vectors[i]);
        repeat(10)@(negedge clk);check(message_valid && kind==2 && message_words==48,"complete layer transaction has independent bounded mailbox");
        release_message();
        display_payload(-1,-1);check(begins==1 && commits==1 && aborts==0 && pixels==342720,"full 816x420 host frame commits after all 342720 pixels");
        display_payload(100,-1);check(commits==1 && aborts==1,"corrupt full frame never publishes dirty DDR bank");
        display_payload(-1,1000);check(commits==1 && aborts==2,"context change during frame aborts old task view");
        context=7;
        tx_start=1;@(negedge clk);tx_start=0;wait(tx_busy);wait(!tx_busy);repeat(3)@(negedge clk);
        check(tx_count==40,"TX exactly matches Python header and payload under backpressure");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_stream %0d checks",checks);else $display("FAIL: inspection_stream %0d",errors);
        $finish;
    end
    initial begin #300000000;$display("FAIL: inspection_stream timeout");$finish;end
endmodule
