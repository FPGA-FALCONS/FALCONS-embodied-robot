`timescale 1ns/1ps
module inspection_tx_tb;
    reg clk=0,rst=0,start=0,consume=0,read_word=0;
    reg [31:0] cycle=0;reg [4:0] index=0;
    wire ready=(cycle%3)!=0;
    wire [7:0] data,kind,sequence;wire busy,valid,last,msg_valid,error,rxready;
    wire [5:0] count;wire [31:0] word;
    integer checks=0,errors=0,last_count=0;
    always #10 clk=~clk;
    always @(posedge clk)begin cycle<=cycle+1;if(last && ready)last_count=last_count+1;end
    inspection_packet_tx tx(.clk(clk),.rst_n(rst),.i_start(start),.i_kind(8'h81),.i_count(4'd8),
        .i_words({32'd8,32'd7,32'd6,32'd5,32'hffffffff,32'd3,32'd2,32'd1}),
        .o_busy(busy),.o_data(data),.o_valid(valid),.o_last(last),.i_ready(ready));
    inspection_packet_rx #(.GAP_CYCLES(1000)) rx(.clk(clk),.rst_n(rst),.i_data(data),.i_valid(valid && ready),.o_ready(rxready),
        .o_message_valid(msg_valid),.i_message_ready(consume),.o_kind(kind),.o_sequence(sequence),.o_count(count),
        .i_word_read(read_word),.i_word_index(index),.o_word(word),.o_error(error));
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;@(negedge clk);start=1;@(negedge clk);start=0;
        wait(msg_valid);@(negedge clk);
        check(kind==8'h81 && count==8 && sequence==0,"CRC survives repeated byte backpressure");
        index=3;read_word=1;repeat(2)@(negedge clk);read_word=0;
        check(word==32'hffffffff,"snapshot preserves full status word");
        index=7;read_word=1;repeat(2)@(negedge clk);read_word=0;
        check(word==8 && last_count==1 && !busy,"one last marker completes full transaction");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_tx %0d checks",checks);else $display("FAIL: inspection_tx %0d",errors);
        $finish;
    end
    initial begin #1000000;$display("FAIL: inspection_tx timeout");$finish;end
endmodule
