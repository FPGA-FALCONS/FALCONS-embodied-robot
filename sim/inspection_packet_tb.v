`timescale 1ns/1ps
module inspection_packet_tb;
    reg clk=0,rst=0,valid=0,accepted=0,read_word=0;
    reg [7:0] data=0;reg [4:0] word_index=0;
    wire ready,message_valid,error;wire [7:0] kind,sequence;
    wire [5:0] count;wire [31:0] word;
    reg [7:0] vector[0:119];
    integer i,checks=0,errors=0,packet_errors=0;
    always #10 clk=~clk;
    inspection_packet_rx #(.GAP_CYCLES(200)) dut(
        .clk(clk),.rst_n(rst),.i_data(data),.i_valid(valid),.o_ready(ready),
        .o_message_valid(message_valid),.i_message_ready(accepted),.o_kind(kind),.o_sequence(sequence),.o_count(count),
        .i_word_read(read_word),.i_word_index(word_index),.o_word(word),.o_error(error));
    always @(posedge clk)if(error)packet_errors=packet_errors+1;
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send_vector;input integer corrupt;
        begin for(i=0;i<120;i=i+1)begin @(negedge clk);valid=1;data=vector[i]^(i==corrupt?8'd1:8'd0);end
        @(negedge clk);valid=0;repeat(8)@(negedge clk);end
    endtask
    task consume;
        begin @(negedge clk);accepted=1;@(negedge clk);accepted=0;end
    endtask
    initial begin
        $readmemh("vectors/inspection_control.hex",vector);
        repeat(3)@(negedge clk);rst=1;
        send_vector(-1);
        check(message_valid && kind==2 && sequence==77 && count==8,"actual Python transaction CRC accepted");
        word_index=5;read_word=1;repeat(2)@(negedge clk);read_word=0;
        check(word==32'hffffffff,"synchronous word RAM preserves uint32 bits");
        repeat(5)@(negedge clk);check(message_valid && !ready,"message held for consumer backpressure");
        consume();check(!message_valid && ready,"consumer releases message");
        send_vector(20);check(!message_valid && packet_errors>0,"corrupt frame cannot publish transaction");
        send_vector(-1);check(message_valid,"valid message recovers after corruption");consume();
        for(i=0;i<24;i=i+1)begin @(negedge clk);valid=1;data=vector[i];end
        @(negedge clk);valid=0;repeat(210)@(negedge clk);
        check(!message_valid,"truncated message times out without publishing");
        send_vector(-1);check(message_valid,"new transaction recovers after timeout");
        $display("CHECKS: %0d",checks);
        if(errors==0)$display("PASS: inspection_packet %0d checks",checks);else $display("FAIL: inspection_packet %0d",errors);
        $finish;
    end
    initial begin #1000000;$display("FAIL: inspection_packet timeout");$finish;end
endmodule
