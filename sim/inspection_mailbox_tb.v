`timescale 1ns/1ps
module inspection_mailbox_tb;
    reg clk=0,rst=0,valid=0,ready=0,read=0;
    reg [8:0] words=0;
    reg [4:0] index=0;
    reg [15:0] data;
    reg [15:0] memory[0:203],layer[0:47],hello[0:17],control[0:59],block[0:41];
    wire release_mail,rd,msg,error;
    wire [7:0] address,kind,seq;
    wire [5:0] count;
    wire [31:0] word;
    integer checks=0,errors=0,releases=0,seen_errors=0,i;
    always #10 clk=~clk;
    always @(posedge clk)begin
        if(rd)data<=memory[address];
        if(release_mail)begin valid<=0;releases=releases+1;end
        if(error)seen_errors=seen_errors+1;
    end
    inspection_mailbox_rx dut(.clk(clk),.rst_n(rst),.i_valid(valid),.i_words(words),.o_release(release_mail),
        .o_read(rd),.o_address(address),.i_data(data),.o_valid(msg),.i_ready(ready),.o_kind(kind),.o_sequence(seq),
        .o_count(count),.i_word_read(read),.i_word_index(index),.o_word(word),.o_error(error));
    task check;input ok;input [639:0] name;
        begin checks=checks+1;if(ok===1'b1)$display("pass %0s",name);else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task settle;begin repeat(2000)@(negedge clk);end endtask
    initial begin
        $readmemh("vectors/inspection_mailbox_0.hex",layer);
        $readmemh("vectors/inspection_mailbox_1.hex",hello);
        $readmemh("vectors/inspection_mailbox_2.hex",control);
        $readmemh("vectors/inspection_mailbox_3.hex",block);
        #100;rst=1;
        for(i=0;i<48;i=i+1)memory[i]=layer[i];words=48;valid=1;settle;
        check(msg && kind==3 && count==6 && seq==9 && releases==1,"Python layer BEGIN survives synchronous mailbox and two CRC validators");
        index=5;read=1;@(negedge clk);read=0;@(negedge clk);
        check(word==6,"message RAM remains held after outer mailbox release");
        ready=1;@(negedge clk);ready=0;settle;
        for(i=0;i<18;i=i+1)memory[i]=hello[i];words=18;valid=1;settle;
        check(!msg && releases==2 && seen_errors==1,"USB HELLO cannot replace the UART session");
        for(i=0;i<60;i=i+1)memory[i]=control[i];words=60;valid=1;settle;
        check(!msg && releases==3 && seen_errors==2,"USB CONTROL cannot grant navigation authority");
        for(i=0;i<42;i=i+1)memory[i]=block[i];memory[20]=memory[20]^1;words=42;valid=1;settle;
        check(!msg && releases==4,"inner corruption is rejected even with an approved outer container");
        for(i=0;i<42;i=i+1)memory[i]=block[i];words=42;valid=1;settle;
        check(msg && kind==4 && count==5 && releases==5,"fresh layer DATA recovers after corrupted and forbidden messages");
        ready=1;@(negedge clk);ready=0;settle;
        // Two otherwise valid transactions cannot share one USB container.
        for(i=0;i<48;i=i+1)begin memory[i]=layer[i];memory[i+48]=layer[i];end
        words=96;valid=1;settle;
        check(!msg && releases==6,"multiple inner transactions in one container are rejected");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_mailbox %0d checks",checks);else $display("FAIL: inspection_mailbox %0d",errors);
        $finish;
    end
    initial begin #1000000;$display("FAIL: inspection_mailbox timeout");$finish;end
endmodule
