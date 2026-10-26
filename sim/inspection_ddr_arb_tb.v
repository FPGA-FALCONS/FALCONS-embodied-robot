`timescale 1ns/1ps
module inspection_ddr_arb_tb;
    reg clk=0,rst=0;
    always #10 clk=~clk;
    integer cycle=0,legacy_count=0,write_count=0,read_count=0;
    integer head=0,tail=0,legacy_head=0,legacy_tail=0,ai_head=0,ai_tail=0;
    integer errors=0,checks=0,i;
    reg [27:0] reads[0:2047],legacy_reads[0:1023],ai_reads[0:1023];
    reg memory_valid=0;
    reg [127:0] memory_data=0;
    wire mr=cycle%7!=2,mwr=cycle%11<7;
    wire lr,lwr,ar,aw,lv,av,men,mwen,mwend,fault;
    wire [127:0] ld,ad,mdata;
    wire [27:0] address;
    wire [2:0] cmd;
    wire legacy_want=legacy_count<600;
    wire legacy_is_read=legacy_count[0];
    wire legacy_enable=legacy_want && lr && (legacy_is_read || lwr);
    wire [27:0] legacy_address=28'h1000+legacy_count*8;
    wire [27:0] write_address=28'h100000+write_count*8;
    wire [27:0] read_address=28'h180000+read_count*8;
    function [127:0] data_at;input [27:0] a;
        begin data_at={4{{4'hd,a}}};end
    endfunction
    inspection_ddr_arb #(.TAG_DEPTH(8)) dut(
        .clk(clk),.rst_n(rst),.i_initialized(1'b1),.i_legacy_want(legacy_want),.i_legacy_en(legacy_enable),
        .i_legacy_cmd(legacy_is_read?3'd1:3'd0),.i_legacy_address(legacy_address),.i_legacy_write_data(data_at(legacy_address)),
        .o_legacy_ready(lr),.o_legacy_write_ready(lwr),.o_legacy_read_valid(lv),.o_legacy_read_data(ld),
        .i_write_valid(write_count<600),.i_write_address(write_address),.i_write_data(data_at(write_address)),.o_write_ready(aw),
        .i_read_valid(read_count<600),.i_read_address(read_address),.o_read_ready(ar),.o_read_valid(av),.o_read_data(ad),
        .i_memory_ready(mr),.i_memory_write_ready(mwr),.i_memory_read_valid(memory_valid),.i_memory_read_data(memory_data),
        .o_memory_en(men),.o_memory_write_en(mwen),.o_memory_write_end(mwend),.o_memory_cmd(cmd),
        .o_memory_address(address),.o_memory_write_data(mdata),.o_protocol_fault(fault));
    always @(negedge clk)begin
        cycle=cycle+1;memory_valid=0;
        if(rst && head<tail && cycle%17==0)begin
            memory_valid=1;memory_data=data_at(reads[head]);head=head+1;
        end
    end
    always @(posedge clk)if(rst)begin
        if(legacy_enable)begin
            if(legacy_is_read)begin legacy_reads[legacy_tail]=legacy_address;legacy_tail=legacy_tail+1;end
            legacy_count<=legacy_count+1;
        end
        if(aw && write_count<600)write_count<=write_count+1;
        if(ar && read_count<600)begin
            ai_reads[ai_tail]=read_address;ai_tail=ai_tail+1;read_count<=read_count+1;
        end
        if(men)begin
            if(!mr || (cmd==0 && (!mwr || !mwen || !mwend || mdata!==data_at(address))))begin
                errors=errors+1;$display("FAIL: physical command/paired write handshake");
            end
            if(cmd==1)begin reads[tail]=address;tail=tail+1;end
        end
        if(lv)begin
            if(legacy_head>=legacy_tail || ld!==data_at(legacy_reads[legacy_head]))begin errors=errors+1;$display("FAIL: legacy response owner/order");end
            legacy_head=legacy_head+1;
        end
        if(av)begin
            if(ai_head>=ai_tail || ad!==data_at(ai_reads[ai_head]))begin errors=errors+1;$display("FAIL: AI response owner/order");end
            ai_head=ai_head+1;
        end
    end
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;
        repeat(20000)@(negedge clk);
        check(legacy_count==600 && write_count==600 && read_count==600,"three concurrent clients make progress under independent memory backpressure");
        check(legacy_head==300 && ai_head==600 && head==tail,"all outstanding reads preserve owner and order through a saturated tag queue");
        check(!fault && errors==0,"read/write commands and data are accepted together without lost transactions");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_ddr_arb %0d checks",checks);else $display("FAIL: inspection_ddr_arb %0d",errors);
        $finish;
    end
endmodule
