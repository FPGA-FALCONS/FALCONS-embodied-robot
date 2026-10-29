`timescale 1ns/1ps
module nav_visual_layer_tb;
    reg clk=0,rst=0,enable=1,begin_layer=0,abort_layer=0,commit=0,stopped=1,inflight=0;
    reg [31:0] epoch=1,begin_epoch=1,expected_crc=32'h915e08be,word=0;
    reg [15:0] version=1;
    reg valid=0,read_req=0;
    reg [2:0] index=0;
    reg [5:0] address=0;
    wire ready,layer_valid,committed,error,blocked,raw,cost;
    wire [15:0] published_version;wire [3:0] error_code;
    integer checks=0,errors=0,last_error=0,commits=0;
    always #10 clk=~clk;
    nav_visual_layer #(.CELL_BITS(6)) dut(
        .clk(clk),.reset_n(rst),.i_enable(enable),.i_epoch(epoch),
        .i_begin(begin_layer),.i_abort(abort_layer),.i_commit(commit),
        .i_begin_epoch(begin_epoch),.i_expected_crc(expected_crc),.i_begin_version(version),
        .i_stopped(stopped),.i_read_inflight(inflight),.i_word_valid(valid),.i_word_index(index),.i_word(word),
        .o_word_ready(ready),.o_layer_valid(layer_valid),.o_version(published_version),
        .o_committed(committed),.o_error(error),.o_error_code(error_code),.o_read_blocked(blocked),
        .i_read(read_req),.i_read_address(address),.o_raw(raw),.o_cost(cost));
    always @(posedge clk) begin if(error)last_error=error_code;if(committed)commits=commits+1;end
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task start_layer;
        begin @(negedge clk);begin_layer=1;@(negedge clk);begin_layer=0;end
    endtask
    task send_word;input [2:0] n;input [31:0] data;
        begin wait(ready);@(negedge clk);index=n;word=data;valid=1;@(negedge clk);valid=0;wait(!blocked);end
    endtask
    task upload;input [31:0] first_cost;
        begin start_layer();send_word(0,32'h00000400);send_word(1,0);send_word(2,first_cost);send_word(3,0);end
    endtask
    task do_commit;
        begin @(negedge clk);commit=1;@(negedge clk);commit=0;repeat(4)@(negedge clk);end
    endtask
    task read_at;input [5:0] a;
        begin wait(!blocked);@(negedge clk);read_req=1;address=a;@(negedge clk);read_req=0;#1;end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        read_at(10);check(!layer_valid && !raw && !cost,"reset masks uninitialized RAM");
        upload(32'h00100400);read_at(10);check(!raw && !cost,"background upload invisible");
        stopped=0;do_commit();check(!layer_valid && last_error==6,"moving commit rejected");
        stopped=1;expected_crc=0;upload(32'h00100400);do_commit();
        check(!layer_valid && last_error==5,"whole layer CRC mismatch rejected");
        expected_crc=32'h915e08be;upload(32'h00100400);inflight=1;do_commit();
        check(!layer_valid && blocked,"commit waits for old read transaction drain");
        inflight=0;repeat(5)@(negedge clk);
        check(layer_valid && published_version==1 && commits==1,"complete layer published once");
        read_at(10);check(raw && cost,"physical obstacle in both planes");
        read_at(20);check(!raw && cost,"inflation remains cost-only");
        enable=0;read_at(10);check(!raw && !cost,"legacy navigation excludes AI layer");enable=1;
        version=2;start_layer();send_word(0,0);@(negedge clk);abort_layer=1;@(negedge clk);abort_layer=0;
        read_at(10);check(raw && cost && published_version==1,"aborted back bank preserves active layer");
        expected_crc=32'h66c72e36;upload(0);do_commit();
        check(last_error==7 && published_version==1,"cost cannot omit a raw obstacle");
        expected_crc=0;start_layer();send_word(1,0);repeat(2)@(negedge clk);
        check(last_error==3 && !ready,"out of order word aborts staging");
        epoch=2;repeat(3)@(negedge clk);read_at(10);
        check(!layer_valid && !raw && published_version==0,"new localization epoch invalidates old layer");
        begin_epoch=1;start_layer();repeat(2)@(negedge clk);
        check(last_error==1 && !ready,"old epoch begin refused");
        $display("CHECKS: %0d",checks);
        if(errors==0)$display("PASS: nav_visual_layer %0d checks",checks);else $display("FAIL: nav_visual_layer %0d",errors);
        $finish;
    end
    initial begin #2000000;$display("FAIL: nav_visual_layer timeout");$finish;end
endmodule
