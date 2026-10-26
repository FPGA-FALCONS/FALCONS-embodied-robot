`timescale 1ns/1ps
module inspection_fx2_tb;
    reg master=0,rst=0;
    always #16.666667 master=~master;
    wire clk;
    assign #4 clk=master;
    wire [15:0] fd;
    wire [1:0] phy_address,address;
    wire phy_rd,phy_wr,phy_oe,phy_end,phy_cs;
    wire rd_n,wr_n,oe_n,end_n,cs_n,rxvalid,txready;
    wire [15:0] rxdata;
    assign #5 address=phy_address;
    assign #5 rd_n=phy_rd;
    assign #5 wr_n=phy_wr;
    assign #5 oe_n=phy_oe;
    assign #5 end_n=phy_end;
    assign #5 cs_n=phy_cs;
    integer cycles=0,rx_index=0,received=0,tx_index=0,physical_writes=0,flushes=0,checks=0,errors=0;
    integer rx_free=4,tx_free=4;
    wire flagb,flagc;
    assign #9.5 flagb=rx_index<200 && rx_index<cycles/8;
    assign #9.5 flagc=tx_free>0;
    wire rxready=rx_free>0;
    wire [15:0] txdata=16'h8000+tx_index;
    // Model real prefetch latency; the word changes after the actual FX2 pop.
    assign #10.5 fd=!oe_n?(16'h1000+rx_index):16'bz;
    inspection_fx2_phy dut(.clk(clk),.rst_n(rst),.fd(fd),.flagb(flagb),.flagc(flagc),.address(phy_address),
        .slrd_n(phy_rd),.slwr_n(phy_wr),.sloe_n(phy_oe),.pktend_n(phy_end),.slcs_n(phy_cs),
        .o_rx_data(rxdata),.o_rx_valid(rxvalid),.i_rx_ready(rxready),
        .i_tx_data(txdata),.i_tx_valid(tx_index<300),.i_tx_last(tx_index==299),.o_tx_ready(txready));
    realtime rd_changed=-1000,wr_changed=-1000,end_changed=-1000,address_changed=-1000,cs_changed=-1000,data_changed=-1000,last_access=-1000;
    always @(rd_n)rd_changed=$realtime;
    always @(wr_n)wr_changed=$realtime;
    always @(end_n)end_changed=$realtime;
    always @(cs_n)cs_changed=$realtime;
    always @(fd)data_changed=$realtime;
    always @(address)begin
        if(rst && $realtime-last_access<10)begin errors=errors+1;$display("FAIL: FIFO address hold <10ns");end
        address_changed=$realtime;
    end
    always @(negedge master)if(rst && cycles%11==0 && tx_free<4)tx_free<=tx_free+1;
    always @(negedge clk)if(rst && cycles%13==0 && rx_free<4)rx_free<=rx_free+1;
    always @(posedge clk)if(rst)begin
        if(txready)tx_index<=tx_index+1;
        if(rxvalid)begin
            if(rx_free<=0 || rxdata!==(16'h1000+received))begin errors=errors+1;$display("FAIL: pipelined EP2 slot/order %0d %h",received,rxdata);end
            received<=received+1;rx_free<=rx_free-1;
        end
    end
    always @(posedge master)begin
        cycles=cycles+1;
        if(rst)begin
            if(!rd_n)begin
                if(oe_n || address!=0 || !flagb || !wr_n || fd!==(16'h1000+rx_index) || cs_n)begin
                    errors=errors+1;$display("FAIL: invalid EP2 read or bus contention");
                end
                if($realtime-rd_changed<18.7)begin errors=errors+1;$display("FAIL: SLRD setup <18.7ns");end
                rx_index<=rx_index+1;
            end
            if(!wr_n)begin
                if(!oe_n || address!=2 || !flagc || !rd_n || fd!==txdata || !txready || cs_n || tx_free<=0)begin
                    errors=errors+1;$display("FAIL: invalid EP6 write or bus contention");
                end
                if($realtime-wr_changed<10.4 || $realtime-data_changed<9.2)begin errors=errors+1;$display("FAIL: write setup violates FX2 specification");end
                tx_free<=tx_free-1;physical_writes<=physical_writes+1;
            end
            if(!end_n)begin
                flushes<=flushes+1;
                if(address!=2 || !wr_n || !rd_n || !oe_n || $realtime-end_changed<14.6)begin
                    errors=errors+1;$display("FAIL: short packet flush direction/setup");
                end
            end
            if(!rd_n || !wr_n || !end_n)begin
                if($realtime-address_changed<25 || $realtime-cs_changed<25)begin
                    errors=errors+1;$display("FAIL: FIFO address/select setup <25ns");
                end
                last_access=$realtime;
            end
        end
    end
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        repeat(3)@(negedge master);rst=1;
        repeat(6000)@(negedge master);
        check(rx_index==200 && received==200,"all EP2 words survive delayed pointer changes and independent sink backpressure");
        check(tx_index==300 && physical_writes==300,"fair arbitration drains EP6 with actual finite endpoint space");
        check(flushes==1,"only final short packet flushes; full 512-byte packet does not emit ZLP");
        check(!errors,"30MHz interface meets chip setup/hold with delayed clock, controls, flags and data");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_fx2 %0d checks",checks);else $display("FAIL: inspection_fx2 %0d",errors);
        $finish;
    end
endmodule
