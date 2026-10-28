`timescale 1ns/1ps
// Production four-channel encoder path, including the real 10ms window.
// Motors remain disabled. A missing phase reproduces spinning/zero feedback.
module encoder_chain_tb;
    reg clk=0,rst=0,clear=0;
    reg [3:0] a=0,b=0;
    wire [127:0] counts,deltas;
    wire valid,stby;
    reg [127:0] last_delta=0;
    integer checks=0,errors=0,windows=0;
    always #10 clk=~clk;
    motor_top #(.CLK_FREQ_HZ(50_000_000),.WINDOW_MS(10),.FILTER_BITS(3)) dut(
        .i_clock(clk),.i_reset_n(rst),.i_enable(4'd0),.i_dir(4'd0),.i_brake(4'd0),.i_duty(44'd0),
        .i_stall_clear(1'b0),.o_stby(stby),.o_in1(),.o_in2(),.o_pwm(),.o_stall(),
        .i_enc_a(a),.i_enc_b(b),.i_count_clear(clear),.o_count(counts),.o_delta(deltas),
        .o_delta_valid(valid),.o_enc_dir());
    always @(posedge clk)if(rst && valid)begin last_delta<=deltas;windows=windows+1;end
    task check;
        input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task zero;
        begin
            a=0;b=0;repeat(30)@(negedge clk);
            clear=1;@(negedge clk);clear=0;repeat(30)@(negedge clk);
        end
    endtask
    task automatic forward;
        input integer lane,hold_clocks,cycles;
        integer n;
        begin
            for(n=0;n<cycles;n=n+1)begin
                {a[lane],b[lane]}=2'b10;repeat(hold_clocks)@(negedge clk);
                {a[lane],b[lane]}=2'b11;repeat(hold_clocks)@(negedge clk);
                {a[lane],b[lane]}=2'b01;repeat(hold_clocks)@(negedge clk);
                {a[lane],b[lane]}=2'b00;repeat(hold_clocks)@(negedge clk);
            end
        end
    endtask
    task missing_b;
        integer n;
        begin
            for(n=0;n<5000;n=n+1)begin
                a[0]=1;repeat(50)@(negedge clk);
                a[0]=0;repeat(50)@(negedge clk);
            end
        end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst=1;zero();
        fork
            forward(0,50,2500);  // 1MHz edges; above expected vehicle speed.
            forward(1,100,1250);
            forward(2,250,500);
            forward(3,500,250);
        join
        repeat(30)@(negedge clk);
        $display("ENCODER_COUNTS A=%0d B=%0d C=%0d D=%0d",$signed(counts[31:0]),$signed(counts[63:32]),$signed(counts[95:64]),$signed(counts[127:96]));
        check(counts[31:0]==10000 && counts[63:32]==5000 && counts[95:64]==2000 && counts[127:96]==1000,
              "all four real decoders retain fast asynchronous quadrature counts");
        check($signed(last_delta[31:0])>=9998 && $signed(last_delta[63:32])>=4998 &&
              $signed(last_delta[95:64])>=1998 && $signed(last_delta[127:96])>=998,
              "10ms shared sample publishes each wheel without A-channel loss");
        check(!stby,"encoder acquisition continues with all motor outputs disabled");
        zero();
        fork missing_b();forward(1,100,1250);join
        repeat(30)@(negedge clk);
        check(counts[31:0]==0 && counts[63:32]==5000,
              "missing A-wheel B phase gives zero net counts while another wheel measures normally");
        check($signed(last_delta[31:0])>=-1 && $signed(last_delta[31:0])<=1 && $signed(last_delta[63:32])>=4998,
              "fast single-phase edges reproduce near-zero speed feedback rather than a display fault");
        zero();forward(0,50,1250);repeat(30)@(negedge clk);
        check(counts[31:0]==5000,"restoring both phases recovers A counting without a fault latch");
        check(windows>=2 && !stby,"sampling stays periodic and the test never enables motors");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: encoder_chain %0d checks",checks);else $display("FAIL: encoder_chain %0d",errors);
        $finish;
    end
    initial begin #80000000;$display("FAIL: encoder_chain timeout");$finish;end
endmodule
