`timescale 1ns/1ps
module yaw_pipeline_tb;
    reg clk=0,rst=0,enable=1,clear=0,sample=0;
    reg signed [31:0] gz=0,previous_yaw=0;
    wire signed [31:0] original,pipelined;
    wire original_valid,pipelined_valid;
    integer cycle=0,compared=0;
    always #10 clk=~clk;
    yaw_integrate #(.CLK_FREQ_HZ(4096),.SUM_PIPELINE(0)) reference(
        .i_clock(clk),.i_reset_n(rst),.i_gyro_z(gz),.i_sample_valid(sample),
        .i_enable(enable),.i_clear(clear),.o_yaw(original),.o_yaw_valid(original_valid),.o_deadzone_hit(),.o_gz_eff());
    yaw_integrate #(.CLK_FREQ_HZ(4096),.SUM_PIPELINE(1)) candidate(
        .i_clock(clk),.i_reset_n(rst),.i_gyro_z(gz),.i_sample_valid(sample),
        .i_enable(enable),.i_clear(clear),.o_yaw(pipelined),.o_yaw_valid(pipelined_valid),.o_deadzone_hit(),.o_gz_eff());
    always @(posedge clk)begin
        #1;
        if(rst)begin
            if(enable && !clear && pipelined!==previous_yaw)$fatal(1,"FAIL: yaw sample mismatch cycle=%0d expected=%0d got=%0d",cycle,previous_yaw,pipelined);
            if((!enable || clear) && pipelined!==0)$fatal(1,"FAIL: pipeline bypassed clear/disable");
            if(original_valid!==pipelined_valid)$fatal(1,"FAIL: yaw validity changed");
            compared=compared+1;
        end
        previous_yaw=original;
    end
    initial begin
        repeat(4)@(negedge clk);rst=1;
        for(cycle=0;cycle<20000;cycle=cycle+1)begin
            @(negedge clk);sample=(cycle%3)==0;clear=(cycle%137)==100;
            enable=cycle%97>=3;
            case((cycle/67)%9)
                0:gz=200000;1:gz=-200000;2:gz=29;3:gz=-29;
                4:gz=30;5:gz=-30;7:gz=32'sh80000000;8:gz=32'sh7fffffff;
                default:gz=(cycle*731)%400001-200000;
            endcase
        end
        @(negedge clk);
        $display("pass %0d cycle comparisons preserve samples, wraps, deadzone, clear and disable",compared);
        $display("CHECKS: 1");$display("0 FAIL");$display("PASS: yaw_pipeline");$finish;
    end
endmodule
