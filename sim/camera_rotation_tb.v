`timescale 1ns/1ps
module camera_rotation_tb;
    reg clk=0;
    reg [7:0] addr=0;
    wire [23:0] normal,rotated;
    integer n,checks=0,errors=0;
    always #10 clk=~clk;
    ov5640_init_table_rgb #(.IMAGE_WIDTH(1024),.IMAGE_HEIGHT(600)) original(
        .clk(clk),.addr(addr),.q(normal));
    ov5640_init_table_rgb #(.IMAGE_WIDTH(1024),.IMAGE_HEIGHT(600),
        .IMAGE_FLIP_EN(1),.IMAGE_MIRROR_EN(1)) rotate180(
        .clk(clk),.addr(addr),.q(rotated));
    initial begin
        for(n=0;n<253;n=n+1)begin
            @(negedge clk);addr=n;
            @(posedge clk);#1;
            checks=checks+1;
            if((^normal)===1'bx || (^rotated)===1'bx)begin
                errors=errors+1;$display("FAIL: unknown camera config %0d",n);
            end else if(n==211)begin
                if(normal!==24'h382040 || rotated!==24'h382047)begin
                    errors=errors+1;$display("FAIL: vertical orientation register");end
                else $display("pass vertical orientation register");
            end else if(n==212)begin
                if(normal!==24'h382107 || rotated!==24'h382100)begin
                    errors=errors+1;$display("FAIL: horizontal orientation register");end
                else $display("pass horizontal orientation register");
            end else if(normal!==rotated)begin
                errors=errors+1;$display("FAIL: unrelated camera config changed %0d",n);
            end else $display("pass unchanged camera config %0d",n);
        end
        $display("CHECKS: %0d",checks);$display("%0d FAIL",errors);
        if(errors==0)$display("PASS: camera_rotation_tb");
        else $display("FAIL: camera_rotation_tb");
        $finish;
    end
endmodule
