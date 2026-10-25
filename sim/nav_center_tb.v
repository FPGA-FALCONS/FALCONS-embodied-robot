`timescale 1ns/1ps
module nav_center_tb;
    reg clk=0,reset_n=0,start=0,in_valid=0,in_last=0;
    always #10 clk=~clk;
    reg [15:0] start_x,start_y,in_x,in_y;
    wire in_ready,out_valid,out_last,busy,request;
    wire [15:0] out_x,out_y,rx,ry;
    reg response=0,raw=0,cost=0,pending=0;
    reg map [0:65535];
    integer cycles=0,checks=0,errors=0,d,dx,dy,nx,ny,t,v,i,mode,address,delay_left;
    integer count,ax,ay,bx,by,cx,cy,ex,ey,wait_count,begin_cycle,segment_length,probe_position,probe_step;
    reg [255:0] observed_probes,expected_probes;
    reg shape_ok,stable_ok,held=0;
    reg [32:0] hold_data;
    wire ready=cycles%7>=2;
    wire map_ready=!pending && cycles%5!=0;
    always @(posedge clk) begin
        cycles<=cycles+1;
        if(dut.state==6) observed_probes[dut.probe_p]=1;
        response<=0;
        if(pending) begin
            if(delay_left==0) begin
                response<=1;
                raw<=map[address];
                cost<=map[address];
                pending<=0;
            end else delay_left<=delay_left-1;
        end
        if(request && map_ready) begin
            pending<=1;
            address<=ry*256+rx;
            delay_left<=cycles%4;
            if(rx>255 || ry>255) begin $display("FAIL: center out-of-map read"); errors=errors+1; end
        end
        if(held && (!out_valid || {out_last,out_y,out_x}!==hold_data)) stable_ok=0;
        held=out_valid && !ready;
        if(held) hold_data={out_last,out_y,out_x};
        if(out_valid && ready) begin
            case(count)
                0: if(out_x!=(mode==0 ? bx+4*nx : bx) || out_y!=(mode==0 ? by+4*ny : by) || out_last) shape_ok=0;
                1: if(out_x!=(mode==0 ? cx+4*nx : cx) || out_y!=(mode==0 ? cy+4*ny : cy) || out_last) shape_ok=0;
                2: if(out_x!=ex || out_y!=ey || !out_last) shape_ok=0;
                default: shape_ok=0;
            endcase
            count=count+1;
        end
    end
    nav_center #(.PATH_INC_MAX(0)) dut (
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(1'b0),
        .start_x(start_x),.start_y(start_y),.start_relaxed(1'b0),
        .in_valid(in_valid),.in_ready(in_ready),.in_x(in_x),.in_y(in_y),.in_last(in_last),
        .out_valid(out_valid),.out_ready(ready),.out_x(out_x),.out_y(out_y),.out_last(out_last),
        .mrd_valid(request),.mrd_ready(map_ready),.mrd_x(rx),.mrd_y(ry),
        .mrd_rsp_valid(response),.mrd_raw(raw),.mrd_cost(cost),.busy(busy)
    );
    task point;
        input [15:0] x,y;
        input last;
        begin
            @(negedge clk); in_x=x; in_y=y; in_last=last; in_valid=1;
            while(!in_ready) @(negedge clk);
            @(negedge clk); in_valid=0;
        end
    endtask
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: center dir=%0d mode=%0d %0s",d,mode,text); end
            else $display("pass center dir=%0d mode=%0d %0s",d,mode,text);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); reset_n=1;
        check(dut.DIAG_D==14 && dut.DIAG_STEP==2 && dut.DIAG_MIN==3,"diagonal probe constants preserve physical scale");
        for(mode=0;mode<3;mode=mode+1) begin
            for(d=0;d<(mode==0 ? 8 : 1);d=d+1) begin
                for(i=0;i<65536;i=i+1) map[i]=0;
                case(d) 0,1,7:dx=1; 3,4,5:dx=-1; default:dx=0; endcase
                case(d) 1,2,3:dy=1; 5,6,7:dy=-1; default:dy=0; endcase
                nx=-dy; ny=dx;
                bx=100; by=100;
                segment_length=mode==0 && d>=4 ? 21 : 20;
                cx=bx+segment_length*dx; cy=by+segment_length*dy;
                ax=bx-12*nx; ay=by-12*ny;
                ex=cx+(mode==1 ? -12 : 12)*nx;
                ey=cy+(mode==1 ? -12 : 12)*ny;
                for(t=3;t<=segment_length-3;t=t+1) begin
                    map[(by+t*dy-2*ny)*256+bx+t*dx-2*nx]=1;
                    map[(by+t*dy+10*ny)*256+bx+t*dx+10*nx]=1;
                end
                // t=10 lies between cardinal probe positions 8 and 12.
                if(mode==2) map[(by+10*dy+4*ny)*256+bx+10*dx+4*nx]=1;
                count=0; shape_ok=1; stable_ok=1;
                observed_probes=0; expected_probes=0;
                probe_step=d%2 ? 2 : 4;
                for(probe_position=0;probe_position<segment_length;probe_position=probe_position+probe_step)
                    expected_probes[probe_position]=1;
                expected_probes[segment_length]=1;
                @(negedge clk); start_x=ax; start_y=ay; start=1; begin_cycle=cycles;
                @(negedge clk); start=0;
                point(bx,by,0); point(cx,cy,0); point(ex,ey,1);
                wait_count=0;
                while(busy && wait_count<200000) begin @(negedge clk); wait_count=wait_count+1; end
                check(!busy && count==3 && shape_ok,"expected shift or rejection preserves three directions and final goal");
                check(stable_ok,"centered waypoints hold under backpressure");
                check(observed_probes===expected_probes,"probes include both endpoints and non-multiple tail exactly");
                check(dut.d_plus==10 && dut.d_minus==2,"all probe locations contribute to minimum clearances");
                $display("METRICS: center dir=%0d mode=%0d cycles=%0d budget=%0d",d,mode,cycles-begin_cycle,dut.cumulative);
                repeat(3) @(negedge clk);
            end
        end
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_center_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: nav_center_tb");
        $finish;
    end
    initial begin #100000000; $display("FAIL: center timeout"); $finish; end
endmodule
