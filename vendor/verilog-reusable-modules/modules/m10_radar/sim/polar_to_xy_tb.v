`timescale 1ns/1ps
module polar_to_xy_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset_n = 0, point_valid = 0, pixel_ready = 0;
    reg [15:0] angle_cdeg = 0, distance_mm = 0;
    reg [1:0] zoom_level = 2;
    wire point_ready, pixel_valid;
    wire [9:0] pixel_x, pixel_y;
    integer checks = 0;
    integer angle, offset_x, offset_y, hold, zoom, range_limit;
    real theta, reference_x, reference_y;
    polar_to_xy dut (
        .clk(clk), .reset_n(reset_n), .point_valid(point_valid),
        .point_ready(point_ready), .angle_cdeg(angle_cdeg),
        .distance_mm(distance_mm), .zoom_level(zoom_level), .pixel_valid(pixel_valid),
        .pixel_ready(pixel_ready), .pixel_x(pixel_x), .pixel_y(pixel_y)
    );

    task submit;
        input integer a, distance;
        begin
            @(negedge clk);
            while (!point_ready) @(negedge clk);
            angle_cdeg = a; distance_mm = distance; point_valid = 1;
            @(posedge clk); #1;
            @(negedge clk); point_valid = 0;
        end
    endtask
    task expect_locked_zoom;
        input integer accept_zoom, next_zoom, a, distance, x, y;
        reg [9:0] held_x, held_y;
        integer cycle;
        begin
            zoom_level = accept_zoom;
            submit(a, distance);
            // submit returns before the LUT/multiply/scale stages execute.
            zoom_level = next_zoom;
            while (!pixel_valid) @(negedge clk);
            if (pixel_x != x || pixel_y != y)
                $fatal(1,"in-flight zoom changed sample: accepted zoom=%0d next=%0d got=(%0d,%0d) expected=(%0d,%0d)",
                       accept_zoom,next_zoom,pixel_x,pixel_y,x,y);
            held_x = pixel_x; held_y = pixel_y;
            for (cycle = 0; cycle < 8; cycle = cycle + 1) begin
                @(negedge clk); zoom_level = cycle % 4;
                #1;
                if (!pixel_valid || point_ready || pixel_x !== held_x || pixel_y !== held_y)
                    $fatal(1,"stalled output changed when zoom changed");
            end
            pixel_ready = 1;
            @(posedge clk); #1;
            if (pixel_valid) $fatal(1,"locked output was not consumed");
            @(negedge clk); pixel_ready = 0;
            checks = checks + 1;
        end
    endtask
    task expect_coordinate;
        input integer a, distance, x, y, tolerance;
        integer dx_error, dy_error;
        reg [9:0] held_x, held_y;
        begin
            submit(a, distance);
            while (!pixel_valid) @(negedge clk);
            dx_error = pixel_x - x; dy_error = pixel_y - y;
            if (dx_error < 0) dx_error = -dx_error;
            if (dy_error < 0) dy_error = -dy_error;
            if (dx_error > tolerance || dy_error > tolerance)
                $fatal(1,"coordinate angle=%0d distance=%0d got=(%0d,%0d) expected=(%0d,%0d)",
                       a,distance,pixel_x,pixel_y,x,y);
            held_x = pixel_x; held_y = pixel_y;
            for (hold = 0; hold < 4; hold = hold + 1) begin
                @(negedge clk);
                if (!pixel_valid || point_ready || pixel_x !== held_x || pixel_y !== held_y)
                    $fatal(1,"output changed while backpressured");
            end
            pixel_ready = 1;
            @(posedge clk); #1;
            if (pixel_valid) $fatal(1,"output was not consumed");
            @(negedge clk); pixel_ready = 0;
            checks = checks + 1;
        end
    endtask
    task expect_discard;
        input integer distance;
        integer cycle;
        begin
            submit(12345, distance);
            for (cycle = 0; cycle < 12; cycle = cycle + 1) begin
                @(negedge clk);
                if (pixel_valid) $fatal(1,"invalid range %0d emitted a point",distance);
            end
            if (!point_ready) $fatal(1,"discard failed to release input");
            checks = checks + 1;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); reset_n = 1;
        expect_coordinate(0,1000,512,275,0);
        expect_coordinate(9000,1000,537,300,0);
        expect_coordinate(18000,1000,512,325,0);
        expect_coordinate(27000,1000,487,300,0);
        expect_coordinate(36000,1000,512,275,0);
        expect_coordinate(4500,10000,689,123,0);
        expect_coordinate(13500,10000,689,477,0);
        expect_coordinate(22500,10000,335,477,0);
        expect_coordinate(31500,10000,335,123,0);
        expect_coordinate(0,1,512,300,0);
        expect_coordinate(9000,1030,538,300,0);
        expect_discard(0);
        expect_discard(10001);
        expect_discard(65535);
        // The same 2 m measurement has radii 200/100/50/25 pixels.
        for (zoom = 0; zoom < 4; zoom = zoom + 1) begin
            zoom_level = zoom;
            expect_coordinate(9000,2000,512+(200 >> zoom),300,0);
            expect_coordinate(0,2000,512,300-(200 >> zoom),0);
            range_limit = 2500 << zoom;
            expect_coordinate(9000,range_limit,762,300,0);
            expect_coordinate(27000,range_limit,262,300,0);
            expect_discard(range_limit+1);
            expect_discard(0);
            expect_discard(65535);
            // The half-pixel tie rounds away from the center at every scale.
            expect_coordinate(9000,5 << zoom,513,300,0);
            expect_coordinate(27000,5 << zoom,511,300,0);
            // Continuous-angle reference is independent of the generated ROM.
            // The 0.1-degree table may differ from this reference by one pixel.
            for (angle = 0; angle < 36000; angle = angle + 137) begin
                theta = angle * 3.141592653589793 / 18000.0;
                reference_x = $sin(theta) * 250.0;
                reference_y = $cos(theta) * 250.0;
                offset_x = $rtoi(reference_x + (reference_x >= 0 ? 0.5 : -0.5));
                offset_y = $rtoi(reference_y + (reference_y >= 0 ? 0.5 : -0.5));
                expect_coordinate(angle,range_limit,512+offset_x,300-offset_y,1);
            end
        end
        zoom_level = 2;
        theta = (65535-36000) * 3.141592653589793 / 18000.0;
        reference_x = $sin(theta) * 250.0; reference_y = $cos(theta) * 250.0;
        offset_x = $rtoi(reference_x + (reference_x >= 0 ? 0.5 : -0.5));
        offset_y = $rtoi(reference_y + (reference_y >= 0 ? 0.5 : -0.5));
        expect_coordinate(65535,10000,512+offset_x,300-offset_y,1);
        expect_locked_zoom(0,3,9000,2000,712,300);
        // A range accepted at the wide scale stays valid when the live scale
        // becomes narrower than that range during the conversion pipeline.
        expect_locked_zoom(3,0,18000,20000,512,550);
        zoom_level = 0;
        expect_discard(20000);
        zoom_level = 3;
        expect_coordinate(18000,20000,512,550,0);
        $display("PASS polar_to_xy_tb: %0d four-scale/range/quadrant/rounding/locked-zoom/backpressure checks",checks);
        $finish;
    end
    initial begin #2000000; $fatal(1,"polar test timeout"); end
endmodule
