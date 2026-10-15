`timescale 1ns/1ps
module point_bitmap_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset_n = 0, point_valid = 0, video_de = 0, frame_start = 0;
    reg [8:0] point_x = 0, point_y = 0;
    reg [11:0] video_x = 0, video_y = 0;
    wire point_ready, pixel_bit, pixel_valid, bitmap_ready;
    wire [15:0] front_points;
    integer checks = 0, clear_cycles;
    point_bitmap #(.REFRESH_FRAMES(2)) dut (
        .clk(clk), .reset_n(reset_n), .point_valid(point_valid),
        .point_ready(point_ready), .point_x(point_x), .point_y(point_y),
        .video_x(video_x), .video_y(video_y), .video_de(video_de),
        .frame_start(frame_start), .pixel_bit(pixel_bit),
        .pixel_valid(pixel_valid), .front_points(front_points),
        .bitmap_ready(bitmap_ready)
    );
    task write_point;
        input integer x,y;
        begin
            @(negedge clk); while (!point_ready) @(negedge clk);
            point_x = x; point_y = y; point_valid = 1;
            @(posedge clk); #1;
            @(negedge clk); point_valid = 0;
        end
    endtask
    task read_point;
        input integer x,y,expected;
        begin
            @(negedge clk); video_x = x + 256; video_y = y + 44; video_de = 1;
            @(posedge clk); #1;
            if (!pixel_valid || pixel_bit !== expected[0])
                $fatal(1,"bitmap (%0d,%0d) expected=%0d got=%b",x,y,expected,pixel_bit);
            checks = checks + 1;
        end
    endtask
    task frame;
        begin
            @(negedge clk); frame_start = 1;
            @(posedge clk); #1;
            @(negedge clk); frame_start = 0;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); reset_n = 1;
        clear_cycles = 0;
        while (!bitmap_ready) begin
            @(posedge clk); #1; clear_cycles = clear_cycles + 1;
            if (!bitmap_ready && point_ready) $fatal(1,"input accepted during boot clear");
        end
        if (clear_cycles != 8192) $fatal(1,"boot clear length %0d",clear_cycles);
        read_point(0,0,0); read_point(511,511,0);
        write_point(0,0); write_point(31,0); write_point(32,0);
        write_point(511,511); write_point(255,255); write_point(256,256);
        write_point(31,0); // Duplicate and adjacent writes must preserve bits.
        read_point(0,0,0); read_point(31,0,0); read_point(511,511,0);
        frame;
        if (front_points != 0) $fatal(1,"swapped too early");
        frame;
        if (front_points != 7 || point_ready) $fatal(1,"swap/count/clear incorrect");
        // A held input survives the entire clear and belongs to the next image.
        point_x = 99; point_y = 77; point_valid = 1;
        read_point(0,0,1); read_point(31,0,1); read_point(32,0,1);
        read_point(511,511,1); read_point(255,255,1); read_point(256,256,1);
        read_point(30,0,0); read_point(33,0,0); read_point(99,77,0);
        while (!point_ready) @(negedge clk);
        @(posedge clk); #1;
        @(negedge clk); point_valid = 0;
        read_point(99,77,0); read_point(0,0,1);
        frame; frame;
        if (front_points != 1) $fatal(1,"held point was lost or duplicated");
        read_point(99,77,1); read_point(0,0,0); read_point(31,0,0);
        read_point(511,511,0); read_point(-1,0,0); read_point(512,511,0);
        while (!point_ready) @(negedge clk);
        // A frame boundary during RMW cannot swap an incompletely written image.
        frame;
        write_point(11,12);
        frame_start = 1;
        @(posedge clk); #1;
        if (front_points != 1) $fatal(1,"swap occurred during write");
        @(negedge clk); frame_start = 0;
        frame;
        if (front_points != 1) $fatal(1,"pending swap/count incorrect");
        read_point(11,12,1); read_point(99,77,0);
        @(negedge clk); video_de = 0;
        @(posedge clk); #1;
        if (pixel_valid || pixel_bit) $fatal(1,"blanking pixel was visible");
        $display("PASS point_bitmap_tb: %0d visibility/RMW/clear/swap/boundary checks",checks);
        $finish;
    end
    initial begin #3000000; $fatal(1,"bitmap test timeout"); end
endmodule
