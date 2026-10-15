`timescale 1ns/1ps

module disp_driver_tb;

    localparam integer H_TOTAL = 1312;
    localparam integer V_TOTAL = 622;
    localparam integer ACTIVE_PIXELS = 1024 * 600;
    localparam integer FRAME_CYCLES = H_TOTAL * V_TOTAL;

    reg clk;
    reg reset_n;
    reg [23:0] pixel_data;
    wire pixel_req;
    wire [11:0] pixel_x;
    wire [11:0] pixel_y;
    wire video_hs;
    wire video_vs;
    wire [7:0] video_r;
    wire [7:0] video_g;
    wire [7:0] video_b;
    wire frame_start;
    wire video_active;
    wire pixel_clk;

    integer frame_count;
    integer cycles_since_frame;
    integer active_pixel_count;
    integer error_count;
    reg previous_frame_start;

    disp_driver dut (
        .ClkDisp     (clk),
        .Rst_n       (reset_n),
        .Data        (pixel_data),
        .DataReq     (pixel_req),
        .H_Addr      (pixel_x),
        .V_Addr      (pixel_y),
        .Disp_HS     (video_hs),
        .Disp_VS     (video_vs),
        .Disp_Red    (video_r),
        .Disp_Green  (video_g),
        .Disp_Blue   (video_b),
        .Frame_Begin (frame_start),
        .Disp_DE     (video_active),
        .Disp_PCLK   (pixel_clk)
    );

    always #10 clk = ~clk;

    always @(posedge clk) begin
        #1;
        if(!reset_n)begin
            frame_count = 0;
            cycles_since_frame = 0;
            active_pixel_count = 0;
            error_count = 0;
            previous_frame_start = 1'b0;
        end
        else begin
            cycles_since_frame = cycles_since_frame + 1;

            if(pixel_req)begin
                active_pixel_count = active_pixel_count + 1;
                if(pixel_x >= 1024 || pixel_y >= 600)begin
                    $display("ERROR: active coordinate out of range: x=%0d y=%0d", pixel_x, pixel_y);
                    error_count = error_count + 1;
                end
            end

            if(frame_start)begin
                if(previous_frame_start)begin
                    $display("ERROR: frame_start lasted more than one pixel clock");
                    error_count = error_count + 1;
                end

                frame_count = frame_count + 1;
                if(frame_count > 1)begin
                    if(cycles_since_frame != FRAME_CYCLES)begin
                        $display("ERROR: frame length expected %0d got %0d", FRAME_CYCLES, cycles_since_frame);
                        error_count = error_count + 1;
                    end
                    if(active_pixel_count != ACTIVE_PIXELS)begin
                        $display("ERROR: active pixels expected %0d got %0d", ACTIVE_PIXELS, active_pixel_count);
                        error_count = error_count + 1;
                    end
                end

                cycles_since_frame = 0;
                active_pixel_count = 0;

                if(frame_count == 3)begin
                    if(error_count == 0)
                        $display("PASS: 1024x600 display timing checks passed");
                    else
                        $fatal(1, "FAIL: display timing detected %0d errors", error_count);
                    $finish;
                end
            end

            previous_frame_start = frame_start;
        end
    end

    initial begin
        clk = 1'b0;
        reset_n = 1'b0;
        pixel_data = 24'h5AA55A;
        frame_count = 0;
        cycles_since_frame = 0;
        active_pixel_count = 0;
        error_count = 0;
        previous_frame_start = 1'b0;

        repeat(5) @(posedge clk);
        reset_n = 1'b1;
    end

    initial begin
        repeat(2500000) @(posedge clk);
        $fatal(1, "FAIL: display timing test timed out");
    end

endmodule
