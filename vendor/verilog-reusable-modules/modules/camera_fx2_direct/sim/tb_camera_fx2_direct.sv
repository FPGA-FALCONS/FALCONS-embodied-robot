`timescale 1ns/1ps
// Self-checking direct-stream testbench.
//
// SCENARIO:
//   0 = clean frames
//   1 = frame decimation
//   2 = USB stall -> overflow -> backoff -> recovery
//   3 = truncated and extra-pixel frames -> recovery
//   4 = packet-aligned total length (no PKTEND)
//   5 = reset in the middle of a frame, then recovery
module tb_camera_fx2_direct;
    parameter integer SOURCE_WIDTH      = 8;
    parameter integer SOURCE_HEIGHT     = 6;
    parameter integer FRAME_DECIMATION  = 1;
    parameter integer BACKOFF_FRAMES    = 2;
    parameter integer START_WAIT_CYCLES = 4096;
    parameter integer FIFO_DEPTH        = 64;
    parameter integer CAM_HALF_NS       = 7;
    parameter integer IF_HALF_NS        = 5;
    parameter integer DECIM_NUM         = 1;
    parameter integer DECIM_DEN         = 1;
    parameter integer FLAGC_DUTY        = 100;
    parameter integer SCENARIO          = 0;

    localparam integer FRAME_PIXELS = SOURCE_WIDTH * SOURCE_HEIGHT;
    localparam integer OUT_WIDTH    = (SOURCE_WIDTH / DECIM_DEN) * DECIM_NUM;
    localparam integer OUT_HEIGHT   = (SOURCE_HEIGHT / DECIM_DEN) * DECIM_NUM;
    localparam integer OUT_PIXELS   = OUT_WIDTH * OUT_HEIGHT;
    localparam integer TOTAL_BYTES  = 24 + OUT_PIXELS * 2 + 4;
    localparam integer TOTAL_WORDS  = TOTAL_BYTES / 2;
    localparam         SHORT_PACKET = (TOTAL_BYTES % 512) != 0;

    reg cam_clk = 0;
    reg ifclk   = 0;
    always #(CAM_HALF_NS) cam_clk = ~cam_clk;
    always #(IF_HALF_NS) ifclk = ~ifclk;

    reg reset_n = 0;
    reg cam_vsync = 0;
    reg cam_valid = 0;
    reg [15:0] cam_rgb565 = 0;
    reg fx2_flagc = 0;
    wire [15:0] fx2_fd;
    wire [1:0]  fx2_faddr;
    wire fx2_slwr_n, fx2_sloe_n, fx2_slrd_n, fx2_pktend_n;
    wire capture_busy;
    wire [31:0] frames_sent, frames_dropped, frames_malformed, frames_aborted;
    wire fault;

    camera_fx2_direct #(
        .SOURCE_WIDTH(SOURCE_WIDTH), .SOURCE_HEIGHT(SOURCE_HEIGHT),
        .FRAME_DECIMATION(FRAME_DECIMATION), .BACKOFF_FRAMES(BACKOFF_FRAMES),
        .START_WAIT_CYCLES(START_WAIT_CYCLES), .FIFO_DEPTH(FIFO_DEPTH),
        .DECIM_NUM(DECIM_NUM), .DECIM_DEN(DECIM_DEN)
    ) dut (
        .cam_clk(cam_clk), .ifclk(ifclk), .reset_n(reset_n),
        .cam_vsync(cam_vsync), .cam_valid(cam_valid), .cam_rgb565(cam_rgb565),
        .fx2_flagc(fx2_flagc), .fx2_fd(fx2_fd), .fx2_faddr(fx2_faddr),
        .fx2_slwr_n(fx2_slwr_n), .fx2_sloe_n(fx2_sloe_n),
        .fx2_slrd_n(fx2_slrd_n), .fx2_pktend_n(fx2_pktend_n),
        .capture_busy(capture_busy),
        .frames_sent(frames_sent), .frames_dropped(frames_dropped),
        .frames_malformed(frames_malformed), .frames_aborted(frames_aborted),
        .fault(fault)
    );

    integer bin_fd, meta_fd;
    integer wire_words = 0;
    integer frame_words = 0;
    integer wire_frames = 0;
    integer pktend_count = 0;
    integer stall_cycles = 0;
    integer pktend_stall_cycles = 0;
    integer tb_frame_index = 0;
    integer i;
    reg [31:0] rng = 32'h13579bdf;
    integer duty_counter = 0;
    reg random_stalls = 0;
    reg force_full = 0;
    reg pktend_hold = 0;
    reg was_stalled = 0;
    reg previous_pktend = 0;
    reg [15:0] stalled_data = 0;
    reg first_boundary = 1;
    string out_dir;

    always @(negedge ifclk) begin
        if (force_full || pktend_hold)
            fx2_flagc = 1'b0;
        else if (FLAGC_DUTY < 100) begin
            duty_counter = (duty_counter == 99) ? 0 : duty_counter + 1;
            fx2_flagc = (duty_counter < FLAGC_DUTY);
        end else if (random_stalls) begin
            rng = {rng[30:0], rng[31]^rng[21]^rng[1]^rng[0]};
            fx2_flagc = (rng[3:0] > 3);
        end else
            fx2_flagc = 1'b1;
    end

    // FX2 write-side protocol checks and wire capture.
    always @(posedge ifclk) begin
        if (!reset_n) begin
            was_stalled = 0;
            previous_pktend = 0;
        end else begin
            if (fx2_faddr !== 2'b10 || fx2_sloe_n !== 1'b1 || fx2_slrd_n !== 1'b1)
                $fatal(1, "FX2 fixed address/direction mismatch");
            if (!fx2_flagc && !fx2_slwr_n)
                $fatal(1, "SLWR asserted while FLAGC reports full");
            if (was_stalled && fx2_fd !== stalled_data)
                $fatal(1, "FD changed while transfer stalled");
            was_stalled = (!fx2_flagc &&
                (dut.state == dut.TX_HDR_SEND || dut.state == dut.PAY_SEND ||
                 dut.state == dut.PAD_SEND || dut.state == dut.TRAILER_SEND));
            stalled_data = fx2_fd;
            if (!fx2_flagc && was_stalled)
                stall_cycles = stall_cycles + 1;

            if (!fx2_slwr_n) begin
                if (frame_words == 0 && fx2_fd !== 16'h4946)
                    $fatal(1, "Frame does not start with FIMG magic word: got %04x frame_words=%0d wire_words=%0d", fx2_fd, frame_words, wire_words);
                if (frame_words == 1 && fx2_fd !== 16'h474d)
                    $fatal(1, "Bad FIMG second magic word");
                if (frame_words == 2 && fx2_fd !== 16'h0102)
                    $fatal(1, "Bad FIMG v2/format word");
                if (frame_words == 3 && fx2_fd !== 16'h0018)
                    $fatal(1, "Bad FIMG header length");
                if (frame_words == 4 && fx2_fd !== OUT_WIDTH[15:0])
                    $fatal(1, "Bad FIMG width");
                if (frame_words == 5 && fx2_fd !== OUT_HEIGHT[15:0])
                    $fatal(1, "Bad FIMG height");
                $fwrite(bin_fd, "%c%c", fx2_fd[7:0], fx2_fd[15:8]);
                wire_words = wire_words + 1;
                frame_words = frame_words + 1;
                if (frame_words == TOTAL_WORDS) begin
                    frame_words = 0;
                    wire_frames = wire_frames + 1;
                    $display("FRAME_DONE t=%0t words=%0d", $time, wire_words);
                end
            end

            if (!fx2_pktend_n) begin
                if (previous_pktend)
                    $fatal(1, "PKTEND longer than one IFCLK");
                if (!fx2_flagc || !fx2_slwr_n)
                    $fatal(1, "PKTEND while full or while SLWR active");
                if (frame_words != 0)
                    $fatal(1, "PKTEND in the middle of a frame");
                pktend_count = pktend_count + 1;
            end
            previous_pktend = !fx2_pktend_n;
        end
    end

    // Deliberate PACKET_END stall coverage: hold FLAGC low when the DUT enters
    // the guard, then release and check that exactly one PKTEND emerges.
    initial begin
        if (SHORT_PACKET && SCENARIO != 5) begin
            wait(reset_n);
            wait(dut.state == dut.PKT_GUARD);
            @(negedge ifclk); pktend_hold = 1;
            repeat(7) begin
                @(negedge ifclk);
                if (fx2_pktend_n !== 1'b1 || dut.state != dut.PKT_GUARD)
                    $fatal(1, "PACKET_END committed while FLAGC full");
                pktend_stall_cycles = pktend_stall_cycles + 1;
            end
            pktend_hold = 0;
        end
    end

    task automatic blank(input integer cycles);
        integer k;
        begin
            for (k = 0; k < cycles; k = k + 1) begin
                @(negedge cam_clk);
                cam_valid = 1'b0;
            end
        end
    endtask

    task automatic boundary;
        begin
            @(negedge cam_clk); cam_valid = 1'b0; cam_vsync = 1'b1;
            @(negedge cam_clk); cam_vsync = 1'b0;
            repeat (4) @(negedge cam_clk);
        end
    endtask

    task automatic pixels(input integer tag, input integer count);
        integer p;
        begin
            for (p = 0; p < count; p = p + 1) begin
                @(negedge cam_clk);
                cam_valid = 1'b1;
                cam_rgb565 = ((tag << 8) + p) & 16'hffff;
                if ((p % 13) == 7) begin
                    @(negedge cam_clk);
                    cam_valid = 1'b0;
                end
            end
            @(negedge cam_clk);
            cam_valid = 1'b0;
        end
    endtask

    task automatic record_frame(input integer tag, input integer count,
                                input integer good_hint);
        begin
            $fdisplay(meta_fd, "%0d %0d %0d %0d",
                      tb_frame_index, tag, count, good_hint);
            tb_frame_index = tb_frame_index + 1;
        end
    endtask

    // A frame occupies the interval after one boundary through the next
    // boundary.  The boundary at the end closes it and opens the next slot.
    task automatic frame(input integer tag, input integer count,
                         input integer good_hint, input integer pre_blank);
        integer ready;
        begin
            if (first_boundary) begin
                boundary();
                first_boundary = 0;
            end
            // A source frame is only pixel-valid if the DUT admitted it.  If
            // decimation/backoff dropped it, do not inject pixels into the
            // next frame's header window.
            if ((dut.cam_state == dut.CAM_WAIT_TX) ||
                (dut.cam_state == dut.CAM_HEADER) ||
                (dut.cam_state == dut.CAM_ACTIVE)) begin
                wait_cam_active_ok(400000, ready);
                if (ready) begin
                    blank(pre_blank);
                    pixels(tag, count);
                    blank(12);
                    record_frame(tag, count, good_hint);
                end else begin
                    blank(12);
                    record_frame(tag, 0, 0);
                end
            end else begin
                blank(pre_blank);
                record_frame(tag, count, 0);
            end
            boundary();
        end
    endtask

    task automatic wait_tx_idle(input integer timeout);
        integer k;
        begin
            k = 0;
            while (dut.tx_idle_if !== 1'b1 && k < timeout) begin
                @(negedge ifclk);
                k = k + 1;
            end
            if (k == timeout) $fatal(1, "TX did not return to idle");
        end
    endtask

    task automatic wait_cam_active_ok(input integer timeout, output integer ok);
        integer k;
        begin
            k = 0;
            while (dut.cam_state != dut.CAM_ACTIVE && k < timeout) begin
                @(negedge cam_clk);
                k = k + 1;
            end
            ok = (dut.cam_state == dut.CAM_ACTIVE);
        end
    endtask

    task automatic wait_cam_active(input integer timeout);
        integer ok;
        begin
            wait_cam_active_ok(timeout, ok);
            if (!ok) $fatal(1, "Camera frame did not become active");
        end
    endtask

    task automatic wait_frames(input integer count, input integer timeout);
        integer k;
        begin
            k = 0;
            while ((frames_sent + frames_aborted) < count && k < timeout) begin
                @(negedge ifclk);
                k = k + 1;
            end
            if (k == timeout) $fatal(1, "Frame drain timeout");
            repeat (40) @(negedge ifclk);
        end
    endtask

    task automatic reset_device;
        begin
            @(negedge ifclk);
            reset_n = 0; cam_valid = 0; cam_vsync = 0; force_full = 0;
            random_stalls = 0;
            repeat (10) @(negedge cam_clk);
            reset_n = 1;
            repeat (16) @(negedge cam_clk);
            tb_frame_index = 0;
            first_boundary = 1;
        end
    endtask

    initial begin
        if (!$value$plusargs("OUT_DIR=%s", out_dir)) out_dir = ".";
        bin_fd = $fopen({out_dir, "/wire.bin"}, "wb");
        meta_fd = $fopen({out_dir, "/meta.txt"}, "w");
        if (!bin_fd || !meta_fd) $fatal(1, "Cannot open output files");
        reset_device();

        case (SCENARIO)
            0: begin
                frame(8'h11, FRAME_PIXELS, 1, 4096);
                frame(8'h12, FRAME_PIXELS, 1, 4096);
                frame(8'h13, FRAME_PIXELS, 1, 4096);
                frame(8'h14, FRAME_PIXELS, 1, 4096);
                wait_frames(4, 200000);
                if (frames_sent < 3) $fatal(1, "Too few clean frames");
                if (frames_aborted != 0) $fatal(1, "Clean stream aborted frames");
            end
            1: begin
                frame(8'h21, FRAME_PIXELS, 1, 4096);
                frame(8'h22, FRAME_PIXELS, 1, 4096);
                frame(8'h23, FRAME_PIXELS, 1, 4096);
                frame(8'h24, FRAME_PIXELS, 1, 4096);
                frame(8'h25, FRAME_PIXELS, 1, 4096);
                frame(8'h26, FRAME_PIXELS, 1, 4096);
                wait_frames(3, 200000);
                if (frames_sent < 3) $fatal(1, "Decimation lost all frames");
                if (frames_dropped < 3) $fatal(1, "Decimation did not drop frames");
            end
            2: begin
                // Fill the small FIFO while USB is held full.
                if (first_boundary) begin
                    boundary();
                    first_boundary = 0;
                end
                blank(96);
                pixels(8'h31, 100);
                force_full = 1;
                pixels(8'h31, FRAME_PIXELS - 100);
                blank(12);
                record_frame(8'h31, FRAME_PIXELS, 0);
                boundary();
                repeat (30) @(negedge cam_clk);
                force_full = 0;
                // BACKOFF_FRAMES=2: these two source frames are deliberately
                // dropped, then the stream must recover.
                frame(8'h2a, 0, 0, 256);
                frame(8'h2b, 0, 0, 256);
                frame(8'h32, FRAME_PIXELS, 1, 256);
                frame(8'h33, FRAME_PIXELS, 1, 256);
                wait_frames(3, 400000);
                if (frames_aborted < 1) $fatal(1, "Overflow frame not aborted");
                if (frames_sent < 1) $fatal(1, "No recovery frame after overflow");
            end
            3: begin
                frame(8'h41, FRAME_PIXELS - 5, 0, 4096);
                frame(8'h42, FRAME_PIXELS + 5, 0, 4096);
                frame(8'h43, FRAME_PIXELS, 1, 4096);
                wait_frames(3, 400000);
                if (frames_aborted < 2) $fatal(1, "Malformed frames not aborted");
                if (frames_sent < 1) $fatal(1, "No recovery after malformed frames");
            end
            4: begin
                // 242x1 -> 24 + 484 + 4 = 512 bytes, packet aligned.
                frame(8'h51, FRAME_PIXELS, 1, 4096);
                frame(8'h52, FRAME_PIXELS, 1, 4096);
                wait_frames(2, 200000);
                if (frames_sent < 1) $fatal(1, "Aligned frame not sent");
            end
            6: begin
                // One real-size OV5640 frame, 5/8 decimation to 640x375.
                frame(8'h71, FRAME_PIXELS, 1, 4096);
                wait_frames(1, 20000000);
                if (frames_sent < 1) $fatal(1, "Full-size frame not sent");
            end
            5: begin
                frame(8'h61, FRAME_PIXELS, 1, 4096);
                wait_cam_active(400000);
                blank(96);
                pixels(8'h62, FRAME_PIXELS / 2);
                @(negedge cam_clk);
                // The partial pre-reset frame is deliberately discarded from
                // the crosscheck stream; reset starts a new protocol session.
                $fclose(bin_fd);
                $fdisplay(meta_fd, "RESET");
                reset_n = 0;
                repeat (10) @(negedge cam_clk);
                reset_n = 1;
                repeat (20) @(negedge cam_clk);
                bin_fd = $fopen({out_dir, "/wire.bin"}, "wb");
                if (!bin_fd) $fatal(1, "Cannot reopen post-reset wire file");
                wire_words = 0;
                frame_words = 0;
                wire_frames = 0;
                pktend_count = 0;
                stall_cycles = 0;
                tb_frame_index = 0;
                first_boundary = 1;
                frame(8'h63, FRAME_PIXELS, 1, 4096);
                frame(8'h64, FRAME_PIXELS, 1, 4096);
                wait_frames(2, 400000);
                if (frames_sent < 1) $fatal(1, "No frame after reset");
            end
            default: $fatal(1, "Unknown SCENARIO");
        endcase

        repeat (200) @(negedge ifclk);
        if (fault) $fatal(1, "DUT asserted fault");
        if (wire_frames != (frames_sent + frames_aborted))
            $fatal(1, "Wire frames %0d != counters %0d",
                  wire_frames, frames_sent + frames_aborted);
        if (SHORT_PACKET && pktend_count != wire_frames)
            $fatal(1, "PKTEND count %0d != frames %0d",
                  pktend_count, wire_frames);
        if (!SHORT_PACKET && pktend_count != 0)
            $fatal(1, "Packet-aligned stream emitted PKTEND");
        if (SHORT_PACKET && SCENARIO != 5 && pktend_stall_cycles < 7)
            $fatal(1, "PKTEND stall coverage missing");
        $display("PASS scenario=%0d frames=%0d good=%0d bad=%0d dropped=%0d malformed=%0d bytes=%0d pktend=%0d stalls=%0d pktend_stalls=%0d",
                 SCENARIO, wire_frames, frames_sent, frames_aborted,
                 frames_dropped, frames_malformed, wire_words * 2,
                 pktend_count, stall_cycles, pktend_stall_cycles);
        $fclose(bin_fd);
        $fclose(meta_fd);
        $finish;
    end

    initial begin
        #200000000;
        $fatal(1, "Global watchdog timeout");
    end
endmodule
