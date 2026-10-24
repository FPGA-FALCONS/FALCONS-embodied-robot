`timescale 1ns/1ps
// Direct, whole-resolution OV5640 RGB565 -> FX2 EP6 transport.
//
// This is deliberately NOT a frame-buffer design.  Pixels are written into a
// small dual-clock FIFO as they arrive and are drained to the FX2 slave FIFO.
// There is no way to backpressure a DVP camera, so a frame that outruns USB is
// terminated with a deliberately invalid CRC and the receiver discards it.
//
// Wire protocol (FIMG v2):
//   24-byte header, RGB565LE payload, 4-byte trailing CRC32.
//   Header field 10 is CRC32 over the first 20 header bytes.
//   Trailer is zlib/IEEE CRC32 over the first 20 header bytes + payload.
//   A bad/truncated frame still transmits the declared payload length (zero
//   padding) and a trailer that does not match, preserving byte framing.
//
// FX2 contract is the same as camera_fx2_stream: external 48 MHz IFCLK,
// 16-bit AUTO-IN EP6 at FIFO address 2'b10, FLAGC active high = writable.
//
// No full-frame storage, no DDR3 access, no camera reconfiguration, and no
// firmware/EEPROM manipulation are performed by this module.
module camera_fx2_direct #(
    parameter integer SOURCE_WIDTH       = 1024,
    parameter integer SOURCE_HEIGHT      = 600,
    parameter integer FRAME_DECIMATION   = 2,
    parameter integer BACKOFF_FRAMES     = 3,
    parameter integer START_WAIT_CYCLES  = 1024,
    parameter integer FIFO_DEPTH         = 4096,
    parameter integer DECIM_NUM          = 1,
    parameter integer DECIM_DEN          = 1
) (
    input  wire        cam_clk,
    input  wire        ifclk,
    input  wire        reset_n,
    input  wire        cam_vsync,
    input  wire        cam_valid,
    input  wire [15:0] cam_rgb565,
    input  wire        fx2_flagc,
    output wire [15:0] fx2_fd,
    output wire [1:0]  fx2_faddr,
    output wire        fx2_slwr_n,
    output wire        fx2_sloe_n,
    output wire        fx2_slrd_n,
    output wire        fx2_pktend_n,
    output wire        capture_busy,
    output reg  [31:0] frames_sent,
    output reg  [31:0] frames_dropped,
    output reg  [31:0] frames_malformed,
    output reg  [31:0] frames_aborted,
    output reg         fault
);
    // Keep NUM of every DEN source columns/rows.  The default 1/1 forwards
    // every pixel; the board build uses 5/8 to make the DVP instantaneous rate
    // compatible with the FX2 drain rate without a full-frame buffer.
    localparam integer SOURCE_PIXELS = SOURCE_WIDTH * SOURCE_HEIGHT;
    localparam integer OUT_WIDTH     = (SOURCE_WIDTH / DECIM_DEN) * DECIM_NUM;
    localparam integer OUT_HEIGHT    = (SOURCE_HEIGHT / DECIM_DEN) * DECIM_NUM;
    localparam integer FRAME_PIXELS  = OUT_WIDTH * OUT_HEIGHT;
    localparam integer HEADER_WORDS  = 12;
    localparam [31:0]  PAYLOAD_BYTES = FRAME_PIXELS * 2;
    localparam [31:0]  TOTAL_BYTES   = 24 + PAYLOAD_BYTES + 4;
    localparam         SHORT_PACKET  = (TOTAL_BYTES % 512) != 0;
    localparam [15:0]  WIDTH_FIELD   = OUT_WIDTH[15:0];
    localparam [15:0]  HEIGHT_FIELD  = OUT_HEIGHT[15:0];

    // Asynchronous assertion, synchronous release per clock domain.
    (* ASYNC_REG = "TRUE" *) reg cam_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg cam_reset_n;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_n;
    always @(posedge cam_clk or negedge reset_n) begin
        if (!reset_n) begin
            cam_reset_meta <= 1'b0;
            cam_reset_n    <= 1'b0;
        end else begin
            cam_reset_meta <= 1'b1;
            cam_reset_n    <= cam_reset_meta;
        end
    end
    always @(posedge ifclk or negedge reset_n) begin
        if (!reset_n) begin
            usb_reset_meta <= 1'b0;
            usb_reset_n    <= 1'b0;
        end else begin
            usb_reset_meta <= 1'b1;
            usb_reset_n    <= usb_reset_meta;
        end
    end

    // ------------------------------------------------------------------
    // Shared stream FIFOs
    // ------------------------------------------------------------------
    wire        data_wr_en;
    wire [15:0] data_wr_data;
    wire        data_wr_full;
    wire [31:0] data_wr_free;
    wire        data_rd_en;
    wire [15:0] data_rd_data;
    wire        data_rd_valid;
    wire        data_rd_empty;

    wire        status_wr_en;
    wire        status_wr_data;
    wire        status_wr_full;
    wire [31:0] status_wr_free;
    wire        status_rd_en;
    wire        status_rd_data;
    wire        status_rd_valid;
    wire        status_rd_empty;

    acg_async_fifo #(.WIDTH(16), .DEPTH(FIFO_DEPTH)) u_data_fifo (
        .wr_clk   (cam_clk),
        .wr_rst_n (cam_reset_n),
        .wr_en    (data_wr_en),
        .wr_data  (data_wr_data),
        .wr_full  (data_wr_full),
        .wr_free  (data_wr_free),
        .rd_clk   (ifclk),
        .rd_rst_n (usb_reset_n),
        .rd_en    (data_rd_en),
        .rd_data  (data_rd_data),
        .rd_valid (data_rd_valid),
        .rd_empty (data_rd_empty)
    );

    // One entry per admitted frame: 1 = complete, 0 = malformed/truncated.
    acg_async_fifo #(.WIDTH(1), .DEPTH(16)) u_status_fifo (
        .wr_clk   (cam_clk),
        .wr_rst_n (cam_reset_n),
        .wr_en    (status_wr_en),
        .wr_data  (status_wr_data),
        .wr_full  (status_wr_full),
        .wr_free  (status_wr_free),
        .rd_clk   (ifclk),
        .rd_rst_n (usb_reset_n),
        .rd_en    (status_rd_en),
        .rd_data  (status_rd_data),
        .rd_valid (status_rd_valid),
        .rd_empty (status_rd_empty)
    );

    function automatic [31:0] crc_byte(input [31:0] crc, input [7:0] value);
        reg [31:0] c;
        integer bit_index;
        begin
            c = crc ^ {24'd0, value};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                c = c[0] ? ((c >> 1) ^ 32'hedb88320) : (c >> 1);
            crc_byte = c;
        end
    endfunction

    function automatic [31:0] crc_word(input [31:0] crc, input [15:0] value);
        crc_word = crc_byte(crc_byte(crc, value[7:0]), value[15:8]);
    endfunction

    function automatic [15:0] header_word(input [3:0] index, input [31:0] seq);
        begin
            case (index)
                4'd0: header_word = 16'h4946; // 'F','I'
                4'd1: header_word = 16'h474d; // 'M','G'
                4'd2: header_word = 16'h0102; // version 2, RGB565LE
                4'd3: header_word = 16'h0018; // 24-byte header
                4'd4: header_word = WIDTH_FIELD;
                4'd5: header_word = HEIGHT_FIELD;
                4'd6: header_word = seq[15:0];
                4'd7: header_word = seq[31:16];
                4'd8: header_word = PAYLOAD_BYTES[15:0];
                4'd9: header_word = PAYLOAD_BYTES[31:16];
                default: header_word = 16'd0;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Capture domain: admit a frame, emit header, then stream pixels.
    // ------------------------------------------------------------------
    localparam [1:0] CAM_IDLE    = 2'd0;
    localparam [1:0] CAM_WAIT_TX = 2'd1;
    localparam [1:0] CAM_HEADER  = 2'd2;
    localparam [1:0] CAM_ACTIVE  = 2'd3;

    reg [1:0]  cam_state;
    reg        vsync_d;
    reg        capture_started;
    reg        overflow;
    reg [31:0] pixel_count;      // emitted (decimated) pixels
    reg [31:0] source_count;     // all source pixels in this frame
    reg [31:0] source_x;         // x position in the source raster
    reg [31:0] x_phase;
    reg [31:0] y_phase;
    reg [3:0]  header_index;
    reg [31:0] header_seq;
    reg [31:0] frame_index;
    reg [31:0] decim_counter;
    reg [31:0] backoff_counter;
    reg [31:0] wait_count;
    reg        data_wr_en_r;
    reg [15:0] data_wr_data_r;
    reg        status_wr_en_r;
    reg        status_wr_data_r;

    wire cam_boundary = cam_vsync && !vsync_d;
    wire keep_pixel = (x_phase < DECIM_NUM) && (y_phase < DECIM_NUM);
    wire frame_good_now = capture_started && !overflow &&
        (source_count == SOURCE_PIXELS) && (pixel_count == FRAME_PIXELS);
    wire [31:0] backoff_after = (capture_started && !frame_good_now) ?
        BACKOFF_FRAMES : ((backoff_counter != 32'd0) ?
        (backoff_counter - 32'd1) : 32'd0);
    wire decim_now = (decim_counter == 32'd0);
    wire [31:0] decim_after = (decim_counter >= (FRAME_DECIMATION - 1)) ?
        32'd0 : (decim_counter + 32'd1);

    // tx_idle_if is registered in the IFCLK domain (assigned below, after the
    // USB state declaration), then synchronized into the capture domain.
    reg tx_idle_if;
    (* ASYNC_REG = "TRUE" *) reg tx_idle_s1;
    (* ASYNC_REG = "TRUE" *) reg tx_idle_s2;
    always @(posedge cam_clk or negedge cam_reset_n) begin
        if (!cam_reset_n) begin
            tx_idle_s1 <= 1'b0;
            tx_idle_s2 <= 1'b0;
        end else begin
            tx_idle_s1 <= tx_idle_if;
            tx_idle_s2 <= tx_idle_s1;
        end
    end

    assign data_wr_en   = data_wr_en_r;
    assign data_wr_data = data_wr_data_r;
    assign status_wr_en = status_wr_en_r;
    assign status_wr_data = status_wr_data_r;
    assign capture_busy = (cam_state != CAM_IDLE);

    always @(posedge cam_clk or negedge cam_reset_n) begin
        if (!cam_reset_n) begin
            cam_state       <= CAM_IDLE;
            vsync_d         <= 1'b0;
            capture_started <= 1'b0;
            overflow        <= 1'b0;
            pixel_count     <= 32'd0;
            source_count    <= 32'd0;
            source_x        <= 32'd0;
            x_phase         <= 32'd0;
            y_phase         <= 32'd0;
            header_index    <= 4'd0;
            header_seq      <= 32'd0;
            frame_index     <= 32'd0;
            decim_counter   <= 32'd0;
            backoff_counter <= 32'd0;
            wait_count      <= 32'd0;
            data_wr_en_r    <= 1'b0;
            data_wr_data_r  <= 16'd0;
            status_wr_en_r  <= 1'b0;
            status_wr_data_r<= 1'b0;
            frames_dropped  <= 32'd0;
            frames_malformed<= 32'd0;
        end else begin
            vsync_d      <= cam_vsync;
            data_wr_en_r <= 1'b0;
            status_wr_en_r <= 1'b0;

            if (cam_boundary) begin
                // Close the frame that just ended.  Exactly one status word is
                // queued for every admitted frame.
                if (capture_started) begin
                    if (!status_wr_full) begin
                        status_wr_en_r   <= 1'b1;
                        status_wr_data_r <= frame_good_now;
                    end
                    if (!frame_good_now)
                        frames_malformed <= frames_malformed + 32'd1;
                end

                backoff_counter <= backoff_after;
                decim_counter   <= decim_after;
                frame_index     <= frame_index + 32'd1;
                pixel_count     <= 32'd0;
                source_count    <= 32'd0;
                source_x        <= 32'd0;
                x_phase         <= 32'd0;
                y_phase         <= 32'd0;
                overflow        <= 1'b0;
                capture_started <= 1'b0;
                header_index    <= 4'd0;
                wait_count      <= 32'd0;

                if (decim_now && (backoff_after == 32'd0) &&
                    (cam_state != CAM_HEADER) &&
                    (cam_state != CAM_WAIT_TX) &&
                    (data_wr_free >= HEADER_WORDS) &&
                    (status_wr_free >= 32'd1) && !status_wr_full) begin
                    header_seq <= frame_index;
                    cam_state  <= CAM_WAIT_TX;
                end else begin
                    cam_state <= CAM_IDLE;
                    frames_dropped <= frames_dropped + 32'd1;
                end
            end else begin
                case (cam_state)
                    CAM_IDLE: begin
                        // Wait for the next VSYNC.
                    end

                    CAM_WAIT_TX: begin
                        // The previous frame may still be draining.  Start the
                        // header only when the transmitter has finished it;
                        // vertical blanking is much longer than this wait.
                        if (cam_valid)
                            overflow <= 1'b1;
                        if (tx_idle_s2 && (data_wr_free >= HEADER_WORDS)) begin
                            cam_state       <= CAM_HEADER;
                            header_index    <= 4'd0;
                            capture_started <= 1'b1;
                        end else if (wait_count >= (START_WAIT_CYCLES - 1)) begin
                            cam_state <= CAM_IDLE;
                            frames_dropped <= frames_dropped + 32'd1;
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    CAM_HEADER: begin
                        // Header writes are 12 consecutive cam_clk cycles.
                        // Real VSYNC/vertical blanking is orders of magnitude
                        // longer; a repeated boundary here is outside contract.
                        if (!data_wr_full) begin
                            data_wr_en_r   <= 1'b1;
                            data_wr_data_r <= header_word(header_index, header_seq);
                        end else begin
                            overflow <= 1'b1;
                        end
                        if (header_index == 4'd11) begin
                            cam_state <= CAM_ACTIVE;
                            header_index <= 4'd0;
                        end else begin
                            header_index <= header_index + 4'd1;
                        end
                    end

                    CAM_ACTIVE: begin
                        if (cam_valid) begin
                            if (source_count < SOURCE_PIXELS) begin
                                if (keep_pixel) begin
                                    if ((pixel_count < FRAME_PIXELS) &&
                                        !overflow && !data_wr_full) begin
                                        data_wr_en_r   <= 1'b1;
                                        data_wr_data_r <= cam_rgb565;
                                    end else begin
                                        overflow <= 1'b1;
                                    end
                                    pixel_count <= pixel_count + 32'd1;
                                end
                            end else begin
                                // Extra source pixels invalidate the raster.
                                overflow <= 1'b1;
                            end
                            source_count <= source_count + 32'd1;
                            if (x_phase == (DECIM_DEN - 1))
                                x_phase <= 32'd0;
                            else
                                x_phase <= x_phase + 32'd1;
                            if (source_x == (SOURCE_WIDTH - 1)) begin
                                source_x <= 32'd0;
                                if (y_phase == (DECIM_DEN - 1))
                                    y_phase <= 32'd0;
                                else
                                    y_phase <= y_phase + 32'd1;
                            end else begin
                                source_x <= source_x + 32'd1;
                            end
                        end
                    end

                    default: cam_state <= CAM_IDLE;
                endcase
            end
        end
    end

    // ------------------------------------------------------------------
    // USB/IFCLK domain: one frame at a time, header -> payload -> trailer.
    // ------------------------------------------------------------------
    localparam [4:0] TX_IDLE        = 5'd0;
    localparam [4:0] HDR_REQ        = 5'd1;
    localparam [4:0] HDR_WAIT       = 5'd2;
    localparam [4:0] TX_HDR_LOAD    = 5'd3;
    localparam [4:0] TX_HDR_SEND    = 5'd4;
    localparam [4:0] PAY_LOAD       = 5'd5;
    localparam [4:0] PAY_WAIT       = 5'd6;
    localparam [4:0] PAY_SEND       = 5'd7;
    localparam [4:0] WAIT_STATUS    = 5'd8;
    localparam [4:0] STATUS_WAIT    = 5'd9;
    localparam [4:0] PAD_LOAD       = 5'd10;
    localparam [4:0] PAD_SEND       = 5'd11;
    localparam [4:0] TRAILER_LOAD   = 5'd12;
    localparam [4:0] TRAILER_SEND   = 5'd13;
    localparam [4:0] PKT_GUARD      = 5'd14;
    localparam [4:0] PKT_END        = 5'd15;
    localparam [4:0] COMPLETE       = 5'd16;
    localparam [4:0] FAULT          = 5'd17;

    reg [4:0]  state;
    reg        slwr_window;
    reg        pktend_window;

    // Registered level, not a decode glitch: capture may start a new header
    // only after the previous trailer has completed.
    always @(posedge ifclk or negedge usb_reset_n)
        if (!usb_reset_n) tx_idle_if <= 1'b1;
        else              tx_idle_if <= (state == TX_IDLE);

    reg [3:0]  hdr_read_index;
    reg [15:0] header_words [0:11];
    reg [31:0] header_crc_acc;
    reg [31:0] payload_crc;
    reg [31:0] trailer_crc;
    reg [31:0] payload_count;
    reg [31:0] pad_remaining;
    reg [3:0]  tx_index;
    reg [15:0] tx_word;
    reg        frame_bad;
    reg [5:0]  settle_count;
    reg [2:0]  status_retry;
    reg        data_rd_en_r;
    reg        status_rd_en_r;

    // Small same-clock payload prefetch FIFO.  The async FIFO has a one-cycle
    // registered read; prefetching hides that latency so the FX2 send path can
    // consume one word every two IFCLK cycles instead of every four.
    reg [15:0] pre_mem [0:7];
    reg [2:0]  pre_wr_ptr;
    reg [2:0]  pre_rd_ptr;
    reg [3:0]  pre_count;
    reg        pre_started;
    reg        pre_clear;
    reg        pre_rd_en;
    reg        pre_wr_en;
    reg [15:0] pre_wr_data;
    wire pre_empty = (pre_count == 4'd0);
    wire pre_full  = (pre_count == 4'd8);
    wire pre_room  = (pre_count < 4'd6);
    wire [15:0] pre_rd_data = pre_mem[pre_rd_ptr];
    wire pre_wr_ok = pre_wr_en && !pre_full;
    wire pre_rd_ok = pre_rd_en && !pre_empty;
    // Keep the async FIFO read stream asserted while there is room.  The FIFO
    // returns one word per IFCLK; the small pre FIFO absorbs the two-cycle
    // send cadence and leaves headroom for the registered read.
    wire fill_active = pre_started && pre_room && !data_rd_empty;
    wire fill_quiescent = !fill_active && !data_rd_valid && !pre_wr_en;

    assign data_rd_en   = data_rd_en_r | fill_active;
    assign status_rd_en = status_rd_en_r;

    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            pre_wr_ptr <= 3'd0;
            pre_rd_ptr <= 3'd0;
            pre_count  <= 4'd0;
        end else if (pre_clear) begin
            pre_wr_ptr <= 3'd0;
            pre_rd_ptr <= 3'd0;
            pre_count  <= 4'd0;
        end else begin
            if (pre_wr_ok) begin
                pre_mem[pre_wr_ptr] <= pre_wr_data;
                pre_wr_ptr <= pre_wr_ptr + 3'd1;
            end
            if (pre_rd_ok)
                pre_rd_ptr <= pre_rd_ptr + 3'd1;
            case ({pre_wr_ok, pre_rd_ok})
                2'b10: pre_count <= pre_count + 4'd1;
                2'b01: pre_count <= pre_count - 4'd1;
                default: pre_count <= pre_count;
            endcase
        end
    end

    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            pre_wr_en   <= 1'b0;
            pre_wr_data <= 16'd0;
        end else begin
            pre_wr_en <= 1'b0;
            if (pre_started && data_rd_valid && !pre_full) begin
                pre_wr_en   <= 1'b1;
                pre_wr_data <= data_rd_data;
            end
        end
    end

    assign fx2_fd       = tx_word;
    assign fx2_faddr    = 2'b10;
    assign fx2_sloe_n   = 1'b1;
    assign fx2_slrd_n   = 1'b1;
    // Data/address are prepared in LOAD states and held across stalls.
    // The write window is registered in the LOAD state, so the pad-to-pad
    // path is only FLAGC -> one AND -> pad.  The data word is stable while
    // the window is open; throughput remains about two IFCLK per word.
    assign fx2_slwr_n   = !(usb_reset_n && fx2_flagc && slwr_window);
    assign fx2_pktend_n = !(usb_reset_n && fx2_flagc && pktend_window);

    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            state          <= TX_IDLE;
            slwr_window    <= 1'b0;
            pktend_window  <= 1'b0;
            hdr_read_index <= 4'd0;
            header_crc_acc <= 32'hffffffff;
            payload_crc    <= 32'hffffffff;
            trailer_crc    <= 32'd0;
            payload_count  <= 32'd0;
            pad_remaining  <= 32'd0;
            tx_index       <= 4'd0;
            tx_word        <= 16'd0;
            frame_bad      <= 1'b0;
            settle_count   <= 6'd0;
            status_retry   <= 3'd0;
            data_rd_en_r   <= 1'b0;
            status_rd_en_r <= 1'b0;
            pre_started    <= 1'b0;
            pre_clear      <= 1'b0;
            pre_rd_en      <= 1'b0;
            frames_sent    <= 32'd0;
            frames_aborted <= 32'd0;
            fault          <= 1'b0;
        end else begin
            data_rd_en_r   <= 1'b0;
            status_rd_en_r <= 1'b0;
            pre_clear      <= 1'b0;
            pre_rd_en      <= 1'b0;
            case (state)
                TX_IDLE: begin
                    if (!data_rd_empty) begin
                        hdr_read_index <= 4'd0;
                        header_crc_acc <= 32'hffffffff;
                        pre_started    <= 1'b0;
                        state          <= HDR_REQ;
                    end
                end

                HDR_REQ: begin
                    data_rd_en_r <= 1'b1;
                    state        <= HDR_WAIT;
                end

                HDR_WAIT: begin
                    if (data_rd_valid) begin
                        if (hdr_read_index == 4'd11) begin
                            // Raw accumulator covers header bytes 0..19.
                            header_words[10] <= ~header_crc_acc[15:0];
                            header_words[11] <= ~header_crc_acc[31:16];
                            payload_crc      <= header_crc_acc;
                            payload_count    <= 32'd0;
                            tx_index         <= 4'd0;
                            pre_started      <= 1'b1;
                            state            <= TX_HDR_LOAD;
                        end else begin
                            header_words[hdr_read_index] <= data_rd_data;
                            if (hdr_read_index < 4'd10)
                                header_crc_acc <= crc_word(header_crc_acc,
                                                           data_rd_data);
                            hdr_read_index <= hdr_read_index + 4'd1;
                            state          <= HDR_REQ;
                        end
                    end else if (data_rd_empty) begin
                        // Header writes may be slower than IFCLK; retry.
                        state <= HDR_REQ;
                    end else begin
                        // Registered FIFO read data appears one IFCLK after
                        // rd_en; stay here until that valid pulse arrives.
                        state <= HDR_WAIT;
                    end
                end

                TX_HDR_LOAD: begin
                    tx_word     <= header_words[tx_index];
                    slwr_window <= 1'b1;
                    state       <= TX_HDR_SEND;
                end

                TX_HDR_SEND: begin
                    if (fx2_flagc) begin
                        slwr_window <= 1'b0;
                        if (tx_index == 4'd11) begin
                            tx_index <= 4'd0;
                            state    <= PAY_LOAD;
                        end else begin
                            tx_index <= tx_index + 4'd1;
                            state    <= TX_HDR_LOAD;
                        end
                    end
                end

                PAY_LOAD: begin
                    if ((payload_count < FRAME_PIXELS) && !pre_empty) begin
                        tx_word       <= pre_rd_data;
                        pre_rd_en     <= 1'b1;
                        slwr_window   <= 1'b1;
                        payload_crc   <= crc_word(payload_crc, pre_rd_data);
                        payload_count <= payload_count + 32'd1;
                        state         <= PAY_SEND;
                    end else if (!status_rd_empty && data_rd_empty &&
                                 !pre_wr_en) begin
                        pre_started <= 1'b0;
                        if (fill_quiescent) begin
                            settle_count <= 6'd63;
                            state        <= WAIT_STATUS;
                        end
                    end
                end

                PAY_WAIT: begin
                    if (data_rd_valid) begin
                        tx_word       <= data_rd_data;
                        payload_crc   <= crc_word(payload_crc, data_rd_data);
                        payload_count <= payload_count + 32'd1;
                        state         <= PAY_SEND;
                    end else if (data_rd_empty) begin
                        // Lost arbitration; retry rather than corrupt order.
                        state <= PAY_LOAD;
                    end else begin
                        state <= PAY_WAIT;
                    end
                end

                PAY_SEND: begin
                    if (fx2_flagc) begin
                        slwr_window <= 1'b0;
                        if (payload_count == FRAME_PIXELS) begin
                            pre_started  <= 1'b0;
                            settle_count <= 6'd63;
                            state        <= WAIT_STATUS;
                        end else begin
                            state <= PAY_LOAD;
                        end
                    end
                end

                WAIT_STATUS: begin
                    if (!status_rd_empty) begin
                        if (settle_count == 6'd0) begin
                            status_rd_en_r <= 1'b1;
                            status_retry   <= 3'd3;
                            state          <= STATUS_WAIT;
                        end else begin
                            settle_count <= settle_count - 6'd1;
                        end
                    end else begin
                        // Status is queued at VSYNC.  Do not advance without it.
                        settle_count <= 6'd63;
                    end
                end

                STATUS_WAIT: begin
                    if (status_rd_valid) begin
                        if ((payload_count == FRAME_PIXELS) &&
                            (status_rd_data == 1'b1)) begin
                            frame_bad <= 1'b0;
                            trailer_crc <= ~payload_crc;
                            tx_index <= 4'd0;
                            state    <= TRAILER_LOAD;
                        end else begin
                            frame_bad <= 1'b1;
                            if (payload_count < FRAME_PIXELS) begin
                                pad_remaining <= FRAME_PIXELS - payload_count;
                                state         <= PAD_LOAD;
                            end else begin
                                trailer_crc <= payload_crc; // deliberately bad
                                tx_index    <= 4'd0;
                                state       <= TRAILER_LOAD;
                            end
                        end
                    end else if (status_retry == 3'd0) begin
                        fault <= 1'b1;
                        state <= FAULT;
                    end else begin
                        // Same registered-read latency as the data FIFO.
                        status_retry <= status_retry - 3'd1;
                        state        <= STATUS_WAIT;
                    end
                end

                PAD_LOAD: begin
                    tx_word     <= 16'd0;
                    slwr_window <= 1'b1;
                    state       <= PAD_SEND;
                end

                PAD_SEND: begin
                    if (fx2_flagc) begin
                        slwr_window <= 1'b0;
                        if (pad_remaining <= 32'd1) begin
                            pad_remaining <= 32'd0;
                            trailer_crc   <= payload_crc; // deliberately bad
                            tx_index      <= 4'd0;
                            state         <= TRAILER_LOAD;
                        end else begin
                            pad_remaining <= pad_remaining - 32'd1;
                            state         <= PAD_LOAD;
                        end
                    end
                end

                TRAILER_LOAD: begin
                    tx_word     <= (tx_index == 4'd0) ? trailer_crc[15:0] :
                                                         trailer_crc[31:16];
                    slwr_window <= 1'b1;
                    state       <= TRAILER_SEND;
                end

                TRAILER_SEND: begin
                    if (fx2_flagc) begin
                        slwr_window <= 1'b0;
                        if (tx_index == 4'd0) begin
                            tx_index <= 4'd1;
                            state    <= TRAILER_LOAD;
                        end else if (SHORT_PACKET) begin
                            state <= PKT_GUARD;
                        end else begin
                            state <= COMPLETE;
                        end
                    end
                end

                PKT_GUARD: begin
                    if (!SHORT_PACKET)
                        state <= COMPLETE;
                    else if (fx2_flagc) begin
                        pktend_window <= 1'b1;
                        state         <= PKT_END;
                    end
                end

                PKT_END: begin
                    if (fx2_flagc) begin
                        pktend_window <= 1'b0;
                        state         <= COMPLETE;
                    end
                end

                COMPLETE: begin
                    if (frame_bad)
                        frames_aborted <= frames_aborted + 32'd1;
                    else
                        frames_sent <= frames_sent + 32'd1;
                    pre_started   <= 1'b0;
                    pre_clear     <= 1'b1;
                    slwr_window   <= 1'b0;
                    pktend_window <= 1'b0;
                    state         <= TX_IDLE;
                end

                default: begin
                    fault <= 1'b1;
                    state <= FAULT;
                end
            endcase
        end
    end

    initial begin
        if (SOURCE_WIDTH < 1 || SOURCE_HEIGHT < 1)
            $fatal(1, "camera_fx2_direct dimensions must be positive");
        if (SOURCE_WIDTH > 65535 || SOURCE_HEIGHT > 65535)
            $fatal(1, "camera_fx2_direct FIMG width/height exceed 16 bits");
        if (FRAME_DECIMATION < 1)
            $fatal(1, "camera_fx2_direct FRAME_DECIMATION must be >= 1");
        if (BACKOFF_FRAMES < 0)
            $fatal(1, "camera_fx2_direct BACKOFF_FRAMES must be >= 0");
        if (START_WAIT_CYCLES < 1)
            $fatal(1, "camera_fx2_direct START_WAIT_CYCLES must be >= 1");
        if (FIFO_DEPTH < HEADER_WORDS)
            $fatal(1, "camera_fx2_direct FIFO_DEPTH must hold one header");
        if (DECIM_DEN < 1 || DECIM_NUM < 1 || DECIM_NUM > DECIM_DEN ||
            DECIM_DEN > 8)
            $fatal(1, "camera_fx2_direct requires 1 <= NUM <= DEN <= 8");
        if ((SOURCE_WIDTH % DECIM_DEN) != 0 ||
            (SOURCE_HEIGHT % DECIM_DEN) != 0)
            $fatal(1, "camera_fx2_direct decimation must divide dimensions");
        if (OUT_WIDTH > 65535 || OUT_HEIGHT > 65535)
            $fatal(1, "camera_fx2_direct output dimensions exceed 16 bits");
    end
endmodule
