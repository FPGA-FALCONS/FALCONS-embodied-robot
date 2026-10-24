`timescale 1ns/1ps
// Whole-frame, loss-tolerant camera -> FX2 EP6 transport.
// Firmware contract: external synchronous IFCLK, 16-bit AUTO-IN, EP6 address
// 2'b10, FLAGC active high means writable, 512-byte USB high-speed packets.
// cam_valid denotes one RGB565 pixel. VSYNC rising closes the previous frame;
// no pixel is accepted on that boundary. A common reset must reach both clocks.
module camera_fx2_stream #(
    parameter integer SOURCE_WIDTH = 1024,
    parameter integer SOURCE_HEIGHT = 600,
    parameter integer DOWNSAMPLE = 2
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
    // Debug counters: sent is IFCLK-domain; dropped/malformed are cam-domain.
    // Snapshot them before crossing domains in a production status interface.
    output reg  [31:0] frames_sent,
    output reg  [31:0] frames_dropped,
    output reg  [31:0] frames_malformed
);
    localparam integer OUT_WIDTH = SOURCE_WIDTH / DOWNSAMPLE;
    localparam integer OUT_HEIGHT = SOURCE_HEIGHT / DOWNSAMPLE;
    localparam integer OUT_PIXELS = OUT_WIDTH * OUT_HEIGHT;
    localparam integer RAM_AW = (OUT_PIXELS > 1) ? $clog2(OUT_PIXELS) : 1;
    localparam [31:0] SOURCE_PIXELS = SOURCE_WIDTH * SOURCE_HEIGHT;
    localparam [31:0] LAST_X = SOURCE_WIDTH - 1;
    localparam [31:0] LAST_PIXEL = OUT_PIXELS - 1;
    localparam [31:0] PAYLOAD_BYTES = OUT_PIXELS * 2;
    localparam [15:0] WIDTH_FIELD = OUT_WIDTH[15:0];
    localparam [15:0] HEIGHT_FIELD = OUT_HEIGHT[15:0];
    localparam SHORT_PACKET = ((24 + OUT_PIXELS * 2) % 512) != 0;

    // Asynchronous assertion, synchronous release separately in each domain.
    (* ASYNC_REG = "TRUE" *) reg cam_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg cam_reset_n;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_n;
    always @(posedge cam_clk or negedge reset_n) begin
        if (!reset_n) begin
            cam_reset_meta <= 1'b0;
            cam_reset_n <= 1'b0;
        end else begin
            cam_reset_meta <= 1'b1;
            cam_reset_n <= cam_reset_meta;
        end
    end
    always @(posedge ifclk or negedge reset_n) begin
        if (!reset_n) begin
            usb_reset_meta <= 1'b0;
            usb_reset_n <= 1'b0;
        end else begin
            usb_reset_meta <= 1'b1;
            usb_reset_n <= usb_reset_meta;
        end
    end

    reg req_toggle;
    reg ack_toggle;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_sync;
    always @(posedge cam_clk or negedge cam_reset_n)
        if (!cam_reset_n) ack_sync <= 2'b00;
        else ack_sync <= {ack_sync[0], ack_toggle};
    always @(posedge ifclk or negedge usb_reset_n)
        if (!usb_reset_n) req_sync <= 2'b00;
        else req_sync <= {req_sync[0], req_toggle};

    // Held bundled data: sequence and RAM cannot change from request through
    // acknowledgement. The synchronised toggle is the sole ownership transfer.
    reg [31:0] captured_sequence;
    reg [31:0] next_sequence;
    reg capture_active;
    reg vsync_d;
    reg [31:0] source_count;
    reg [31:0] pixel_x;
    reg [31:0] pixel_y;
    reg [31:0] write_count;
    wire cam_boundary = cam_vsync && !vsync_d;
    wire buffer_owned = (req_toggle != ack_sync[1]);
    wire frame_complete = capture_active && source_count == SOURCE_PIXELS;
    assign capture_busy = buffer_owned || capture_active;

    // One write clock / one synchronous read clock, no RAM reset or clear.
    // Default 512x300x16 buffer consumes 2,457,600 RAM bits.
    (* ram_style = "block" *) reg [15:0] frame_ram [0:OUT_PIXELS-1];
    wire cam_write_enable = cam_reset_n && capture_active && !buffer_owned &&
        !cam_boundary && cam_valid && source_count < SOURCE_PIXELS &&
        (pixel_x % DOWNSAMPLE == 0) && (pixel_y % DOWNSAMPLE == 0);
    always @(posedge cam_clk)
        if (cam_write_enable)
            frame_ram[write_count[RAM_AW-1:0]] <= cam_rgb565;

    always @(posedge cam_clk or negedge cam_reset_n) begin
        if (!cam_reset_n) begin
            req_toggle <= 1'b0;
            captured_sequence <= 32'd0;
            next_sequence <= 32'd0;
            capture_active <= 1'b0;
            vsync_d <= 1'b0;
            source_count <= 32'd0;
            pixel_x <= 32'd0;
            pixel_y <= 32'd0;
            write_count <= 32'd0;
            frames_dropped <= 32'd0;
            frames_malformed <= 32'd0;
        end else begin
            vsync_d <= cam_vsync;
            if (cam_boundary) begin
                source_count <= 32'd0;
                pixel_x <= 32'd0;
                pixel_y <= 32'd0;
                write_count <= 32'd0;
                if (frame_complete) begin
                    req_toggle <= ~req_toggle;
                    captured_sequence <= next_sequence;
                    next_sequence <= next_sequence + 32'd1;
                    capture_active <= 1'b0;
                    frames_dropped <= frames_dropped + 32'd1;
                end else if (buffer_owned) begin
                    capture_active <= 1'b0;
                    frames_dropped <= frames_dropped + 32'd1;
                end else begin
                    if (capture_active)
                        frames_malformed <= frames_malformed + 32'd1;
                    capture_active <= 1'b1;
                end
            end else if (capture_active && cam_valid) begin
                // Saturate one beyond expected so extra pixels invalidate it.
                if (source_count <= SOURCE_PIXELS)
                    source_count <= source_count + 32'd1;
                if (source_count < SOURCE_PIXELS) begin
                    if (pixel_x == LAST_X) begin
                        pixel_x <= 32'd0;
                        pixel_y <= pixel_y + 32'd1;
                    end else pixel_x <= pixel_x + 32'd1;
                    if (cam_write_enable) write_count <= write_count + 32'd1;
                end
            end
        end
    end

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

    reg [31:0] usb_sequence;
    reg [31:0] final_crc;
    function automatic [15:0] header_word(input [3:0] index);
        begin
            case (index)
                4'd0: header_word = 16'h4946; // 'F', 'I'
                4'd1: header_word = 16'h474d; // 'M', 'G'
                4'd2: header_word = 16'h0101; // version, RGB565LE
                4'd3: header_word = 16'h0018;
                4'd4: header_word = WIDTH_FIELD;
                4'd5: header_word = HEIGHT_FIELD;
                4'd6: header_word = usb_sequence[15:0];
                4'd7: header_word = usb_sequence[31:16];
                4'd8: header_word = PAYLOAD_BYTES[15:0];
                4'd9: header_word = PAYLOAD_BYTES[31:16];
                4'd10: header_word = final_crc[15:0];
                4'd11: header_word = final_crc[31:16];
                default: header_word = 16'd0;
            endcase
        end
    endfunction

    localparam [3:0] IDLE = 4'd0, CRC_HEADER = 4'd1, CRC_REQ = 4'd2,
        CRC_WAIT = 4'd3, CRC_DATA = 4'd4, TX_HEADER_LOAD = 4'd5,
        TX_HEADER_SEND = 4'd6, TX_REQ = 4'd7, TX_WAIT = 4'd8,
        TX_LOAD = 4'd9, TX_SEND = 4'd10, PACKET_GUARD = 4'd11,
        PACKET_END = 4'd12, COMPLETE = 4'd13;
    reg [3:0] state;
    reg [3:0] header_index;
    reg [31:0] read_index;
    reg [31:0] crc_acc;
    reg [15:0] ram_q;
    reg [15:0] tx_word;
    wire ram_read_enable = state == CRC_REQ || state == TX_REQ;
    always @(posedge ifclk)
        if (ram_read_enable)
            ram_q <= frame_ram[read_index[RAM_AW-1:0]];

    assign fx2_fd = tx_word;
    assign fx2_faddr = 2'b10;
    assign fx2_sloe_n = 1'b1;
    assign fx2_slrd_n = 1'b1;
    // Data/address are prepared in LOAD states and stay stable during stalls.
    // FX2 and FLAGC share IFCLK; no asynchronous FLAG synchronizer is inserted.
    assign fx2_slwr_n = !(usb_reset_n && fx2_flagc &&
        (state == TX_HEADER_SEND || state == TX_SEND));
    assign fx2_pktend_n = !(usb_reset_n && fx2_flagc && state == PACKET_END);

    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            ack_toggle <= 1'b0;
            usb_sequence <= 32'd0;
            final_crc <= 32'd0;
            state <= IDLE;
            header_index <= 4'd0;
            read_index <= 32'd0;
            crc_acc <= 32'hffffffff;
            tx_word <= 16'd0;
            frames_sent <= 32'd0;
        end else begin
            case (state)
                IDLE: if (req_sync[1] != ack_toggle) begin
                    usb_sequence <= captured_sequence;
                    crc_acc <= 32'hffffffff;
                    header_index <= 4'd0;
                    read_index <= 32'd0;
                    state <= CRC_HEADER;
                end
                CRC_HEADER: begin
                    crc_acc <= crc_word(crc_acc, header_word(header_index));
                    if (header_index == 4'd9) state <= CRC_REQ;
                    else header_index <= header_index + 4'd1;
                end
                CRC_REQ: state <= CRC_WAIT;
                CRC_WAIT: state <= CRC_DATA;
                CRC_DATA: begin
                    crc_acc <= crc_word(crc_acc, ram_q);
                    if (read_index == LAST_PIXEL) begin
                        final_crc <= ~crc_word(crc_acc, ram_q);
                        header_index <= 4'd0;
                        read_index <= 32'd0;
                        state <= TX_HEADER_LOAD;
                    end else begin
                        read_index <= read_index + 32'd1;
                        state <= CRC_REQ;
                    end
                end
                TX_HEADER_LOAD: begin
                    tx_word <= header_word(header_index);
                    state <= TX_HEADER_SEND;
                end
                TX_HEADER_SEND: if (fx2_flagc) begin
                    if (header_index == 4'd11) state <= TX_REQ;
                    else begin
                        header_index <= header_index + 4'd1;
                        state <= TX_HEADER_LOAD;
                    end
                end
                TX_REQ: state <= TX_WAIT;
                TX_WAIT: state <= TX_LOAD;
                TX_LOAD: begin
                    tx_word <= ram_q;
                    state <= TX_SEND;
                end
                TX_SEND: if (fx2_flagc) begin
                    if (read_index == LAST_PIXEL) state <= PACKET_GUARD;
                    else begin
                        read_index <= read_index + 32'd1;
                        state <= TX_REQ;
                    end
                end
                PACKET_GUARD: begin
                    if (!SHORT_PACKET) state <= COMPLETE;
                    else if (fx2_flagc) state <= PACKET_END;
                end
                PACKET_END: if (fx2_flagc) state <= COMPLETE;
                COMPLETE: begin
                    ack_toggle <= req_sync[1];
                    frames_sent <= frames_sent + 32'd1;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end

    // Elaboration guards intentionally fail before any invalid RAM access.
    initial begin
        if (SOURCE_WIDTH < 1 || SOURCE_HEIGHT < 1 || DOWNSAMPLE < 1)
            $fatal(1, "camera_fx2_stream dimensions/downsample must be positive");
        if (SOURCE_WIDTH % DOWNSAMPLE != 0 || SOURCE_HEIGHT % DOWNSAMPLE != 0)
            $fatal(1, "camera_fx2_stream requires exactly divisible dimensions");
        if (OUT_WIDTH > 65535 || OUT_HEIGHT > 65535)
            $fatal(1, "camera_fx2_stream FIMG width/height exceed 16 bits");
    end
endmodule
