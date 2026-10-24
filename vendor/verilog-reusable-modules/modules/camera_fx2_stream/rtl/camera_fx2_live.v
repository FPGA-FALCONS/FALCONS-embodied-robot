`timescale 1ns/1ps
// FIM2 v2 loss-tolerant line transport.  Camera pixels are never backpressured.
// Matching FX2 firmware: external IFCLK, synchronous16-bit EP6 AUTO-IN,
// FLAGC=1 means writable; FIFOADR=2. Each record has a CRC32LE trailer.
// Source supplies exactly SOURCE_WIDTH*SOURCE_HEIGHT valid pixels per frame.
// VSYNC rising closes the old frame and ignores simultaneous cam_valid.
// One blank camera cycle after VSYNC is required to admit the next START.
module camera_fx2_live #(
    parameter integer SOURCE_WIDTH = 1024,
    parameter integer SOURCE_HEIGHT = 600,
    parameter integer LINE_BANKS = 4,
    parameter integer DESC_DEPTH = 16
) (
    input wire cam_clk,
    input wire ifclk,
    input wire reset_n,
    input wire cam_vsync,
    input wire cam_valid,
    input wire [15:0] cam_rgb565,
    input wire fx2_flagc,
    output wire [15:0] fx2_fd,
    output wire [1:0] fx2_faddr,
    output wire fx2_slwr_n,
    output wire fx2_sloe_n,
    output wire fx2_slrd_n,
    output wire fx2_pktend_n,
    // Camera-domain open-frame indication, not an upstream ready signal.
    output wire capture_busy,
    // sent: IFCLK-domain completed END records. Others: camera domain.
    output reg [31:0] frames_sent,
    output reg [31:0] frames_dropped,
    output reg [31:0] frames_malformed
);
    localparam integer BANK_W = (LINE_BANKS > 1) ? $clog2(LINE_BANKS) : 1;
    localparam integer DESC_AW = $clog2(DESC_DEPTH);
    localparam integer PTR_W = DESC_AW + 1;
    localparam integer RAM_WORDS = SOURCE_WIDTH * LINE_BANKS;
    localparam integer RAM_AW = (RAM_WORDS > 1) ? $clog2(RAM_WORDS) : 1;
    localparam integer DESC_W = 8 + 32 + 32 + BANK_W + 1;
    localparam [PTR_W-1:0] FIFO_DEPTH = DESC_DEPTH[PTR_W-1:0];
    localparam [PTR_W-1:0] PTR_ONE = 1;
    localparam [PTR_W-1:0] PTR_TWO = 2;
    localparam [31:0] WIDTH_32 = SOURCE_WIDTH;
    localparam [31:0] HEIGHT_32 = SOURCE_HEIGHT;
    localparam [31:0] LAST_COLUMN = SOURCE_WIDTH - 1;
    localparam [31:0] ROW_BYTES = SOURCE_WIDTH * 2;
    localparam [15:0] WIDTH_16 = SOURCE_WIDTH[15:0];
    localparam [15:0] HEIGHT_16 = SOURCE_HEIGHT[15:0];
    localparam [7:0] KIND_START = 8'd1, KIND_DATA = 8'd2,
        KIND_END = 8'd3, KIND_ABORT = 8'd4;

    (* ASYNC_REG = "TRUE" *) reg cam_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg cam_reset_n;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg usb_reset_n;
    always @(posedge cam_clk or negedge reset_n) begin
        if (!reset_n) begin cam_reset_meta <= 1'b0; cam_reset_n <= 1'b0; end
        else begin cam_reset_meta <= 1'b1; cam_reset_n <= cam_reset_meta; end
    end
    always @(posedge ifclk or negedge reset_n) begin
        if (!reset_n) begin usb_reset_meta <= 1'b0; usb_reset_n <= 1'b0; end
        else begin usb_reset_meta <= 1'b1; usb_reset_n <= usb_reset_meta; end
    end

    function [PTR_W-1:0] binary_to_gray;
        input [PTR_W-1:0] value;
        begin binary_to_gray = (value >> 1) ^ value; end
    endfunction
    function [PTR_W-1:0] gray_to_binary;
        input [PTR_W-1:0] value;
        integer b;
        begin
            gray_to_binary[PTR_W-1] = value[PTR_W-1];
            for (b = PTR_W-2; b >= 0; b = b-1)
                gray_to_binary[b] = gray_to_binary[b+1] ^ value[b];
        end
    endfunction

    reg [PTR_W-1:0] wr_binary, wr_gray;
    reg [PTR_W-1:0] rd_binary, rd_gray;
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] rd_gray_cam_meta;
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] rd_gray_cam_sync;
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] wr_gray_usb_meta;
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] wr_gray_usb_sync;
    reg [LINE_BANKS-1:0] bank_owner;
    reg [LINE_BANKS-1:0] bank_ack;
    (* ASYNC_REG = "TRUE" *) reg [LINE_BANKS-1:0] bank_ack_cam_meta;
    (* ASYNC_REG = "TRUE" *) reg [LINE_BANKS-1:0] bank_ack_cam_sync;
    always @(posedge cam_clk or negedge cam_reset_n) begin
        if (!cam_reset_n) begin
            rd_gray_cam_meta <= {PTR_W{1'b0}};
            rd_gray_cam_sync <= {PTR_W{1'b0}};
            bank_ack_cam_meta <= {LINE_BANKS{1'b0}};
            bank_ack_cam_sync <= {LINE_BANKS{1'b0}};
        end else begin
            rd_gray_cam_meta <= rd_gray;
            rd_gray_cam_sync <= rd_gray_cam_meta;
            bank_ack_cam_meta <= bank_ack;
            bank_ack_cam_sync <= bank_ack_cam_meta;
        end
    end
    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            wr_gray_usb_meta <= {PTR_W{1'b0}};
            wr_gray_usb_sync <= {PTR_W{1'b0}};
        end else begin
            wr_gray_usb_meta <= wr_gray;
            wr_gray_usb_sync <= wr_gray_usb_meta;
        end
    end
    wire [PTR_W-1:0] fifo_used = wr_binary - gray_to_binary(rd_gray_cam_sync);
    wire [PTR_W-1:0] fifo_free = FIFO_DEPTH - fifo_used;
    wire fifo_has_control = fifo_free >= PTR_ONE;
    wire fifo_has_admission = fifo_free >= PTR_TWO;
    wire fifo_empty = rd_gray == wr_gray_usb_sync;

    reg vsync_d;
    reg frame_open;
    reg start_pending;
    reg [31:0] boundary_sequence;
    reg [31:0] pending_frame_id;
    reg [31:0] current_frame_id;
    reg [31:0] row_index;
    reg [31:0] column_index;
    reg [BANK_W-1:0] row_bank;
    wire cam_boundary = cam_vsync && !vsync_d;
    assign capture_busy = frame_open;

    reg free_bank_valid;
    reg [BANK_W-1:0] free_bank;
    integer bank_scan;
    always @* begin
        free_bank_valid = 1'b0;
        free_bank = {BANK_W{1'b0}};
        for (bank_scan = 0; bank_scan < LINE_BANKS; bank_scan = bank_scan+1)
            if (!free_bank_valid && bank_owner[bank_scan] == bank_ack_cam_sync[bank_scan]) begin
                free_bank_valid = 1'b1;
                free_bank = bank_scan[BANK_W-1:0];
            end
    end
    wire [BANK_W-1:0] write_bank = column_index == 32'd0 ? free_bank : row_bank;
    wire row_can_start = free_bank_valid && fifo_has_admission;
    wire pixel_is_extra = row_index >= HEIGHT_32;
    wire row_admission_failed = column_index == 32'd0 && !row_can_start;
    wire pixel_allowed = frame_open && !start_pending && !cam_boundary && cam_valid &&
        !pixel_is_extra && !row_admission_failed;
    wire cam_write_enable = cam_reset_n && pixel_allowed;
    wire row_complete = pixel_allowed && column_index == LAST_COLUMN;
    wire open_frame_failed = frame_open && !cam_boundary && !start_pending && cam_valid &&
        (pixel_is_extra || row_admission_failed || (row_complete && !fifo_has_admission));
    wire boundary_complete = row_index == HEIGHT_32 && column_index == 32'd0;

    // Descriptor fields: kind, frame id, row/record index, bank, owner toggle.
    // The RAM and descriptor slot are held until the consumer advances rd_gray.
    reg [DESC_W-1:0] descriptor_ram [0:DESC_DEPTH-1];
    reg descriptor_write;
    reg [DESC_W-1:0] descriptor_value;
    always @* begin
        descriptor_write = 1'b0;
        descriptor_value = {DESC_W{1'b0}};
        if (cam_reset_n) begin
            if (cam_boundary && frame_open && fifo_has_control) begin
                descriptor_write = 1'b1;
                descriptor_value = {boundary_complete ? KIND_END : KIND_ABORT,
                    current_frame_id, row_index, {BANK_W{1'b0}}, 1'b0};
            end else if (!cam_boundary && start_pending && !cam_valid && fifo_has_admission) begin
                descriptor_write = 1'b1;
                descriptor_value = {KIND_START, pending_frame_id, 32'd0, {BANK_W{1'b0}}, 1'b0};
            end else if (open_frame_failed && fifo_has_control) begin
                descriptor_write = 1'b1;
                descriptor_value = {KIND_ABORT, current_frame_id, row_index, {BANK_W{1'b0}}, 1'b0};
            end else if (row_complete && fifo_has_admission) begin
                descriptor_write = 1'b1;
                descriptor_value = {KIND_DATA, current_frame_id, row_index,
                    write_bank, ~bank_owner[write_bank]};
            end
        end
    end
    always @(posedge cam_clk)
        if (descriptor_write) descriptor_ram[wr_binary[DESC_AW-1:0]] <= descriptor_value;

    (* ram_style = "block" *) reg [15:0] line_ram [0:RAM_WORDS-1];
    wire [31:0] write_address_wide = ({{(32-BANK_W){1'b0}}, write_bank} * WIDTH_32) + column_index;
    always @(posedge cam_clk)
        if (cam_write_enable) line_ram[write_address_wide[RAM_AW-1:0]] <= cam_rgb565;

    always @(posedge cam_clk or negedge cam_reset_n) begin
        if (!cam_reset_n) begin
            wr_binary <= {PTR_W{1'b0}};
            wr_gray <= {PTR_W{1'b0}};
            bank_owner <= {LINE_BANKS{1'b0}};
            vsync_d <= 1'b0;
            frame_open <= 1'b0;
            start_pending <= 1'b0;
            boundary_sequence <= 32'd0;
            pending_frame_id <= 32'd0;
            current_frame_id <= 32'd0;
            row_index <= 32'd0;
            column_index <= 32'd0;
            row_bank <= {BANK_W{1'b0}};
            frames_dropped <= 32'd0;
            frames_malformed <= 32'd0;
        end else begin
            vsync_d <= cam_vsync;
            if (descriptor_write) begin
                wr_binary <= wr_binary + PTR_ONE;
                wr_gray <= binary_to_gray(wr_binary + PTR_ONE);
            end
            if (cam_boundary) begin
                // There is always a reserved slot to close an admitted frame.
                if (frame_open && !boundary_complete) begin
                    frames_malformed <= frames_malformed + 32'd1;
                    frames_dropped <= frames_dropped + 32'd1;
                end
                if (start_pending) frames_dropped <= frames_dropped + 32'd1;
                frame_open <= 1'b0;
                start_pending <= 1'b1;
                pending_frame_id <= boundary_sequence;
                boundary_sequence <= boundary_sequence + 32'd1;
                row_index <= 32'd0;
                column_index <= 32'd0;
            end else if (start_pending) begin
                // Never begin halfway into an unbackpressured source frame.
                if (cam_valid || !fifo_has_admission) begin
                    start_pending <= 1'b0;
                    frames_dropped <= frames_dropped + 32'd1;
                end else begin
                    start_pending <= 1'b0;
                    frame_open <= 1'b1;
                    current_frame_id <= pending_frame_id;
                end
            end else if (open_frame_failed) begin
                frame_open <= 1'b0;
                frames_dropped <= frames_dropped + 32'd1;
                if (pixel_is_extra) frames_malformed <= frames_malformed + 32'd1;
            end else if (pixel_allowed) begin
                if (column_index == 32'd0) row_bank <= write_bank;
                if (row_complete) begin
                    bank_owner[write_bank] <= ~bank_owner[write_bank];
                    column_index <= 32'd0;
                    row_index <= row_index + 32'd1;
                end else column_index <= column_index + 32'd1;
            end
        end
    end

    function [31:0] crc_byte;
        input [31:0] crc;
        input [7:0] value;
        reg [31:0] c;
        integer bit_index;
        begin
            c = crc ^ {24'd0, value};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index+1)
                c = c[0] ? ((c >> 1) ^ 32'hedb88320) : (c >> 1);
            crc_byte = c;
        end
    endfunction
    function [31:0] crc_word;
        input [31:0] crc;
        input [15:0] value;
        begin crc_word = crc_byte(crc_byte(crc, value[7:0]), value[15:8]); end
    endfunction

    localparam [3:0] IDLE = 4'd0, DESC_WAIT = 4'd1, DESC_LOAD = 4'd2,
        HEADER_LOAD = 4'd3, HEADER_SEND = 4'd4, PAYLOAD = 4'd5,
        CRC_LOAD = 4'd6, CRC_LOW_SEND = 4'd7, CRC_HIGH_SEND = 4'd8,
        PACKET_GUARD = 4'd9, PACKET_END = 4'd10, COMPLETE = 4'd11;
    reg [3:0] state;
    reg [DESC_W-1:0] descriptor_q;
    reg [7:0] record_kind;
    reg [31:0] record_frame_id;
    reg [31:0] record_index;
    reg [BANK_W-1:0] record_bank;
    reg record_bank_toggle;
    reg [3:0] header_index;
    reg [31:0] crc_acc;
    reg [31:0] final_crc;
    reg [15:0] tx_word;
    wire [31:0] record_bytes = record_kind == KIND_DATA ? ROW_BYTES : 32'd0;
    wire [31:0] crc_final = ~crc_acc;
    wire record_short_packet = ((record_bytes + 32'd28) % 32'd512) != 32'd0;
    function [15:0] header_word;
        input [3:0] index;
        begin
            case (index)
                4'd0: header_word = 16'h4946;
                4'd1: header_word = 16'h324d;
                4'd2: header_word = {record_kind, 8'd2};
                4'd3: header_word = 16'd0;
                4'd4: header_word = record_frame_id[15:0];
                4'd5: header_word = record_frame_id[31:16];
                4'd6: header_word = record_index[15:0];
                4'd7: header_word = record_index[31:16];
                4'd8: header_word = WIDTH_16;
                4'd9: header_word = HEIGHT_16;
                4'd10: header_word = record_bytes[15:0];
                4'd11: header_word = record_bytes[31:16];
                default: header_word = 16'd0;
            endcase
        end
    endfunction

    // Two-word skid queue plus a one-clock synchronous RAM read pipeline.
    // Occupancy includes the in-flight RAM read before issuing another read.
    reg [31:0] read_issued;
    reg [31:0] payload_sent;
    reg read_pending;
    reg [15:0] ram_q;
    reg [15:0] queue0, queue1;
    reg [1:0] queue_count;
    wire payload_pop = state == PAYLOAD && queue_count != 2'd0 && fx2_flagc;
    wire [2:0] reserved_words = {1'b0, queue_count} + {2'b00, read_pending} - {2'b00, payload_pop};
    wire ram_read_enable = state == PAYLOAD && read_issued < WIDTH_32 && reserved_words < 3'd2;
    wire [31:0] read_address_wide = ({{(32-BANK_W){1'b0}}, record_bank} * WIDTH_32) + read_issued;
    always @(posedge ifclk) begin
        if (state == IDLE && !fifo_empty) descriptor_q <= descriptor_ram[rd_binary[DESC_AW-1:0]];
        if (ram_read_enable) ram_q <= line_ram[read_address_wide[RAM_AW-1:0]];
    end

    assign fx2_fd = state == PAYLOAD ? queue0 : tx_word;
    assign fx2_faddr = 2'b10;
    assign fx2_sloe_n = 1'b1;
    assign fx2_slrd_n = 1'b1;
    assign fx2_slwr_n = !(usb_reset_n && fx2_flagc &&
        (state == HEADER_SEND || state == CRC_LOW_SEND || state == CRC_HIGH_SEND ||
        (state == PAYLOAD && queue_count != 2'd0)));
    assign fx2_pktend_n = !(usb_reset_n && fx2_flagc && state == PACKET_END);

    always @(posedge ifclk or negedge usb_reset_n) begin
        if (!usb_reset_n) begin
            rd_binary <= {PTR_W{1'b0}};
            rd_gray <= {PTR_W{1'b0}};
            bank_ack <= {LINE_BANKS{1'b0}};
            state <= IDLE;
            record_kind <= 8'd0;
            record_frame_id <= 32'd0;
            record_index <= 32'd0;
            record_bank <= {BANK_W{1'b0}};
            record_bank_toggle <= 1'b0;
            header_index <= 4'd0;
            crc_acc <= 32'hffffffff;
            final_crc <= 32'd0;
            tx_word <= 16'd0;
            read_issued <= 32'd0;
            payload_sent <= 32'd0;
            read_pending <= 1'b0;
            queue0 <= 16'd0;
            queue1 <= 16'd0;
            queue_count <= 2'd0;
            frames_sent <= 32'd0;
        end else begin
            read_pending <= ram_read_enable;
            if (ram_read_enable) read_issued <= read_issued + 32'd1;
            if (state == PAYLOAD) begin
                case ({read_pending, payload_pop})
                    2'b10: begin
                        if (queue_count == 2'd0) queue0 <= ram_q;
                        else queue1 <= ram_q;
                        queue_count <= queue_count + 2'd1;
                    end
                    2'b01: begin
                        if (queue_count == 2'd2) queue0 <= queue1;
                        queue_count <= queue_count - 2'd1;
                    end
                    2'b11: begin
                        if (queue_count == 2'd1) queue0 <= ram_q;
                        else begin queue0 <= queue1; queue1 <= ram_q; end
                    end
                    default: begin end
                endcase
            end
            case (state)
                IDLE: if (!fifo_empty) state <= DESC_WAIT;
                DESC_WAIT: state <= DESC_LOAD;
                DESC_LOAD: begin
                    {record_kind, record_frame_id, record_index,
                        record_bank, record_bank_toggle} <= descriptor_q;
                    header_index <= 4'd0;
                    crc_acc <= 32'hffffffff;
                    read_issued <= 32'd0;
                    payload_sent <= 32'd0;
                    read_pending <= 1'b0;
                    queue_count <= 2'd0;
                    state <= HEADER_LOAD;
                end
                HEADER_LOAD: begin tx_word <= header_word(header_index); state <= HEADER_SEND; end
                HEADER_SEND: if (fx2_flagc) begin
                    crc_acc <= crc_word(crc_acc, tx_word);
                    if (header_index == 4'd11) begin
                        if (record_kind == KIND_DATA) state <= PAYLOAD;
                        else state <= CRC_LOAD;
                    end else begin header_index <= header_index + 4'd1; state <= HEADER_LOAD; end
                end
                PAYLOAD: if (payload_pop) begin
                    crc_acc <= crc_word(crc_acc, queue0);
                    payload_sent <= payload_sent + 32'd1;
                    if (payload_sent == LAST_COLUMN) state <= CRC_LOAD;
                end
                CRC_LOAD: begin
                    final_crc <= crc_final;
                    tx_word <= crc_final[15:0];
                    state <= CRC_LOW_SEND;
                end
                CRC_LOW_SEND: if (fx2_flagc) begin tx_word <= final_crc[31:16]; state <= CRC_HIGH_SEND; end
                CRC_HIGH_SEND: if (fx2_flagc) state <= PACKET_GUARD;
                PACKET_GUARD: begin
                    if (!record_short_packet) state <= COMPLETE;
                    else if (fx2_flagc) state <= PACKET_END;
                end
                PACKET_END: if (fx2_flagc) state <= COMPLETE;
                COMPLETE: begin
                    if (record_kind == KIND_DATA) bank_ack[record_bank] <= record_bank_toggle;
                    if (record_kind == KIND_END) frames_sent <= frames_sent + 32'd1;
                    rd_binary <= rd_binary + PTR_ONE;
                    rd_gray <= binary_to_gray(rd_binary + PTR_ONE);
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end

    initial begin
        if (SOURCE_WIDTH < 1 || SOURCE_HEIGHT < 1 || SOURCE_WIDTH > 65535 || SOURCE_HEIGHT > 65535)
            $fatal(1, "camera_fx2_live dimensions must fit unsigned16");
        if (LINE_BANKS < 2 || LINE_BANKS > 16 || (LINE_BANKS & (LINE_BANKS-1)) != 0)
            $fatal(1, "camera_fx2_live LINE_BANKS must be power of two2..16");
        if (DESC_DEPTH < 4 || DESC_DEPTH > 256 || (DESC_DEPTH & (DESC_DEPTH-1)) != 0)
            $fatal(1, "camera_fx2_live DESC_DEPTH must be power of two4..256");
    end
endmodule
