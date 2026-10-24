// Shared UART protocol isolation. Only a complete MOT1 magic claims a frame.
// Once claimed, all 12 bytes (including bad CRC/unknown commands) are binary.
// The short tagged queue replays unmatched magic prefixes without losing ASCII.
module uart_command_router #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BYTE_TIMEOUT_MS = 20
)(
    input wire i_clock,
    input wire i_reset_n,
    input wire [7:0] i_rx_data,
    input wire i_rx_valid,
    output reg [7:0] o_ascii_data,
    output reg o_ascii_valid,
    output reg o_ascii_abort,
    output reg [7:0] o_motion_data,
    output reg o_motion_valid
);
    localparam integer TIMEOUT_CYCLES_RAW = (CLK_FREQ_HZ / 1000) * BYTE_TIMEOUT_MS;
    localparam integer TIMEOUT_CYCLES = TIMEOUT_CYCLES_RAW < 1 ? 1 : TIMEOUT_CYCLES_RAW;
    localparam integer TIMER_BITS = TIMEOUT_CYCLES < 2 ? 1 : $clog2(TIMEOUT_CYCLES + 1);
    localparam integer LAST_TICK = TIMEOUT_CYCLES - 1;
    localparam [TIMER_BITS-1:0] TIMEOUT_LAST = LAST_TICK[TIMER_BITS-1:0];
    // tag 0: ASCII; tag 1: motion; tag 2: first motion byte + ASCII abort.
    reg [7:0] queue_data [0:7], next_data [0:7];
    reg [1:0] queue_tag [0:7], next_tag [0:7];
    reg [3:0] count, next_count;
    reg [1:0] prefix, next_prefix;
    reg [3:0] remaining, next_remaining;
    reg [TIMER_BITS-1:0] idle_ticks, next_ticks;
    integer j;
    integer used;

    function [7:0] magic_byte;
        input integer index;
        begin
            case (index)
                0: magic_byte = 8'h4d;
                1: magic_byte = 8'h4f;
                2: magic_byte = 8'h54;
                default: magic_byte = 8'h31;
            endcase
        end
    endfunction

    always @* begin
        for (j = 0; j < 8; j = j + 1) begin
            next_data[j] = queue_data[j];
            next_tag[j] = queue_tag[j];
        end
        used = {28'd0, count};
        if (count != 0) begin
            for (j = 0; j < 7; j = j + 1) begin
                next_data[j] = queue_data[j+1];
                next_tag[j] = queue_tag[j+1];
            end
            used = used - 1;
        end
        next_prefix = prefix;
        next_remaining = remaining;
        next_ticks = idle_ticks;
        // A partial magic is ordinary ASCII if it never becomes a full magic.
        // A partial claimed binary frame is discarded, never replayed as ASCII.
        if ((prefix != 0 || remaining != 0) && idle_ticks >= TIMEOUT_LAST) begin
            for (j = 0; j < 3; j = j + 1) begin
                if (j < prefix) begin
                    next_data[used] = magic_byte(j);
                    next_tag[used] = 0;
                    used = used + 1;
                end
            end
            next_prefix = 0;
            next_remaining = 0;
            next_ticks = 0;
        end else if (prefix != 0 || remaining != 0) begin
            next_ticks = idle_ticks + 1'b1;
        end else begin
            next_ticks = 0;
        end
        if (i_rx_valid) begin
            next_ticks = 0;
            if (next_remaining != 0) begin
                next_data[used] = i_rx_data;
                next_tag[used] = 1;
                used = used + 1;
                next_remaining = next_remaining - 1'b1;
            end else if (i_rx_data == magic_byte({30'd0, next_prefix})) begin
                if (next_prefix == 3) begin
                    for (j = 0; j < 4; j = j + 1) begin
                        next_data[used] = magic_byte(j);
                        next_tag[used] = j == 0 ? 2 : 1;
                        used = used + 1;
                    end
                    next_prefix = 0;
                    next_remaining = 8;
                end else begin
                    next_prefix = next_prefix + 1'b1;
                end
            end else begin
                for (j = 0; j < 3; j = j + 1) begin
                    if (j < next_prefix) begin
                        next_data[used] = magic_byte(j);
                        next_tag[used] = 0;
                        used = used + 1;
                    end
                end
                if (i_rx_data == 8'h4d) begin
                    next_prefix = 1;
                end else begin
                    next_data[used] = i_rx_data;
                    next_tag[used] = 0;
                    used = used + 1;
                    next_prefix = 0;
                end
            end
        end
        next_count = used[3:0];
    end

    integer k;
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            count <= 0;
            prefix <= 0;
            remaining <= 0;
            idle_ticks <= 0;
            o_ascii_data <= 0;
            o_ascii_valid <= 0;
            o_ascii_abort <= 0;
            o_motion_data <= 0;
            o_motion_valid <= 0;
            for (k = 0; k < 8; k = k + 1) begin
                queue_data[k] <= 0;
                queue_tag[k] <= 0;
            end
        end else begin
            count <= next_count;
            prefix <= next_prefix;
            remaining <= next_remaining;
            idle_ticks <= next_ticks;
            for (k = 0; k < 8; k = k + 1) begin
                queue_data[k] <= next_data[k];
                queue_tag[k] <= next_tag[k];
            end
            o_ascii_valid <= count != 0 && queue_tag[0] == 0;
            o_ascii_abort <= count != 0 && queue_tag[0] == 2;
            o_motion_valid <= count != 0 && queue_tag[0] != 0;
            if (count != 0) begin
                o_ascii_data <= queue_data[0];
                o_motion_data <= queue_data[0];
            end
        end
    end
endmodule
