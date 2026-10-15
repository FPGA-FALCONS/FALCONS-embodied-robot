// Two 512x512 one-bit images stored as 8192x32-bit words. Word-wide clears
// last 8192 clocks (167 us at 49 MHz), rather than clearing every pixel.
// Each bank has one synchronous read port and one write port. Video reads
// the front bank while a read/modify/write engine updates the back bank.
module point_bitmap #(
    parameter integer REFRESH_FRAMES = 12,
    parameter integer ORIGIN_X = 256,
    parameter integer ORIGIN_Y = 44
) (
    input             clk,
    input             reset_n,
    input             point_valid,
    output            point_ready,
    input      [8:0]  point_x,
    input      [8:0]  point_y,
    input     [11:0]  video_x,
    input     [11:0]  video_y,
    input             video_de,
    input             frame_start,
    output            pixel_bit,
    output            pixel_valid,
    output reg [15:0] front_points,
    output            bitmap_ready
);
    localparam [1:0] S_BOOT_CLEAR = 0, S_IDLE = 1,
                     S_WRITE = 2, S_BACK_CLEAR = 3;
    reg [1:0] state;
    reg front_bank;
    reg initialized;
    reg [12:0] clear_addr, write_addr;
    reg [31:0] write_mask;
    reg [15:0] back_points, frame_count;
    reg [31:0] bank0 [0:8191];
    reg [31:0] bank1 [0:8191];
    reg [31:0] bank0_q, bank1_q;

    wire [11:0] local_video_x = video_x - ORIGIN_X;
    wire [11:0] local_video_y = video_y - ORIGIN_Y;
    wire video_inside = video_x >= ORIGIN_X && video_x < ORIGIN_X + 512 &&
                        video_y >= ORIGIN_Y && video_y < ORIGIN_Y + 512;
    wire [12:0] video_addr = {local_video_y[8:0], local_video_x[8:5]};
    wire [12:0] incoming_addr = {point_y, point_x[8:5]};
    wire [12:0] back_read_addr = state == S_IDLE ? incoming_addr : write_addr;
    wire [12:0] bank0_read_addr = front_bank ? back_read_addr : video_addr;
    wire [12:0] bank1_read_addr = front_bank ? video_addr : back_read_addr;
    wire [31:0] back_word = front_bank ? bank0_q : bank1_q;
    wire due = frame_count >= REFRESH_FRAMES - 1;
    wire swap_now = initialized && state == S_IDLE && frame_start && due;

    // No asynchronous reset on RAM read registers or RAM arrays. This pattern
    // preserves simple dual-port block-RAM inference in the Gowin toolchain.
    always @(posedge clk) begin
        bank0_q <= bank0[bank0_read_addr];
        bank1_q <= bank1[bank1_read_addr];
        if (state == S_BOOT_CLEAR) begin
            bank0[clear_addr] <= 32'b0;
            bank1[clear_addr] <= 32'b0;
        end else if (state == S_BACK_CLEAR) begin
            if (front_bank)
                bank0[clear_addr] <= 32'b0;
            else
                bank1[clear_addr] <= 32'b0;
        end else if (state == S_WRITE) begin
            if (front_bank)
                bank0[write_addr] <= back_word | write_mask;
            else
                bank1[write_addr] <= back_word | write_mask;
        end
    end

    reg front_bank_q, inside_q, de_q, initialized_q;
    reg [4:0] bit_index_q;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            front_bank_q <= 0;
            inside_q <= 0;
            de_q <= 0;
            initialized_q <= 0;
            bit_index_q <= 0;
        end else begin
            front_bank_q <= front_bank;
            inside_q <= video_inside;
            de_q <= video_de;
            initialized_q <= initialized;
            bit_index_q <= local_video_x[4:0];
        end
    end
    wire [31:0] front_word = front_bank_q ? bank1_q : bank0_q;
    assign pixel_bit = initialized_q && inside_q && de_q && front_word[bit_index_q];
    assign pixel_valid = de_q;
    assign bitmap_ready = initialized;
    assign point_ready = state == S_IDLE && !swap_now;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_BOOT_CLEAR;
            initialized <= 0;
            front_bank <= 0;
            front_points <= 0;
            back_points <= 0;
            frame_count <= 0;
            clear_addr <= 0;
            write_addr <= 0;
            write_mask <= 0;
        end else begin
            if (frame_start && initialized && !due)
                frame_count <= frame_count + 1'b1;
            case (state)
                S_BOOT_CLEAR: begin
                    if (clear_addr == 13'd8191) begin
                        state <= S_IDLE;
                        initialized <= 1'b1;
                        clear_addr <= 0;
                    end else begin
                        clear_addr <= clear_addr + 1'b1;
                    end
                end
                S_IDLE: begin
                    if (swap_now) begin
                        front_bank <= ~front_bank;
                        front_points <= back_points;
                        back_points <= 0;
                        frame_count <= 0;
                        clear_addr <= 0;
                        state <= S_BACK_CLEAR;
                    end else if (point_valid) begin
                        write_addr <= incoming_addr;
                        write_mask <= 32'b1 << point_x[4:0];
                        if (back_points != 16'hffff)
                            back_points <= back_points + 1'b1;
                        state <= S_WRITE;
                    end
                end
                S_WRITE: state <= S_IDLE;
                S_BACK_CLEAR: begin
                    if (clear_addr == 13'd8191) begin
                        state <= S_IDLE;
                        clear_addr <= 0;
                    end else begin
                        clear_addr <= clear_addr + 1'b1;
                    end
                end
                default: state <= S_BOOT_CLEAR;
            endcase
        end
    end
endmodule
