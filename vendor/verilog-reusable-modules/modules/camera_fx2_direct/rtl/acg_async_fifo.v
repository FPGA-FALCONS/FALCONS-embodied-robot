`timescale 1ns/1ps
// Dual-clock FIFO for the direct camera stream.
//
// Gray-coded pointers cross through two-stage synchronizers.  The write side
// also exposes a conservative free-word count so a frame header can be
// admitted atomically (all 12 words or none).  The read side is registered:
// asserting rd_en while !rd_empty makes rd_data/rd_valid available on the next
// rd_clk edge.  Memory contents are deliberately not reset.
module acg_async_fifo #(
    parameter integer WIDTH = 16,
    parameter integer DEPTH = 4096
) (
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             wr_full,
    output reg  [31:0]      wr_free,
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    input  wire             rd_en,
    output reg  [WIDTH-1:0] rd_data,
    output reg              rd_valid,
    output wire             rd_empty
);
    localparam integer AW = $clog2(DEPTH);
    localparam integer PW = AW + 1;

    (* ram_style = "block" *) reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg [PW-1:0] wr_bin;
    reg [PW-1:0] wr_gray;
    reg [PW-1:0] rd_bin;
    reg [PW-1:0] rd_gray;

    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rd_gray_wr1;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rd_gray_wr2;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wr_gray_rd1;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wr_gray_rd2;

    function automatic [PW-1:0] bin2gray(input [PW-1:0] value);
        begin
            bin2gray = (value >> 1) ^ value;
        end
    endfunction

    function automatic [PW-1:0] gray2bin(input [PW-1:0] value);
        integer i;
        begin
            gray2bin[PW-1] = value[PW-1];
            for (i = PW-2; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ value[i];
        end
    endfunction

    // Pointer synchronizers.  Reset is already synchronized per domain by the
    // caller; these registers only need an asynchronous assertion.
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_wr1 <= {PW{1'b0}};
            rd_gray_wr2 <= {PW{1'b0}};
        end else begin
            rd_gray_wr1 <= rd_gray;
            rd_gray_wr2 <= rd_gray_wr1;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_rd1 <= {PW{1'b0}};
            wr_gray_rd2 <= {PW{1'b0}};
        end else begin
            wr_gray_rd1 <= wr_gray;
            wr_gray_rd2 <= wr_gray_rd1;
        end
    end

    wire [PW-1:0] wr_bin_next  = wr_bin + {{(PW-1){1'b0}}, 1'b1};
    wire [PW-1:0] wr_gray_next = bin2gray(wr_bin_next);
    wire [PW-1:0] rd_bin_next  = rd_bin + {{(PW-1){1'b0}}, 1'b1};
    wire [PW-1:0] rd_gray_next = bin2gray(rd_bin_next);

    // Classic full/empty tests.  DEPTH must be a power of two >= 4.
    assign wr_full = (wr_gray ==
        {~rd_gray_wr2[PW-1:PW-2], rd_gray_wr2[PW-3:0]});
    assign rd_empty = (rd_gray == wr_gray_rd2);

    wire [PW-1:0] rd_bin_wr = gray2bin(rd_gray_wr2);
    wire [PW-1:0] used      = wr_bin - rd_bin_wr;
    wire [31:0]   used32    = {{(32-PW){1'b0}}, used};
    always @* begin
        if (used32 >= DEPTH)
            wr_free = 32'd0;
        else
            wr_free = DEPTH - used32;
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin  <= {PW{1'b0}};
            wr_gray <= {PW{1'b0}};
        end else if (wr_en && !wr_full) begin
            mem[wr_bin[AW-1:0]] <= wr_data;
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin   <= {PW{1'b0}};
            rd_gray  <= {PW{1'b0}};
            rd_data  <= {WIDTH{1'b0}};
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            if (rd_en && !rd_empty) begin
                rd_data  <= mem[rd_bin[AW-1:0]];
                rd_bin   <= rd_bin_next;
                rd_gray  <= rd_gray_next;
                rd_valid <= 1'b1;
            end
        end
    end

    initial begin
        if (DEPTH < 4)
            $fatal(1, "acg_async_fifo requires DEPTH >= 4");
        if ((DEPTH & (DEPTH - 1)) != 0)
            $fatal(1, "acg_async_fifo requires a power-of-two DEPTH");
        if (WIDTH < 1)
            $fatal(1, "acg_async_fifo requires WIDTH >= 1");
    end
endmodule
