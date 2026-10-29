// Two independent 1-bit planes, double buffered. RAM data/read registers are
// never reset. Upload and IEEE CRC32 advance one bit per clock, not 32 xor
// iterations on the packet-to-map path. Commit waits for stopped + read drain.
module nav_visual_layer #(
    parameter integer CELL_BITS=16
)(
    input wire clk,reset_n,
    input wire i_enable,
    input wire [31:0] i_epoch,
    input wire i_begin,i_abort,i_commit,
    input wire [31:0] i_begin_epoch,i_expected_crc,
    input wire [15:0] i_begin_version,
    input wire i_stopped,i_read_inflight,
    input wire i_word_valid,
    input wire [CELL_BITS-4:0] i_word_index,
    input wire [31:0] i_word,
    output wire o_word_ready,
    output reg o_layer_valid,
    output reg [15:0] o_version,
    output reg o_committed,o_error,
    output reg [3:0] o_error_code,
    output wire o_read_blocked,
    input wire i_read,
    input wire [CELL_BITS-1:0] i_read_address,
    output wire o_raw,o_cost
);
    localparam integer CELLS=1<<CELL_BITS;
    localparam integer TOTAL_BITS=2*CELLS;
    reg raw_mem[0:2*CELLS-1];
    reg cost_mem[0:2*CELLS-1];
    reg front_bank,stage_bank,staging,writing,commit_pending;
    reg [CELL_BITS+1:0] written;
    reg [5:0] remaining;
    reg [31:0] shift_word,crc,expected_crc,epoch_q;
    reg [15:0] stage_version;
    reg invalid_cost;
    reg raw_q,cost_q,verify_valid,verify_bit;
    wire cost_write=writing && written[CELL_BITS];
    wire [CELL_BITS:0] write_address={stage_bank,written[CELL_BITS-1:0]};
    wire [CELL_BITS:0] read_address={front_bank,i_read_address};
    wire raw_write=writing && !written[CELL_BITS];
    wire [CELL_BITS:0] raw_read_address=cost_write?write_address:read_address;
    // A single synchronous raw read port is shared by normal reads and the
    // back-bank cost-superset check. The caller blocks reads while writing.
    always @(posedge clk) begin
        // Explicit write-first bypass avoids Gowin's unsupported DPB
        // read-before-write mode. Consumers are blocked during writes; the
        // bypass value is never presented as a navigation read response.
        if(raw_write) begin
            raw_mem[write_address]<=shift_word[0];raw_q<=shift_word[0];
        end else if(cost_write || i_read) raw_q<=raw_mem[raw_read_address];
        if(cost_write) begin
            cost_mem[write_address]<=shift_word[0];cost_q<=shift_word[0];
        end else if(i_read) cost_q<=cost_mem[read_address];
    end
    assign o_raw=i_enable && o_layer_valid ? raw_q : 1'b0;
    assign o_cost=i_enable && o_layer_valid ? cost_q : 1'b0;
    assign o_word_ready=staging && !writing && !commit_pending && written<TOTAL_BITS;
    assign o_read_blocked=writing || commit_pending;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            front_bank<=0;stage_bank<=1;staging<=0;writing<=0;commit_pending<=0;
            written<=0;remaining<=0;shift_word<=0;crc<=32'hffffffff;expected_crc<=0;
            stage_version<=0;epoch_q<=0;invalid_cost<=0;verify_valid<=0;verify_bit<=0;
            o_layer_valid<=0;o_version<=0;o_committed<=0;o_error<=0;o_error_code<=0;
        end else begin
            o_committed<=0;o_error<=0;
            verify_valid<=cost_write;
            if(cost_write) verify_bit<=shift_word[0];
            if(verify_valid && raw_q && !verify_bit) invalid_cost<=1;
            if(i_abort || i_epoch!=epoch_q) begin
                staging<=0;writing<=0;commit_pending<=0;verify_valid<=0;
                if(i_epoch!=epoch_q) begin
                    epoch_q<=i_epoch;o_layer_valid<=0;o_version<=0;
                end
            end else if(i_begin) begin
                if(writing || commit_pending || i_begin_epoch!=i_epoch ||
                   i_begin_version==0 || i_begin_version<=o_version) begin
                    staging<=0;o_error<=1;o_error_code<=1;
                end else begin
                    stage_bank<=!front_bank;staging<=1;written<=0;crc<=32'hffffffff;
                    expected_crc<=i_expected_crc;stage_version<=i_begin_version;
                    invalid_cost<=0;verify_valid<=0;
                end
            end else if(writing) begin
                crc<=(crc>>1)^((crc[0]^shift_word[0])?32'hedb88320:32'd0);
                shift_word<=shift_word>>1;written<=written+1'b1;
                remaining<=remaining-1'b1;
                if(remaining==1) writing<=0;
            end else if(i_word_valid && o_word_ready) begin
                if({1'b0,i_word_index}!=written[CELL_BITS+1:5]) begin
                    staging<=0;o_error<=1;o_error_code<=3;
                end else begin
                    writing<=1;remaining<=32;shift_word<=i_word;
                end
            end else if(i_commit) begin
                if(!staging || written!=TOTAL_BITS) begin
                    staging<=0;o_error<=1;o_error_code<=4;
                end else if((~crc)!=expected_crc) begin
                    staging<=0;o_error<=1;o_error_code<=5;
                end else if(!i_stopped) begin
                    staging<=0;o_error<=1;o_error_code<=6;
                end else commit_pending<=1;
            end else if(commit_pending && !verify_valid) begin
                if(invalid_cost || !i_stopped) begin
                    commit_pending<=0;staging<=0;o_error<=1;o_error_code<=7;
                end else if(!i_read_inflight) begin
                    front_bank<=stage_bank;o_layer_valid<=1;o_version<=stage_version;
                    o_committed<=1;commit_pending<=0;staging<=0;
                end
            end
        end
    end
endmodule
