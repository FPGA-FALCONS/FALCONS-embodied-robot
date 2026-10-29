// ACG720 manufacturer ch65/ch66 pin contract: IFCLK input, 16-bit Slave FIFO,
// EP2 OUT at 00/FLAGB high=readable; EP6 IN at 10/FLAGC high=writable.
// Registered direction states add setup and high-Z turnaround cycles. Only an
// accepted read pops EP2. Backpressure stops SLRD immediately. No motor ports.
module inspection_fx2_phy(
    input wire clk,rst_n,
    inout wire [15:0] fd,
    input wire flagb,flagc,
    output reg [1:0] address,
    output reg slrd_n,slwr_n,sloe_n,pktend_n,slcs_n,
    output wire [15:0] o_rx_data,
    output wire o_rx_valid,
    input wire i_rx_ready,
    input wire [15:0] i_tx_data,
    input wire i_tx_valid,i_tx_last,
    output wire o_tx_ready
);
    localparam IDLE=0,R_SETUP=1,R_READ=2,TURN=3,W_SETUP=4,W_WRITE=5,FLUSH=6;
    reg [2:0] state;
    reg prefer_write,flush_pending;
    reg [15:0] drive_pin /* synthesis syn_preserve=1 */;
    reg flagb_r,flagc_r,rx_valid_r;
    reg [15:0] rx_data_r,pin_input_sample;
    reg read_granted,write_granted;
    reg [15:0] pin_data;
    reg [7:0] packet_words;
    // Physical pins, including data and OE, are clocked at the I/O boundary.
    // SETUP reserves a FIFO slot for the next edge; the producer holds its
    // current word until that registered strobe is accepted at the FX2.
    wire read_word=state==R_READ && read_granted;
    wire write_word=state==W_WRITE && write_granted;
    genvar pin;
    generate for(pin=0;pin<16;pin=pin+1)begin:g_pins
        assign fd[pin]=drive_pin[pin]?pin_data[pin]:1'bz;
    end endgenerate
    assign o_rx_data=rx_data_r;
    assign o_rx_valid=rx_valid_r;
    assign o_tx_ready=write_word;
    // FX2 changes flags/data after its rising IFCLK edge. Sample on the
    // falling edge, before the next qualified FIFO pop, then forward the
    // held word/valid together on rising edges. No guessed input hold time.
    always @(negedge clk or negedge rst_n)begin
        if(!rst_n)begin flagb_r<=0;flagc_r<=0;pin_input_sample<=0;end
        else begin flagb_r<=flagb;flagc_r<=flagc;pin_input_sample<=fd;end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;prefer_write<=0;packet_words<=0;flush_pending<=0;
            read_granted<=0;write_granted<=0;drive_pin<=0;pin_data<=0;
            rx_data_r<=0;rx_valid_r<=0;
            address<=0;slrd_n<=1;slwr_n<=1;sloe_n<=1;pktend_n<=1;slcs_n<=1;
        end else begin
            slcs_n<=0;slrd_n<=1;slwr_n<=1;pktend_n<=1;
            // No FLAG-dependent clock enable on the wide physical data bank.
            pin_data<=i_tx_data;
            rx_data_r<=pin_input_sample;rx_valid_r<=read_word;
            case(state)
                IDLE:begin
                    if(i_tx_valid && flagc_r && (prefer_write || !flagb_r || !i_rx_ready))begin
                        address<=2;drive_pin<=16'hffff;sloe_n<=1;state<=W_SETUP;
                    end else if(flagb_r && i_rx_ready)begin
                        address<=0;drive_pin<=0;sloe_n<=0;state<=R_SETUP;
                    end
                end
                R_SETUP:begin
                    read_granted<=flagb_r && i_rx_ready;
                    slrd_n<=!(flagb_r && i_rx_ready);state<=R_READ;
                end
                R_READ:begin
                    if(read_word)prefer_write<=1;
                    sloe_n<=1;drive_pin<=0;state<=TURN;
                end
                W_SETUP:begin
                    write_granted<=flagc_r && i_tx_valid;
                    slwr_n<=!(flagc_r && i_tx_valid);state<=W_WRITE;
                end
                W_WRITE:begin
                    drive_pin<=0;state<=TURN;
                    if(write_word)begin
                        packet_words<=packet_words+1'b1;prefer_write<=0;
                        if(i_tx_last && packet_words!=255)flush_pending<=1;
                    end
                end
                TURN:if(flush_pending)begin
                    state<=FLUSH;address<=2;pktend_n<=0;
                end else state<=IDLE;
                FLUSH:begin flush_pending<=0;packet_words<=0;state<=IDLE;end
                default:begin state<=IDLE;drive_pin<=0;sloe_n<=1;end
            endcase
        end
    end
endmodule
