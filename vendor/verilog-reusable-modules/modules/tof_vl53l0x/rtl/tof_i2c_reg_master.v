// Single-register, single-byte I2C master. All timing uses clk enables.
// External pull-ups are required. Never drive SCL or SDA high.
module tof_i2c_reg_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000,
    parameter integer I2C_TIMEOUT_US = 2000,
    parameter [6:0] DEVICE_ADDR = 7'h29
) (
    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    input wire cmd_read,
    input wire [7:0] cmd_reg,
    input wire [7:0] cmd_wdata,
    output wire cmd_ready,
    output reg rsp_valid,
    output reg [7:0] rsp_rdata,
    output reg [2:0] rsp_error,
    inout wire scl,
    inout wire sda
);
    // Integer parameters must yield at least four clocks per half period.
    // Dividing first avoids overflow for the default 50 MHz / 2 ms timeout.
    localparam [31:0] HALF_CLKS = CLK_HZ / I2C_HZ / 2;
    localparam [31:0] LIMIT_CLKS = (CLK_HZ / 1000000) * I2C_TIMEOUT_US;
    localparam [4:0] IDLE = 5'd0, ACQUIRE = 5'd1,
        START_HOLD = 5'd2, TX_LOW = 5'd3, WAIT_HIGH = 5'd4,
        TX_HIGH = 5'd5, ACK_LOW = 5'd6, ACK_HIGH = 5'd7,
        ACK_FINISH = 5'd8, RESTART_LOW = 5'd9,
        RESTART_HIGH = 5'd10, RX_LOW = 5'd11,
        RX_HIGH = 5'd12, RX_FINISH = 5'd13,
        NACK_LOW = 5'd14, NACK_HIGH = 5'd15,
        STOP_LOW = 5'd16, STOP_HIGH = 5'd17,
        STOP_CHECK = 5'd18, FINISH = 5'd19;
    reg [4:0] state, high_destination;
    reg scl_low, sda_low;
    reg scl_meta, scl_sync, sda_meta, sda_sync;
    reg [31:0] delay_count, wait_count;
    reg read_latched;
    reg [7:0] reg_latched, data_latched, tx_byte, rx_byte;
    reg [2:0] bit_index;
    reg [1:0] transaction_phase;

    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;
    assign cmd_ready = (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_meta <= 1'b1;
            scl_sync <= 1'b1;
            sda_meta <= 1'b1;
            sda_sync <= 1'b1;
        end else begin
            scl_meta <= scl;
            scl_sync <= scl_meta;
            sda_meta <= sda;
            sda_sync <= sda_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            high_destination <= IDLE;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            delay_count <= 32'd0;
            wait_count <= 32'd0;
            read_latched <= 1'b0;
            reg_latched <= 8'd0;
            data_latched <= 8'd0;
            tx_byte <= 8'd0;
            rx_byte <= 8'd0;
            bit_index <= 3'd7;
            transaction_phase <= 2'd0;
            rsp_valid <= 1'b0;
            rsp_rdata <= 8'd0;
            rsp_error <= 3'd0;
        end else begin
            rsp_valid <= 1'b0;
            if (delay_count != 32'd0) begin
                delay_count <= delay_count - 32'd1;
            end else begin
                case (state)
                    IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        if (cmd_valid) begin
                            read_latched <= cmd_read;
                            reg_latched <= cmd_reg;
                            data_latched <= cmd_wdata;
                            tx_byte <= {DEVICE_ADDR, 1'b0};
                            rx_byte <= 8'd0;
                            rsp_rdata <= 8'd0;
                            rsp_error <= 3'd0;
                            transaction_phase <= 2'd0;
                            bit_index <= 3'd7;
                            wait_count <= 32'd0;
                            // Allow synchronizers to observe the released bus.
                            delay_count <= HALF_CLKS - 32'd1;
                            state <= ACQUIRE;
                        end
                    end
                    ACQUIRE: begin
                        if (scl_sync && sda_sync) begin
                            sda_low <= 1'b1;
                            delay_count <= HALF_CLKS - 32'd1;
                            state <= START_HOLD;
                        end else if (wait_count >= LIMIT_CLKS - 32'd1) begin
                            rsp_error <= scl_sync ? 3'd3 : 3'd2;
                            state <= FINISH;
                        end else wait_count <= wait_count + 32'd1;
                    end
                    START_HOLD: begin
                        scl_low <= 1'b1;
                        sda_low <= ~tx_byte[7];
                        delay_count <= HALF_CLKS - 32'd1;
                        state <= TX_LOW;
                    end
                    TX_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= TX_HIGH;
                        state <= WAIT_HIGH;
                    end
                    WAIT_HIGH: begin
                        if (scl_sync) begin
                            delay_count <= HALF_CLKS - 32'd1;
                            state <= high_destination;
                        end else if (wait_count >= LIMIT_CLKS - 32'd1) begin
                            // Abort cannot issue STOP while SCL is held low.
                            scl_low <= 1'b0;
                            sda_low <= 1'b0;
                            rsp_error <= 3'd2;
                            state <= FINISH;
                        end else wait_count <= wait_count + 32'd1;
                    end
                    TX_HIGH: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            sda_low <= 1'b0;
                            state <= ACK_LOW;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            sda_low <= ~tx_byte[bit_index - 3'd1];
                            state <= TX_LOW;
                        end
                        delay_count <= HALF_CLKS - 32'd1;
                    end
                    ACK_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= ACK_HIGH;
                        state <= WAIT_HIGH;
                    end
                    ACK_HIGH: begin
                        // Sample before lowering SCL, after a full high period.
                        scl_low <= 1'b1;
                        if (sda_sync) begin
                            rsp_error <= 3'd1;
                            sda_low <= 1'b1;
                            state <= STOP_LOW;
                        end else state <= ACK_FINISH;
                        // Preserve low hold time before any next START/STOP.
                        delay_count <= HALF_CLKS - 32'd1;
                    end
                    ACK_FINISH: begin
                        bit_index <= 3'd7;
                        case (transaction_phase)
                            2'd0: begin
                                tx_byte <= reg_latched;
                                sda_low <= ~reg_latched[7];
                                transaction_phase <= 2'd1;
                                delay_count <= HALF_CLKS - 32'd1;
                                state <= TX_LOW;
                            end
                            2'd1: begin
                                if (read_latched) begin
                                    sda_low <= 1'b0;
                                    delay_count <= HALF_CLKS - 32'd1;
                                    state <= RESTART_LOW;
                                end else begin
                                    tx_byte <= data_latched;
                                    sda_low <= ~data_latched[7];
                                    transaction_phase <= 2'd2;
                                    delay_count <= HALF_CLKS - 32'd1;
                                    state <= TX_LOW;
                                end
                            end
                            2'd2: begin
                                sda_low <= 1'b1;
                                delay_count <= HALF_CLKS - 32'd1;
                                state <= STOP_LOW;
                            end
                            default: begin
                                sda_low <= 1'b0;
                                delay_count <= HALF_CLKS - 32'd1;
                                state <= RX_LOW;
                            end
                        endcase
                    end
                    RESTART_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= RESTART_HIGH;
                        state <= WAIT_HIGH;
                    end
                    RESTART_HIGH: begin
                        sda_low <= 1'b1;
                        tx_byte <= {DEVICE_ADDR, 1'b1};
                        transaction_phase <= 2'd3;
                        delay_count <= HALF_CLKS - 32'd1;
                        state <= START_HOLD;
                    end
                    RX_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= RX_HIGH;
                        state <= WAIT_HIGH;
                    end
                    RX_HIGH: begin
                        rx_byte[bit_index] <= sda_sync;
                        scl_low <= 1'b1;
                        delay_count <= HALF_CLKS - 32'd1;
                        state <= RX_FINISH;
                    end
                    RX_FINISH: begin
                        if (bit_index == 3'd0) begin
                            rsp_rdata <= rx_byte;
                            // Release SDA for the final master's NACK bit.
                            state <= NACK_LOW;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            state <= RX_LOW;
                        end
                    end
                    NACK_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= NACK_HIGH;
                        state <= WAIT_HIGH;
                    end
                    NACK_HIGH: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        delay_count <= HALF_CLKS - 32'd1;
                        state <= STOP_LOW;
                    end
                    STOP_LOW: begin
                        scl_low <= 1'b0;
                        wait_count <= 32'd0;
                        high_destination <= STOP_HIGH;
                        state <= WAIT_HIGH;
                    end
                    STOP_HIGH: begin
                        sda_low <= 1'b0;
                        delay_count <= HALF_CLKS - 32'd1;
                        wait_count <= 32'd0;
                        state <= STOP_CHECK;
                    end
                    STOP_CHECK: begin
                        if (scl_sync && sda_sync) state <= FINISH;
                        else if (wait_count >= LIMIT_CLKS - 32'd1) begin
                            rsp_error <= scl_sync ? 3'd3 : 3'd2;
                            state <= FINISH;
                        end else wait_count <= wait_count + 32'd1;
                    end
                    FINISH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        rsp_valid <= 1'b1;
                        state <= IDLE;
                    end
                    default: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        rsp_error <= 3'd3;
                        state <= FINISH;
                    end
                endcase
            end
        end
    end
endmodule
