// Whole CSV lines and 12-byte AI control envelopes share one physical UART.
// A control snapshot is sent every 100ms, even while the AI master is off.
module ai_uart_tx #(
    parameter integer CLK_HZ=50_000_000, BAUD_RATE=115200,
    parameter integer INSPECTION_ENABLE=0,
    parameter integer REPORT_CYCLES=5_000_000
)(
    input wire clk,rst_n,
    input wire [7:0] i_csv_data,
    input wire i_csv_valid,
    output wire o_csv_ready,
    input wire [15:0] i_flags,
    input wire [7:0] i_controls,
    input wire i_ai_link_active,
    input wire [7:0] i_inspection_data,
    input wire i_inspection_valid,i_inspection_last,
    output wire o_inspection_ready,
    output wire o_uart_tx
);
    reg [31:0] timer;
    reg due,in_csv,in_inspection,active;
    reg [3:0] index;
    reg [7:0] seq;
    reg [15:0] flags,crc;
    reg [7:0] controls;
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] value;integer n;reg [15:0] c;
        begin c=old^{value,8'd0};for(n=0;n<8;n=n+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
    endfunction
    reg [7:0] packet_byte;
    always @* begin
        case(index)
            0:packet_byte=8'h4d;1:packet_byte=8'h4f;2:packet_byte=8'h54;3:packet_byte=8'h31;
            4:packet_byte=seq;5:packet_byte=8'h50;
            6:packet_byte=flags[15:8];7:packet_byte=flags[7:0];
            8:packet_byte=controls;9:packet_byte=0;
            10:packet_byte=crc[15:8];default:packet_byte=crc[7:0];
        endcase
    end
    wire ready;
    wire inspection_requested=INSPECTION_ENABLE && i_inspection_valid;
    wire control_boundary=due && !in_csv && !in_inspection && i_ai_link_active;
    wire inspection_grant=!active && !in_csv && (in_inspection || (!control_boundary && inspection_requested));
    wire [7:0] data=active?packet_byte:inspection_grant?i_inspection_data:i_csv_data;
    // Reserve the boundary before accepting the first CSV byte.
    wire csv_grant=!active && !in_inspection && !inspection_grant && !control_boundary;
    wire valid=active || (inspection_grant && i_inspection_valid) || (csv_grant && i_csv_valid);
    assign o_csv_ready=csv_grant && ready;
    assign o_inspection_ready=inspection_grant && ready;
    uart_tx_queue #(.CLK_FREQ_HZ(CLK_HZ),.BAUD_RATE(BAUD_RATE),.FIFO_DEPTH_LOG2(7)) tx(
        .i_clock(clk),.i_reset_n(rst_n),.i_tx_data(data),.i_tx_valid(valid),
        .o_tx_ready(ready),.o_uart_tx(o_uart_tx));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin timer<=0;due<=1;in_csv<=0;in_inspection<=0;active<=0;index<=0;seq<=0;flags<=0;controls<=0;crc<=16'hffff;end
        else begin
            if(timer>=REPORT_CYCLES-1) begin timer<=0;due<=1;end else timer<=timer+1'b1;
            if(csv_grant && i_csv_valid && ready) in_csv<=i_csv_data!=8'h0a;
            if(inspection_grant && i_inspection_valid && ready)in_inspection<=!i_inspection_last;
            if(!active && control_boundary) begin
                active<=1;index<=0;flags<=i_flags;controls<=i_controls;crc<=16'hffff;due<=0;
            end else if(active && ready) begin
                if(index<10) crc<=crc_byte(crc,packet_byte);
                if(index==11) begin active<=0;seq<=seq+1'b1;end else index<=index+1'b1;
            end
        end
    end
endmodule
