// Three bounded clients share the original DDR controller, never its clocks.
// Legacy camera keeps its FIFO adapter. AI write/read are independent clients.
// Read commands carry an in-order owner tag; responses are routed a registered
// clock later. Client ready is asserted only for the selected physical command.
module inspection_ddr_arb #(
    parameter integer TAG_DEPTH=32
)(
    input wire clk,rst_n,i_initialized,
    input wire i_legacy_want,i_legacy_en,
    input wire [2:0] i_legacy_cmd,
    input wire [27:0] i_legacy_address,
    input wire [127:0] i_legacy_write_data,
    output wire o_legacy_ready,o_legacy_write_ready,
    output reg o_legacy_read_valid,
    output reg [127:0] o_legacy_read_data,
    input wire i_write_valid,
    input wire [27:0] i_write_address,
    input wire [127:0] i_write_data,
    output wire o_write_ready,
    input wire i_read_valid,
    input wire [27:0] i_read_address,
    output wire o_read_ready,
    output reg o_read_valid,
    output reg [127:0] o_read_data,
    input wire i_memory_ready,i_memory_write_ready,i_memory_read_valid,
    input wire [127:0] i_memory_read_data,
    output wire o_memory_en,o_memory_write_en,o_memory_write_end,
    output reg [2:0] o_memory_cmd,
    output reg [27:0] o_memory_address,
    output reg [127:0] o_memory_write_data,
    output reg o_protocol_fault
);
    localparam integer AW=$clog2(TAG_DEPTH);
    reg [1:0] selected;
    reg [AW-1:0] read_pointer,write_pointer;
    reg [AW:0] count;
    reg tags[0:TAG_DEPTH-1];
    wire room=count<TAG_DEPTH;
    wire legacy_read=i_legacy_cmd==1;
    wire enabled=rst_n && i_initialized;
    wire [2:0] want={i_read_valid && room && !o_protocol_fault,
        i_write_valid && !o_protocol_fault,i_legacy_want && (!legacy_read || room)};
    assign o_legacy_ready=enabled && selected==0 && (!legacy_read || room) && i_memory_ready;
    assign o_legacy_write_ready=enabled && selected==0 && i_memory_write_ready;
    assign o_write_ready=enabled && selected==1 && i_memory_ready && i_memory_write_ready && !o_protocol_fault;
    assign o_read_ready=enabled && selected==2 && i_memory_ready && room && !o_protocol_fault;
    wire legacy_accept=selected==0 && i_legacy_en && o_legacy_ready && (legacy_read || o_legacy_write_ready);
    wire write_accept=i_write_valid && o_write_ready;
    wire read_accept=i_read_valid && o_read_ready;
    assign o_memory_en=legacy_accept || write_accept || read_accept;
    assign o_memory_write_en=write_accept || (legacy_accept && !legacy_read);
    assign o_memory_write_end=o_memory_write_en;
    wire enqueue=read_accept || (legacy_accept && legacy_read);
    wire dequeue=i_memory_read_valid && count!=0;
    always @*begin
        o_memory_cmd=0;o_memory_address=i_write_address;o_memory_write_data=i_write_data;
        case(selected)
            0:begin o_memory_cmd=i_legacy_cmd;o_memory_address=i_legacy_address;o_memory_write_data=i_legacy_write_data;end
            2:begin o_memory_cmd=1;o_memory_address=i_read_address;end
            default:begin end
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            selected<=0;read_pointer<=0;write_pointer<=0;count<=0;o_protocol_fault<=0;
            o_legacy_read_valid<=0;o_read_valid<=0;o_legacy_read_data<=0;o_read_data<=0;
        end else begin
            o_legacy_read_valid<=0;o_read_valid<=0;
            if(enqueue)begin tags[write_pointer]<=read_accept;write_pointer<=write_pointer+1'b1;end
            if(dequeue)begin
                read_pointer<=read_pointer+1'b1;
                o_read_valid<=tags[read_pointer];o_legacy_read_valid<=!tags[read_pointer];
                o_read_data<=i_memory_read_data;o_legacy_read_data<=i_memory_read_data;
            end
            if(i_memory_read_valid && count==0)o_protocol_fault<=1;
            case({enqueue,dequeue})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:begin end
            endcase
            // A blocked write cannot hold a scan-out read behind wr_data_rdy.
            // Switch only the registered grant; accepted commands keep their
            // existing owner tags. With AI idle the legacy path is unchanged.
            if(selected!=2 && want[2] && !o_memory_en)selected<=2;
            else if(o_memory_en || !want[selected])case(selected)
                0:if(want[1])selected<=1;else if(want[2])selected<=2;else selected<=0;
                1:if(want[2])selected<=2;else if(want[0])selected<=0;else selected<=1;
                default:if(want[0])selected<=0;else if(want[1])selected<=1;else selected<=2;
            endcase
        end
    end
    initial begin
        if(TAG_DEPTH<4 || (TAG_DEPTH&(TAG_DEPTH-1)))$fatal(1,"Power-of-two tag depth >=4 required");
    end
endmodule
