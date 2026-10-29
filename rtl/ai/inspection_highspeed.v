// Physical Slave FIFO and protocol CDC boundary. A diagnostic round trip is
// always available without an AI task. It echoes bounded data, never motors.
// Control mailbox ownership and the DDR frame sink are explicit sys ports.
module inspection_highspeed #(
    parameter integer CLK_HZ=50_000_000,FIFO_DEPTH=1024,CLOCK_PROBE_ENABLE=0
)(
    input wire clk,rst_n,fx2_clk,
    inout wire [15:0] fx2_fd,
    input wire fx2_flagb,fx2_flagc,
    output wire [1:0] fx2_address,
    output wire fx2_slrd_n,fx2_slwr_n,fx2_sloe_n,fx2_pktend_n,fx2_slcs_n,
    input wire i_display_enabled,
    input wire [31:0] i_session,i_task,i_epoch,i_context,
    input wire [1:0] i_view,
    output wire o_frame_begin,o_frame_commit,o_frame_abort,
    output wire [31:0] o_frame_sequence,
    input wire i_frame_ready,
    output wire [15:0] o_frame_data,
    output wire o_frame_valid,
    input wire i_frame_word_ready,
    output wire o_control_valid,
    output wire [8:0] o_control_words,
    input wire i_control_ready,i_control_read,
    input wire [7:0] i_control_address,
    output wire [15:0] o_control_data,
    input wire i_route_valid,
    output reg o_route_release,
    input wire [31:0] i_route_session,i_route_task,i_route_epoch,i_route_context,i_route_sequence,i_route_crc,
    input wire [1:0] i_route_view,
    input wire [6:0] i_route_words,
    output wire o_route_read,
    output wire [6:0] o_route_address,
    input wire [15:0] i_route_data,
    input wire i_map_valid,
    output reg o_map_release,
    input wire [31:0] i_map_session,i_map_task,i_map_epoch,i_map_context,i_map_sequence,i_map_crc,
    input wire [1:0] i_map_view,
    output wire o_map_read,
    output wire [11:0] o_map_address,
    input wire [15:0] i_map_data,
    output reg [31:0] o_diagnostics
);
    wire clock_alive;
    wire link_reset_n=rst_n && clock_alive;
    reg [2:0] sys_link_reset,fx_reset;
    always @(posedge clk or negedge link_reset_n)begin
        if(!link_reset_n)sys_link_reset<=0;else sys_link_reset<={sys_link_reset[1:0],1'b1};
    end
    always @(posedge fx2_clk or negedge link_reset_n)begin
        if(!link_reset_n)fx_reset<=0;else fx_reset<={fx_reset[1:0],1'b1};
    end
    wire [31:0] measured_hz;
    wire frequency_valid;
    reg measure_start,diagnostic_wait;
    generate if(CLOCK_PROBE_ENABLE)begin:g_clock_probe
        inspection_fx2_clock #(.CLK_HZ(CLK_HZ)) probe(.clk(clk),.rst_n(rst_n),.fx2_clk(fx2_clk),.i_measure(measure_start),.o_hz(measured_hz),.o_valid(frequency_valid),.o_alive(clock_alive));
    end else begin:g_fixed_test_clock
        assign measured_hz=30_000_000;
        assign frequency_valid=1'b1;
        assign clock_alive=1'b1;
    end endgenerate
    wire [15:0] pin_rx,rx_data,pin_tx;
    wire pin_rx_valid,pin_rx_ready,rx_valid,rx_ready,pin_tx_valid,pin_tx_ready,pin_tx_last;
    wire [16:0] tx_fifo_data;
    wire [15:0] tx_data;
    wire tx_valid,tx_ready,tx_last;
    inspection_fx2_phy phy(.clk(fx2_clk),.rst_n(fx_reset[2]),.fd(fx2_fd),.flagb(fx2_flagb),.flagc(fx2_flagc),
        .address(fx2_address),.slrd_n(fx2_slrd_n),.slwr_n(fx2_slwr_n),.sloe_n(fx2_sloe_n),.pktend_n(fx2_pktend_n),.slcs_n(fx2_slcs_n),
        .o_rx_data(pin_rx),.o_rx_valid(pin_rx_valid),.i_rx_ready(pin_rx_ready),
        .i_tx_data(tx_fifo_data[15:0]),.i_tx_valid(pin_tx_valid),.i_tx_last(tx_fifo_data[16]),.o_tx_ready(pin_tx_ready));
    stream_async_fifo #(.WIDTH(16),.DEPTH(FIFO_DEPTH)) rx_fifo(
        .wclk(fx2_clk),.wrst_n(fx_reset[2]),.i_valid(pin_rx_valid),.i_data(pin_rx),.o_ready(pin_rx_ready),
        .rclk(clk),.rrst_n(sys_link_reset[2]),.o_valid(rx_valid),.o_data(rx_data),.i_ready(rx_ready));
    stream_async_fifo #(.WIDTH(17),.DEPTH(FIFO_DEPTH)) tx_fifo(
        .wclk(clk),.wrst_n(sys_link_reset[2]),.i_valid(tx_valid),.i_data({tx_last,tx_data}),.o_ready(tx_ready),
        .rclk(fx2_clk),.rrst_n(fx_reset[2]),.o_valid(pin_tx_valid),.o_data(tx_fifo_data),.i_ready(pin_tx_ready));
    wire message_valid,message_read,message_ready;
    wire [7:0] message_kind,message_address;
    wire [8:0] message_words;
    wire [15:0] message_data,message_view;
    wire [31:0] message_crc,message_session,message_task,message_epoch,message_context;
    wire [31:0] message_sequence;
    inspection_stream_rx #(.CLK_HZ(CLK_HZ)) receiver(
        .clk(clk),.rst_n(rst_n),.i_data(rx_data),.i_valid(rx_valid),.o_ready(rx_ready),
        .i_display_enabled(i_display_enabled),.i_session(i_session),.i_task(i_task),.i_epoch(i_epoch),.i_context(i_context),.i_view(i_view),
        .o_frame_begin(o_frame_begin),.o_frame_commit(o_frame_commit),.o_frame_abort(o_frame_abort),.o_sequence(message_sequence),
        .i_frame_ready(i_frame_ready),.o_frame_data(o_frame_data),.o_frame_valid(o_frame_valid),.i_frame_word_ready(i_frame_word_ready),
        .o_message_valid(message_valid),.i_message_ready(message_ready),.o_message_kind(message_kind),.o_message_words(message_words),
        .o_message_crc(message_crc),.o_message_session(message_session),.o_message_task(message_task),
        .o_message_epoch(message_epoch),.o_message_context(message_context),.o_message_view(message_view),
        .i_message_read(message_read),.i_message_address(message_address),.o_message_data(message_data),.o_error());
    assign o_frame_sequence=message_sequence;
    assign o_control_valid=message_valid && message_kind==2;
    assign o_control_words=message_words;
    assign o_control_data=message_data;
    wire tx_busy,tx_read;
    wire [11:0] tx_address;
    reg diagnostic_active,map_active,route_active,tx_start,diagnostic_ack;
    assign o_route_read=route_active && tx_read;
    assign o_route_address=tx_address[6:0];
    assign o_map_read=map_active && tx_read;
    assign o_map_address=tx_address;
    assign message_read=diagnostic_active?tx_read:i_control_read;
    assign message_address=diagnostic_active?tx_address[7:0]:i_control_address;
    assign message_ready=diagnostic_ack || (o_control_valid && i_control_ready);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin measure_start<=0;diagnostic_wait<=0;diagnostic_active<=0;map_active<=0;route_active<=0;o_route_release<=0;o_map_release<=0;tx_start<=0;diagnostic_ack<=0;o_diagnostics<=0;end
        else begin
            measure_start<=0;tx_start<=0;diagnostic_ack<=0;o_route_release<=0;o_map_release<=0;
            if(message_valid && message_kind==4 && !diagnostic_wait && !diagnostic_active && !tx_busy && !map_active && !route_active && !diagnostic_ack && !o_map_release && !o_route_release)begin
                measure_start<=1;diagnostic_wait<=1;
            end else if(i_route_valid && !diagnostic_wait && !route_active && !map_active && !diagnostic_active && !tx_busy && !diagnostic_ack && !o_map_release && !o_route_release)begin
                tx_start<=1;route_active<=1;
            end else if(i_map_valid && !diagnostic_wait && !map_active && !route_active && !diagnostic_active && !tx_busy && !diagnostic_ack && !o_map_release && !o_route_release)begin
                tx_start<=1;map_active<=1;
            end
            if(diagnostic_wait && !measure_start && frequency_valid && clock_alive && !tx_busy && !map_active && !route_active)begin
                diagnostic_wait<=0;tx_start<=1;diagnostic_active<=1;
            end
            if(diagnostic_active && tx_valid && tx_ready && tx_last)begin
                diagnostic_ack<=1;diagnostic_active<=0;o_diagnostics<=o_diagnostics+1'b1;
            end
            if(route_active && tx_valid && tx_ready && tx_last)begin
                o_route_release<=1;route_active<=0;
            end
            if(map_active && tx_valid && tx_ready && tx_last)begin
                o_map_release<=1;map_active<=0;
            end
        end
    end
    inspection_stream_tx transmitter(.clk(clk),.rst_n(rst_n),.i_start(tx_start),.i_kind(route_active?8'd5:map_active?8'd3:8'd4),
        .i_session(route_active?i_route_session:map_active?i_map_session:measured_hz),.i_task(route_active?i_route_task:map_active?i_map_task:message_task),
        .i_epoch(route_active?i_route_epoch:map_active?i_map_epoch:message_epoch),.i_context(route_active?i_route_context:map_active?i_map_context:message_context),
        .i_sequence(route_active?i_route_sequence:map_active?i_map_sequence:(message_sequence ^ 32'h46504741)),
        .i_bytes(route_active?{24'd0,i_route_words,1'b0}:map_active?32'd8192:{22'd0,message_words,1'b0}),.i_crc(route_active?i_route_crc:map_active?i_map_crc:message_crc),
        .i_width((map_active || route_active)?16'd256:16'd0),.i_height((map_active || route_active)?16'd256:16'd0),.i_stride(route_active?16'd2:map_active?16'd32:16'd0),
        .i_view(route_active?{14'd0,i_route_view}:map_active?{14'd0,i_map_view}:message_view),
        .o_busy(tx_busy),.o_payload_read(tx_read),.o_payload_address(tx_address),.i_payload_data(route_active?i_route_data:map_active?i_map_data:message_data),
        .o_data(tx_data),.o_valid(tx_valid),.o_last(tx_last),.i_ready(tx_ready));
endmodule
