// Full-resolution double-bank AI display path. It adds clients to the one
// existing DDR controller and leaves original camera FIFO clocks untouched.
module inspection_frame_store #(
    parameter integer CLK_HZ=50_000_000,TOKEN_DEPTH=1024,DISPLAY_DEPTH=128
)(
    input wire clk,rst_n,ddr_clk,i_ddr_initialized,pixel_clk,
    input wire i_begin,i_commit,i_abort,
    input wire [31:0] i_session,i_task,i_epoch,i_context,i_sequence,
    input wire [1:0] i_view,
    output wire o_frame_ready,
    input wire [15:0] i_pixel,
    input wire i_pixel_valid,
    output wire o_pixel_ready,
    output wire o_write_valid,
    output wire [27:0] o_write_address,
    output wire [127:0] o_write_data,
    input wire i_write_ready,
    output wire o_read_valid,
    output wire [27:0] o_read_address,
    input wire i_read_ready,i_response_valid,
    input wire [127:0] i_response_data,
    input wire i_frame_sync,i_pixel_req,i_show,
    input wire [11:0] i_x,i_y,
    output wire [23:0] o_rgb,
    output wire o_rgb_valid,o_fault
);
    reg fresh,read_enable;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)read_enable<=0;else read_enable<=i_show && fresh;
    end
    wire reset_all_n=rst_n && i_ddr_initialized;
    reg [2:0] sys_reset,ddr_reset,pixel_reset;
    always @(posedge clk or negedge reset_all_n)
        if(!reset_all_n)sys_reset<=0;else sys_reset<={sys_reset[1:0],1'b1};
    always @(posedge ddr_clk or negedge reset_all_n)
        if(!reset_all_n)ddr_reset<=0;else ddr_reset<={ddr_reset[1:0],1'b1};
    always @(posedge pixel_clk or negedge reset_all_n)
        if(!reset_all_n)pixel_reset<=0;else pixel_reset<={pixel_reset[1:0],1'b1};
    wire [17:0] token,ddr_token;
    wire token_valid,token_ready,ddr_token_valid,ddr_token_ready,bank_ready,done_toggle;
    (* ASYNC_REG="TRUE" *) reg [1:0] bank_sync,done_sync;
    always @(posedge clk or negedge reset_all_n)begin
        if(!reset_all_n)begin bank_sync<=0;done_sync<=0;end
        else begin bank_sync<={bank_sync[0],bank_ready};done_sync<={done_sync[0],done_toggle};end
    end
    inspection_frame_tokens frontend(.clk(clk),.rst_n(sys_reset[2]),.i_bank_ready(bank_sync[1]),.i_done_toggle(done_sync[1]),
        .i_begin(i_begin),.i_commit(i_commit),.i_abort(i_abort),.i_session(i_session),.i_task(i_task),.i_epoch(i_epoch),
        .i_context(i_context),.i_sequence(i_sequence),.i_view(i_view),.o_frame_ready(o_frame_ready),
        .i_pixel(i_pixel),.i_pixel_valid(i_pixel_valid),.o_pixel_ready(o_pixel_ready),
        .o_token(token),.o_token_valid(token_valid),.i_token_ready(token_ready));
    stream_async_fifo #(.WIDTH(18),.DEPTH(TOKEN_DEPTH)) input_fifo(
        .wclk(clk),.wrst_n(sys_reset[2]),.i_valid(token_valid),.i_data(token),.o_ready(token_ready),
        .rclk(ddr_clk),.rrst_n(ddr_reset[2]),.o_valid(ddr_token_valid),.o_data(ddr_token),.i_ready(ddr_token_ready));
    wire publish_toggle,publish_bank,publish_done,writer_error;
    wire [175:0] metadata;
    inspection_frame_writer writer(.clk(ddr_clk),.rst_n(ddr_reset[2]),.i_initialized(i_ddr_initialized),
        .i_token(ddr_token),.i_token_valid(ddr_token_valid),.o_token_ready(ddr_token_ready),.o_bank_ready(bank_ready),.o_done_toggle(done_toggle),
        .o_write_valid(o_write_valid),.o_write_address(o_write_address),.o_write_data(o_write_data),.i_write_ready(i_write_ready),
        .o_publish_toggle(publish_toggle),.o_publish_bank(publish_bank),.o_publish_metadata(metadata),.i_publish_done(publish_done),.o_error(writer_error));
    wire tick,publish_ack,read_fifo_reset,fifo_write,fifo_full,fifo_empty,fifo_read,fifo_valid,reader_fault;
    wire [127:0] fifo_write_data,fifo_read_data;
    wire [31:0] free;
    wire [15:0] colour;
    wire data_reset_n=read_fifo_reset && pixel_reset[2];
    inspection_frame_reader reader(.clk(ddr_clk),.rst_n(ddr_reset[2]),.i_enable(read_enable),.i_pixel_tick(tick),.i_pixel_publish_ack(publish_ack),
        .i_publish_toggle(publish_toggle),.i_publish_bank(publish_bank),.o_publish_done(publish_done),
        .o_read_valid(o_read_valid),.o_read_address(o_read_address),.i_read_ready(i_read_ready),
        .i_response_valid(i_response_valid),.i_response_data(i_response_data),.i_fifo_free(free),
        .o_fifo_write(fifo_write),.o_fifo_data(fifo_write_data),.o_fifo_reset_n(read_fifo_reset),.o_fault(reader_fault));
    acg_async_fifo #(.WIDTH(128),.DEPTH(DISPLAY_DEPTH)) output_fifo(
        .wr_clk(ddr_clk),.wr_rst_n(read_fifo_reset && ddr_reset[2]),.wr_en(fifo_write),.wr_data(fifo_write_data),.wr_full(fifo_full),.wr_free(free),
        .rd_clk(pixel_clk),.rd_rst_n(data_reset_n),.rd_en(fifo_read),.rd_data(fifo_read_data),.rd_valid(fifo_valid),.rd_empty(fifo_empty));
    reg fifo_fault;
    always @(posedge ddr_clk or negedge reset_all_n)begin
        if(!reset_all_n)fifo_fault<=0;
        else if(fifo_write && fifo_full)fifo_fault<=1;
    end
    assign o_fault=fifo_fault || reader_fault;
    localparam AGE_BITS=$clog2(CLK_HZ+1);
    reg [AGE_BITS-1:0] age;
    always @(posedge clk or negedge reset_all_n)begin
        if(!reset_all_n)begin age<=0;fresh<=0;end
        else if(i_commit)begin age<=0;fresh<=1;end
        else if(age<CLK_HZ)age<=age+1'b1;else fresh<=0;
    end
    inspection_frame_pixels pixels(.clk(pixel_clk),.rst_n(pixel_reset[2]),.i_frame_sync(i_frame_sync),.i_pixel_req(i_pixel_req),
        .i_x(i_x),.i_y(i_y),.i_show(i_show && fresh && !o_fault),.i_session(i_session),.i_task(i_task),.i_epoch(i_epoch),
        .i_context(i_context),.i_view(i_view),.i_publish_toggle(publish_toggle),.i_publish_metadata(metadata),.o_tick(tick),.o_publish_ack(publish_ack),
        .i_fifo_reset_n(data_reset_n),.i_fifo_empty(fifo_empty),.i_fifo_valid(fifo_valid),.i_fifo_data(fifo_read_data),.o_fifo_read(fifo_read),
        .o_pixel(colour),.o_pixel_valid(o_rgb_valid),.o_underflow());
    assign o_rgb={colour[15:11],3'b000,colour[10:5],2'b00,colour[4:0],3'b000};
endmodule
