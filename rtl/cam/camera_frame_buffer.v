// OV5640 three-bank store. Camera cannot be backpressured: only an armed
// start-of-frame is captured; incomplete/overflowed frames are never published.
// All bank ownership lives in ui_clk. Display tickets cross as toggles; a bank
// changes only after old read responses drain, before resetting/prefilling FIFO.
// Address units are 16-bit words. Banks 0x200000/0x300000/0x400000 do not overlap
// the AI banks 0x100000/0x180000. No full-frame storage is added to BSRAM.
module camera_frame_buffer #(
    parameter integer PIXELS=1024*600,
    parameter integer READ_TARGET=128
)(
    input wire ui_clk,rst_n,init_calib_complete,
    input wire app_rdy,app_wdf_rdy,app_rd_data_valid,
    input wire [127:0] app_rd_data,
    output wire [27:0] app_addr,
    output wire o_request,app_en,app_wdf_wren,app_wdf_end,
    output wire [2:0] app_cmd,
    output wire [127:0] app_wdf_data,
    input wire rd_load,wr_load,wr_clk,wfifo_wren,
    input wire [15:0] wfifo_din,
    output wire wrfifo_full,
    input wire rd_clk,rfifo_rden,
    output wire rdfifo_empty,
    output wire [15:0] rfifo_dout
);
    localparam integer BLOCKS=PIXELS/8;
    localparam W_ALLOC=0,W_RESET=1,W_CAPTURE=2,W_FINISH=3;
    localparam R_WAIT=0,R_DRAIN=1,R_RESET=2,R_RUN=3;
    reg [1:0] ws,rs;
    reg [1:0] front_bank,pending_bank,write_bank;
    reg front_valid,pending_valid;
    reg [16:0] write_index,read_index;
    reg [5:0] outstanding;
    reg [4:0] write_reset_count,read_reset_count;
    reg [2:0] drain_guard;
    reg write_fifo_reset,read_fifo_reset;
    reg arm_toggle,last_done;
    (* ASYNC_REG="TRUE" *) reg [1:0] done_sync,tick_sync,blank_sync;
    reg last_tick;

    // Writer request and completion are stable-data toggle handshakes. A new
    // request cannot be issued until the previous capture and DDR writes finish.
    (* ASYNC_REG="TRUE" *) reg [1:0] arm_sync;
    (* ASYNC_REG="TRUE" *) reg [2:0] capture_reset;
    reg seen_arm,waiting_frame,capturing,capture_bad,capture_done;
    reg wr_load_q;
    reg [4:0] arm_delay;
    reg [19:0] pixel_count;
    wire capture_rst_n=rst_n && !write_fifo_reset;
    wire camera_boundary=wr_load && !wr_load_q;
    always @(posedge wr_clk or negedge rst_n)begin
        if(!rst_n)arm_sync<=0;else arm_sync<={arm_sync[0],arm_toggle};
    end
    always @(posedge wr_clk or negedge capture_rst_n)begin
        if(!capture_rst_n)capture_reset<=0;
        else capture_reset<={capture_reset[1:0],1'b1};
    end
    // capture_done is not reset by the per-frame FIFO reset: no false tickets.
    always @(posedge wr_clk or negedge rst_n)begin
        if(!rst_n)begin
            seen_arm<=0;waiting_frame<=0;capturing<=0;capture_bad<=0;
            capture_done<=0;wr_load_q<=0;arm_delay<=0;pixel_count<=0;
        end else begin
            wr_load_q<=wr_load;
            if(arm_sync[1]!=seen_arm)begin
                seen_arm<=arm_sync[1];arm_delay<=16;waiting_frame<=0;
            end else if(arm_delay!=0 && capture_reset[2])begin
                arm_delay<=arm_delay-1'b1;
                if(arm_delay==1)waiting_frame<=1;
            end
            if(camera_boundary)begin
                if(capturing)begin
                    capturing<=0;capture_bad<=1;capture_done<=!capture_done;
                end else if(waiting_frame && capture_reset[2])begin
                    waiting_frame<=0;capturing<=1;pixel_count<=0;capture_bad<=0;
                end
            end else if(capturing && wfifo_wren)begin
                if(wrfifo_full)capture_bad<=1;
                pixel_count<=pixel_count+1'b1;
                if(pixel_count==PIXELS-1)begin capturing<=0;capture_done<=!capture_done;end
            end
        end
    end

    // Preserve the existing, already placed 16<->128 bit Gowin FIFO IPs.
    wire [127:0] write_data;
    wire [9:0] write_count,read_count;
    wire write_empty,read_full;
    wire write_accept,read_accept;
    wire fifo_write=capturing && capture_reset[2] && wfifo_wren && !wrfifo_full && !camera_boundary;
    wr_data_fifo capture_fifo(.Data(wfifo_din),.Reset(!rst_n || write_fifo_reset),
        .WrClk(wr_clk),.RdClk(ui_clk),.WrEn(fifo_write),.RdEn(write_accept),
        .Wnum(),.Rnum(write_count),.Almost_Empty(),.Almost_Full(),
        .Q(write_data),.Empty(write_empty),.Full(wrfifo_full));

    reg display_tick,rd_load_q,blank_window;
    (* ASYNC_REG="TRUE" *) reg [2:0] display_reset;
    (* ASYNC_REG="TRUE" *) reg [1:0] valid_sync;
    reg display_bad;
    wire display_rst_n=rst_n && !read_fifo_reset;
    wire [15:0] pixel_data;
    wire fifo_read=rfifo_rden && display_reset[2] && valid_sync[1] && !rdfifo_empty && !display_bad;
    assign rfifo_dout=display_reset[2] && valid_sync[1] && !display_bad && !rdfifo_empty ? pixel_data : 16'd0;
    always @(posedge rd_clk or negedge rst_n)begin
        if(!rst_n)begin display_tick<=0;rd_load_q<=0;valid_sync<=0;blank_window<=0;display_bad<=0;end
        else begin
            rd_load_q<=rd_load;valid_sync<={valid_sync[0],front_valid};
            if(rd_load && !rd_load_q)begin
                display_tick<=!display_tick;blank_window<=1;display_bad<=0;
            end else if(rfifo_rden)begin
                blank_window<=0;
                if(!display_reset[2] || !valid_sync[1] || rdfifo_empty)display_bad<=1;
            end
        end
    end
    always @(posedge rd_clk or negedge display_rst_n)begin
        if(!display_rst_n)display_reset<=0;
        else begin
            display_reset<={display_reset[1:0],1'b1};
        end
    end
    wire tick=tick_sync[1]!=last_tick;
    wire return_word=app_rd_data_valid && outstanding!=0;
    rd_data_fifo display_fifo(.Data(app_rd_data),.Reset(!rst_n || read_fifo_reset),
        .WrClk(ui_clk),.RdClk(rd_clk),.WrEn(return_word && rs==R_RUN && !tick),.RdEn(fifo_read),
        .Wnum(read_count),.Rnum(),.Almost_Empty(),.Almost_Full(),
        .Q(pixel_data),.Empty(rdfifo_empty),.Full(read_full));

    // One registered command, at most 32 outstanding reads. Scheduling and
    // address formation happen before the ready handshake, not through it.
    reg command_valid,command_read;
    reg [27:0] command_address;
    wire command_allowed=command_valid && (command_read ? (rs==R_RUN && !tick) : ws==W_CAPTURE);
    assign o_request=command_allowed;
    assign app_en=command_allowed && app_rdy && (command_read || app_wdf_rdy);
    assign app_cmd=command_read ? 3'd1 : 3'd0;
    assign app_addr=command_address;
    assign app_wdf_data=write_data;
    assign app_wdf_wren=app_en && !command_read;
    assign app_wdf_end=app_wdf_wren;
    assign write_accept=app_wdf_wren;
    assign read_accept=app_en && command_read;
    wire [10:0] reserved_words={1'b0,read_count}+{5'd0,outstanding};
    wire need_read=rs==R_RUN && !tick && front_valid && read_index<BLOCKS &&
        outstanding<32 && reserved_words<READ_TARGET && !read_full;
    wire need_write=ws==W_CAPTURE && write_index<BLOCKS && !write_empty && write_count!=0;
    wire urgent_read=reserved_words<32;
    wire writer_publish=ws==W_FINISH && !capture_bad && write_index==BLOCKS;
    wire reader_switch=rs==R_DRAIN && !tick && blank_sync[1] && drain_guard==3 && outstanding==0;
    reg [1:0] free_bank;
    always @*begin
        if((!front_valid || front_bank!=0) && (!pending_valid || pending_bank!=0))free_bank=0;
        else if((!front_valid || front_bank!=1) && (!pending_valid || pending_bank!=1))free_bank=1;
        else free_bank=2;
    end
    function [2:0] bank_code;
        input [1:0] bank;
        begin case(bank)0:bank_code=2;1:bank_code=3;default:bank_code=4;endcase end
    endfunction
    always @(posedge ui_clk or negedge rst_n)begin
        if(!rst_n)begin done_sync<=0;tick_sync<=0;blank_sync<=0;end
        else begin
            done_sync<={done_sync[0],capture_done};tick_sync<={tick_sync[0],display_tick};
            blank_sync<={blank_sync[0],blank_window};
        end
    end
    always @(posedge ui_clk or negedge rst_n)begin
        if(!rst_n)begin
            ws<=W_ALLOC;rs<=R_WAIT;front_bank<=0;front_valid<=0;pending_bank<=0;pending_valid<=0;
            write_bank<=1;write_index<=0;read_index<=0;outstanding<=0;
            write_reset_count<=0;read_reset_count<=0;drain_guard<=0;
            write_fifo_reset<=1;read_fifo_reset<=1;arm_toggle<=0;last_done<=0;last_tick<=0;
            command_valid<=0;command_read<=0;command_address<=0;
        end else begin
            if(command_valid)begin
                if(app_en || (command_read && (tick || rs!=R_RUN)) || (!command_read && ws!=W_CAPTURE))command_valid<=0;
            end else if(init_calib_complete)begin
                if(need_read && (urgent_read || !need_write))begin
                    command_valid<=1;command_read<=1;
                    command_address<={5'd0,bank_code(front_bank),read_index,3'b000};
                end else if(need_write)begin
                    command_valid<=1;command_read<=0;
                    command_address<={5'd0,bank_code(write_bank),write_index,3'b000};
                end
            end
            if(write_accept)write_index<=write_index+1'b1;
            if(read_accept)read_index<=read_index+1'b1;
            case({read_accept,return_word})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:outstanding<=outstanding-1'b1;
                default:begin end
            endcase
            case(ws)
                W_ALLOC:if(init_calib_complete)begin
                    write_bank<=free_bank;write_index<=0;write_fifo_reset<=1;
                    write_reset_count<=0;ws<=W_RESET;
                end
                W_RESET:if(write_reset_count==15)begin
                    write_fifo_reset<=0;arm_toggle<=!arm_toggle;ws<=W_CAPTURE;
                end else write_reset_count<=write_reset_count+1'b1;
                W_CAPTURE:if(done_sync[1]!=last_done)begin
                    if(capture_bad || write_index==BLOCKS)begin
                        last_done<=done_sync[1];ws<=W_FINISH;
                    end
                end
                W_FINISH:ws<=W_ALLOC;
                default:ws<=W_ALLOC;
            endcase
            if(tick)begin last_tick<=tick_sync[1];rs<=R_DRAIN;drain_guard<=0;end
            else case(rs)
                R_DRAIN:if(drain_guard!=3)drain_guard<=drain_guard+1'b1;
                    else if(reader_switch)begin
                        read_fifo_reset<=1;read_reset_count<=0;read_index<=0;rs<=R_RESET;
                    end
                R_RESET:if(read_reset_count==15)begin read_fifo_reset<=0;rs<=R_RUN;end
                    else read_reset_count<=read_reset_count+1'b1;
                default:begin end
            endcase
            if(reader_switch && pending_valid)begin
                front_bank<=pending_bank;front_valid<=1;pending_valid<=0;
            end
            if(writer_publish)begin pending_bank<=write_bank;pending_valid<=1;end
        end
    end
    initial if(PIXELS<8 || PIXELS%8 || PIXELS>=1048576)$fatal(1,"Camera bank must contain complete blocks and fit its stride");
endmodule
