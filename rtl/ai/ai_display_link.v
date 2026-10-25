// CRC checked, ordered double-buffer display updates. UI frames never drive motors.
// The existing router isolates all MOT1 envelopes from the ASCII tuning parser.
module ai_display_link #(
    parameter integer CLK_HZ=50_000_000
)(
    input wire clk,rst_n,
    input wire [7:0] rx_data,
    input wire rx_valid,
    input wire pixel_clk,pixel_rst_n,
    input wire [11:0] pixel_x,pixel_y,
    input wire [1:0] selected,
    input wire i_inspection,
    output wire o_glyph,
    output wire o_connected,o_local_region,o_highlight,
    output reg [7:0] o_display_ack,
    output wire o_display_bank,o_host_alive
);
    localparam integer CELLS=828, BYTE_TIMEOUT=CLK_HZ/50, LINK_TIMEOUT=CLK_HZ*3;
    reg [3:0] pos;
    reg [7:0] frame[0:10];
    reg [15:0] crc;
    reg [31:0] gap,age;
    reg staging,stage_bank,published_bank;
    reg [7:0] transaction;
    reg [10:0] expected,written,published_count;
    reg [8:0] mem[0:2047];
    reg [15:0] font_mem[0:16383];
    reg [511:0] font_valid0,font_valid1;
    reg font_upload;
    reg [8:0] font_slot;
    reg [4:0] font_next_row;
    reg [7:0] published_sequence;
    reg bank_ack1,bank_ack2;
    reg read_bank;
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] value;integer n;reg [15:0] c;
        begin c=old^{value,8'd0};for(n=0;n<8;n=n+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
    endfunction
    function [7:0] magic;
        input [3:0] n;
        begin case(n) 0:magic=8'h4d;1:magic=8'h4f;2:magic=8'h54;default:magic=8'h31;endcase end
    endfunction
    wire [15:0] value={frame[6],frame[7]};
    wire [9:0] address={frame[9][1:0],frame[8]};
    wire [8:0] font_address={frame[9][4],frame[8]};
    wire glyph_ready=stage_bank?font_valid1[value[8:0]]:font_valid0[value[8:0]];
    assign o_display_bank=bank_ack2;
    assign o_host_alive=age<(CLK_HZ*3/4);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pos<=0;crc<=16'hffff;gap<=0;age<=LINK_TIMEOUT;
            staging<=0;stage_bank<=1;published_bank<=0;transaction<=0;
            expected<=0;written<=0;published_count<=0;bank_ack1<=0;bank_ack2<=0;
            font_valid0<=0;font_valid1<=0;font_upload<=0;font_slot<=0;font_next_row<=0;
            published_sequence<=0;o_display_ack<=0;
        end else begin
            bank_ack1<=read_bank;bank_ack2<=bank_ack1;
            if(bank_ack2==published_bank)o_display_ack<=published_sequence;
            if(age<LINK_TIMEOUT) age<=age+1'b1;
            if(pos!=0) begin
                if(gap>=BYTE_TIMEOUT-1) begin pos<=0;crc<=16'hffff;gap<=0;staging<=0;end
                else gap<=gap+1'b1;
            end
            if(rx_valid) begin
                gap<=0;
                if(pos<4 && rx_data!=magic(pos)) begin
                    if(rx_data==8'h4d) begin pos<=1;crc<=crc_byte(16'hffff,rx_data);frame[0]<=rx_data;end
                    else begin pos<=0;crc<=16'hffff;end
                end else if(pos==11) begin
                    pos<=0;crc<=16'hffff;
                    if(crc=={frame[10],rx_data}) begin
                        case(frame[5])
                            8'h40: begin
                                age<=0;
                                staging<=0;
                                if(value<=CELLS && published_bank==bank_ack2) begin
                                    staging<=1;stage_bank<=~published_bank;transaction<=frame[4];
                                    expected<=value[10:0];written<=0;font_upload<=0;
                                end
                            end
                            8'h41: begin
                                age<=0;
                                if(staging && frame[4]==transaction && frame[9]<4 &&
                                   {1'b0,address}==written && written<expected && value<512 && glyph_ready) begin
                                    mem[{stage_bank,address}]<=value[8:0];written<=written+1'b1;
                                end else staging<=0;
                            end
                            8'h42: begin
                                age<=0;
                                if(staging && frame[4]==transaction && written==expected && value==expected) begin
                                    published_bank<=stage_bank;published_count<=written;
                                    published_sequence<=frame[4];
                                end
                                staging<=0;
                            end
                            8'h43: age<=0;
                            8'h44: begin
                                age<=0;
                                if(staging && frame[4]==transaction && written==0 && frame[9][7:5]==0) begin
                                    if(frame[9][3:0]==0) begin
                                        font_upload<=1;font_slot<=font_address;font_next_row<=1;
                                        if(stage_bank)font_valid1[font_address]<=0;else font_valid0[font_address]<=0;
                                        font_mem[{stage_bank,font_address,4'd0}]<=value;
                                    end else if(font_upload && font_slot==font_address && font_next_row=={1'b0,frame[9][3:0]}) begin
                                        font_mem[{stage_bank,font_address,frame[9][3:0]}]<=value;
                                        font_next_row<=font_next_row+1'b1;
                                        if(frame[9][3:0]==15) begin
                                            font_upload<=0;
                                            if(stage_bank)font_valid1[font_address]<=1;else font_valid0[font_address]<=1;
                                        end
                                    end else staging<=0;
                                end else staging<=0;
                            end
                            default: ;
                        endcase
                    end else if(frame[5]>=8'h40 && frame[5]<=8'h44) staging<=0;
                end else begin
                    frame[pos]<=rx_data;
                    if(pos<10) crc<=pos==0?crc_byte(16'hffff,rx_data):crc_byte(crc,rx_data);
                    pos<=pos+1'b1;
                end
            end
        end
    end
    reg bank_p1,bank_p2,alive_p1,alive_p2;
    reg [10:0] count_p1,count_p2,read_count;
    reg [1:0] selected_p1,selected_p2;
    reg task_p1,task_p2;
    // Commit only at frame blanking; acknowledge before reusing the old bank.
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            bank_p1<=0;bank_p2<=0;read_bank<=0;count_p1<=0;count_p2<=0;read_count<=0;
            alive_p1<=0;alive_p2<=0;
            selected_p1<=0;selected_p2<=0;
            task_p1<=0;task_p2<=0;
        end else begin
            bank_p1<=published_bank;bank_p2<=bank_p1;
            count_p1<=published_count;count_p2<=count_p1;
            alive_p1<=age<LINK_TIMEOUT;alive_p2<=alive_p1;
            selected_p1<=selected;selected_p2<=selected_p1;
            task_p1<=i_inspection;task_p2<=task_p1;
            if(pixel_x==0 && pixel_y==0 && bank_p2!=read_bank) begin
                read_bank<=bank_p2;read_count<=count_p2;
            end
        end
    end
    assign o_connected=alive_p2;
    wire [11:0] look_x=pixel_x+12'd3;
    wire [11:0] x=look_x-12'd240,y=pixel_y-12'd128;
    reg row_hit;
    reg [9:0] row_base;
    reg [3:0] row_index;
    integer r;
    always @* begin
        row_hit=0;row_base=0;row_index=0;
        for(r=0;r<18;r=r+1)begin
            if(pixel_y>=128+r*20 && pixel_y<144+r*20)begin
                row_hit=1;row_base=r*46;row_index=pixel_y-(128+r*20);
            end
        end
    end
    reg [9:0] base_p1;
    reg [5:0] col_p1;
    reg [4:0] row_p1;
    reg [4:0] bit_p1,bit_q,bit_d;
    reg inside_p1,local_p1;
    reg [8:0] menu_p1;
    wire [9:0] char_addr=base_p1+{4'd0,col_p1};
    reg [8:0] char_q;
    reg [4:0] row_q;
    reg valid_q,valid_d,local_q,local_d;
    reg menu_hit;
    reg [5:0] menu_index;
    reg [4:0] menu_col;
    integer m,h;
    // 216x44 boxes centered at x=340/580/820, y=86. Four/four/five
    // 24px cells start at x=292/532/760 and y=74, independent of host text.
    always @* begin
        menu_hit=0;menu_index=13;menu_col=0;
        if(pixel_y>=74 && pixel_y<98)begin
            for(m=0;m<5;m=m+1)begin
                if(m<(task_p2?3:4) && look_x>=(task_p2?304:292)+m*24 && look_x<(task_p2?328:316)+m*24)begin
                    menu_hit=1;menu_index=m+(task_p2?14:0);menu_col=look_x-((task_p2?304:292)+m*24);
                end
                if(m<4 && look_x>=532+m*24 && look_x<556+m*24)begin
                    menu_hit=1;menu_index=m+(task_p2?17:4);menu_col=look_x-(532+m*24);
                end
                if(m<(task_p2?4:5) && look_x>=(task_p2?772:760)+m*24 && look_x<(task_p2?796:784)+m*24)begin
                    menu_hit=1;menu_index=m+(task_p2?21:8);menu_col=look_x-((task_p2?772:760)+m*24);
                end
            end
        end
        // Local breadcrumb identifies a recognition child page; the original
        // three feature labels remain on the AI overview. This needs no RK.
        if(task_p2 && pixel_y>=24 && pixel_y<48)begin
            for(h=0;h<9;h=h+1)begin
                if(look_x>=440+h*24 && look_x<464+h*24)begin
                    menu_hit=1;menu_index=25+h;menu_col=look_x-(440+h*24);
                end
                if(h<5 && look_x>=832+h*24 && look_x<856+h*24)begin
                    menu_hit=1;menu_index=34+h;menu_col=look_x-(832+h*24);
                end
            end
        end
    end
    always @(posedge pixel_clk) begin
        base_p1<=row_base;col_p1<=x[9:4];row_p1<=menu_hit?(pixel_y<60?pixel_y-12'd24:pixel_y-12'd74):{1'b0,row_index};
        bit_p1<=23-menu_col;bit_q<=bit_p1;bit_d<=bit_q;
        inside_p1<=menu_hit || (look_x>=240 && look_x<976 && row_hit);
        local_p1<=menu_hit;menu_p1<={3'd0,menu_index};
        char_q<=local_p1?menu_p1:mem[{read_bank,char_addr}];row_q<=row_p1;
        valid_q<=inside_p1 && (local_p1 || {1'b0,char_addr}<read_count);valid_d<=valid_q;
        local_q<=local_p1;local_d<=local_q;
    end
    reg [15:0] dynamic_bits;
    always @(posedge pixel_clk)dynamic_bits<=font_mem[{read_bank,char_q,row_q[3:0]}];
    wire [23:0] menu_bits;
    wire [9:0] menu_addr={char_q[5:0],4'd0}+{1'b0,char_q[5:0],3'd0}+{5'd0,row_q};
    ai_menu_font font(.clk(pixel_clk),.addr(menu_addr),.bits(menu_bits));
    wire [11:0] selected_left=12'd232+selected_p2*12'd240;
    assign o_highlight=pixel_x>=selected_left && pixel_x<selected_left+216 && pixel_y>=64 && pixel_y<108;
    assign o_local_region=(pixel_x>=208 && pixel_y>=60 && pixel_y<112) ||
        (task_p2 && pixel_x>=432 && pixel_x<976 && pixel_y>=16 && pixel_y<52);
    assign o_glyph=valid_d && (local_d?menu_bits[bit_d]:(o_connected && dynamic_bits[15-pixel_x[3:0]]));
endmodule
