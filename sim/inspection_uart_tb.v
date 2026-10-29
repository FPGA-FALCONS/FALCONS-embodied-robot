`timescale 1ns/1ps
module inspection_uart_tb;
    reg clk=0,rst=0,start=0,csv_valid=0;reg [7:0] csv_data=0;
    wire busy,ins_valid,ins_last,ins_ready,csv_ready,uart,rx_valid;
    wire [7:0] ins_data,rx_data,kind,seq;
    wire msg_valid,error;wire [5:0] word_count;
    reg [7:0] received[0:255];integer received_count=0,checks=0,errors=0,i,csv_position=-1,begin_position=-1,end_position=-1;
    always #500 clk=~clk;
    inspection_packet_tx source(.clk(clk),.rst_n(rst),.i_start(start),.i_kind(8'h81),.i_count(4'd8),
        .i_words({32'd8,32'd7,32'd6,32'd5,32'd4,32'd3,32'd2,32'd1}),.o_busy(busy),
        .o_data(ins_data),.o_valid(ins_valid),.o_last(ins_last),.i_ready(ins_ready));
    ai_uart_tx #(.CLK_HZ(1_000_000),.BAUD_RATE(100_000),.REPORT_CYCLES(100000),.INSPECTION_ENABLE(1)) mux(
        .clk(clk),.rst_n(rst),.i_csv_data(csv_data),.i_csv_valid(csv_valid),.o_csv_ready(csv_ready),
        .i_flags(16'd19),.i_controls(8'd18),.i_ai_link_active(1'b1),
        .i_inspection_data(ins_data),.i_inspection_valid(ins_valid),.i_inspection_last(ins_last),.o_inspection_ready(ins_ready),.o_uart_tx(uart));
    uart_byte_rx #(.CLK_FREQ_HZ(1_000_000),.BAUD_RATE(100_000),.RECOVERY_EN(0)) rx(
        .i_clock(clk),.i_reset_n(rst),.i_uart_rx(uart),.o_rx_data(rx_data),.o_rx_valid(rx_valid),.i_rx_ready(1'b1),.o_rx_error());
    inspection_packet_rx #(.CLK_HZ(1_000_000)) parse(.clk(clk),.rst_n(rst),.i_data(rx_data),.i_valid(rx_valid),.o_ready(),
        .o_message_valid(msg_valid),.i_message_ready(1'b0),.o_kind(kind),.o_sequence(seq),.o_count(word_count),
        .i_word_read(1'b0),.i_word_index(5'd0),.o_word(),.o_error(error));
    always @(posedge clk)if(rx_valid && received_count<256)begin received[received_count]=rx_data;received_count=received_count+1;end
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send_csv;input [7:0] value;
        begin @(negedge clk);csv_data=value;csv_valid=1;@(posedge clk);while(!csv_ready)@(posedge clk);@(negedge clk);csv_valid=0;end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst=1;
        fork
            begin send_csv("x");repeat(30)@(negedge clk);send_csv("y");send_csv(10);end
            begin repeat(20)@(negedge clk);start=1;@(negedge clk);start=0;end
        join
        wait(msg_valid);repeat(1000)@(negedge clk);
        check(kind==8'h81 && word_count==8,"physical UART delivers complete CRC checked inspection transaction");
        for(i=0;i<received_count-2;i=i+1)begin
            if(received[i]=="x" && received[i+1]=="y" && received[i+2]==10)csv_position=i;
            if(i+5<received_count && received[i]==8'h4d && received[i+1]==8'h4f && received[i+5]==8'h60)begin_position=i;
            if(i+5<received_count && received[i]==8'h4d && received[i+1]==8'h4f && received[i+5]==8'h62)end_position=i;
        end
        check(csv_position>=0,"CSV line remains contiguous under inspection contention");
        check(begin_position>=0 && end_position-begin_position==108,"no control or CSV byte splits inspection transaction");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_uart %0d checks",checks);else $display("FAIL: inspection_uart %0d",errors);
        $finish;
    end
    initial begin #50000000;$display("FAIL: inspection_uart timeout");$finish;end
endmodule
