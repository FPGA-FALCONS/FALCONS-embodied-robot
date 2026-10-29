`timescale 1ns/1ps
module inspection_dispatch_tb;
    reg clk=0,rst=0,message=0;reg [7:0] kind=0;reg [5:0] count=0;
    reg force_not_ready=0;
    reg [31:0] memory[0:31],read_data;
    wire consumed,read_word;wire [4:0] read_index;
    wire hello,control,heartbeat,begin_layer,commit_layer,word_valid,error;
    wire [31:0] session,task_id,epoch,command,flags,gx,gy,layer_speed,crc,word;
    wire [12:0] index;wire [15:0] version,published_version;
    wire layer_ready,layer_valid,committed,layer_error,busy;
    integer checks=0,errors=0,hello_count=0,control_count=0,error_count=0,commits=0,i;
    always #10 clk=~clk;
    always @(posedge clk)begin
        if(read_word)read_data<=memory[read_index];
        if(hello)hello_count=hello_count+1;
        if(control)control_count=control_count+1;
        if(error)error_count=error_count+1;
        if(committed)commits=commits+1;
    end
    inspection_dispatch #(.TIMEOUT_CYCLES(1000)) dispatch(
        .clk(clk),.rst_n(rst),.i_valid(message),.i_kind(kind),.i_count(count),.o_ready(consumed),
        .o_read(read_word),.o_read_index(read_index),.i_word(read_data),
        .i_current_session(32'd5),.i_current_epoch(32'd1),.i_current_task(32'd1),.i_current_command(32'd1),
        .i_current_layer(published_version),.i_upload_allowed(1'b1),.i_layer_ready(layer_ready && !force_not_ready),.i_layer_busy(busy),
        .o_hello(hello),.o_control(control),.o_heartbeat(heartbeat),.o_begin(begin_layer),.o_commit(commit_layer),.o_word_valid(word_valid),
        .o_session(session),.o_task(task_id),.o_epoch(epoch),.o_command(command),.o_flags(flags),.o_goal_x(gx),.o_goal_y(gy),
        .o_layer_speed(layer_speed),.o_expected_crc(crc),.o_layer_word(word),.o_word_index(index),.o_layer_version(version),.o_error(error));
    nav_visual_layer #(.CELL_BITS(6)) layer(
        .clk(clk),.reset_n(rst),.i_enable(1'b1),.i_epoch(32'd1),.i_begin(begin_layer),.i_abort(1'b0),.i_commit(commit_layer),
        .i_begin_epoch(epoch),.i_expected_crc(crc),.i_begin_version(version),.i_stopped(1'b1),.i_read_inflight(1'b0),
        .i_word_valid(word_valid),.i_word_index(index[2:0]),.i_word(word),.o_word_ready(layer_ready),.o_layer_valid(layer_valid),
        .o_version(published_version),.o_committed(committed),.o_error(layer_error),.o_error_code(),.o_read_blocked(busy),
        .i_read(1'b0),.i_read_address(6'd0),.o_raw(),.o_cost());
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send;input [7:0] t;input [5:0] n;
        begin @(negedge clk);kind=t;count=n;message=1;wait(consumed);@(negedge clk);message=0;repeat(3)@(negedge clk);end
    endtask
    initial begin
        for(i=0;i<32;i=i+1)memory[i]=0;
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        memory[0]=5;send(1,1);check(hello_count==1 && session==5,"hello decoded after synchronous word read");
        memory[1]=1;memory[2]=1;memory[3]=2;memory[4]=32'h00020101;memory[5]=256;memory[6]=128;memory[7]=32'h001e0001;
        send(2,8);check(control_count==1 && flags[7:0]==8'hff && error_count==1,"invalid goal generates rejection instead of wrapped cell");
        memory[4]=1;memory[5]=32'h915e08be;send(3,6);
        check(layer_ready && version==1 && epoch==1,"authenticated layer begin reaches RAM writer");
        memory[0]=5;memory[1]=1;memory[2]=1;memory[3]=0;
        memory[4]=32'h00000400;memory[5]=0;memory[6]=32'h00100400;memory[7]=0;send(4,8);
        memory[0]=5;memory[1]=1;memory[2]=1;memory[3]=2;memory[4]=1;send(5,5);
        repeat(10)@(negedge clk);
        check(layer_valid && published_version==1 && commits==1,"streamed words and delayed commit retain exact CRC");
        memory[0]=5;memory[1]=1;memory[2]=1;send(4,3);
        check(error_count==2,"short data header refused without out of range reads");
        memory[0]=5;memory[1]=1;memory[2]=2;memory[3]=3;memory[4]=2;memory[5]=0;send(3,6);
        check(error_count==3 && published_version==1,"wrong epoch cannot begin replacement layer");
        memory[2]=1;send(3,6);force_not_ready=1;
        memory[0]=5;memory[1]=1;memory[2]=2;memory[3]=0;memory[4]=0;send(4,5);
        check(error_count==4 && published_version==1,"stalled writer times out and releases packet without publishing");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_dispatch %0d checks",checks);else $display("FAIL: inspection_dispatch %0d",errors);
        $finish;
    end
    initial begin #1000000;$display("FAIL: inspection_dispatch timeout");$finish;end
endmodule
