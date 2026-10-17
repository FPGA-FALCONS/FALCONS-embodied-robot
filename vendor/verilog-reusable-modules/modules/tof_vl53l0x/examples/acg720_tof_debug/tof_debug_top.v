// ACG720 physical ToF-to-seven-segment bring-up, with no simulated data source.
// All three ToF pins are open-drain through the real controller.
module tof_debug_top #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=100000,
    parameter integer I2C_TIMEOUT_US=2000,
    parameter integer BOOT_DELAY_US=10000,
    parameter integer RESET_LOW_US=1000,
    parameter integer POLL_INTERVAL_US=1000,
    parameter integer INIT_TIMEOUT_US=100000,
    parameter integer MEAS_TIMEOUT_US=100000,
    parameter integer SAMPLE_PERIOD_US=50000,
    parameter integer STALE_TIMEOUT_US=200000,
    parameter integer DIGIT_US=500,
    parameter integer SCLK_DIVIDER=25,
    parameter integer HEARTBEAT_US=500000
) (
    input wire clk,
    input wire rst_n,
    inout wire tof_scl,
    inout wire tof_sda,
    inout wire tof_xshut_n,
    output wire ds,
    output wire sh_cp,
    output wire st_cp,
    output wire [7:0] led
);
    reg reset_meta,reset_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin reset_meta<=1'b0; reset_sync<=1'b0; end
        else begin reset_meta<=1'b1; reset_sync<=reset_meta; end
    end
    wire [15:0] distance_mm;
    wire range_valid,sample_pulse,initialized,busy;
    wire [3:0] unused_raw_range_status;
    wire [4:0] fault_code;
    tof_vl53l0x #(
        .CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.I2C_TIMEOUT_US(I2C_TIMEOUT_US),
        .BOOT_DELAY_US(BOOT_DELAY_US),.RESET_LOW_US(RESET_LOW_US),
        .POLL_INTERVAL_US(POLL_INTERVAL_US),.INIT_TIMEOUT_US(INIT_TIMEOUT_US),
        .MEAS_TIMEOUT_US(MEAS_TIMEOUT_US),.SAMPLE_PERIOD_US(SAMPLE_PERIOD_US),
        .STALE_TIMEOUT_US(STALE_TIMEOUT_US)
    ) sensor (
        .clk(clk),.rst_n(rst_n),.retry(1'b0),
        .scl(tof_scl),.sda(tof_sda),.xshut_n(tof_xshut_n),
        .distance_mm(distance_mm),.range_valid(range_valid),
        .sample_pulse(sample_pulse),.initialized(initialized),.busy(busy),
        .raw_range_status(unused_raw_range_status),.fault_code(fault_code)
    );
    wire [63:0] glyphs;
    wire warning,stop;
    tof_display_formatter formatter (
        .clk(clk),.rst_n(reset_sync),.distance_mm(distance_mm),
        .range_valid(range_valid),.initialized(initialized),.fault_code(fault_code),
        .glyphs(glyphs),.warning(warning),.stop(stop)
    );
    tof_hc595_display #(.CLK_HZ(CLK_HZ),.DIGIT_US(DIGIT_US),
        .SCLK_DIVIDER(SCLK_DIVIDER)) display (
        .clk(clk),.rst_n(reset_sync),.glyphs(glyphs),
        .ds(ds),.sh_cp(sh_cp),.st_cp(st_cp)
    );
    function integer counter_width;
        input integer maximum;
        integer remainder;
        begin
            remainder=maximum; counter_width=0;
            while(remainder>0) begin
                counter_width=counter_width+1; remainder=remainder/2;
            end
            if(counter_width<1) counter_width=1;
        end
    endfunction
    localparam integer HEARTBEAT_CYCLES=(CLK_HZ/1000000)*HEARTBEAT_US;
    localparam integer HEARTBEAT_BITS=counter_width(HEARTBEAT_CYCLES-1);
    localparam [HEARTBEAT_BITS-1:0] HEARTBEAT_LAST=
        HEARTBEAT_CYCLES[HEARTBEAT_BITS-1:0]-1'b1;
    reg [HEARTBEAT_BITS-1:0] heartbeat_counter;
    reg heartbeat,sample_toggle;
    always @(posedge clk or negedge reset_sync) begin
        if(!reset_sync) begin
            heartbeat_counter<={HEARTBEAT_BITS{1'b0}};
            heartbeat<=1'b0; sample_toggle<=1'b0;
        end else begin
            if(sample_pulse) sample_toggle<=~sample_toggle;
            if(heartbeat_counter==HEARTBEAT_LAST) begin
                heartbeat_counter<={HEARTBEAT_BITS{1'b0}};
                heartbeat<=~heartbeat;
            end else heartbeat_counter<=heartbeat_counter+1'b1;
        end
    end
    assign led={heartbeat,(fault_code!=5'd0),sample_toggle,busy,
        stop,warning,range_valid,initialized};
endmodule
