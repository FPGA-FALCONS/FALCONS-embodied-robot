// S1/S2 active-low keys: synchronize, debounce, and act once per press.
// Apply requested scale only at a video frame boundary. Clearing the plot
// leaves UART reception, packet queues, diagnostics and video timing running.
module zoom_control #(
    parameter integer DEBOUNCE_CLKS = 980000
) (
    input wire clk,
    input wire reset_n,
    input wire zoom_in_n,
    input wire zoom_out_n,
    input wire frame_start,
    output reg [1:0] zoom_level,
    output reg plot_clear
);
    function integer counter_width;
        input integer value;
        integer remainder;
        begin
            counter_width = 1;
            remainder = value - 1;
            while (remainder > 1) begin
                counter_width = counter_width + 1;
                remainder = remainder >> 1;
            end
        end
    endfunction
    localparam integer COUNT_WIDTH = counter_width(DEBOUNCE_CLKS);
    reg [1:0] key_meta, key_sync, stable_keys, press_pulse;
    reg [COUNT_WIDTH-1:0] stable_count [0:1];
    reg [1:0] requested_zoom;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            key_meta <= 2'b11;
            key_sync <= 2'b11;
        end else begin
            key_meta <= {zoom_out_n, zoom_in_n};
            key_sync <= key_meta;
        end
    end
    genvar key;
    generate for (key = 0; key < 2; key = key + 1) begin: debounce
        always @(posedge clk or negedge reset_n) begin
            if (!reset_n) begin
                stable_keys[key] <= 1;
                stable_count[key] <= 0;
                press_pulse[key] <= 0;
            end else begin
                press_pulse[key] <= 0;
                if (key_sync[key] == stable_keys[key])
                    stable_count[key] <= 0;
                else if (stable_count[key] == DEBOUNCE_CLKS - 1) begin
                    stable_keys[key] <= key_sync[key];
                    stable_count[key] <= 0;
                    press_pulse[key] <= !key_sync[key];
                end else
                    stable_count[key] <= stable_count[key] + 1'b1;
            end
        end
    end endgenerate
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            requested_zoom <= 2;
            zoom_level <= 2;
            plot_clear <= 0;
        end else begin
            plot_clear <= 0;
            // Ignore simultaneous presses and a press while the other key is
            // held down. Limits are clamped; holding a key does not repeat.
            if (press_pulse == 2'b01 && stable_keys[1]) begin
                if (requested_zoom != 0) requested_zoom <= requested_zoom - 1'b1;
            end else if (press_pulse == 2'b10 && stable_keys[0]) begin
                if (requested_zoom != 3) requested_zoom <= requested_zoom + 1'b1;
            end
            if (frame_start && requested_zoom != zoom_level) begin
                zoom_level <= requested_zoom;
                plot_clear <= 1;
            end
        end
    end
endmodule
