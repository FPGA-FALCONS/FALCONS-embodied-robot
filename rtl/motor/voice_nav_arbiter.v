`timescale 1ns/1ps
// Select by ownership, not by the current action's valid flag. A safety
// gate may suppress voice_valid, but that must never expose stale navigation.
module voice_nav_arbiter (
    input wire i_voice_owned,
    input wire i_voice_valid,
    input wire signed [31:0] i_voice_center,
    input wire signed [31:0] i_voice_diff,
    input wire i_nav_valid,
    input wire signed [31:0] i_nav_center,
    input wire signed [31:0] i_nav_diff,
    output wire o_valid,
    output wire signed [31:0] o_center,
    output wire signed [31:0] o_diff
);
    assign o_valid = i_voice_owned ? i_voice_valid : i_nav_valid;
    assign o_center = i_voice_owned ? (i_voice_valid ? i_voice_center : 32'sd0) : i_nav_center;
    assign o_diff = i_voice_owned ? (i_voice_valid ? i_voice_diff : 32'sd0) : i_nav_diff;
endmodule
