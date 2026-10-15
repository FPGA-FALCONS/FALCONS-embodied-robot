//////////////////////////////////////////////////////////////////////////////////
// 模块：dyn_text_ram —— 动态文本字符缓冲（sys 域写 / pixel 域读）
//
// 【为什么需要它】
//   界面上的"实时数值"（四轮转速、中心速度…）不能像菜单文案那样预先烧进 ROM，
//   必须在运行中生成。生成在 sys 域（50 MHz，与控制器同域），渲染在 pixel 域
//   （49 MHz，HDMI 像素时钟），所以这里要做一次跨域传递。
//
// 【跨域方式】简单双口 RAM：一个写端口接 sys 域，一个读端口接 pixel 域
//   —— Gowin 的 BSRAM 原生支持这种"一写一读、两套时钟"的模式。
//   读侧不做同步器：字符是"显示用"数据，某一帧上出现新旧混排（撕裂）在视觉上
//   可以接受（数值更新周期 100 ms，远慢于 60 Hz 帧率）；这比加握手省得多。
//   若将来要严格无撕裂，改成双缓冲 + 一个"提交指针"同步位即可（接口不用变）。
//
// 【数据格式】每个字节 = 字库里的字符索引
//   字库前 95 项就是 ASCII 0x20~0x7E，故索引 = ASCII码 − 32：
//      空格 = 0   '-' = 13   '0'~'9' = 16~25
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module dyn_text_ram #(
    parameter integer DEPTH     = 64,
    parameter integer ADDR_BITS = 6
)(
    input  wire                  i_clk_w,        // 写时钟（sys 域）
    input  wire                  i_we,
    input  wire [ADDR_BITS-1:0]  i_waddr,
    input  wire [7:0]            i_wdata,

    input  wire                  i_clk_r,        // 读时钟（pixel 域）
    input  wire [ADDR_BITS-1:0]  i_raddr,
    output reg  [7:0]            o_rdata
);

    reg [7:0] mem [0:DEPTH-1];

    // 写端口（sys 域）
    always @(posedge i_clk_w) begin
        if (i_we) mem[i_waddr] <= i_wdata;
    end

    // 读端口（pixel 域）
    always @(posedge i_clk_r) begin
        o_rdata <= mem[i_raddr];
    end

endmodule
