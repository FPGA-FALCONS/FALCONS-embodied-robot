// ============================================================================
// cam_overlay —— 摄像头画面叠加层（OV5640 → DDR3 帧缓存 → 显示像素二选一）
//
// 架构**照模块示例**：vendor\verilog-reusable-modules\modules\ov5640_capture\
//   examples\acg720_ddr3_hdmi\rtl\acg720_ov5640_ddr3_hdmi_top.v（该示例已在实板验证）
//   链路 = ov5640_capture → ddr3_ctrl_2port → HDMI 像素
//
// 本模块相对示例的**唯一必要偏离**：示例把 DDR 读出的像素直接当整屏输出；
// 本工程需求原文 §17/§21 要求「左侧约 1/5~1/6 宽度为菜单，选中摄像头页时
// **菜单右侧全部画面**显示摄像头画面」，所以我们在**输出端做二选一**：
//   o_rgb = (当前页==摄像头页 && pixel_x >= MENU_W) ? DDR像素 : UI像素
//
// ★★ 一个容易踩错的点：`rfifo_rden` **必须绑 pixel_req 全程连续读**。
//    不能改成「只读菜单右侧区域」—— DDR 读是顺序突发流，跳过若干点会让
//    读写地址累积错位，画面整片错行。
//
// 时钟：摄像头 PCLK 与显示 pixel_clk 异步，跨域由 ddr3_ctrl_2port 内部的读写 FIFO 完成
//       （这正是库文档强调的「必须用异步 FIFO 或 DDR 帧缓存」）。
// ============================================================================
module cam_overlay #(
    parameter integer IMAGE_WIDTH  = 1024,
    parameter integer IMAGE_HEIGHT = 600,
    parameter integer MENU_W       = 208,      // 与 ui_render 的菜单宽度一致
    parameter integer SYS_CLOCK    = 50_000_000
)(
    // ---- 系统 ----
    input  wire        sys_clk,          // 板载 50 MHz
    input  wire        sys_rst_n,

    // ---- 摄像头物理引脚（OV5640 DVP；模块自带 24 MHz 晶振，无 XCLK）----
    output wire        camera_sclk,
    inout  wire        camera_sdat,
    input  wire        camera_vsync,
    input  wire        camera_href,
    input  wire        camera_pclk,
    input  wire [7:0]  camera_data,
    output wire        camera_rst_n,
    output wire        camera_pwdn,

    // ---- DDR3 物理引脚 ----
    output wire [13:0] O_ddr_addr,
    output wire [2:0]  O_ddr_ba,
    output wire        O_ddr_cs_n,
    output wire        O_ddr_ras_n,
    output wire        O_ddr_cas_n,
    output wire        O_ddr_we_n,
    output wire        O_ddr_clk,
    output wire        O_ddr_clk_n,
    output wire        O_ddr_cke,
    output wire        O_ddr_odt,
    output wire        O_ddr_reset_n,
    output wire [1:0]  O_ddr_dqm,
    inout  wire [15:0] IO_ddr_dq,
    inout  wire [1:0]  IO_ddr_dqs,
    inout  wire [1:0]  IO_ddr_dqs_n,

    // ---- 显示侧（pixel_clk 域）----
    input  wire        pixel_clk,
    input  wire        pixel_req,
    input  wire [11:0] pixel_x,
    input  wire        frame_sync,       // 显示帧同步（每帧一次）
    input  wire        show_cam,         // 当前页 = 摄像头页
    input  wire [23:0] ui_rgb,           // ui_render 生成的像素
    output wire [23:0] o_rgb,            // 最终送给 hdmi_1024x600 的像素

    // ---- 状态（可接 LED）----
    output wire        cam_init_done,
    output wire        ddr_init_done,
    output wire        pll_lock
);

    localparam [27:0] APP_ADDR_MAX = IMAGE_WIDTH * IMAGE_HEIGHT;  // 614400
    localparam [7:0]  BURST_LEN    = IMAGE_WIDTH / 8;             // 128

    // ---------------------------------------------------------------
    // DDR3 需要 200 MHz + 50 MHz（照示例用同一颗 PLL，保证同源低抖动）
    // ---------------------------------------------------------------
    wire clk_200m;
    wire clk_50m_pll;
    wire camera_pixel_clk;
    wire camera_frame_sync;
    wire camera_pixel_valid;
    wire [15:0] camera_pixel_rgb565;
    wire [15:0] ddr_pixel_rgb565;
    wire cam_rst_n = sys_rst_n & pll_lock;

    Gowin_PLL u_system_pll (
        .lock    (pll_lock),
        .clkout0 (clk_200m),
        .clkout1 (clk_50m_pll),
        .clkin   (sys_clk),
        .reset   (~sys_rst_n)
    );

    // ---------------------------------------------------------------
    // 摄像头采集（模块库 ov5640_capture）
    // ---------------------------------------------------------------
    ov5640_capture #(
        .IMAGE_WIDTH  (IMAGE_WIDTH),
        .IMAGE_HEIGHT (IMAGE_HEIGHT),
        .SYS_CLOCK    (SYS_CLOCK)
    ) u_camera (
        .cfg_clk       (clk_50m_pll),
        .reset_n       (cam_rst_n),
        .camera_sclk   (camera_sclk),
        .camera_sdat   (camera_sdat),
        .camera_vsync  (camera_vsync),
        .camera_href   (camera_href),
        .camera_pclk   (camera_pclk),
        .camera_data   (camera_data),
        .camera_rst_n  (camera_rst_n),
        .camera_pwdn   (camera_pwdn),
        .init_done     (cam_init_done),
        .init_error    (),
        .pixel_clk     (camera_pixel_clk),
        .capture_clear (),
        .pixel_valid   (camera_pixel_valid),
        .pixel_rgb565  (camera_pixel_rgb565),
        .line_valid    (),
        .frame_sync    (camera_frame_sync),
        .pixel_x       (),
        .pixel_y       ()
    );

    // ---------------------------------------------------------------
    // DDR3 帧缓存（模块示例的 ddr3_ctrl_2port）
    //   写侧：摄像头 PCLK 域，按 camera_frame_sync 对齐帧首
    //   读侧：显示 pixel_clk 域，按 frame_sync 对齐帧首，rden 绑 pixel_req
    // ---------------------------------------------------------------
    ddr3_ctrl_2port u_frame_buffer (
        .clk                 (clk_50m_pll),
        .pll_lock            (pll_lock),
        .clk_200m            (clk_200m),
        .sys_rst_n           (sys_rst_n),
        .init_calib_complete (ddr_init_done),

        .rd_load             (frame_sync),
        .wr_load             (camera_frame_sync),
        .app_addr_rd_min     (28'd0),
        .app_addr_rd_max     (APP_ADDR_MAX),
        .rd_bust_len         (BURST_LEN),
        .app_addr_wr_min     (28'd0),
        .app_addr_wr_max     (APP_ADDR_MAX),
        .wr_bust_len         (BURST_LEN),

        .wr_clk              (camera_pixel_clk),
        .wfifo_wren          (camera_pixel_valid),
        .wfifo_din           (camera_pixel_rgb565),
        .rd_clk              (pixel_clk),
        .rfifo_rden          (pixel_req),      // ★ 必须全程连续读，不能只读显示区
        .rfifo_dout          (ddr_pixel_rgb565),

        .ddr3_dq             (IO_ddr_dq),
        .ddr3_dqs_n          (IO_ddr_dqs_n),
        .ddr3_dqs_p          (IO_ddr_dqs),
        .ddr3_addr           (O_ddr_addr),
        .ddr3_ba             (O_ddr_ba),
        .ddr3_ras_n          (O_ddr_ras_n),
        .ddr3_cas_n          (O_ddr_cas_n),
        .ddr3_we_n           (O_ddr_we_n),
        .ddr3_reset_n        (O_ddr_reset_n),
        .ddr3_ck_p           (O_ddr_clk),
        .ddr3_ck_n           (O_ddr_clk_n),
        .ddr3_cke            (O_ddr_cke),
        .ddr3_cs_n           (O_ddr_cs_n),
        .ddr3_dm             (O_ddr_dqm),
        .ddr3_odt            (O_ddr_odt)
    );

    // ---------------------------------------------------------------
    // RGB565 → RGB888（照示例的高位对齐扩展）
    // ---------------------------------------------------------------
    wire [23:0] cam_rgb888 = {
        ddr_pixel_rgb565[15:11], 3'b000,
        ddr_pixel_rgb565[10:5],  2'b00,
        ddr_pixel_rgb565[4:0],   3'b000
    };

    // ---------------------------------------------------------------
    // 输出二选一：摄像头页且落在菜单右侧 → 摄像头画面；否则 → UI
    // ---------------------------------------------------------------
    wire cam_region = show_cam && (pixel_x >= MENU_W);
    assign o_rgb = cam_region ? cam_rgb888 : ui_rgb;

endmodule
