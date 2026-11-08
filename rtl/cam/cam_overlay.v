// ============================================================================
// cam_overlay —— 摄像头画面叠加层（OV5640 → DDR3 帧缓存 → 显示像素二选一）
//
// 采集仍使用模块库 ov5640_capture；DDR 控制器内由 camera_frame_buffer
// 管理三块互斥帧区。完整帧写完后发布，在显示垂直空白期切换读帧，
// 避免异步采集/显示读写同一帧造成运动画面撕裂。AI 保留独立双缓冲。
// 输出端按菜单区域叠加，摄像头画面仍保持原有 180° 旋转。
//
// ★★ 一个容易踩错的点：`rfifo_rden` **必须绑 pixel_req 全程连续读**。
//    不能改成「只读菜单右侧区域」—— DDR 读是顺序突发流，跳过若干点会让
//    读写地址累积错位，画面整片错行。
//
// 时钟：摄像头 PCLK 与显示 pixel_clk 异步，跨域由 ddr3_ctrl_2port 内部的读写 FIFO 完成
//       （这正是库文档强调的「必须用异步 FIFO 或 DDR 帧缓存」）。
// ============================================================================
module cam_overlay #(
    parameter integer AI_DISPLAY_ENABLE=0,
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
    input  wire [11:0] pixel_y,
    input  wire        frame_sync,       // 显示帧同步（每帧一次）
    input  wire        show_cam,         // 当前页 = 摄像头页
    input  wire [23:0] ui_rgb,           // ui_render 生成的像素
    output wire [23:0] o_rgb,            // 最终送给 hdmi_1024x600 的像素

    // AI input is an independent RK stream, never the FPGA camera source.
    input wire ai_begin,ai_commit,ai_abort,ai_pixel_valid,ai_show,
    input wire [15:0] ai_pixel,
    input wire [31:0] ai_session,ai_task,ai_epoch,ai_context,ai_sequence,
    input wire [1:0] ai_view,
    output wire ai_frame_ready,ai_pixel_ready,ai_fault,
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
        // 同时上下翻转和左右镜像，使相机源画面旋转180度；UI菜单不参与翻转。
        .IMAGE_FLIP_EN (1),
        .IMAGE_MIRROR_EN(1),
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
    // DDR3 三帧缓存（ddr3_ctrl_2port + camera_frame_buffer）
    //   写侧：摄像头 PCLK 域，按 camera_frame_sync 对齐帧首
    //   读侧：显示 pixel_clk 域，按 frame_sync 对齐帧首，rden 绑 pixel_req
    // ---------------------------------------------------------------
    wire ai_ddr_clk,ai_write_valid,ai_write_ready,ai_read_valid,ai_read_ready,ai_response_valid,ai_arb_fault;
    wire [27:0] ai_write_address,ai_read_address;
    wire [127:0] ai_write_data,ai_response_data;
    wire [23:0] ai_rgb;
    wire ai_rgb_valid,ai_store_fault;
    generate if(AI_DISPLAY_ENABLE)begin:g_ai_store
        inspection_frame_store #(.CLK_HZ(SYS_CLOCK)) store(
            .clk(sys_clk),.rst_n(sys_rst_n),.ddr_clk(ai_ddr_clk),.i_ddr_initialized(ddr_init_done),.pixel_clk(pixel_clk),
            .i_begin(ai_begin),.i_commit(ai_commit),.i_abort(ai_abort),.i_session(ai_session),.i_task(ai_task),.i_epoch(ai_epoch),
            .i_context(ai_context),.i_sequence(ai_sequence),.i_view(ai_view),.o_frame_ready(ai_frame_ready),
            .i_pixel(ai_pixel),.i_pixel_valid(ai_pixel_valid),.o_pixel_ready(ai_pixel_ready),
            .o_write_valid(ai_write_valid),.o_write_address(ai_write_address),.o_write_data(ai_write_data),.i_write_ready(ai_write_ready),
            .o_read_valid(ai_read_valid),.o_read_address(ai_read_address),.i_read_ready(ai_read_ready),
            .i_response_valid(ai_response_valid),.i_response_data(ai_response_data),
            .i_frame_sync(frame_sync),.i_pixel_req(pixel_req),.i_show(ai_show && !ai_arb_fault),.i_x(pixel_x),.i_y(pixel_y),
            .o_rgb(ai_rgb),.o_rgb_valid(ai_rgb_valid),.o_fault(ai_store_fault));
    end else begin:g_no_ai_store
        assign ai_frame_ready=0,ai_pixel_ready=0,ai_write_valid=0,ai_write_address=0,ai_write_data=0;
        assign ai_read_valid=0,ai_read_address=0,ai_rgb=0,ai_rgb_valid=0,ai_store_fault=0;
    end endgenerate
    assign ai_fault=ai_arb_fault || ai_store_fault;
    ddr3_ctrl_2port #(.AI_DISPLAY_ENABLE(AI_DISPLAY_ENABLE),.CAMERA_PIXELS(IMAGE_WIDTH*IMAGE_HEIGHT)) u_frame_buffer (
        .ai_ddr_clk(ai_ddr_clk),.ai_write_valid(ai_write_valid),.ai_write_address(ai_write_address),.ai_write_data(ai_write_data),
        .ai_write_ready(ai_write_ready),.ai_read_valid(ai_read_valid),.ai_read_address(ai_read_address),.ai_read_ready(ai_read_ready),
        .ai_response_valid(ai_response_valid),.ai_response_data(ai_response_data),.ai_protocol_fault(ai_arb_fault),
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
    assign o_rgb = cam_region ? cam_rgb888 : ai_rgb_valid ? ai_rgb : ui_rgb;

endmodule
