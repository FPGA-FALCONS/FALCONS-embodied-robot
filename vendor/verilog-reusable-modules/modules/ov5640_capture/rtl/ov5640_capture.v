module ov5640_capture #(
    parameter integer IMAGE_WIDTH     = 1024,
    parameter integer IMAGE_HEIGHT    = 600,
    parameter integer IMAGE_FLIP_EN   = 0,
    parameter integer IMAGE_MIRROR_EN = 0,
    parameter integer SYS_CLOCK       = 50_000_000,
    parameter integer SCL_CLOCK       = 400_000,
    parameter integer INIT_TIMEOUT_MS = 1000,
    parameter integer CAMERA_PWDN_LEVEL = 0
)(
    input             cfg_clk,
    input             reset_n,

    output            camera_sclk,
    inout             camera_sdat,
    input             camera_vsync,
    input             camera_href,
    input             camera_pclk,
    input      [7:0]  camera_data,
    output reg        camera_rst_n,
    output reg        camera_pwdn,

    output            init_done,
    output reg        init_error,
    output            pixel_clk,
    output            capture_clear,
    output            pixel_valid,
    output     [15:0] pixel_rgb565,
    output            line_valid,
    output            frame_sync,
    output     [10:0] pixel_x,
    output     [10:0] pixel_y
);

    localparam [31:0] INIT_TIMEOUT_CYCLES =
        (SYS_CLOCK / 1000) * INIT_TIMEOUT_MS;

    wire init_done_internal;
    wire camera_init_reset_n;
    wire camera_rst_n_internal;
    wire cfg_reset_n;
    wire capture_reset_n;
    reg [31:0] init_timeout_count;
    reg [1:0] cfg_reset_pipe;
    reg [1:0] pclk_reset_pipe;

    assign cfg_reset_n     = cfg_reset_pipe[1];
    assign capture_reset_n = pclk_reset_pipe[1];

    always@(posedge cfg_clk or negedge reset_n)
    if(!reset_n)
        cfg_reset_pipe <= 2'b00;
    else
        cfg_reset_pipe <= {cfg_reset_pipe[0], 1'b1};

    always@(posedge camera_pclk or negedge reset_n)
    if(!reset_n)
        pclk_reset_pipe <= 2'b00;
    else
        pclk_reset_pipe <= {pclk_reset_pipe[0], 1'b1};

    assign pixel_clk          = camera_pclk;
    assign init_done          = init_done_internal & ~init_error;
    assign camera_init_reset_n = cfg_reset_n & ~init_error;

    always@(posedge cfg_clk or negedge reset_n)
    if(!reset_n)begin
        init_timeout_count <= 32'd0;
        init_error <= 1'b0;
    end
    else if(!cfg_reset_n)begin
        init_timeout_count <= 32'd0;
        init_error <= 1'b0;
    end
    else if(init_done_internal)begin
        init_timeout_count <= 32'd0;
        init_error <= 1'b0;
    end
    else if(!init_error)begin
        if(init_timeout_count >= INIT_TIMEOUT_CYCLES - 1'b1)
            init_error <= 1'b1;
        else
            init_timeout_count <= init_timeout_count + 1'b1;
    end

    always@(posedge cfg_clk or negedge reset_n)
    if(!reset_n)begin
        camera_rst_n <= 1'b0;
        camera_pwdn <= CAMERA_PWDN_LEVEL;
    end
    else begin
        camera_rst_n <= camera_rst_n_internal & cfg_reset_n & ~init_error;
        camera_pwdn <= CAMERA_PWDN_LEVEL;
    end

    camera_init #(
        .SYS_CLOCK       (SYS_CLOCK),
        .SCL_CLOCK       (SCL_CLOCK),
        .CAMERA_TYPE     ("ov5640"),
        .IMAGE_TYPE      (0),
        .IMAGE_WIDTH     (IMAGE_WIDTH),
        .IMAGE_HEIGHT    (IMAGE_HEIGHT),
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN)
    ) u_camera_init (
        .Clk          (cfg_clk),
        .Rst_n        (camera_init_reset_n),
        .Init_Done    (init_done_internal),
        .camera_rst_n (camera_rst_n_internal),
        .camera_pwdn  (),
        .i2c_sclk     (camera_sclk),
        .i2c_sdat     (camera_sdat)
    );

    DVP_Capture u_dvp_capture (
        .Rst_n      (capture_reset_n),
        .PCLK       (camera_pclk),
        .Vsync      (camera_vsync),
        .Href       (camera_href),
        .Data       (camera_data),
        .ImageState (capture_clear),
        .DataValid  (pixel_valid),
        .DataPixel  (pixel_rgb565),
        .DataHs     (line_valid),
        .DataVs     (frame_sync),
        .Xaddr      (pixel_x),
        .Yaddr      (pixel_y)
    );

endmodule
