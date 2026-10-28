// ============================================================================
// 模块：radar_navigation
// 职责：整车自动导航入口；只连接 nav_radar_adapter 的正式接口。
// 约定：50 MHz，异步低有效复位；M10 GPS UART 460800 / 8N1。
//       世界位置为毫米，初始车头为 +x、车左为 +y；朝向逆时针 0.01°。
//       本层不增加逻辑或延迟；调试输出留在内部 adapter，不接到整车。
// 接线、坐标、启动和控制流程见 ../docs/delivery_handoff.md。
// ============================================================================
module radar_navigation #(
    parameter integer AI_LAYER_ENABLE = 0,
    // 规划时障碍向横纵各预留的格数，同时用于停后局部接出和路线复核。
    parameter integer PLAN_INFLATE_CELLS  = 4
) (
    // 公共时钟与复位；除串口输入外，输入应已完整同步到此时钟域。
    input  wire               clk,
    input  wire               reset_n,

    // Optional AI visual map transaction. Disabled builds retain the original
    // map path; production enables it and supplies stopped/epoch explicitly.
    input wire ai_enable,ai_begin,ai_abort,ai_commit,ai_word_valid,ai_stopped,
    input wire [31:0] ai_epoch,ai_begin_epoch,ai_expected_crc,ai_word,
    input wire [15:0] ai_begin_version,
    input wire [12:0] ai_word_index,
    output wire ai_word_ready,ai_layer_valid,ai_committed,ai_error,ai_layer_busy,
    output wire [15:0] ai_layer_version,
    output wire [3:0] ai_error_code,

    // 运行时前探配置；每次扫描开始锁存。距离从定位中心点起算。
    input  wire         [7:0] forward_check_cells,
    input  wire        [15:0] vehicle_width_mm,
    // 包含起点的规划窗口格数，2..64，默认 5；下一轮规划生效。
    input  wire         [7:0] w_max,

    // 建图、规划和自动动作分别授权；撤销运动授权会先要求停车。
    input  wire               map_enable,
    input  wire               plan_enable,
    input  wire               motion_enable,

    // 雷达串口；持续定位，位置和朝向作为一组更新。
    input  wire               radar_rx,
    input  wire               pose_valid,
    input  wire signed [31:0] pose_x_mm,
    input  wire signed [31:0] pose_y_mm,
    input  wire signed [31:0] pose_yaw_cdeg,

    // 目标持续有效；格号 0..255，每格 50 mm，原点在中心格线。
    input  wire               goal_valid,
    input  wire         [7:0] goal_x_cell,
    input  wire         [7:0] goal_y_cell,

    // 控制器完成且停稳后置 1，保持到请求撤销；急停需新的低→高确认。
    input  wire               ctrl_done,
    // 1：即将或正在直行；0：原地转向或等待，不产生平移。
    input  wire               ctrl_linear_active,
    output wire               cmd_valid,
    output wire               emergency_stop,    // 抢占正常动作的停车请求。
    output wire signed [31:0] target_x_mm,
    output wire signed [31:0] target_y_mm,
    // 显示当前任务线型：0 水平、1 正45°、2 竖直、3 负45°，仅 cmd_valid 有效时使用。
    output wire         [1:0] target_line_dir,

    // 界面任务状态：0 关闭，1 运行中，2 暂停，3 已到达，4 无路。
    output wire         [3:0] nav_state,
    output wire               sensors_valid,

    // 同时控制地图和路线显示；点云输出不受此开关影响。
    input  wire               display_enable,
    // 全地图按 x 递增、再 y 递增输出；首尾标志均伴随有效格。
    output wire [15:0]        route_id,
    output wire [15:0]        map_version, // UI export version; no new map reader
    output wire               map_disp_valid,
    output wire               map_disp_occupied,
    output wire               map_disp_frame_start,
    output wire               map_disp_frame_done,
    // 路线按航点顺序输出；空路线 start/done 同拍为 1，valid 为 0。
    output wire               route_disp_valid,
    output wire         [7:0] route_disp_x_cell,
    output wire         [7:0] route_disp_y_cell,
    output wire               route_disp_frame_start,
    output wire               route_disp_frame_done,

    // 雷达自身坐标的测量事件，无 ready；0° 朝车头、顺时针 0.01°。
    output wire               cloud_valid,
    output wire        [15:0] cloud_angle_cdeg,
    output wire        [15:0] cloud_distance_mm   // 毫米；0/FFFF 等值由显示端过滤。
);
    nav_radar_adapter #(
        .PLAN_INFLATE_CELLS  (PLAN_INFLATE_CELLS),
        .AI_LAYER_ENABLE (AI_LAYER_ENABLE)
    ) u_adapter (
        .clk                    (clk),
        .reset_n                (reset_n),

        .ai_enable(ai_enable),
        .ai_begin(ai_begin),
        .ai_abort(ai_abort),
        .ai_commit(ai_commit),
        .ai_word_valid(ai_word_valid),
        .ai_stopped(ai_stopped),
        .ai_epoch(ai_epoch),
        .ai_begin_epoch(ai_begin_epoch),
        .ai_expected_crc(ai_expected_crc),
        .ai_word(ai_word),
        .ai_begin_version(ai_begin_version),
        .ai_word_index(ai_word_index),
        .ai_word_ready(ai_word_ready),
        .ai_layer_valid(ai_layer_valid),
        .ai_committed(ai_committed),
        .ai_error(ai_error),
        .ai_layer_version(ai_layer_version),
        .ai_error_code(ai_error_code),
        .ai_layer_busy(ai_layer_busy),
        .forward_check_cells    (forward_check_cells),
        .w_max               (w_max),
        .vehicle_width_mm       (vehicle_width_mm),
        .map_enable             (map_enable),
        .plan_enable            (plan_enable),
        .motion_enable          (motion_enable),
        .radar_rx               (radar_rx),
        .pose_valid             (pose_valid),
        .pose_x_mm              (pose_x_mm),
        .pose_y_mm              (pose_y_mm),
        .pose_yaw_cdeg          (pose_yaw_cdeg),
        .goal_valid             (goal_valid),
        .goal_x_cell            (goal_x_cell),
        .goal_y_cell            (goal_y_cell),
        .ctrl_done              (ctrl_done),
        .ctrl_linear_active     (ctrl_linear_active),
        .cmd_valid              (cmd_valid),
        .emergency_stop         (emergency_stop),
        .target_x_mm            (target_x_mm),
        .target_y_mm            (target_y_mm),
        .target_line_dir          (target_line_dir),
        .nav_state              (nav_state),
        .display_enable         (display_enable),
        .map_disp_valid         (map_disp_valid),
        .map_disp_occupied      (map_disp_occupied),
        .map_disp_frame_start   (map_disp_frame_start),
        .map_disp_frame_done    (map_disp_frame_done),
        .route_disp_valid       (route_disp_valid),
        .route_disp_x_cell      (route_disp_x_cell),
        .route_disp_y_cell      (route_disp_y_cell),
        .route_disp_frame_start (route_disp_frame_start),
        .route_disp_frame_done  (route_disp_frame_done),
        .cloud_valid            (cloud_valid),
        .cloud_angle_cdeg       (cloud_angle_cdeg),
        .cloud_distance_mm      (cloud_distance_mm),

        // 不向交付接口引出诊断信号；不改变内部导航和停车判断。
        .st_sensors_valid       (sensors_valid),
        .st_brake_clear         (),
        .st_stop_cause          (),
        .st_at_target           (),
        .st_segment_blocked     (),
        .st_plan_busy           (),
        .st_target_index        (),
        .st_route_length        (),
        .st_route_id            (route_id),
        .st_map_version         (map_version),
        .st_map_dropped         (),
        .st_plan_rounds         (),
        .st_occupied_cells      (),
        .scan_valid             (),
        .radar_frame_done       (),
        .sample_rejected        (),
        .radar_error            (),
        .radar_overflow         ()
    );
endmodule
