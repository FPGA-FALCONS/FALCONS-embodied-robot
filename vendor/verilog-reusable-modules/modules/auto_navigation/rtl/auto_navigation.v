// ============================================================================
// 模块：auto_navigation
// ----------------------------------------------------------------------------
// 职责：
//   - 已解析 M10 点流的自动导航交付入口；完成坐标变换、建图、后台 BFS 和动作握手。
//   - 只输出目标点任务及抢占停车请求；转向、速度、制动和停稳判断由控制器完成。
//   - 不处理选点像素、手动控制仲裁、定位计算或电机 PWM。
// 接口约定：
//   - 全部端口同一 50 MHz 时钟域，reset_n 异步低有效；多位异步输入由框架整体同步。
//   - 随点位姿对应距离接收时刻，背压时整份载荷保持；不接收时间戳。
//   - 世界 x/y 为 signed mm，yaw 为逆时针 0.01°；雷达 0° 朝前、顺时针增大。
//   - 地图 256×256、50 mm/格，世界范围 [-6400,6400)；目标输入是格号。
//   - goal_valid 是持续电平，无接受握手；四使能独立，撤使能不清地图或重置坐标。
//   - 完整端口、初始化及控制器协议见 ../docs/delivery_handoff.md。
// 关键不变量：
//   - point_valid && point_ready 同时接受角度、距离、槽号及接收时的位姿。
//   - 目标点任务 valid↑→done↑→valid↓→done↓；同向终点可更新，完成包含转向和直行。
//   - emergency_stop 抢占；必须先见 ctrl_done=0，再接收本次停稳的 ctrl_done=1。
// 禁止事项：
//   - 不要把屏幕像素或毫米坐标直接接到 goal_*_cell；(128,128) 格中心是 (25,25) mm。
//   - 不要用显示流控制底盘，也不要把诊断状态替代 cmd_valid / emergency_stop。
// ============================================================================
module auto_navigation #(
    parameter integer PLAN_INFLATE_CELLS  = 4
) (
    // 50 MHz 公共时钟；所有控制/位姿多位输入需已同步到此域。
    input  wire               clk,
    // 异步低有效复位；同时清任务、路线、整圈状态并重新初始化地图。
    input  wire               reset_n,

    // 运行时前探配置；每次扫描开始锁存。距离从定位中心点起算。
    input  wire         [7:0] forward_check_cells,
    input  wire        [15:0] vehicle_width_mm,

    // 允许启动新规划轮次；撤销不取消在途计算，不清活动路线。
    input  wire               plan_enable,
    // 授权自动动作；上升先停车握手，撤销也先完成停车握手。
    input  wire               motion_enable,
    // 同时使能两路显示；为 0 时两路有效数据和帧边界均为 0。
    input  wire               display_enable,
    // 使能 raw/cost 更新；为 0 接收并丢弃新观测，不清已有地图。
    input  wire               map_enable,

    // 点与全部旁带有效；背压时保持到 point_ready。
    input  wire               point_valid,
    // 为 1 才可在本拍接受点；UART 内部缓存由 adapter 管理。
    output wire               point_ready,
    // 雷达角 0..35999，0.01°，0° 朝车头，顺时针增加。
    input  wire        [15:0] point_angle_cdeg,
    // 距离 mm；0/10000 无回波，1..9999 命中，>10000（含 FFFF）拒绝。
    input  wire        [15:0] point_distance_mm,
    // M10 报文内槽号 0..41，包括填充槽，用于整圈完整性。
    input  wire         [5:0] point_index,
    // 该点采样位姿有效；与点一起锁存，不能用当前有效标志替代。
    input  wire               point_pose_valid,
    // 采样时世界 x，signed mm；[-6400,6400)。
    input  wire signed [31:0] point_pose_x_mm,
    // 采样时世界 y，signed mm；[-6400,6400)。
    input  wire signed [31:0] point_pose_y_mm,
    // 采样时世界偏航，signed 0.01°，逆时针，[-18000,18000)。
    input  wire signed [31:0] point_pose_yaw_cdeg,
    // 点流传输/序列故障事件；使整圈失效并触发数据停车。
    input  wire               stream_error,

    // 当前位置有效；定位失效须主动撤销，冻结且保持 1 无法被识别。
    input  wire               pose_valid,
    // 当前世界 x，signed mm；初始车头为 +x，范围 [-6400,6400)。
    input  wire signed [31:0] pose_x_mm,
    // 当前世界 y，signed mm；初始车左为 +y，范围 [-6400,6400)。
    input  wire signed [31:0] pose_y_mm,
    // 当前绝对 yaw，signed 0.01°，逆时针，[-18000,18000)。
    input  wire signed [31:0] pose_yaw_cdeg,

    // 控制器完成且停稳电平，保持至请求撤销；急停需要新的低→高确认。
    input  wire               ctrl_done,
    // 1：即将或正在直行；0：原地转向或等待，不产生平移。
    input  wire               ctrl_linear_active,

    // 持续目标有效电平，无接受脉冲；撤销取消自动动作，不清地图。
    input  wire               goal_valid,
    // 目标世界格 x=0..255；格中心 x=50*x-6375 mm。
    input  wire         [7:0] goal_x_cell,
    // 目标世界格 y=0..255；正向与世界 +y 相同，非屏幕行号。
    input  wire         [7:0] goal_y_cell,

    // 目标点任务请求；同向终点可更新，四相握手后切换方向。
    output wire               cmd_valid,
    // 抢占停车请求，优先于正常动作；已停稳也必须完成本次握手。
    output wire               emergency_stop,
    // 任务连续世界目标 x，signed mm；纵向可更新，横向锁存。
    output wire signed [31:0] target_x_mm,
    // 任务连续世界目标 y，signed mm；与 x 一起交给控制器。
    output wire signed [31:0] target_y_mm,
    // 显示当前任务线型：0 水平、1 正45°、2 竖直、3 负45°，仅 cmd_valid 有效时使用。
    output wire         [1:0] target_line_dir,

    // 正式任务接口：供整车界面读取。
    // 正式任务状态：0关闭/1运行中/2暂停/3已到达/4无路；不用于底盘控制。
    output wire         [3:0] nav_state,

    // 诊断输出：可不连接，不作为任务完成或停车确认依据。
    // 数据健康电平：定位就绪/范围、整圈和流错误均合格。
    output wire               st_sensors_valid,
    // 同 st_sensors_valid；不表示 raw 前探无障碍，也不判断底盘制动。
    output wire               st_brake_clear,
    // 诊断：0无本项原因/1数据失效/2前探未放行/3位置范围越界。
    output wire         [1:0] st_stop_cause,
    // 接受控制器正常直行完成后的单拍提示；不做位置误差验收。
    output wire               st_at_target,
    // 当前方向有效前探命中 raw/边界；0 不保证检查已就绪。
    output wire               st_segment_blocked,
    // 规划请求、BFS、装载、完整验证至发布/取消的整个轮次忙。
    output wire               st_plan_busy,
    // 活动路线当前点索引，从 0 起；仅调试/显示。
    output wire        [15:0] st_target_index,
    // 活动 bank 合并后航点数，可能是可执行前缀。
    output wire        [15:0] st_route_length,
    // 成功发布计数，16 位回绕；不是给控制器的指令编号。
    output wire        [15:0] st_route_id,
    // raw 实际改变且膨胀事务结束后加一，16 位回绕。
    output wire        [15:0] st_map_version,
    // 禁用建图时丢弃的观测数，16 位回绕；不代表 UART 溢出。
    output wire        [15:0] st_map_dropped,
    // 已启动的规划轮次数，32 位回绕，包含后续失败/取消轮次。
    output wire        [31:0] st_plan_rounds,
    // 当前 raw=1 的格数；不是 cost 格数或雷达点累计数。
    output wire        [31:0] st_occupied_cells,
    // 整圈序列有效电平；断流不单独清零，不检查观测年龄。
    output wire               scan_valid,
    // 完整合格整圈完成的单拍事件，不是地图更新完成。
    output wire               radar_frame_done,
    // 单拍诊断：非法输入或端点出图；出图射线仍可更新图内段。
    output wire               sample_rejected,

    // 地图按 x 递增、再 y 递增扫描；首尾均与有效格同拍。
    output wire               map_disp_valid,
    output wire               map_disp_occupied,
    output wire               map_disp_frame_start,
    output wire               map_disp_frame_done,
    // 路线按航点顺序输出；空路线 start/done 同拍、valid 为 0。
    output wire               route_disp_valid,
    output wire         [7:0] route_disp_x_cell,
    output wire         [7:0] route_disp_y_cell,
    output wire               route_disp_frame_start,
    output wire               route_disp_frame_done
);
    wire pose_inside = pose_x_mm >= -32'sd6400 && pose_x_mm < 32'sd6400 &&
        pose_y_mm >= -32'sd6400 && pose_y_mm < 32'sd6400 && pose_yaw_cdeg >= -32'sd18000 &&
        pose_yaw_cdeg < 32'sd18000;
    wire navigation_pose_valid = pose_valid && pose_inside;

    // 单一观测握手同时传递射线与可选障碍端点；无命中仍需清理射线。
    // nav_scanner 是下游唯一背压来源，显示旁路不能驱动 ready。
    // M10 每包第一槽的角度回绕表示换圈。标记随点穿过极坐标流水，
    // 不能直接使用输入侧整圈脉冲清理仍在下游处理的上一条射线。
    reg packet_seen;
    reg [15:0] previous_packet_angle;
    wire point_scan_start = packet_seen && point_index == 0 &&
                            point_angle_cdeg < previous_packet_angle;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            packet_seen <= 1'b0;
            previous_packet_angle <= 0;
        end else if (point_valid && point_ready && point_index == 0) begin
            packet_seen <= 1'b1;
            previous_packet_angle <= point_angle_cdeg;
        end
    end
    wire grid_valid, grid_ready, grid_scan_start;
    wire [15:0] grid_x, grid_y;
    // 图内真实障碍命中；无回波或图外端点为 0，但仍有射线观测。
    wire obs_grid_hit;
    wire obs_ready;  // 由 nav_scanner 驱动，是这条路径上唯一的背压来源
    assign grid_ready = obs_ready;
    wire signed [31:0] observation_origin_x_mm, observation_origin_y_mm;
    wire signed [31:0] observation_ray_dx_q16, observation_ray_dy_q16;

    nav_polar u_polar (
        .clk             (clk),
        .reset_n         (reset_n),
        .in_valid        (point_valid),
        .in_scan_start   (point_scan_start),
        .in_ready        (point_ready),
        .in_angle_cdeg   (point_angle_cdeg),
        .in_distance_mm  (point_distance_mm),
        .in_pose_valid   (point_pose_valid),
        .in_pose_x_mm    (point_pose_x_mm),
        .in_pose_y_mm    (point_pose_y_mm),
        .in_pose_yaw_cdeg(point_pose_yaw_cdeg),
        .out_valid       (grid_valid),
        .out_scan_start  (grid_scan_start),
        .out_ready       (grid_ready),
        .out_hit         (obs_grid_hit),
        .out_hit_x       (grid_x),
        .out_hit_y       (grid_y),
        .out_origin_x_mm (observation_origin_x_mm),
        .out_origin_y_mm (observation_origin_y_mm),
        .out_ray_dx_q16  (observation_ray_dx_q16),
        .out_ray_dy_q16  (observation_ray_dy_q16),
        .out_rejected    (sample_rejected)
    );

    lidar_scan_monitor u_scan (
        .clk             (clk),
        .reset_n         (reset_n),
        .point_accepted  (point_valid && point_ready),
        .point_angle_cdeg(point_angle_cdeg),
        .point_index     (point_index),
        .stream_error    (stream_error),
        .radar_frame_done(radar_frame_done),
        .scan_valid      (scan_valid)
    );

    // 目标为持续电平；每轮锁存，换目标立即撤销旧路线的运动资格。
    // 当前位姿只用于控制和发布验证，不能替换上游点携带的采样位姿。

    nav_top #(
        .GRID_W             (256),
        .GRID_H             (256),
        .CELL_MM            (50),
        .HALF_MM            (6400),
        .MAX_WP             (1024),
        .PLAN_INFLATE_CELLS (PLAN_INFLATE_CELLS)
    ) u_nav (
        .clk               (clk),
        .reset_n           (reset_n),
        .forward_check_cells (forward_check_cells),
        .vehicle_width_mm    (vehicle_width_mm),
        .pose_valid        (navigation_pose_valid),
        .pose_x_mm         (pose_x_mm),
        .pose_y_mm         (pose_y_mm),
        .goal_valid        (goal_valid),
        .goal_x            ({8'd0, goal_x_cell}),
        .goal_y            ({8'd0, goal_y_cell}),
        .plan_enable       (plan_enable),
        .motion_enable     (motion_enable),
        .display_enable    (display_enable),
        .map_enable        (map_enable),
        .ctrl_done         (ctrl_done),
        .ctrl_linear_active     (ctrl_linear_active),
        .obs_valid         (grid_valid),
        .obs_scan_start    (grid_scan_start),
        .obs_ready         (obs_ready),
        .obs_grid_x        (grid_x),
        .obs_grid_y        (grid_y),
        .obs_grid_valid    (obs_grid_hit),
        .obs_origin_x_mm   (observation_origin_x_mm),
        .obs_origin_y_mm   (observation_origin_y_mm),
        .obs_ray_dx_q16    (observation_ray_dx_q16),
        .obs_ray_dy_q16    (observation_ray_dy_q16),
        .scan_valid        (scan_valid),
        .stream_error      (stream_error),
        .cmd_valid         (cmd_valid),
        .emergency_stop    (emergency_stop),
        .target_x_mm       (target_x_mm),
        .target_y_mm       (target_y_mm),
        .target_line_dir     (target_line_dir),
        .nav_state         (nav_state),
        .st_commit_valid   (),
        .st_sensors_valid  (st_sensors_valid),
        .st_brake_clear    (st_brake_clear),
        .st_stop_cause     (st_stop_cause),
        .st_at_target      (st_at_target),
        .st_segment_blocked(st_segment_blocked),
        .st_plan_busy      (st_plan_busy),
        .st_target_index   (st_target_index),
        .st_route_length   (st_route_length),
        .st_route_id       (st_route_id),
        .st_map_version    (st_map_version),
        .st_map_dropped    (st_map_dropped),
        .st_plan_rounds    (st_plan_rounds),
        .st_occupied_cells (st_occupied_cells),
        // 地图顺序扫描与路线航点分别输出，无来源切换。
        .map_disp_valid        (map_disp_valid),
        .map_disp_occupied     (map_disp_occupied),
        .map_disp_frame_start  (map_disp_frame_start),
        .map_disp_frame_done   (map_disp_frame_done),
        .route_disp_valid      (route_disp_valid),
        .route_disp_x_cell     (route_disp_x_cell),
        .route_disp_y_cell     (route_disp_y_cell),
        .route_disp_frame_start(route_disp_frame_start),
        .route_disp_frame_done (route_disp_frame_done)
    );

endmodule
