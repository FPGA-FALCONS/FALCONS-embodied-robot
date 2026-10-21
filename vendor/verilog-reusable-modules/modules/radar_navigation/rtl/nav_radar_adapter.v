// ============================================================================
// 模块：nav_radar_adapter
// ----------------------------------------------------------------------------
// 职责：
//   - M10 GPS UART 与自动导航的交付集成入口，内部例化雷达解析和 auto_navigation。
//   - 距离低字节接收时捕获位姿，整份上下文随点排队。
//   - 只折叠雷达错误、连接导航接口，不负责定位、模式仲裁或控制律。
// 接口约定：
//   - 所有逻辑统一 50 MHz、异步低有效复位；UART 460800、8N1，空闲高电平。
//   - 信任雷达和定位持续及时更新，不接收时间戳，也不检查观测年龄。
//   - 位姿 x/y 为 signed mm，yaw 为逆时针 0.01°；雷达 0° 朝前、顺时针增大。
//   - 目标点任务与急停握手同 auto_navigation，完整交接见 ../../auto_navigation/docs/delivery_handoff.md。
// 关键不变量：
//   - 97 位上下文顺序为 {有效, x32, y32, yaw32}，解包与打包必须一致。
//   - UART 接收时刻只是采集关联近似，不是光学同步；没有雷达安装外参补偿。
//   - stream_error=包错|UART错|缓存溢出；公开 radar_error 不含缓存溢出。
// 禁止事项：
//   - 不要改用出队时当前位姿；整套导航固定使用 50 MHz。
//   - 定位源准备好后置 pose_valid，位置与朝向必须作为一组原子更新。
// ============================================================================
module nav_radar_adapter #(
    parameter integer FORWARD_CHECK_CELLS = 3,
    parameter integer VEHICLE_WIDTH_MM    = 180,
    parameter integer FRONT_OFFSET_MM     = 150,
    parameter integer LATERAL_MARGIN_MM   = 20,
    parameter integer PLAN_INFLATE_CELLS  = 2
) (
    // 50 MHz 公共时钟；所有控制/位姿多位输入需已同步到此域。
    input  wire               clk,
    // 异步低有效复位；同时清任务、路线、整圈状态并重新初始化地图。
    input  wire               reset_n,

    // 允许启动新规划轮次；撤销不取消在途计算，不清活动路线。
    input  wire               plan_enable,
    // 授权自动动作；上升先停车握手，撤销也先完成停车握手。
    input  wire               motion_enable,
    // 同时使能两路显示；为 0 时两路有效数据和帧边界均为 0。
    input  wire               display_enable,
    // 使能 raw/cost 更新；为 0 接收并丢弃新观测，不清已有地图。
    input  wire               map_enable,
    // M10 UART 输入，460800 / 8N1，空闲高。
    input  wire               radar_rx,

    // 当前位置有效；定位失效须主动撤销，冻结且保持 1 无法被识别。
    input  wire               pose_valid,
    // 当前世界 x，signed mm；初始车头为 +x，范围 [-12800,12800)。
    input  wire signed [31:0] pose_x_mm,
    // 当前世界 y，signed mm；初始车左为 +y，范围 [-12800,12800)。
    input  wire signed [31:0] pose_y_mm,
    // 当前绝对 yaw，signed 0.01°，逆时针，[-18000,18000)。
    input  wire signed [31:0] pose_yaw_cdeg,

    // 控制器完成且停稳电平，保持至请求撤销；急停需要新的低→高确认。
    input  wire               ctrl_done,
    // 1：即将或正在直行；0：原地转向或等待，不产生平移。
    input  wire               ctrl_linear_active,

    // 持续目标有效电平，无接受脉冲；撤销取消自动动作，不清地图。
    input  wire               goal_valid,
    // 目标世界格 x=0..255；格中心 x=100*x-12750 mm。
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
    output wire               route_disp_frame_done,
    // 整圈序列有效电平；断流不单独清零，不检查观测年龄。
    output wire               scan_valid,
    // 完整合格整圈完成的单拍事件，不是地图更新完成。
    output wire               radar_frame_done,
    // 单拍诊断：非法输入或端点出图；出图射线仍可更新图内段。
    output wire               sample_rejected,
    // 单拍包错误或 UART 接收错误；不包含 radar_overflow。
    output wire               radar_error,
    // 雷达双报文缓存满而丢报文的事件，内部同时使整圈失效。
    output wire               radar_overflow,

    // 雷达自身坐标的测量事件，无 ready；接收端逐拍接收或写入显示 FIFO。
    // 每个解析槽只输出一次，不依赖定位、建图、规划或显示使能。
    output wire               cloud_valid,
    // 雷达 0° 朝车头、顺时针增加，单位 0.01°，范围 0..35999。
    output wire        [15:0] cloud_angle_cdeg,
    // 解析后的毫米距离，保留 0/FFFF 等槽值，由显示端过滤；不做 10 m 截断。
    output wire        [15:0] cloud_distance_mm
);
    wire point_valid, point_ready, packet_error, uart_error;
    wire [15:0] point_angle_cdeg, point_distance_mm;
    wire [5:0] point_index;
    wire [96:0] point_context;

    // 旁听唯一消费者的成交事件，不能把显示忙接入导航背压链。
    assign cloud_valid       = point_valid && point_ready;
    assign cloud_angle_cdeg  = point_angle_cdeg;
    assign cloud_distance_mm = point_distance_mm;

    // 位姿在距离接收时锁存，随点排队；出队时不读取新的定位值。
    wire [96:0] capture_context = {pose_valid, pose_x_mm, pose_y_mm, pose_yaw_cdeg};

    // 错误分类（taxonomy）仍然对本外壳的集成方可见；导航只得到一个布尔量。
    assign radar_error = packet_error || uart_error;

    // 包格式、双包缓存与 UART 全部封在雷达内部。未接出的端口是它的诊断面（包内容、
    // 各类计数、粘滞故障位），公共契约不导出它们。
    m10_radar #(
        .CLK_FREQ_HZ(50_000_000)
    ) u_radar (
        .clk                (clk),
        .reset_n            (reset_n),
        .radar_rx           (radar_rx),
        .capture_context    (capture_context),
        .point_context      (point_context),
        .point_valid        (point_valid),
        .point_ready        (point_ready),
        .point_angle_cdeg   (point_angle_cdeg),
        .point_distance_mm  (point_distance_mm),
        .point_index        (point_index),
        .packet_error       (packet_error),
        .packet_overflow    (radar_overflow),
        .rx_error           (uart_error),
        .packet_valid       (),
        .packet_error_reason(),
        .packet_speed       (),
        .rx_data            (),
        .rx_valid           (),
        .data_active        (),
        .error_sticky       (),
        .overflow_sticky    (),
        .packet_count       (),
        .rx_byte_count      (),
        .uart_error_count   (),
        .bad_packet_count   (),
        .last_rx_byte       (),
        .last_error_reason  ()
    );

    // 上下文按下述位切片解包，顺序必须与上面的打包顺序一致。stream_error 是三类错误的
    // 或：radar_error 已含包错与 UART 错，再或上双包缓存溢出。
    auto_navigation #(
        .FORWARD_CHECK_CELLS(FORWARD_CHECK_CELLS),
        .VEHICLE_WIDTH_MM   (VEHICLE_WIDTH_MM),
        .FRONT_OFFSET_MM    (FRONT_OFFSET_MM),
        .LATERAL_MARGIN_MM  (LATERAL_MARGIN_MM),
        .PLAN_INFLATE_CELLS (PLAN_INFLATE_CELLS)
    ) u_navigation (
        .clk                (clk),
        .reset_n            (reset_n),
        .plan_enable        (plan_enable),
        .motion_enable      (motion_enable),
        .display_enable     (display_enable),
        .map_enable         (map_enable),
        .point_valid        (point_valid),
        .point_ready        (point_ready),
        .point_angle_cdeg   (point_angle_cdeg),
        .point_distance_mm  (point_distance_mm),
        .point_index        (point_index),
        .point_pose_valid   (point_context[96]),
        .point_pose_x_mm    (point_context[95:64]),
        .point_pose_y_mm    (point_context[63:32]),
        .point_pose_yaw_cdeg(point_context[31:0]),
        .stream_error       (radar_error || radar_overflow),
        .pose_valid         (pose_valid),
        .pose_x_mm          (pose_x_mm),
        .pose_y_mm          (pose_y_mm),
        .pose_yaw_cdeg      (pose_yaw_cdeg),
        .ctrl_done          (ctrl_done),
        .ctrl_linear_active     (ctrl_linear_active),
        .goal_valid         (goal_valid),
        .goal_x_cell        (goal_x_cell),
        .goal_y_cell        (goal_y_cell),
        .cmd_valid          (cmd_valid),
        .emergency_stop     (emergency_stop),
        .target_x_mm        (target_x_mm),
        .target_y_mm        (target_y_mm),
        .nav_state          (nav_state),
        .st_sensors_valid   (st_sensors_valid),
        .st_brake_clear     (st_brake_clear),
        .st_stop_cause      (st_stop_cause),
        .st_at_target       (st_at_target),
        .st_segment_blocked (st_segment_blocked),
        .st_plan_busy       (st_plan_busy),
        .st_target_index    (st_target_index),
        .st_route_length    (st_route_length),
        .st_route_id        (st_route_id),
        .st_map_version     (st_map_version),
        .st_map_dropped     (st_map_dropped),
        .st_plan_rounds     (st_plan_rounds),
        .st_occupied_cells  (st_occupied_cells),
        .map_disp_valid        (map_disp_valid),
        .map_disp_occupied     (map_disp_occupied),
        .map_disp_frame_start  (map_disp_frame_start),
        .map_disp_frame_done   (map_disp_frame_done),
        .route_disp_valid      (route_disp_valid),
        .route_disp_x_cell     (route_disp_x_cell),
        .route_disp_y_cell     (route_disp_y_cell),
        .route_disp_frame_start(route_disp_frame_start),
        .route_disp_frame_done (route_disp_frame_done),
        .scan_valid         (scan_valid),
        .radar_frame_done   (radar_frame_done),
        .sample_rejected    (sample_rejected)
    );
endmodule
