// ============================================================================
// 模块：nav_polar
// ----------------------------------------------------------------------------
// 职责：
//   - 将 M10 极坐标及随点位姿转换为世界 Q16 射线和地图端点，交给 nav_scanner。
//   - 16 级 CORDIC 逐点旋转；0/10000 mm 按无回波，其他合法距离按真实命中处理。
//   - 检查输入范围；非法输入消费后报拒绝，合法射线出图仍继续输出图内清理所需几何。
// 接口约定：
//   - 单时钟、异步低有效复位；输入/输出各一组 valid/ready，同时只有一份采样在途。
//   - 雷达角 0..35999，单位 0.01°、顺时针；yaw 为 [-18000,18000)、逆时针。
//   - 位置 signed mm，范围 [-12800,12800)；距离 0..10000 mm，超过 10000 包括 FFFF 拒绝。
//   - out_ray_* 是 signed Q16 mm 位移；out_hit_x/y 是 signed 格号，图外端点不截边。
//   - out_hit 仅表示图内真实命中；接收时的位姿保持至观测被接受。
//   - out_rejected 单拍：非法输入无输出；合法端点出图时同时保留后续 out_valid。
// 关键不变量：
//   - 世界角 = yaw - 雷达角，归一化至 [-18000,18000)；四个轴用精确值保住格界。
//   - 格号 floor((世界毫米+12800)/100)；负的图外坐标也必须向下取整。
//   - 旋转、偏移量锁存、常数乘法量化分拍，输出背压时几何数据全部保持。
// 禁止事项：
//   - 不要用出队时当前位姿替换采样位姿；本模块不补偿雷达安装平移或安装偏角。
//   - 39797、119305、41944 分别对应 CORDIC 增益、二进制角度刻度、100 mm 量化常数。
// ============================================================================
module nav_polar (
    input  wire               clk,
    input  wire               reset_n,

    input  wire               in_valid,
    output wire               in_ready,
    input  wire        [15:0] in_angle_cdeg,
    input  wire        [15:0] in_distance_mm,
    input  wire               in_pose_valid,
    input  wire signed [31:0] in_pose_x_mm,
    input  wire signed [31:0] in_pose_y_mm,
    input  wire signed [31:0] in_pose_yaw_cdeg,

    output wire               out_valid,
    input  wire               out_ready,
    output wire               out_hit,
    output wire signed [15:0] out_hit_x,
    output wire signed [15:0] out_hit_y,
    output wire signed [31:0] out_origin_x_mm,
    output wire signed [31:0] out_origin_y_mm,
    output reg  signed [31:0] out_ray_dx_q16,
    output reg  signed [31:0] out_ray_dy_q16,
    output reg                out_rejected
);
    localparam integer MAP_MM = 25600;  // 256 个单元格 x 100 mm
    localparam integer HALF_MM = 12800;  // MAP_MM / 2，把世界坐标搬进 [0, MAP_MM)
    localparam integer CELL_MM = 100;
    localparam integer MAX_RANGE_MM = 10000;
    localparam integer CORDIC_ITERS = 16;

    // QUANT2 是独立的流水（pipeline）级，把单元格乘法与旋转的组合逻辑路径隔开；
    // 理由见 offset_x_r 处。
    localparam [2:0] IDLE     = 3'd0,
                     ROTATE   = 3'd1,
                     QUANTIZE = 3'd2,
                     QUANT2   = 3'd3,
                     OUTPUT   = 3'd4;

    localparam signed [63:0] MAP_EXTENT_Q16 = 64'sd1677721600;  // MAP_MM << 16

    reg [ 2:0] state;
    reg        pending;
    reg        hit_inside_q;  // 地图内判定结果，与端点一起锁存
    reg [14:0] latched_pose_x_map_mm, latched_pose_y_map_mm;
    reg        [15:0] latched_distance_mm;
    reg signed [15:0] latched_world_angle_cdeg;
    reg signed [31:0] cordic_x, cordic_y, cordic_z;
    reg        [ 3:0] iteration;
    reg signed [31:0] atan_rom  [0:CORDIC_ITERS-1];

    // round(atan(2^-i) * 2^32 / (2*pi))，一个带符号的二进制角度 ROM
    initial begin
        atan_rom[0]  = 32'sd536870912;
        atan_rom[1]  = 32'sd316933406;
        atan_rom[2]  = 32'sd167458907;
        atan_rom[3]  = 32'sd85004756;
        atan_rom[4]  = 32'sd42667331;
        atan_rom[5]  = 32'sd21354465;
        atan_rom[6]  = 32'sd10679838;
        atan_rom[7]  = 32'sd5340245;
        atan_rom[8]  = 32'sd2670163;
        atan_rom[9]  = 32'sd1335087;
        atan_rom[10] = 32'sd667544;
        atan_rom[11] = 32'sd333772;
        atan_rom[12] = 32'sd166886;
        atan_rom[13] = 32'sd83443;
        atan_rom[14] = 32'sd41722;
        atan_rom[15] = 32'sd20861;
    end

    wire in_no_echo = (in_distance_mm == 16'd0) || (in_distance_mm >= MAX_RANGE_MM[15:0]);
    wire [15:0] effective_distance_mm = in_no_echo ? MAX_RANGE_MM[15:0] : in_distance_mm;


    // 世界角 = yaw - angle，先归一化到 [-18000, 18000) 厘度；再折叠 180 度到
    // [-9000, 9000] 以限制 CORDIC 的收敛范围，折叠时把初始 x 取负以保证落在同一象限。
    reg signed [16:0] normalized_angle_cdeg;
    reg signed [14:0] folded_angle_cdeg;
    reg signed [31:0] initial_cordic_x;
    always @* begin
        normalized_angle_cdeg = $signed(in_pose_yaw_cdeg[16:0]) - $signed({1'b0, in_angle_cdeg});
        if (normalized_angle_cdeg < -17'sd18000)
            normalized_angle_cdeg = normalized_angle_cdeg + 17'sd36000;
        else if (normalized_angle_cdeg >= 17'sd18000)
            normalized_angle_cdeg = normalized_angle_cdeg - 17'sd36000;

        folded_angle_cdeg = normalized_angle_cdeg;
        initial_cordic_x  = $signed({16'd0, effective_distance_mm}) * 32'sd39797;
        if (normalized_angle_cdeg > 17'sd9000) begin
            folded_angle_cdeg = normalized_angle_cdeg - 17'sd18000;
            initial_cordic_x  = -initial_cordic_x;
        end else if (normalized_angle_cdeg < -17'sd9000) begin
            folded_angle_cdeg = normalized_angle_cdeg + 17'sd18000;
            initial_cordic_x  = -initial_cordic_x;
        end
    end

    // 无回波（no-echo）：合法的零距离或恰为量程上限表示没有真实命中，
    // 两者含义相同 —— 一直到量程上限都是自由空间，不是障碍物。它仍然是合法的
    // observation（下游要靠它清除波束扫过的单元格），所以这里推到满量程继续走几何，
    // 到输出端再用 out_hit 为低与真实障碍物分开。把零当非法，射线就永远清不掉；
    // 把量程上限当命中，则会在每个空旷方向上凭空造出正好 10 m 处的障碍物。
    wire input_legal = in_pose_valid && in_angle_cdeg < 16'd36000 &&
        in_distance_mm <= MAX_RANGE_MM[15:0] && in_pose_x_mm >= -32'sd12800 &&
        in_pose_x_mm < 32'sd12800 && in_pose_y_mm >= -32'sd12800 && in_pose_y_mm < 32'sd12800 &&
        in_pose_yaw_cdeg >= -32'sd18000 && in_pose_yaw_cdeg < 32'sd18000;

    wire signed [63:0] range_q16 = $signed({48'd0, latched_distance_mm}) <<< 16;
    reg signed [63:0] rotated_x_q16, rotated_y_q16;
    always @* begin
        rotated_x_q16 = {{32{cordic_x[31]}}, cordic_x};
        rotated_y_q16 = {{32{cordic_y[31]}}, cordic_y};
        // 精确轴（exact axis）保住了半开的地图边界与栅格线向下取整，否则有限的
        // CORDIC 残差会让端点跨过一个单元格。
        if (latched_world_angle_cdeg == 32'sd0) begin
            rotated_x_q16 = range_q16;
            rotated_y_q16 = 64'sd0;
        end else if (latched_world_angle_cdeg == 32'sd9000) begin
            rotated_x_q16 = 64'sd0;
            rotated_y_q16 = range_q16;
        end else if (latched_world_angle_cdeg == -32'sd9000) begin
            rotated_x_q16 = 64'sd0;
            rotated_y_q16 = -range_q16;
        end else if (latched_world_angle_cdeg == -32'sd18000) begin
            rotated_x_q16 = -range_q16;
            rotated_y_q16 = 64'sd0;
        end
    end

    // 量化（quantize）直接在带偏置的坐标系里做，不需要反向减偏置：锁存的位姿偏移
    // 已经落在 [0, MAP_MM) 内。
    wire signed [63:0] offset_world_x_q16 =
        ($signed({49'd0, latched_pose_x_map_mm}) <<< 16) + rotated_x_q16;
    wire signed [63:0] offset_world_y_q16 =
        ($signed({49'd0, latched_pose_y_map_mm}) <<< 16) + rotated_y_q16;
    wire world_inside = offset_world_x_q16 >= 64'sd0 && offset_world_x_q16 < MAP_EXTENT_Q16 &&
        offset_world_y_q16 >= 64'sd0 && offset_world_y_q16 < MAP_EXTENT_Q16;
    // 有符号 floor(offset_mm)，图外约 -10000..35600，不能截到图内范围。
    wire signed [16:0] offset_world_x_mm = offset_world_x_q16 >>> 16;
    wire signed [16:0] offset_world_y_mm = offset_world_y_q16 >>> 16;
    // 乘法的输入必须取自寄存器，不能直接接旋转与 64 位偏移加法的组合输出，否则会
    // 闭合成一条 latched_world_angle_cdeg → rotate → add → DSP 乘法 →
    // nav_scanner 的 Bresenham 误差链，超出 50 MHz（20 ns）的时序预算。拆成独立一级
    // 的代价是一个 observation 时钟的延迟，valid/ready 握手（handshake）可以吸收它，
    // 同时让乘法独占一个完整周期。
    //
    // ceil(2^22 / 100) = 41944；对每个 15 位输入，完整乘积都给出精确的向下取整，
    // 因此不需要"除以常数单元格尺寸"的除法器。
    reg signed [16:0] offset_x_r, offset_y_r;
    reg signed [15:0] hit_x, hit_y;
    // 保留图外端点的符号和方向，禁止先截到 15 位再量化。
    function signed [15:0] signed_cell;
        input signed [16:0] offset_mm;
        reg [16:0] magnitude;
        reg [32:0] product;
        begin
            magnitude = offset_mm < 0 ? (-offset_mm + 17'd99) : offset_mm;
            product   = magnitude * 16'd41944;
            if (offset_mm < 0) signed_cell = -$signed({1'b0, product[32:22]});
            else signed_cell = $signed({1'b0, product[32:22]});
        end
    endfunction

    assign in_ready  = reset_n && state == IDLE;
    assign out_valid = reset_n && pending;
    // out_hit 的含义是"端点是一个真实障碍物"，而不只是"端点在地图内"。无回波
    // （no-echo）采样的端点被推到量程上限，因此距离等于上限就能认出它：这种采样下游
    // 仍然看到 out_valid 为高（沿射线清除单元格），但 out_hit 为低，不写入任何障碍物。
    wire latched_no_echo = (latched_distance_mm >= MAX_RANGE_MM[15:0]);
    assign out_hit         = pending && hit_inside_q && !latched_no_echo;
    assign out_hit_x       = hit_x;
    assign out_hit_y       = hit_y;
    assign out_origin_x_mm = $signed({17'd0, latched_pose_x_map_mm}) - 32'sd12800;
    assign out_origin_y_mm = $signed({17'd0, latched_pose_y_map_mm}) - 32'sd12800;

    // 地图内判定结果与端点一起锁存，这样即使下游停顿（stall），out_hit 也保持稳定。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state                    <= IDLE;
            pending                  <= 1'b0;
            hit_inside_q             <= 1'b0;
            out_ray_dx_q16           <= 32'sd0;
            out_ray_dy_q16           <= 32'sd0;
            latched_pose_x_map_mm    <= 15'd12800;
            latched_pose_y_map_mm    <= 15'd12800;
            latched_distance_mm      <= 16'd0;
            latched_world_angle_cdeg <= 16'sd0;
            cordic_x                 <= 32'sd0;
            cordic_y                 <= 32'sd0;
            cordic_z                 <= 32'sd0;
            iteration                <= 4'd0;
            out_rejected             <= 1'b0;
        end else begin
            out_rejected <= 1'b0;
            case (state)
                IDLE: begin
                    if (in_valid && in_ready) begin
                        latched_pose_x_map_mm    <= in_pose_x_mm + 32'sd12800;
                        latched_pose_y_map_mm    <= in_pose_y_mm + 32'sd12800;
                        latched_distance_mm      <= effective_distance_mm;
                        latched_world_angle_cdeg <= normalized_angle_cdeg;
                        if (input_legal) begin
                            cordic_x  <= initial_cordic_x;
                            cordic_y  <= 32'sd0;
                            // 119305 = 2^32 / 36000：把厘度换成 atan_rom 使用的
                            // 32 位归一化角度单位（一整圈 = 2^32）。
                            cordic_z  <= folded_angle_cdeg * 32'sd119305;
                            iteration <= 4'd0;
                            state     <= ROTATE;
                        end else begin
                            out_rejected <= 1'b1;
                        end
                    end
                end

                ROTATE: begin
                    // 两次更新都用时钟沿之前的值，这是 CORDIC 的要求
                    if (cordic_z >= 32'sd0) begin
                        cordic_x <= cordic_x - (cordic_y >>> iteration);
                        cordic_y <= cordic_y + (cordic_x >>> iteration);
                        cordic_z <= cordic_z - atan_rom[iteration];
                    end else begin
                        cordic_x <= cordic_x + (cordic_y >>> iteration);
                        cordic_y <= cordic_y - (cordic_x >>> iteration);
                        cordic_z <= cordic_z + atan_rom[iteration];
                    end
                    if (iteration == CORDIC_ITERS - 1) state <= QUANTIZE;
                    else iteration <= iteration + 1'b1;
                end

                QUANTIZE: begin
                    // 在这里打一拍，让乘法与它驱动的单元格索引各自处在独立的周期里，
                    // 而不是从旋转和偏移加法上一路串下来。
                    offset_x_r     <= offset_world_x_mm;
                    offset_y_r     <= offset_world_y_mm;
                    hit_inside_q   <= world_inside;
                    out_ray_dx_q16 <= rotated_x_q16[31:0];
                    out_ray_dy_q16 <= rotated_y_q16[31:0];
                    if (!world_inside) out_rejected <= 1'b1;
                    state <= QUANT2;
                end

                QUANT2: begin
                    // 量化输出再寄存，隔开符号/乘移位与扫描器的误差链；本级完成后置 pending。
                    hit_x   <= signed_cell(offset_x_r);
                    hit_y   <= signed_cell(offset_y_r);
                    pending <= 1'b1;
                    state   <= OUTPUT;
                end

                OUTPUT: begin
                    if (out_ready) begin
                        pending <= 1'b0;
                        state   <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
