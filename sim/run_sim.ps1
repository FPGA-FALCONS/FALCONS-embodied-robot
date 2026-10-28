# run_sim.ps1 —— 本工程的仿真批量入口（每个测试开独立 vsim 进程）
#
# 为什么必须"每个测试一个进程"：
#   ModelSim 在 -c（命令行）模式下，测试平台里的 $finish 会【终结整个 vsim 进程】。
#   把多个 tb 串在同一个 do 脚本里跑，只有第一个会执行，后面静默跳过且不报错 ——
#   极易误判成"全部通过"（这是模块库 README 与本工程交接文档都强调过的坑）。
#
# 覆盖的测试（都是自检 tb，末尾打印 PASS:/FAIL:）：
#   rc_led_test_tb    RC测试顶层：真实PWM/按键/LED/电机恒禁能
#   rc_probe_report_tb 诊断CSV：逐字节UART接收/快照/边界
#   reset_sync_tb      冷启动与复位去抖
#   ht10a_input_tb  HT10A 通道/标定/开关/失效/启动门禁集成
#   rc_input_tb      遥控槽位映射、归一化/死区/模式两档/按钮(select+confirm)/丢信号/取反/摇杆拨动/掩码有效性
#   chassis_ctrl_tb  差速分解、限幅、安全逻辑、闭环方向、rpm 换算      （23 项）
#   num_text_tb      实时数值格式化（含负数与前导空格）                （96 项）
#   ui_render_tb     中文界面渲染（菜单/高亮/状态栏/门禁提示/动态数值/占位）（36 项）
#   speed_report_tb  串口 CSV 上行 12 列格式（整行逐字符比对）          （4 行）
#   odom_tb          航位推算（静止/直行/航向参与/解缠/★中点积分）      （20 项）
#   cordic_atan2_tb  CORDIC 四象限 atan2（象限/正前后/零向量/对角/幅值）（12 项）
#   nav_ctrl_tb      导航控制 10 组状态转移边（ALIGN⇄TRACK/L1/L2/暂停/故障）（91 项）
#   nav_sys_tb       UI+控制 联合台：按键序列 → 电机指令（暂停停车/完成回 SETUP）/（61 项）
#   tune_ctrl_tb     参数整定页控制器（选行/调值/钳位/连发/复位/退出/手动档）（60 项）
#   tune_page_tb     参数整定页像素级渲染（菜单项/标题/11 行标签与数值/单位/高亮条）（45 项）
#
# 编码：本文件必须保存为 UTF-8 with BOM（PowerShell 5.1 靠 BOM 判编码）。
#
# 输出：屏幕只回显"检查行 + 结论"；每个测试的**完整输出**落盘
#       logs\sim_<test>.log（UTF-8 无 BOM，logs\ 已被 .gitignore 忽略）。
#       ★ 判 PASS 的三条硬条件（2026-10-16 收紧，见判定段注释）：
#         ① PASS:/FAIL 都**锚行首**匹配；② 先做 zeroFail 抵消（"0 FAIL" 不算失败）；
#         ③ tb 打印的 `CHECKS: N` 必须与实测 pass 行数一致（**仅对打印 CHECKS 的 tb 生效** —— 当前新增诊断与复位测试及ht10a测试均打印CHECKS）。
#       ★ 每次运行前会**清空并重建**仿真库（**ModelSim 引擎**；Icarus 分支每台全量重编，不建库；只清 sim\work_sim 这一个目录）——
#         防止库里残留上次编译的旧模块，把"漏列源文件"掩盖成假通过。
#
# 用法：
#   powershell -File sim\run_sim.ps1
#   powershell -File sim\run_sim.ps1 -ModelSim 'D:\Apps\modeltech64_10.6d\win64'

[CmdletBinding()]
param(
    [string]$ModelSim = 'D:\Apps\modeltech64_10.6d\win64',
    [string]$Library  = 'work_sim',
    [ValidateSet('ModelSim','Icarus')][string]$Simulator = 'ModelSim',
    [string[]]$Only = @()
)

$ErrorActionPreference = 'Stop'

# 【已知限制，2026-10-16 记录】logs\sim_<test>.log 里 tb 的**中文** $display 是乱码
#   （形如 "??????"、夹杂少量怪字），而 ASCII 部分（pass / FAIL / 断言名 / 数值）完好。
#   根因在 ModelSim 侧：vsim 把 UTF-8 无 BOM 的 tb 源文件读进来后，$display 字符串按
#   本地代码页输出，无法表示的字符在**字节层面**就已经是 '?'（0x3F，实测 dump 过）——
#   信息在进入 PowerShell 之前就丢了，**脚本侧修不了**。
#   （试过把 [Console]::OutputEncoding 设成 UTF-8：残余字节只会从"怪汉字"变成 U+FFFD，
#     可读性没有任何改善，所以不保留这个设置 —— 免得给人"已经修好了"的错觉。）
#   影响面：仅段标题行（"-- xxx --"）可读性差；判定与断言定位不受影响，
#   因为断言名、pass/FAIL、检查数全部是 ASCII。
#   真要可读的中文日志，得从 tb 侧解决（$display 只用英文，或让 vsim 以 UTF-8 输出），
#   不在本次改动范围内。

$ProjectRoot = Split-Path -Parent $PSScriptRoot          # sim\ 的上一级 = 工程根
$VendorRoot  = Join-Path $ProjectRoot 'vendor\verilog-reusable-modules\modules'
$UiIncDir    = Join-Path $ProjectRoot 'rtl\ui'           # ui_text_index.vh 所在目录
$HdmiIncDir  = Join-Path $VendorRoot 'hdmi_1024x600\rtl'      # disp_parameter_cfg.vh 所在目录
$LogDir      = Join-Path $ProjectRoot 'logs'             # 完整测试输出落盘目录（.gitignore 已忽略 logs\）

# ------------------------------------------------------------------
# 运行一个原生命令并完整捕获 stdout+stderr，返回 { Output, ExitCode }
#   ★ 为什么必须有这个包装（2026-10-16 加）：
#     本脚本开头是 `$ErrorActionPreference = 'Stop'`，而 `2>&1` 会把子进程的
#     stderr 行包装成 **ErrorRecord** —— 在 Stop 语义下第一行 stderr 就抛终止性
#     异常，脚本直接崩掉：既看不到测试输出、也拿不到非零退出码。
#     所以这些原生调用期间临时把偏好设为 'Continue'，调用结束再恢复；
#     非零退出码由调用方判定成 FAIL（而不是让脚本崩）。
# ------------------------------------------------------------------
function Invoke-Capture {
    param([scriptblock]$Command)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = & $Command 2>&1
        $code   = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
    return [pscustomobject]@{ Output = @($output); ExitCode = $code }
}

if ($Simulator -eq 'ModelSim') {
    $vlib = Join-Path $ModelSim 'vlib.exe'
    $vlog = Join-Path $ModelSim 'vlog.exe'
    $vsim = Join-Path $ModelSim 'vsim.exe'
    # vdel 是"每次运行前清库"用的（见下面的建库段）。
    #   ★ 它是**可选**的：不存在就回退成直接删库目录，所以不参与下面的存在性检查。
    $vdel = Join-Path $ModelSim 'vdel.exe'
    foreach ($exe in @($vlib, $vlog, $vsim)) {
        if (-not (Test-Path $exe)) { throw "Missing $exe; specify -ModelSim or use -Simulator Icarus." }
    }
} else {
    $iverilog = (Get-Command iverilog -ErrorAction Stop).Source
    $vvp = Join-Path (Split-Path -Parent $iverilog) 'vvp.exe'
    if (-not (Test-Path $vvp)) { throw "Missing $vvp" }
}

Set-Location $PSScriptRoot

# ---------- 源文件（顺序无关，vlog 会各自独立编译）----------
$sources = @(
    # 内嵌模块库（只编译本工程用到的）
    "$VendorRoot\rc_pwm_receiver\rtl\rc_pwm_capture.v",
    "$VendorRoot\motor_tb6612\rtl\motor_pwm.v",
    "$VendorRoot\motor_tb6612\rtl\quad_decoder.v",
    "$VendorRoot\motor_tb6612\rtl\speed_measure.v",
    "$VendorRoot\motor_tb6612\rtl\stall_detect.v",
    "$VendorRoot\motor_tb6612\rtl\motor_unit.v",
    "$VendorRoot\motor_tb6612\rtl\motor_top.v",
    # 串口包（speed_report 内部例化 uart_tx_queue；falcons_top 用 uart_byte_rx）
    #   ★ 漏这几个文件时【编译仍然全过】—— vlog 是逐文件独立编译的，不做模块解析；
    #     直到 vsim 加载 design 才报 "Module 'uart_tx_queue' is not defined"，
    #     在 run_sim.ps1 的输出里表现为 speed_report 测试 "Error loading design"。
    "$VendorRoot\uart\rtl\uart_byte_tx.v",
    "$VendorRoot\uart\rtl\uart_tx_queue.v",
    "$VendorRoot\uart\rtl\uart_byte_rx.v",
    "$VendorRoot\uart\rtl\uart_rx_queue.v",
    # FIFO 包：uart_tx_queue / uart_rx_queue 内部都例化 sync_fifo
    #   ★ 依赖是【两层】的：speed_report → uart_tx_queue → sync_fifo。
    #     补依赖时只补一层仍会在 vsim 加载时报下一个没定义的模块，
    #     所以这里一次性把闭包补全（sync_fifo.v 是叶子，无下级例化）。
    "$VendorRoot\sync_fifo\rtl\sync_fifo.v",
    # 应用层
    "$ProjectRoot\rtl\common\font_rom.v",
    # ★ 2026-10-17：24x24 大字库（左侧菜单条用）。ui_render 现在有大小两条并行通路，
    #   必须两个都编进来，否则 u_font_rom_lg 会 "not defined"。
    "$ProjectRoot\rtl\common\font_rom_lg.v",
    # ★ 2026-10-17 晚：16x24 紧凑数字字库（数据页数值用）。不编进来 u_font_num 会 not defined。
    "$ProjectRoot\rtl\common\font_num.v",
    "$ProjectRoot\rtl\common\ui_text_rom.v",
    "$ProjectRoot\rtl\common\app_key.v",
    "$ProjectRoot\rtl\common\reset_sync.v",
    "$ProjectRoot\rtl\rc\rc_input.v",
    "$ProjectRoot\rtl\rc\ht10a_input.v",
    "$ProjectRoot\rtl\rc\rc_drive_guard.v",
    "$ProjectRoot\rtl\rc\manual_takeover.v",
    "$ProjectRoot\rtl\rc\rc_led_observer.v",
    "$ProjectRoot\rtl\debug\rc_probe_report.v",
    "$ProjectRoot\rtl\top\rc_led_test_top.v",
    "$ProjectRoot\rtl\motor\speed_pid.v",
    "$ProjectRoot\rtl\motor\chassis_ctrl.v",
    "$ProjectRoot\rtl\ui\dyn_text_ram.v",
    "$ProjectRoot\rtl\debug\speed_report.v",
    "$ProjectRoot\rtl\ui\num_text.v",
    "$ProjectRoot\rtl\ui\ui_render.v",
    "$ProjectRoot\rtl\ui\nav_free_ctrl.v",
    "$ProjectRoot\rtl\ui\div_iter.v",
    "$ProjectRoot\rtl\ui\tune_ctrl.v",
    # 导航控制层（2026-10-19 新增）：cordic_atan2 先编，nav_ctrl 里例化它
    "$ProjectRoot\rtl\nav\cordic_atan2.v",
    "$ProjectRoot\rtl\nav\nav_ctrl.v",
    # 测试平台
    "$PSScriptRoot\rc_input_tb.v",
    "$PSScriptRoot\rc_led_test_tb.v",
    "$PSScriptRoot\rc_probe_report_tb.v",
    "$PSScriptRoot\reset_sync_tb.v",
    "$PSScriptRoot\ht10a_input_tb.v",
    "$PSScriptRoot\chassis_ctrl_tb.v",
    "$PSScriptRoot\encoder_chain_tb.v",
    "$PSScriptRoot\num_text_tb.v",
    "$PSScriptRoot\ui_render_tb.v",
    "$PSScriptRoot\ui_pipeline_tb.v",
    "$PSScriptRoot\nav_free_tb.v",
    "$PSScriptRoot\cordic_atan2_tb.v",
    "$PSScriptRoot\nav_ctrl_tb.v",
    "$PSScriptRoot\nav_sys_tb.v",
    "$PSScriptRoot\tune_ctrl_tb.v",
    "$PSScriptRoot\tune_page_tb.v",
    "$PSScriptRoot\speed_report_tb.v",
    # 航位推算（2026-10-18 新增）
    "$ProjectRoot\rtl\odom\sin_lut_rom.v",
    "$ProjectRoot\rtl\odom\odom.v",
    "$PSScriptRoot\odom_tb.v",
    # 2026-10-20 新增：三处时序优化各自的模块级 tb
    #   ★ disp_driver 依赖 disp_parameter_cfg.vh（同目录），必须给 +incdir
    "$VendorRoot\hdmi_1024x600\rtl\disp_driver.v",
    #   ★ imu_top 的闭包是两层：imu_top → spi_master_mode0 / bmi270_cfg_rom
    "$VendorRoot\imu660ra\rtl\imu_top.v",
    "$VendorRoot\imu660ra\rtl\spi_master_mode0.v",
    "$VendorRoot\imu660ra\rtl\bmi270_cfg_rom.v",
    "$PSScriptRoot\disp_driver_tb.v",
    "$PSScriptRoot\dyn_text_ram_tb.v",
    "$PSScriptRoot\imu_top_tb.v",
    "$PSScriptRoot\imu_calib_tb.v"
)

# 导航的正式依赖，不编译独立雷达/显示示例。
$sources += @(Get-ChildItem "$VendorRoot\auto_navigation\rtl\*.v" | ForEach-Object { $_.FullName })
$sources += @(Get-ChildItem "$VendorRoot\radar_navigation\rtl\*.v" | ForEach-Object { $_.FullName })
$sources += @("$VendorRoot\m10_radar\rtl\m10_radar.v", "$VendorRoot\m10_radar\rtl\m10_packet_parser.v",
              "$ProjectRoot\rtl\nav\nav_point_ctrl.v", "$PSScriptRoot\nav_point_ctrl_tb.v",
              "$ProjectRoot\rtl\nav\nav_pose.v", "$PSScriptRoot\radar_map_tb.v", "$PSScriptRoot\nav_pose_tb.v",
              "$ProjectRoot\rtl\ui\nav_map_view.v", "$PSScriptRoot\nav_map_view_tb.v", "$PSScriptRoot\nav_command_tb.v",
              "$ProjectRoot\rtl\ui\nav_obstacle_ui.v",
              "$PSScriptRoot\obstacle_page_tb.v",
              "$PSScriptRoot\obstacle_display_tb.v", "$PSScriptRoot\obstacle_plan_tb.v",
              "$PSScriptRoot\obstacle_motion_tb.v", "$PSScriptRoot\nav_scan_accumulator_tb.v", "$PSScriptRoot\nav_geometry_tb.v", "$PSScriptRoot\radar_coordinates_tb.v", "$PSScriptRoot\nav_planner_tb.v", "$PSScriptRoot\nav_dijkstra_tb.v", "$PSScriptRoot\nav_dijkstra_pool_tb.v", "$PSScriptRoot\nav_line_cells_tb.v", "$PSScriptRoot\nav_route_tb.v", "$PSScriptRoot\nav_center_tb.v", "$PSScriptRoot\nav_planner_limit_tb.v", "$PSScriptRoot\nav_drive_tb.v")

$sources += @("$ProjectRoot\rtl\common\display_mm_to_cm.v", "$PSScriptRoot\display_mm_to_cm_tb.v", "$PSScriptRoot\ui_layers_tb.v")

# Voice motion uses production stream isolation and ownership arbitration.
$sources += @("$VendorRoot\voice_motion_bridge\rtl\voice_motion_bridge.v",
              "$ProjectRoot\rtl\debug\uart_command_router.v", "$ProjectRoot\rtl\debug\uart_cmd_rx.v",
              "$ProjectRoot\rtl\motor\voice_nav_arbiter.v",
              "$VendorRoot\voice_motion_bridge\sim\tb_voice_motion_bridge.sv",
              "$PSScriptRoot\tb_uart_command_router.sv", "$PSScriptRoot\voice_motion_integration_tb.sv")

$sources += @("$ProjectRoot\rtl\ai\ai_video_gate.v", "$PSScriptRoot\ai_video_gate_tb.v")
$sources += @("$ProjectRoot\rtl\ai\ai_rc_control.v", "$ProjectRoot\rtl\ai\ai_uart_tx.v",
              "$ProjectRoot\rtl\ai\ai_display_link.v", "$ProjectRoot\rtl\common\ai_menu_font.v",
              "$PSScriptRoot\manual_takeover_tb.v", "$PSScriptRoot\ai_rc_control_tb.v", "$PSScriptRoot\ai_uart_tx_tb.v", "$PSScriptRoot\ai_display_link_tb.v")
$sources += @("$VendorRoot\yaw_integrate\rtl\yaw_integrate.v", "$PSScriptRoot\yaw_pipeline_tb.v")
$sources += @("$VendorRoot\ov5640_capture\rtl\ov5640_init_table_rgb.v", "$PSScriptRoot\camera_rotation_tb.v")
$sources += @("$ProjectRoot\rtl\ai\inspection_packet_rx.v", "$PSScriptRoot\inspection_packet_tb.v",
              "$PSScriptRoot\nav_visual_layer_tb.v", "$PSScriptRoot\visual_navigation_tb.v")
$sources += @("$ProjectRoot\rtl\ai\inspection_nav_control.v", "$ProjectRoot\rtl\ai\inspection_dispatch.v",
              "$ProjectRoot\rtl\ai\inspection_speed_params.v", "$PSScriptRoot\inspection_speed_tb.v",
              "$ProjectRoot\rtl\ai\inspection_packet_tx.v", "$ProjectRoot\rtl\ai\inspection_telemetry.v", "$ProjectRoot\rtl\ai\inspection_application.v",
              "$PSScriptRoot\inspection_control_tb.v", "$PSScriptRoot\inspection_tx_tb.v", "$PSScriptRoot\inspection_dispatch_tb.v", "$PSScriptRoot\inspection_uart_tb.v", "$PSScriptRoot\inspection_application_tb.v", "$PSScriptRoot\inspection_ui_tb.v")
$sources += @("$ProjectRoot\rtl\ai\inspection_fx2_phy.v", "$ProjectRoot\rtl\ai\inspection_stream_rx.v", "$ProjectRoot\rtl\ai\inspection_highspeed.v", "$ProjectRoot\rtl\ai\inspection_fx2_clock.v",
              "$ProjectRoot\rtl\ai\inspection_stream_tx.v", "$ProjectRoot\rtl\common\stream_async_fifo.v",
              "$VendorRoot\camera_fx2_direct\rtl\acg_async_fifo.v", "$PSScriptRoot\inspection_fx2_tb.v",
              "$PSScriptRoot\inspection_stream_tb.v", "$PSScriptRoot\stream_async_fifo_tb.v", "$PSScriptRoot\inspection_highspeed_tb.v", "$PSScriptRoot\nav_map_arb_tb.v", "$PSScriptRoot\stall_monitor_tb.v", "$PSScriptRoot\inspection_fx2_clock_tb.v")
$sources += @(
    "$ProjectRoot\rtl\ai\inspection_mailbox_rx.v",
    "$PSScriptRoot\inspection_mailbox_tb.v",
    "$PSScriptRoot\inspection_timebase_tb.v",
    "$ProjectRoot\rtl\ai\inspection_epoch_tracker.v",
    "$PSScriptRoot\inspection_epoch_tb.v",
    "$ProjectRoot\rtl\ai\inspection_route_export.v",
    "$PSScriptRoot\inspection_route_export_tb.v",
    "$ProjectRoot\rtl\ai\inspection_ddr_arb.v",
    "$ProjectRoot\rtl\ai\inspection_frame_tokens.v",
    "$ProjectRoot\rtl\ai\inspection_frame_writer.v",
    "$ProjectRoot\rtl\ai\inspection_frame_reader.v",
    "$ProjectRoot\rtl\ai\inspection_frame_pixels.v",
    "$ProjectRoot\rtl\ai\inspection_frame_store.v",
    "$ProjectRoot\rtl\ai\inspection_map_export.v",
    "$PSScriptRoot\inspection_ddr_arb_tb.v",
    "$PSScriptRoot\inspection_frame_tb.v", "$PSScriptRoot\inspection_ddr_deadline_tb.v",
    "$PSScriptRoot\inspection_map_export_tb.v"
)
$tests = @(
    @{ Name = 'inspection_epoch'; Top = 'inspection_epoch_tb'; Desc = 'Held IMU valid level / event-only epoch / origin clear / reboot handshake' },
    @{ Name = 'inspection_route_export'; Top = 'inspection_route_export_tb'; Desc = 'Passive navigation prefix / bounded CRC / context and publication' },
    @{ Name = 'inspection_timebase'; Top = 'inspection_timebase_tb'; Desc = 'Device clock exchange / nonce and epoch isolation / no authority' },
    @{ Name = 'inspection_mailbox'; Top = 'inspection_mailbox_tb'; Desc = 'Validated layer-only USB mailbox / UART authority isolation' },
    @{ Name = 'inspection_ddr_arb'; Top = 'inspection_ddr_arb_tb'; Desc = 'Independent DDR AI display / passive map export' },
    @{ Name = 'inspection_frame'; Top = 'inspection_frame_tb'; Desc = 'Independent DDR AI display / passive map export' },
    @{ Name = 'inspection_ddr_deadline'; Top = 'inspection_ddr_deadline_tb'; Desc = 'Display read deadline while camera writes are backpressured' },
    @{ Name = 'inspection_map_export'; Top = 'inspection_map_export_tb'; Desc = 'Independent DDR AI display / passive map export' },

    @{ Name = 'inspection_fx2_clock'; Top = 'inspection_fx2_clock_tb'; Desc = 'Independent Gray frequency measurement / wrong clock / loss and return' },
    @{ Name = 'nav_map_arb'; Top = 'nav_map_arb_tb'; Desc = 'Registered selection / cancellation / stable owner / timeout' },
    @{ Name = 'stall_monitor'; Top = 'stall_monitor_tb'; Desc = 'Coherent monitor pipeline / live PWM inhibit / explicit clear' },
    @{ Name = 'inspection_highspeed'; Top = 'inspection_highspeed_tb'; Desc = 'Whole diagnostic / physical bus / two independent clocks / small FIFO backpressure' },
    @{ Name = 'inspection_stream'; Top = 'inspection_stream_tb'; Desc = 'Host AIF1 vectors / full 816x420 CRC / stale view / bounded mailboxes / transmit backpressure' },
    @{ Name = 'inspection_fx2'; Top = 'inspection_fx2_tb'; Desc = 'Bidirectional Slave FIFO / direction turnaround / short-packet flush / fairness' },
    @{ Name = 'stream_async_fifo'; Top = 'stream_async_fifo_tb'; Desc = 'Independent clocks / full FIFO / tagged stream / backpressure' },
    @{ Name = 'inspection_ui'; Top = 'inspection_ui_tb'; Desc = 'Nested AI task views / CH8 scope / manual preservation / child back' },
    @{ Name = 'inspection_application'; Top = 'inspection_application_tb'; Desc = 'Python transaction to production authority and actual-stop handshake' },
    @{ Name = 'inspection_speed'; Top = 'inspection_speed_tb'; Desc = 'RK physical cruise / iterative slow gain / actual point controller / base independence' },
    @{ Name = 'inspection_dispatch'; Top = 'inspection_dispatch_tb'; Desc = 'Synchronous message RAM / authenticated layer stream / stopped commit' },
    @{ Name = 'inspection_uart'; Top = 'inspection_uart_tb'; Desc = 'Physical UART arbitration / whole CSV / indivisible inspection transaction' },
    @{ Name = 'inspection_control'; Top = 'inspection_control_tb'; Desc = 'Actual stopped handshake, physical confirm, manual priority and local lease' },
    @{ Name = 'inspection_tx'; Top = 'inspection_tx_tb'; Desc = 'Status snapshot / whole transaction / CRC under byte backpressure' },
    @{ Name = 'visual_navigation'; Top = 'visual_navigation_tb'; Desc = 'Camera-only wall changes actual Dijkstra route; AI exit restores radar map' },
    @{ Name = 'inspection_packet'; Top = 'inspection_packet_tb'; Desc = 'Python/RTL transaction CRC, synchronous RAM, corruption and timeout' },
    @{ Name = 'nav_visual_layer'; Top = 'nav_visual_layer_tb'; Desc = 'Atomic visual raw/cost layer, stopped commit, CRC and epoch isolation' },
    @{ Name = 'camera_rotation'; Top = 'camera_rotation_tb'; Desc = 'Camera flip and mirror / all 253 initialization register entries' },
    @{ Name = 'nav_planner_limit'; Top = 'nav_planner_limit_tb'; Desc = 'Post-centering relays / capacity prefix / repeated request / same goal' },
    @{ Name = 'nav_center'; Top = 'nav_center_tb'; Desc = 'Eight-direction intersections / minimum gaps / budget / full-segment rejection' },
    @{ Name = 'nav_route'; Top = 'nav_route_tb'; Desc = 'Eight-direction locked prefix / farthest target / candidate isolation' },
    @{ Name = 'nav_line_cells'; Top = 'nav_line_cells_tb'; Desc = 'Eight-direction inclusive segments and flank cells' },
    @{ Name = 'nav_drive'; Top = 'nav_drive_tb'; Desc = 'Eight-direction raw front checks / lateral / behind / invalid pose' },
    @{ Name = 'nav_dijkstra_pool'; Top = 'nav_dijkstra_pool_tb'; Desc = 'Shared pool full / reuse / restart / reset' },
    @{ Name = 'nav_dijkstra'; Top = 'nav_dijkstra_tb'; Desc = 'Four buckets / 2-3 cost / straight first / no corner cuts / heap reference' },
    @{ Name = 'yaw_pipeline'; Top = 'yaw_pipeline_tb'; Desc = '20000 cycle equivalence for optional yaw sum/wrap pipeline' },
    @{ Name = 'manual_takeover'; Top = 'manual_takeover_tb'; Desc = 'SWC stop before local stick takeover, fresh encoder feedback' },
    @{ Name = 'ai_video_gate'; Top = 'ai_video_gate_tb'; Desc = 'Shared display gate without inspection navigation authority' },
    @{ Name = 'ai_rc_control'; Top = 'ai_rc_control_tb'; Desc = 'AI master/submenu/SWB recording/failsafe' },
    @{ Name = 'ai_uart_tx'; Top = 'ai_uart_tx_tb'; Desc = 'AI control CRC and whole-line CSV UART arbitration' },
    @{ Name = 'ai_display_link'; Top = 'ai_display_link_tb'; Desc = 'Atomic display commit, CRC, stale host and transaction isolation' },
    @{ Name = 'voice_motion_bridge'; Top = 'tb_voice_motion_bridge'; Desc = 'MOT1 ownership, IMU freshness, signed turns, watchdog and command priority' },
    @{ Name = 'uart_command_router'; Top = 'tb_uart_command_router'; Desc = 'Binary isolation, ASCII compatibility and truncated frame recovery' },
    @{ Name = 'voice_motion_integration'; Top = 'voice_motion_integration_tb'; Desc = 'Production router, bridge and authority mux against nonzero navigation' },
    @{ Name = 'ui_layers'; Top = 'ui_layers_tb'; Desc = 'Parallel layer priority / 131072 overlap and palette combinations' },
    @{ Name = 'display_mm_to_cm'; Top = 'display_mm_to_cm_tb'; Desc = 'Signed display conversion / int32 limits / continuous pipeline / reset' },
    @{ Name = 'nav_planner'; Top = 'nav_planner_tb'; Desc = 'Indoor rooms / diagonal walls / 2-3 Dijkstra / centered segments / queue wrap' },
    @{ Name = 'ui_pipeline'; Top = 'ui_pipeline_tb'; Desc = 'Real HDMI scan / six complete pages / baseline pixel fingerprints' },
    @{ Name = 'imu_calib'; Top = 'imu_calib_tb'; Desc = 'IMU calibration pipeline / coherent samples / seed / reject / completion / timeout' },
    @{ Name = 'radar_coordinates'; Top = 'radar_coordinates_tb'; Desc = 'UART quadrants / world map / display / 72 captured packets' },
    @{ Name = 'nav_geometry'; Top = 'nav_geometry_tb'; Desc = '50 mm grid quantization / signed map boundaries / 10 m rays' },
    @{ Name = 'nav_scan_accumulator'; Top = 'nav_scan_accumulator_tb'; Desc = 'Circle hit priority / deferred clears / concurrent circle updates' },
    @{ Name = 'nav_pose'; Top = 'nav_pose_tb'; Desc = 'Real odometry pipeline to navigation pose' },
    @{ Name = 'obstacle_motion'; Top = 'obstacle_motion_tb'; Desc = 'UI / real BFS / point controller / moving differential vehicle' },
    @{ Name = 'nav_command'; Top = 'nav_command_tb'; Desc = 'Active task axis / live target / direction change handshake' },
    @{ Name = 'nav_map_view'; Top = 'nav_map_view_tb'; Desc = 'Map / route double buffering / projected current segment / live targets' },
    @{ Name = 'nav_point_ctrl'; Top = 'nav_point_ctrl_tb'; Desc = 'Point task / live target / emergency handshake' },
    @{ Name = 'obstacle_display'; Top = 'obstacle_display_tb'; Desc = 'Obstacle UI result / resume / retry labels, disabled actions and crosshair' },
    @{ Name = 'obstacle_page'; Top = 'obstacle_page_tb'; Desc = 'Obstacle page and multiple destinations' },
    @{ Name = 'obstacle_plan'; Top = 'obstacle_plan_tb'; Desc = 'Obstacle UI to real BFS preplanning' },
    @{ Name = 'radar_map'; Top = 'radar_map_tb'; Desc = 'UART radar to world map and coherent pose' },
    @{ Name = 'rc_led_test'; Top = 'rc_led_test_tb'; Desc = 'RC-only board top: PWM to LEDs, keys and permanent motor inhibit' },
    @{ Name = 'rc_probe_report'; Top = 'rc_probe_report_tb'; Desc = 'RC diagnostic CSV: real UART decode and atomic snapshot' },
    @{ Name = 'reset_sync'; Top = 'reset_sync_tb'; Desc = 'Cold boot and key debounce without prior reset press' },
    @{ Name = 'ht10a_input'; Top = 'ht10a_input_tb'; Desc = 'HT10A mapping/calibration/switches/loss/manual drive integration' },
    @{ Name = 'rc_input';     Top = 'rc_input_tb';     Desc = '遥控槽位映射：归一化/死区/模式两档/按钮 select+confirm/丢信号/取反/摇杆拨动/掩码有效性' },
    @{ Name = 'chassis_ctrl'; Top = 'chassis_ctrl_tb'; Desc = '★ 差速分解/限幅/安全逻辑/闭环方向/rpm 换算' },
    @{ Name = 'encoder_chain'; Top = 'encoder_chain_tb'; Desc = 'Four real encoder channels / fast pulses / 10ms windows / missing phase reproduction' },
    @{ Name = 'num_text';     Top = 'num_text_tb';     Desc = '实时数值格式化（9 字段、负数、前导空格、饱和）' },
    @{ Name = 'ui_render';    Top = 'ui_render_tb';    Desc = '★ 中文界面：菜单/高亮随 menu_sel/状态栏/光标档文案/动态数值/占位页/非避障导航页' },
    @{ Name = 'nav_free';     Top = 'nav_free_tb';     Desc = '原非避障调试选点（保持原功能）' },
    @{ Name = 'speed_report'; Top = 'speed_report_tb'; Desc = '串口 CSV 上行：12 列格式/负数/边界/大数' },
    @{ Name = 'odom';      Top = 'odom_tb';      Desc = '★ 航位推算：静止/直行位移/航向参与/角度解缠' },
    @{ Name = 'cordic_atan2'; Top = 'cordic_atan2_tb'; Desc = '★ CORDIC 四象限 atan2：象限/正前后/零向量/对角/幅值 10mm~10m' },
    @{ Name = 'nav_ctrl';  Top = 'nav_ctrl_tb';  Desc = '★ 导航控制：ALIGN⇄TRACK 迟滞/L1 到点/L2 切点四种降级/暂停制动/故障/避障准心' },
    @{ Name = 'nav_sys';   Top = 'nav_sys_tb';   Desc = '原非避障 UI 与控制器联合回归' },
    @{ Name = 'tune_ctrl'; Top = 'tune_ctrl_tb'; Desc = '★ 参数整定页：选行回绕/单步调值/上下限钳位/长按连发/复位/退出/手动档禁调/K_SLOW 重算' },
    @{ Name = 'tune_page'; Top = 'tune_page_tb'; Desc = '★ 参数整定页渲染：菜单第 6 项/标题/11 行标签与数值/单位 5 行白字/高亮条/占位页互斥' },
    @{ Name = 'disp_driver'; Top = 'disp_driver_tb'; Desc = '★ HDMI 像素驱动：扫描计数/DE 窗口边界/逐拍像素透传' },
    @{ Name = 'dyn_text_ram'; Top = 'dyn_text_ram_tb'; Desc = '★ 动态字库 RAM：64 地址读写/1 拍读延迟/双时钟并行' },
    @{ Name = 'imu_top';    Top = 'imu_top_tb';    Desc = '★ IMU 角速度/加速度换算：增益系数与符号扩展/三轴独立' }
)

if ($Only.Count) {
    $Only = @($Only -split ',')
    $tests = @($tests | Where-Object { $Only -contains $_.Name })
    if (-not $tests.Count) { throw 'No tests selected' }
}

# 生成确定性的室内地图与独立堆式 Dijkstra 参考，产物留在 logs/。
if (@($tests | Where-Object { $_.Name -in @('nav_dijkstra','nav_planner') }).Count) {
    & python (Join-Path $PSScriptRoot 'dijkstra_eight.py') --output (Join-Path $LogDir 'dijkstra/fixtures')
    if ($LASTEXITCODE -ne 0) { throw 'Eight-direction fixture generation failed' }
}

# ---------- 建库并编译 ----------
Write-Host ''
Write-Host '================ 编译（FALCONS 仿真）================' -ForegroundColor Cyan

$libraryPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot $Library))
# ★ 库的**逻辑名**：vsim 的 design 名必须写成 `<库逻辑名>.<top>`，
#   不能拿 $Library 变量或整条路径去拼（路径形式在 -work 里能用，但 design 名不行）。
$libName     = Split-Path -Leaf $libraryPath
$simPrefix = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\') + '\'
if (-not $libraryPath.StartsWith($simPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Library must stay within sim/.'
}
# ★★ 破坏面守卫（2026-10-16 加固，**不要删**）：上面的 StartsWith 只判"是否落在 sim\ 前缀之内"，
#   而 **sim\ 根路径本身也算"之内"** —— 传 `-Library .` 时 GetFullPath 得到的正是 sim\，
#   守卫通过，紧接着的 `Remove-Item -Recurse -Force $libraryPath` 会**递归删掉整个 sim\**
#   （6 个测试台、sim\README.md、本脚本自身）。所以这里再挡一次：库路径去掉结尾反斜杠后
#   等于 sim\ 根路径 → 直接拒绝。
$simRootPath = [System.IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\')
if ($libraryPath.TrimEnd('\') -eq $simRootPath) {
    throw 'Library must not be the sim\ root itself (-Library . would delete the whole sim\ directory).'
}
if ($Simulator -eq 'ModelSim') {
    # ★ 2026-10-16：每次运行前**先清空、再重建**这个库（原来是"库存在就复用"）。
    #   为什么必须清：复用会让库里留着**上一次运行编译出的旧模块**。实测
    #   sim\work_sim\_info 里存在模块 `vfalcons_top`，而 $sources 里根本没有这个源文件
    #   —— 历史遗留（顶层曾改名过一次）。危害是：以后把某个源文件从 $sources 里
    #   **漏掉**时，vlog 照样成功、vsim 也照样能加载（库里还留着上次编出来的那份），
    #   于是"漏列依赖"不再暴露 = **假通过**。
    #   实现：优先用 vdel -lib <该库> -all 清空；vdel 不存在或没清干净时，
    #   再直接删这一个库目录，然后 vlib 重建。
    #   ★ 只清这一个仿真库目录（sim\work_sim），不动 work_synt / work_check 等其它产物。
    if (Test-Path $vdel) {
        # 同样用 Invoke-Capture：Stop 语义下原生命令的 stderr 会变成终止性异常
        $null = Invoke-Capture { & $vdel -lib $libraryPath -all }
    }
    if (Test-Path $libraryPath) { Remove-Item -Recurse -Force $libraryPath }
    & $vlib $libraryPath | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    foreach ($f in $sources) {
        if (-not (Test-Path $f)) { throw "Missing source: $f" }
        $r = Invoke-Capture { & $vlog -work $libraryPath "+incdir+$UiIncDir" "+incdir+$HdmiIncDir" $f 2>&1 }
        if ($r.ExitCode -ne 0) { $r.Output | ForEach-Object { Write-Host $_ }; throw "Compile failed: $f" }
    }
} else {
    New-Item -ItemType Directory -Path $libraryPath -Force | Out-Null
}
Write-Host "Source list: $($sources.Count) files; simulator: $Simulator" -ForegroundColor Green

# ---------- 逐个测试（每个独立进程）----------
$results = @()

foreach ($t in $tests) {
    Write-Host ''
    Write-Host "================ 测试：$($t.Name) ================" -ForegroundColor Cyan
    Write-Host "  $($t.Desc)"

    if ($Simulator -eq 'ModelSim') {
        # design 名用**库逻辑名** $libName（原来用 $Library 变量拼，一旦 -Library
        # 传的是"库名\子目录"这类形式就会拼错；现在统一从解析后的路径取名字）
        $r = Invoke-Capture { & $vsim -c -work $libraryPath -do "run -all; quit -f" "$libName.$($t.Top)" 2>&1 }
    } else {
        $image = Join-Path $libraryPath "$($t.Top).out"
        $c = Invoke-Capture { & $iverilog -g2012 -I $UiIncDir -I $HdmiIncDir -s $t.Top -o $image @sources 2>&1 }
        if ($c.ExitCode -ne 0) { $c.Output | ForEach-Object { Write-Host $_ }; throw "Compile failed: $($t.Top)" }
        $r = Invoke-Capture { & $vvp $image 2>&1 }
    }
    $testExit = $r.ExitCode
    $out      = @($r.Output)
    $text     = ($out | ForEach-Object { $_.ToString() }) -join "`n"

    # ---------- 完整输出落盘 logs\sim_<test>.log（UTF-8 无 BOM）----------
    #   ★ 原来只把匹配 'pass |FAIL|PASS:|FAIL:|-- ' 的行回显到屏幕，其余**全丢**：
    #     "Error loading design"、"Module 'uart_tx_queue' is not defined"、
    #     编译告警等**最关键的失败信息恰好都不匹配那几个模式** → 屏幕上什么都没有，
    #     只剩一句 NO-RESULT，根本无从定位。
    #     现在：完整输出一律落盘（logs\ 已被 .gitignore 忽略），屏幕仍只回显检查行。
    if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }
    $logFile = Join-Path $LogDir "sim_$($t.Name).log"
    $logText = "# 测试：$($t.Name)  top=$($t.Top)  模拟器=$Simulator  退出码=$testExit`n" +
               "# 下面是完整输出（屏幕只回显了检查行与结论）`n" + $text + "`n"
    [System.IO.File]::WriteAllText($logFile, $logText, (New-Object System.Text.UTF8Encoding($false)))

    # 只回显检查行与结论（tb 输出很多；完整输出见上面的日志文件）
    $out | ForEach-Object {
        $s = $_.ToString().TrimStart('#',' ')
        if ($s -match '^\s*(pass |FAIL|PASS:|FAIL:|-- )') { Write-Host "  $s" }
    }
    Write-Host "  （完整输出：$logFile）" -ForegroundColor DarkGray

    # ---------- PASS / FAIL 判定（2026-10-16 收紧）----------
    #   ① 匹配**必须锚行首**：原写法 `(?im)^\s*(?:#\s*)?(?:FAIL|FATAL|\*\* Error)`
    #      能匹配任何"行首就是这些词"的内容（含 tb 里打印的说明句），
    #      而 PASS 用的是**不锚行首**的 'PASS:' —— 只要输出里出现过 "PASS:" 就算过。
    #      现在两边都锚行首：`(?m)^\s*#?\s*FAIL` 与 `(?m)^\s*#?\s*PASS:`。
    #   ② 先做 zeroFail 抵消：形如 "0 FAIL" / "FAIL: 0 ..." 是**通过**的汇总句，
    #      不能当失败，先在文本里剔掉再判。
    #   ③ 解析 tb 自己打印的 `CHECKS: N`，与实测 pass 行数比对：不一致即 FAIL ——
    #      防"断言被删/被跳过、但结论照样打印 PASS"。tb 不打印 CHECKS 时跳过这一项。
    $textNZF = $text -replace '(?m)^\s*#?\s*(?:0\s+FAIL\b.*|FAIL:\s*0\b.*)$', ''
    $passCount = ([regex]::Matches($textNZF, '(?m)^\s*#?\s*pass\b')).Count
    $checksM   = [regex]::Match($textNZF, '(?m)^\s*#?\s*CHECKS:\s*(\d+)\s*$')
    $checkTotal = if ($checksM.Success) { [int]$checksM.Groups[1].Value } else { -1 }

    #   ④ 兜底：有一类 tb（cordic_atan2 / nav_ctrl / nav_sys）逐项打印用 "ok" 前缀、
    #      不打 `CHECKS: N`，只把总数放在结论行 `PASS: <名> <N> 项全过` 里。
    #      这类 tb 的 pass 行数恒为 0，汇总会显示 pass=0 让人误判成"没跑"。
    #      此时从结论行取 N 作为总数并让 passCount 对齐。
    #      ★ 注意：这不做交叉校验（本来就没有 pass 行可对），只是把 tb 自报的数字显示出来；
    #        有 CHECKS 行的 tb 仍走上面的严格比对，语义不变。
    #      ★ 正则只用 ASCII 锚定：ModelSim 输出被 PowerShell 捕获时中文会变成乱码，
    #        写成 `(\d+)\s*项` 会因字面量"项"对不上而永远不匹配。
    if ($checkTotal -lt 0 -and $passCount -eq 0) {
        $sumM = [regex]::Match($textNZF, '(?m)^\s*#?\s*PASS:\s*\S+\s+(\d+)\b')
        if ($sumM.Success) {
            $checkTotal = [int]$sumM.Groups[1].Value
            $passCount  = $checkTotal
        }
    }

    if ($testExit -ne 0) {
        $status = 'FAIL'; $why = "退出码 $testExit"
    } elseif ($textNZF -match '(?m)^\s*#?\s*(?:FAIL|FATAL|\*\* Error)') {
        $status = 'FAIL'; $why = '输出里有 FAIL/FATAL/Error'
    } elseif ($textNZF -notmatch '(?m)^\s*#?\s*PASS:') {
        $status = 'NO-RESULT'; $why = '没有行首 PASS: 结论行'
    } elseif ($checkTotal -ge 0 -and $checkTotal -ne $passCount) {
        $status = 'FAIL'; $why = "CHECKS=$checkTotal 与实测 pass=$passCount 不一致"
    } else {
        $status = 'PASS'
        $why = if ($checkTotal -ge 0) { "pass=$passCount / CHECKS=$checkTotal" } else { "pass=$passCount" }
    }

    $results += [pscustomobject]@{ 测试 = $t.Name; 结果 = $status; 检查数 = $passCount; 说明 = $why }
}

# ---------- 汇总 ----------
Write-Host ''
Write-Host '================ 汇总 ================' -ForegroundColor Cyan
$results | Format-Table 测试, 结果, 检查数, 说明 -AutoSize

$bad = @($results | Where-Object { $_.结果 -ne 'PASS' })
if ($bad.Count -eq 0) {
    Write-Host "全部 $($tests.Count) 个测试通过" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$($bad.Count) 个测试未通过：$(($bad.测试) -join ', ')" -ForegroundColor Red
    exit 1
}
