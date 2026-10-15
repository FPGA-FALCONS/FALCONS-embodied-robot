# ModelSim 脚本：gyro_tilt_comp 单元测试
#
# 用法（ModelSim GUI 的 Transcript）：
#       cd {<本模块>/sim}
#       do run_test.do
#
# ⚠️ 本脚本用「vsim work.xxx / run -all / quit -sim」的内建写法，只适用于
#    GUI Transcript。在 `-c`（命令行）模式下，tb 里的 $finish 会【终结整个
#    vsim 进程】，必须给每个测试开独立进程；本模块只有一个单元测试平台，
#    命令行下直接执行：
#       vsim -c -do "run -all; quit -f" work.gyro_tilt_comp_tb
#    即可，无需本脚本。
#
# 集成测试见同目录的 run_integration.do（需要 imu660ra 模块）。

if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work

# 被测 RTL
vlog ../rtl/gyro_tilt_comp.v

# 验证环境
vlog gyro_tilt_comp_tb.v

echo "======== gyro_tilt_comp：倾斜补偿单元测试（9 个用例）========"
vsim -quiet work.gyro_tilt_comp_tb
run -all
quit -sim

echo ""
echo "======== 测试执行完毕 ========"
