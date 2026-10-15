# ModelSim 脚本：motor_tb6612 模块包测试
#
# 用法（ModelSim GUI 的 Transcript）：
#       cd {<本模块>/sim}
#       do run_test.do
#
# ⚠️ 本脚本用「vsim work.xxx / run -all / quit -sim」的内建写法，只适用于
#    GUI Transcript。在 `-c`（命令行）模式下，tb 里的 $finish 会【终结整个
#    vsim 进程】—— 因此串行跑多个 tb 时只有第一个会执行，后面的静默跳过、
#    且不报错。命令行请改用 run_motor.ps1（给每个测试开独立进程）。
#
# 本包含 5 个测试平台：4 个原子模块单元测试 + 1 个四路集成测试。

if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work

# 被测 RTL
vlog ../rtl/motor_pwm.v
vlog ../rtl/quad_decoder.v
vlog ../rtl/speed_measure.v
vlog ../rtl/stall_detect.v
vlog ../rtl/motor_unit.v
vlog ../rtl/motor_top.v

# 验证环境
vlog motor_pwm_tb.v
vlog quad_decoder_tb.v
vlog speed_measure_tb.v
vlog stall_detect_tb.v
vlog motor_top_tb.v

echo "======== 1/5 motor_pwm：单路 PWM（占空比 / 方向 / 刹车 / 安全态）========"
vsim -quiet work.motor_pwm_tb
run -all
quit -sim

echo "======== 2/5 quad_decoder：正交解码（4 倍频 / 去毛刺 / 非法跳变）========"
vsim -quiet work.quad_decoder_tb
run -all
quit -sim

echo "======== 3/5 speed_measure：定窗口测速 ========"
vsim -quiet work.speed_measure_tb
run -all
quit -sim

echo "======== 4/5 stall_detect：堵转保护（净位移判据）========"
vsim -quiet work.stall_detect_tb
run -all
quit -sim

echo "======== 5/5 motor_top：★ 四路集成 ========"
vsim -quiet work.motor_top_tb
run -all
quit -sim

echo ""
echo "======== 全部测试执行完毕 ========"
