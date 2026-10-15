# ModelSim 脚本：imu660ra 模块测试
#
# 用法（ModelSim GUI 的 Transcript）：
#       cd {<本模块>/sim}
#       do run_test.do
#
# ⚠️ 本脚本用「vsim work.xxx / run -all / quit -sim」的内建写法，只适用于
#    GUI Transcript。**不要**写成 `vsim -c -do "run -all; quit -f" work.xxx`：
#    那样会启动子进程并执行 quit -f，把整个 ModelSim 一起杀掉，结果是脚本
#    只跑完第一个测试就静默退出（后面的测试不执行、也不报错）。
#    命令行下请给每个测试开独立进程，例如：
#       vsim -c -do "run -all; quit -f" work.spi_master_mode0_tb
#       vsim -c -do "run -all; quit -f" work.imu_top_tb
#
# 本模块不依赖其他任何模块，故编译列表只有自身 rtl 与验证环境。

if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work

# 被测 RTL
vlog ../rtl/bmi270_cfg_rom.v
vlog ../rtl/spi_master_mode0.v
vlog ../rtl/imu_top.v

# 验证环境
vlog bmi270_model.v
vlog spi_master_mode0_tb.v
vlog imu_top_tb.v

echo "======== 测试1：spi_master_mode0 ========"
vsim -quiet work.spi_master_mode0_tb
run -all
quit -sim

echo "======== 测试2：imu_top（BMI270 初始化与读取）========"
vsim -quiet work.imu_top_tb
run -all
quit -sim

echo ""
echo "======== 测试执行完毕 ========"
