# ModelSim 脚本：yaw_integrate 模块测试
#
# 用法（ModelSim GUI 的 Transcript）：
#       cd {<本模块>/sim}
#       do run_test.do
#
# ⚠️ 本脚本用「vsim work.xxx / run -all / quit -sim」的内建写法，只适用于
#    GUI Transcript。在 `-c`（命令行）模式下，tb 里的 $finish 会【终结整个
#    vsim 进程】，必须给每个测试开独立进程；本模块只有一个测试平台，命令行
#    下直接执行：
#       vsim -c -do "run -all; quit -f" work.yaw_integrate_tb
#    即可，无需本脚本。
#
# 本模块不依赖其他任何模块，编译列表只有自身 rtl 与验证环境。

if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work

# 被测 RTL
vlog ../rtl/yaw_integrate.v

# 验证环境
vlog yaw_integrate_tb.v

echo "======== yaw_integrate：偏航角积分（11 个用例）========"
vsim -quiet work.yaw_integrate_tb
run -all
quit -sim

echo ""
echo "======== 测试执行完毕 ========"
