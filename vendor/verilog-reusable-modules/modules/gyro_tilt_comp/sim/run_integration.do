# ModelSim 脚本：gyro_tilt_comp 集成测试（imu_top + gyro_tilt_comp）
#
# 用法（ModelSim GUI 的 Transcript）：
#       cd {<本模块>/sim}
#       do run_integration.do
#
# 【为什么要单独一个脚本】本测试把 imu660ra 的 imu_top 与 gyro_tilt_comp 按
# 真实连接方式接起来，专门覆盖"标定窗口内能否采到加速度样本"这个跨模块契约：
# imu_top 的 o_data_valid 只在 RUN 态产生，若误接到 gyro_tilt_comp 的
# i_sample_valid，标定期一个样本都采不到、增益永远停在 1.0 —— 单元测试对此
# 完全无感。详见本模块 README「标定窗口」一节。
#
# ⚠️ 依赖模块库中的 imu660ra 模块（用相对路径引用，不复制文件）。
#    若只取走本模块，本脚本会因找不到文件而失败，此时请改用 run_test.do。

if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work

# 被测链路：imu660ra 的 RTL
vlog ../../imu660ra/rtl/bmi270_cfg_rom.v
vlog ../../imu660ra/rtl/spi_master_mode0.v
vlog ../../imu660ra/rtl/imu_top.v
vlog ../rtl/gyro_tilt_comp.v

# 验证环境
vlog ../../imu660ra/sim/bmi270_model.v
vlog tilt_integration_tb.v

echo "======== 集成测试：imu_top + gyro_tilt_comp（3 个用例）========"
vsim -quiet work.tilt_integration_tb
run -all
quit -sim

echo ""
echo "======== 测试执行完毕 ========"
