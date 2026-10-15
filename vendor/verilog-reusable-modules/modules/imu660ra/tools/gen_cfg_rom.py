# -*- coding: utf-8 -*-
"""
把 rom/imu660ra_config_file.mem 转换成自包含的 Verilog ROM 模块。

背景：实测 Gowin 综合器对 $readmemh 的处理不可靠——综合后网表里 BSRAM 的
      INIT_RAM 参数并不包含 .mem 的数据（无论用相对路径还是绝对路径都一样），
      导致 BMI270 的 8KB 配置灌入的是无效数据、初始化失败。
      因此改为把 8192 字节直接内嵌到源文件，彻底消除外部文件与路径依赖。
      （做法与模块库中已上板验证的 ov5640_init_table_rgb.v 一致）

用法：在模块根目录执行 python tools/gen_cfg_rom.py
"""

import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEM  = os.path.join(BASE, "rom", "imu660ra_config_file.mem")
OUT  = os.path.join(BASE, "rtl", "bmi270_cfg_rom.v")

#---------------------------------------------------------------------
# 读取并校验数据
#---------------------------------------------------------------------
if not os.path.exists(MEM):
    sys.exit("找不到 %s" % MEM)

with open(MEM, "r") as f:
    data = [ln.strip() for ln in f if ln.strip()]

if len(data) != 8192:
    sys.exit("期望 8192 字节，实际 %d" % len(data))

for i, b in enumerate(data):
    try:
        int(b, 16)
    except ValueError:
        sys.exit("第 %d 行不是合法十六进制：%r" % (i, b))

#---------------------------------------------------------------------
# 文件头
#---------------------------------------------------------------------
HEADER = """//////////////////////////////////////////////////////////////////////////////////
// 模块：bmi270_cfg_rom —— IMU660RA(BMI270) 8KB 初始化配置 ROM
//
// 数据来源：从逐飞 TC264 开源库静态文件 zf_device_config.a 中提取的
//           imu660ra_config_file[8192]（Bosch BMI270 官方 config file）。
//           提取脚本见 ref_tc264/extract_config.py。
//
// 本文件由 tools/gen_cfg_rom.py 自动生成，请勿手工编辑。
//    数据源：rom/imu660ra_config_file.mem
//    修改配置数据后请重新运行：python tools/gen_cfg_rom.py
//
// 为什么内嵌数据而不用 $readmemh：
//    内嵌数据消除了对外部文件和综合时工作目录的依赖，模块提取/移植时
//    只带一个 .v 文件即可，工程更健壮。
//
// 说明：综合后本模块会被 Gowin 实现为 pROM 原语（实测 8 位宽会拆成 4 个
//       BIT_WIDTH=2 的 pROM，每个 INIT_RAM 参数装 128 个 2 位数据、低位
//      对应低地址）。验证脚本 tools/verify_rom_netlist.py 会从综合网表
//      还原数据并与 .mem 逐字节比对。
//
// 端口：
//   i_clock       输入  系统时钟
//   i_addr        输入  地址 0~8191
//   o_data        输出  配置字节（打一拍）
//////////////////////////////////////////////////////////////////////////////////

"""

#---------------------------------------------------------------------
# 生成
#---------------------------------------------------------------------
with open(OUT, "w", newline="\n", encoding="utf-8") as f:
    f.write(HEADER)
    f.write("module bmi270_cfg_rom (\n")
    f.write("    input  wire        i_clock,\n")
    f.write("    input  wire [12:0] i_addr,\n")
    f.write("    output reg  [7:0]  o_data\n")
    f.write(");\n\n")
    f.write("    reg [7:0] rom [0:8191];\n\n")
    f.write("    initial begin\n")
    for i, b in enumerate(data):
        f.write("        rom[%d] = 8'h%s;\n" % (i, b.upper()))
    f.write("    end\n\n")
    f.write("    always @(posedge i_clock) begin\n")
    f.write("        o_data <= rom[i_addr];\n")
    f.write("    end\n\n")
    f.write("endmodule\n")

size = os.path.getsize(OUT)
print("已生成：%s" % OUT)
print("数据字节数：%d" % len(data))
print("文件大小：%.1f KB" % (size / 1024.0))
print("前 4 字节：%s" % " ".join(data[0:4]))
print("后 4 字节：%s" % " ".join(data[-4:]))
