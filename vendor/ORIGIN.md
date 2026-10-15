# vendor 目录的来源与维护方式

| 项 | 值 |
| --- | --- |
| 来源 | `git@github.com:HITSZ-FPGA-FALCONS/verilog-reusable-modules.git` |
| 最后同步的 commit | **`c9b8025`**（`c9b8025589dcde7c3d5f261ec28d7358d08d9953`，2026-10-14，main） |
| 转入直接维护 | 2026-10-15 |
| 维护方式 | **本项目直接维护** —— `vendor\verilog-reusable-modules\` 里的模块就是本工程的代码，直接改、直接提交 |

## 为什么改成本项目直接维护

这批模块**只有本工程一个消费者**（`class\car_fpga` 是已完成使命的试验场，模块已提取完毕）。
此时"库"的复用属性不存在，而同步机制（`VENDOR.lock` + `deps\vendor_manifest.sha256` +
`tools\sync_vendor.ps1` + `tools\check_vendor.ps1` + `doc\模块库升级流程.md` 六步手册）
的成本明显大于收益 —— 队友反馈"同步机制太复杂"，故于 2026-10-15 改为直接维护，
上述 5 个文件已删除（需要时可用 `git checkout <提交> -- <路径>` 从历史恢复）。

## 改模块时的纪律（★ 唯一不能省的是第 2 步）

1. **直接编辑** `vendor\verilog-reusable-modules\modules\<包>\rtl\*.v`
2. **必须跑仿真回归**：`powershell -File sim\run_sim.ps1`
   —— 模块是整车的一部分（电机驱动、IMU 驱动、遥控捕获、HDMI 显示都在 vendor 里），
   改了它们的行为就变了整车行为
3. 可选：改了某个包就顺手跑那个包自带的测试
   `powershell -File tools\run_all_modules.ps1 -Packages <包名>`
4. 提交：`git add -A; git commit -m "fix(<包名>): 改了什么、为什么"`

## 源库的去向

`D:\Programs\FPGA Projects\verilog-reusable-modules` **原地存档、只读**：
不再同步、不再维护，但保留模块的开发历史与完整包结构，以备将来需要复用或考古。
**本工程不再依赖它** —— 构建（`build.tcl`）、仿真（`sim\run_sim.ps1`）、综合
都只读 `vendor\` 里的内容。