# auto_navigation

自动导航内部模块：雷达坐标转换、建图、后台 BFS 规划、分段路线管理、原始障碍前探和控制器握手。整车应例化 [radar_navigation](../radar_navigation/README.md)。

工程源文件见 [rtl/files.f](rtl/files.f)，共 14 个 RTL；不包含测量顶层、测试台和仿真脚本。完整接线与操作见 [交接文档](../radar_navigation/docs/delivery_handoff.md)。
