# auto_navigation

自动导航内部模块：雷达坐标转换、建图、后台 BFS 规划、分段路线管理、原始障碍前探和控制器握手。整车应例化 [radar_navigation](../radar_navigation/README.md)。

四邻接反向 BFS 保留 FIFO 和最短格距，每个搜索节点优先沿进入方向继续扩展，再左转、右转、反向，以减少斜墙栅格边缘的连续小转弯。队列项为 2 位搜索方向加 16 位格号，不增加公开接口；这是同距离下的选择偏好，不保证全局最少转弯。现有长段居中和完整线段检查保持不变。

工程源文件见 [rtl/files.f](rtl/files.f)，共 14 个 RTL；不包含测量顶层、测试台和仿真脚本。完整接线与操作见 [交接文档](../radar_navigation/docs/delivery_handoff.md)。
