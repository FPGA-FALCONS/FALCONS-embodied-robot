# RK去畸变地面H标定接口

正式模型为`inspection.calibration.projection_model="undistorted_homography"`，profile位于`profiles/inspection_ground_homography_20261028.json`；实际生效配置以RK config.json为准。

## 坐标链与绑定

原图像素 → undistortPoints(K,D,P=K) → H并齐次归一化 → 右/前厘米 → `[forward/100,-right/100,0]`前/左米 → 保存的FPGA地图位姿旋转和平移。

当前为14点H，绑定相机序列200901010001、1280×720、focus260及自动对焦关闭。每帧V4L2对焦读回随元数据提供；用于H的像素必须是P=K的去畸变像素，不能用归一化坐标、预览new_K或重复去畸变。

有效区按配置实测凸包，前向40cm起、40cm处横向±20cm，径向上限2.1m；凸包外和退化分母不输出坐标。安装高度/角度初始候选不参与H求解。完整数值与来源见[标定记录](../../../doc/七点地面单应性标定_20261028.md)，原始JSON保持字节SHA。

## 正式调用

`Calibration.project()`返回静态车体点，`locate_stopped()`进一步用停车等待后保存的车辆位姿得到地图坐标。巡检STOP回执后等0.5秒，处理新轮次三帧并平均，不用动态20ms门槛、位姿插值或传感器年龄判断。

框底边中点作为演示接地点；位置余量0.03m、物体半径0.05m、车体半径0.20m是明确的演示常数，不由内参RMS直接推导为误差保证。H只用于点投影，视频显示和模型输入不整帧去畸变。

`inspection.calibration_model`报告静态配置有效性、几何问题、模式/focus及凸包；巡检还需相机、FPGA接口、地图和已提交图层。操作与状态见[巡检指南](../../../doc/AI视觉巡检.md)、[验证记录](../../../doc/AI巡检验证记录.md)。

## 显式部署

```bash
python3 -m ai_coordinator.deploy_inspection --config <现有config.json> --release <已测试release> \
  --calibration-profile <release>/host/ai_coordinator/profiles/inspection_ground_homography_20261028.json --activate
```

使用事务安装器保留模型、设备身份及启动策略；不在带清单的历史release中改文件。现有近40cm配置证据位于运行根`calibration/ground_homography_near40_ff4224fa7642/`。相机安装、focus或图像模式改变后重新标定/验证，不套用不配套的H。
