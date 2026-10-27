# RK去畸变地面H标定

正式配置通过`inspection.calibration.projection_model="undistorted_homography"`选择。
`profiles/inspection_ground_homography_20261028.json`保存用户选定的14点H及绑定的最新720p K/D。
它采用原图像素，经`undistortPoints(K,D,P=K)`后乘H、齐次归一化，输出右/前cm，
随后转换为前/左m：`[forward/100,-right/100,0]`。不使用原先不吻合的20.6cm高度/20°俯角。

每次调用还检查相机序列号、1280×720模式、ground_verified接地点与本帧focus260逐帧读回证据。
H仅允许在配置指定的实测/独立验证凸包内输出，不把矩形范围或远处都视为已标定；径向上限2.1m。
凸包边界外的拟合结果也会拒绝，不依据训练误差自动扩大范围。H自身分母接近0时拒绝。
原始像素H、默认归一化坐标或Windows预览new_K对应像素不能传入此分支。

`Calibration.project()`用于静态车体坐标核对，不返回可执行任务或已验收的动态观测。
`Calibration.locate()`仍要求拍摄/位姿时序、定位代次、地面接点和已接受的XY误差界限。
H部署配置确认静态地面映射，但`timing_verified=false`、`xy_error_m=null`；
不把本次0.832cm独立RMS、1.002cm最大误差直接当作运动安全界限。
控制器的类别接地点/足迹、速度、车体制动与时序设置继续保持原值，未完成时不能发送巡检GOTO。

单目标投影失败仅跳过该目标，不再丢弃同帧其余目标。
状态接口`inspection.calibration_model`显示所选模型、静态配置有效性和仍缺少的动态条件，
便于区分“已接入H”和“允许自动巡检”。H不改变视频显示、模型输入、同步录像或既有FPGA基础功能。

部署必须显式选择标定文件，默认旧部署行为不自动切换：

```bash
python3 -m ai_coordinator.deploy_inspection --config <现有config.json> --release <已测试release> \
  --calibration-profile <release>/host/ai_coordinator/profiles/inspection_ground_homography_20261028.json --activate
```

安装沿用原用户服务事务备份/回滚，保留原模型、音频身份、运行开关和启动策略。
发布包同时保留14点输入/H、最新内参及第三组独立验证的轻量来源JSON，原始照片仍留现场证据目录。

用户随后批准将最近有效前向距离由60cm扩展至40cm。H、K/D不变，边界改为原区域与
已独立检查的近五点凸包，40cm处横向±20cm，仍拒绝更近或凸包外的点。
标定profile中的`homography.region_extension`保存近点证据SHA与原矩阵未重拟合记录。
RK本轮仅更新配置，继续使用`inspection_src_a3d091ea0c2d`原代码；新增版本化配置及证据目录为
`calibration/ground_homography_near40_ff4224fa7642/`，正式`config.json`为实际生效来源，
不改写旧release内已校验的历史profile文件。后续显式重装profile时应使用更新后的配置来源。
