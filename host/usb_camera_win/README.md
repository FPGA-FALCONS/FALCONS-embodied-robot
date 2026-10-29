# Windows 端 USB 摄像头固定焦距（2026-10-27）

在这台 Windows 电脑上把外接 USB 摄像头的对焦锁死，用于标定、YOLO 取帧或任何
"不能容忍镜头自己呼吸"的场景。RK3588 板端的 V4L2 流程见
[../usb_camera_v4l2/README.md](../usb_camera_v4l2/README.md)，两边互不影响。

## 实测事实（2026-10-27，本机）

| 项 | 实测值 |
|---|---|
| 设备 | `USB Cam`，`USB\VID_0BDA&PID_5856`，即项目那只 UVC 相机 |
| DirectShow 索引 | `1`（索引 `0` 是笔记本内置 `Integrated Camera`，`CAP_PROP_FOCUS` 读回 `-1`，不支持对焦控制） |
| 分辨率/帧率 | **默认 `1280x720@30`**（MJPG）；`1920x1080@30` 也可协商，需要时显式传 `--width 1920 --height 1080` |
| 对焦量程 | `CAP_PROP_FOCUS` `0..1000`，写入后读回精确一致 |
| 自动对焦读回 | `1` = auto，`2` = manual（DirectShow `CameraControl` flags 语义，**不是** V4L2 的 0/1） |

**后端必须用 DirectShow。** 实测 MSMF 后端下 `set(CAP_PROP_AUTOFOCUS, 0)` 返回
`True` 但 `CAP_PROP_FOCUS` 写入被忽略（读回恒为 `68`，画面不变）；换成
`cv2.CAP_DSHOW` 后同样的调用真正驱动镜头马达。

## 用法

```powershell
cd "D:\Programs\FPGA Projects\FALCONS-embodied-robot\host\usb_camera_win"

# 1. 看有哪些相机、哪些支持对焦
python lock_focus.py --list

# 2. 扫一遍，找当前工作距离的最佳焦距（会打印每档清晰度）
python lock_focus.py --scan --scan-step 50 --out-dir scan_20261027

# 3. 锁定到某个值，并验证 60 帧内设备焦点寄存器不动
python lock_focus.py --focus 260 --verify --verify-frames 60 --out locked_260.png

# 4. 手动微调（需要桌面显示）：a/d ±25，j/l ±100，s 存图，q 退出
python lock_focus.py --preview

# 可选：换成 1080p / 其他格式 / 指定索引
python lock_focus.py --focus 260 --width 1920 --height 1080 --fourcc MJPG
python lock_focus.py --focus 260 --index 1
```

不传 `--index` 时脚本自动挑"第一个 `CAP_PROP_FOCUS >= 0` 的相机"，因此内置摄像头
不会被误选。`--json-out` 把完整状态写成 JSON，便于留档。

## 本机合焦带（桌面距离约 40–50 cm，棋盘格标定板）

清晰度用 `Laplacian` 方差衡量（越高越锐）：

| focus | 0 | 100 | 200 | 300 | 400 | 500 | 700 | 1000 |
|---|---|---|---|---|---|---|---|---|
| sharpness | 7.6 | 7.9 | 13.3 | 125–162 | 8.0 | 6.6 | 6.7 | 6.4 |

细扫 `250..295`（步长 5）：`250:78 → 265:153 → 275:218 → 280:234 → 285:266 →
290:228`，峰值落在 **285**。也就是说这只镜头在 40–50 cm 处的合焦带只有
**约 240–300**，量程两端（0–200、400–1000）全部失焦。移动工作距离后必须重新
`--scan`，不要照抄 285。

`--verify` 的直接证据是设备寄存器：60 帧内 `CAP_PROP_FOCUS` 读回恒为 `285.0`
（`focus_stable: true`）。`sharpness_std` 会随画面里手/人移动而变大，它只作参考，
判据看 `focus_stable`。

## 在自己的代码里复用

```python
import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)      # 必须是 DSHOW
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)            # 读回 2 = manual
cap.set(cv2.CAP_PROP_FOCUS, 260)              # 0..1000，按实际工作距离标定
```

标定时把这两行放在取图循环之外，且**每次进程启动都设一次**——相机不会记住设置。

## 棋盘格标定：`calib_pinhole.py`

微信群里那两份模板（`pinhole_calib.py`、`camera_capture.py`）直接拿来用会踩三个坑，
所以按本机情况重写了：

| 模板的做法 | 本机的问题 | 本脚本的处理 |
|---|---|---|
| `config.json` 里写死 `index=3`、`1280x800@120` | 本机是 DSHOW 索引 `1`，默认 `1280x720@30`（原探测为 `1920x1080@30`） | 自动挑"第一个支持 `CAP_PROP_FOCUS` 的相机" |
| 不锁焦，采图中途镜头可能自己动 | 内参会整批漂移，标出来自相矛盾 | 标定脚本采图前 `--focus` 锁焦（默认 260），数值写进结果 |
| 采图无门槛，糊图/重复姿态照样进标定 | RMS 变差且看不出是哪张坏 | 质量只做画面提示，按空格直接拍；靠逐图重投影误差在 `r` 里剔除坏图 |

### 用法

```powershell
cd "D:\Programs\FPGA Projects\FALCONS-embodied-robot\host\usb_camera_win"

# 0. 先确认环境：合成图自检，不需要相机，应全部 PASS
python calib_selftest.py

# 1. 交互采集（推荐）：预览窗里按空格就直接拍，拍够按 c 标定
python calib_pinhole.py
#    空格=直接拍  c=标定  r=重标定（剔除离群）  d=删最后一张  ESC/q=退出
#    画面上的覆盖率/清晰度/姿态接近只是提示，不会阻止采集

# 2. 无窗口自动采集（板子摆好后自己缓慢平移/倾斜）
python calib_pinhole.py --auto-collect 15 --no-window

# 3. 对已有图片批量标定（不同分辨率的图不要混在同一目录）
python calib_pinhole.py --folder calib_images

# 4. 不确定棋盘格内角点行列数时，先让脚本自己试
python calib_pinhole.py --detect-pattern
```

默认棋盘格就是项目那块：**9×6 内角点、方格 25 mm**（`--cols 9 --rows 6 --size 25`）。
打印后如果缩放打印过，务必用卡尺量实际格子边长再改 `--size`。统一的尺寸比例误差
会影响恢复的平移和物理尺度；不会仅因统一改变棋盘坐标单位而使像素焦距 `fx` 同比例变化。

### 采图建议

- 分辨率 **`1280x720`**（两个脚本的 `--width/--height` 默认值），帧率 `30`；采集、标定、推理
  三处必须同分辨率，换分辨率就得重新标定；
- 距离 40–50 cm（与锁定的 focus 匹配；本机实测 `--auto-focus` 给出 260，早先一次扫描是 285，
  以你自己那次 `--auto-focus` 的结果为准），板子占画面宽度 1/4–1/3；
- 采 **15–20 张**，让板子在画面里覆盖 3×3 九宫格（预览窗右上角就是这块提示格），
  并带上 ±20–30° 的俯仰/偏转；
- 避免反光斑和过曝（白格糊成一片时角点定位会偏），必要时 `--exposure` 锁死曝光；
- 板子**整块**入画时最可靠；缺一角时画面会提示 `NO BOARD`，此时按空格仍会存图，
  但那张不进标定（只是留个记录），挪好整块再拍效率更高。

### 提示阈值与可调参数

这些数**不拦截采集**，只在预览画面/日志里提醒，方便判断这一张值不值得留：

| 参数 | 默认 | 作用 |
|---|---|---|
| `--min-coverage` | `0.10` | 棋盘格外接框占画面面积低于此值时提示"覆盖率偏低" |
| `--min-sharpness` | `80` | Laplacian 方差低于此值时提示"清晰度偏低" |
| `--min-shift` | 交互 `40` / 自动 `20` | 与已采集帧角点平均位移小于此值时提示"姿态与已有帧接近"（自动模式据此静默跳过） |
| `--outlier-px` | `1.0` | 单张重投影误差超过 `max(1.0, 3×中位数)` 时在 `r` 重标定里剔除 |
| `--focus` | `260` | 当前锁定对焦位置，必须与所用标定一致 |

### 输出

结果写到 `calib_out/`：

| 文件 | 内容 |
|---|---|
| `pinhole_calib.npz` | `K`、`D`、`rms`、`img_size`、`pattern`、`square_mm`、`focus`、`fps` |
| `pinhole_calib.yaml` | OpenCV FileStorage，RK3588 端 `cv::FileStorage` 直接读 |
| `pinhole_calib.json` | 相机/棋盘/曝光信息、逐图重投影误差、RMS，便于留档 |
| `undistort_check.png` | 原图与去畸变并排对比，肉眼确认畸变模型是否对 |

采集的原始帧留在 `calib_images/`（默认），删改只影响重新标定。

**当前结果**（2026-10-27 23:30:22）：1280×720、focus=260、44个有效视图，
拟合RMS约0.6723px，`fx=1015.380 fy=1016.038 cx=606.570 cy=410.941`，
`D=[0.077399,-0.099653,0.010924,-0.000558,0]`。当前CLI默认固定k3=0；
普通标定、剔除离群和文件夹入口统一使用该选项，`--no-fix-k3`恢复同时估计k3。
固定k3是模型选择，不代表独立验证通过。逐视图最大误差约1.5956px，仍需独立验证。

### 两条硬约束

1. **内参与 focus 绑定**：这只镜头靠移动镜组对焦，改 `focus` 就等于改了内参。
   标定用哪个值，推理就必须用哪个值（当前 `260`）；如需改变对焦，重新标定并独立验证。
2. **内参与分辨率绑定**：脚本默认 `1280x720`，`calib_out/` 当前结果也对应此模式，历史1080p参数不可混用。
   只有确认两模式保持同一视野、无额外裁切且为纯等比例缩放时，才可按精确比例缩放
   `fx/fy/cx/cy`（1080p→720p 为 `2/3`，720p→1080p 为 `1.5`）。实际重采样若使用半像素中心约定，
   主点对应变换为 `(c+0.5)*s-0.5`；不能在未知硬件映射时直接套用该偏移。
   畸变系数保持不变。硬件模式若改变裁切或视野，应在目标模式重标定。

## RK3588 / Linux 等价做法

板端是 V4L2，不走 DirectShow，等价命令（先用 by-id 路径选中 UVC 节点）：

```bash
v4l2-ctl -d /dev/v4l/by-id/usb-<相机>-video-index0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/v4l/by-id/usb-<相机>-video-index0 --set-ctrl=focus_absolute=260
v4l2-ctl -d /dev/v4l/by-id/usb-<相机>-video-index0 --list-ctrls   # 看可用范围
```

`focus_absolute` 的范围随内核/驱动而异（常见 `0..250` 或 `0..1023`），**不要照搬
其他设备的刻度**，以 `--list-ctrls` 输出为准。本项目原相机已在RK实际验证0..1023、步长1、读回260，
自动对焦读回0；正式采集还会逐帧检查。板端仍然不允许为了这件事升级软件包。

## 边界与注意事项

- 重新插拔 USB、切换分辨率、或用厂商相机 App 打开过，都可能把自动对焦重新打开；
  重跑脚本即可。脚本每次运行都会先关自动对焦再写焦点。
- 部分 UVC 相机是定焦模组，`CAP_PROP_FOCUS` 可写但画面不变；本机实测画面随值
  明显变化，属于可调焦模组；马达刻度不是光学焦距毫米值。
- `--preview` 需要交互式桌面；在无显示的会话里用 `--focus`/`--scan` 的批处理模式。
- 标定脚本自检（`python calib_selftest.py`，2026-10-27 实测）：合成 12 张图全部检出，
  估计 `fx=1202.2 / fy=1202.3 / cx=960.0 / cy=540.0`（真值 `1200/1200/960/540`），
  `k1=-0.0803`（真值 `-0.08`），RMS `0.167 px`，6 项检查全 PASS。
  2026-10-28补测720p默认设置、普通/稳健入口固定与释放k3，共11项检查通过。
- `--detect-pattern` 在本机真实画面上检出 `(9, 6)` 与 `(6, 4)`（后者是局部误检），
  确认项目那块板子的内角点就是9×6。已完成内参拟合，安装外参和独立地面验证仍待完成。
- `scan_20261027/`、`scan_fine/`、`locked_285.png` 是本次实测证据，可直接作为
  文档附件；重新标定后覆盖或换目录即可。

## 当前标定与地面H

正式K/D为calib_out/pinhole_calib.json，1280×720、focus260、44个有效视图，RMS约0.6723px。内参拟合误差不直接代表地面位置精度，原始照片、不同模式和对焦扫描记录保留各自来源；运行只选择当前模式配套参数。

`fit_ground_homography.py`用原图像素和实测地面点离线拟合，分别输出原始像素近似H与去畸变像素H。当前已部署14点去畸变H：先undistortPoints(K,D,P=K)，再乘H；不能重复去畸变或套用预览new_K像素。原点为雷达中心地面投影，测量轴右/前厘米，运行转换前/左米。

```powershell
python fit_ground_homography.py --input calib_out/ground_homography_points_combined_20261028.json --intrinsics calib_out/pinhole_calib.json --out ground_homography_candidate.json
```

| 资料 | 用途 |
| --- | --- |
| ground_homography_points_combined_20261028.json / ground_homography_combined_20261028.json | 14点拟合输入/候选及来源 |
| ground_homography_points_validation3_20261028.json / ground_homography_independent_validation3_20261028.json | 独立非规则七点，不并入训练 |
| ground_homography_points_near_20261028.json / ground_homography_near_validation_20261028.json | 40～55cm近五点验证；H未重拟合 |
| ai_coordinator/profiles/inspection_ground_homography_20261028.json | 当前14点H、K/D及40cm起有效凸包 |

40cm处横向±20cm，边界与原远处实测范围组成凸包，径向上限2.1m；范围外不作定位。安装高度/角度的初始候选没有用于当前H。详细误差及测量来源见[地面H记录](../../doc/七点地面单应性标定_20261028.md)，实际调用见[H接口](../ai_coordinator/inspection/HOMOGRAPHY.md)。

原始标定JSON禁用Git换行转换，来源SHA按原字节关联，不能因整理文档改写。候选生成时的状态保留在原始证据，当前运行配置由RK config.json决定；当前巡检为停车后0.5秒等待/三帧平均，操作见[巡检指南](../../doc/AI视觉巡检.md)。
