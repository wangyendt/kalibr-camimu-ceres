# Cam-IMU 仿真系统

这个目录提供一个面向本工程 `calibrate_cam_imu` 的相机-IMU 标定数据仿真器。它从一个仿真 YAML 配置生成：

- 所有标定板角点在 `target0` 坐标系下的 3D 坐标。
- 每个 camera 带时间戳的角点像素观测 CSV。
- 每个 IMU 带时间戳的 gyro/accel CSV。
- Ceres/Kalibr 风格的 `camchain.yaml`、`imu*.yaml`、`aprilgrid.yaml`。
- ground truth、连续轨迹 keyframes、所有传感器位姿采样、manifest 和可直接运行的标定脚本。

## 坐标系约定

仿真使用和当前 Ceres 标定器一致的约定：`T_ab` 表示把 `b` frame 中表达的点变换到 `a` frame。

| 名称 | 含义 |
|---|---|
| `target0` | world frame，所有输出 3D target point 都写在这里 |
| `body` | 参考 IMU frame，也就是优化器里的 body frame |
| `T_cam_body` | camera 外参，`p_cam = T_cam_body * p_body` |
| `T_imu_body` | IMU 外参，旋转为 `R_imu_body`；lever arm `r_b = -R_imu_body^T t_imu_body` |
| `T_target0_target` | 第 k 个标定板局部点到 `target0` 的固定变换 |

## 连续轨迹

仿真输入需要连续轨迹，因为 camera/IMU fps、时间偏移、晶振漂移会要求在任意时间取样。

当前实现支持两种轨迹来源：

1. `trajectory.type: analytic`：用于例子的标定扫动轨迹，先生成高频关键帧，再拟合连续三次样条。
2. `trajectory.type: pose_csv`：从真实/产线离散 pose 文件读入，再拟合连续三次样条。

旋转插值由 `trajectory.rotation_interpolation` 控制：

| 值 | 用途 |
|---|---|
| `rotation_spline` | 默认例子使用。SciPy `RotationSpline`，在 SO(3) 上做 cubic rotation spline，并直接输出 body-frame angular rate/acceleration，适合生成 IMU。 |
| `rotvec_cubic` | 兼容旧实现。对 SciPy rotation-vector 做欧式 natural cubic spline；仅建议用于复现旧数据或调试。 |

`pose_csv` 支持带表头的两种格式：

```text
timestamp_s,tx,ty,tz,qw,qx,qy,qz
```

或：

```text
timestamp_ns,T00,T01,...,T33
```

实现上 translation 仍做 natural cubic spline；默认旋转用 `RotationSpline`。IMU 的 gyro/accel 由连续轨迹的一阶、二阶运动量得到：

```text
omega_body = RotationSpline(t, order=1)
alpha_body = RotationSpline(t, order=2)
specific_force_body = R_body_world * (a_world - gravity_world)
```

若使用 `rotvec_cubic`，SciPy 约定下 body angular velocity 必须用 `J_right(r) r_dot = J_left(-r) r_dot`。旧版错误地用了 `-J_left(r) r_dot`，会生成与 pose 不一致的 IMU gyro，并导致 camera-IMU rotation 约 160 度系统偏差。

这仍不是完整的 Lie-group cumulative B-spline。后续如果要逐项复刻 Kalibr 的 cumulative SE(3) B-spline，可以继续在 `trajectory.py` 后端替换。

## 配置结构

例子配置位于：

```bash
simulation/generated/one_cam_one_imu/sim_config.yaml
simulation/generated/one_cam_four_imus/sim_config.yaml
simulation/generated/two_cams_two_imus/sim_config.yaml
```

重要字段：

| 字段 | 说明 |
|---|---|
| `paths.*` | 输出完整路径；生成脚本会写绝对路径 |
| `simulation.start_s/end_s/gravity_m_s2` | 仿真时间窗口和世界系重力 |
| `trajectory` | `analytic` 或 `pose_csv` 连续轨迹输入 |
| `targets[]` | 多标定板数组，含 AprilGrid 参数、`T_target0_target`、角点噪声矩阵 |
| `cameras[]` | camera 数组，含 fps、外参、内参、畸变、clock、像素噪声/漏检/外点 |
| `imus[]` | IMU 数组，含 fps、外参、clock、white noise、random walk、Allan 分组别名、bias、scale/misalignment、g-sensitivity、量化/饱和 |

角点 3D 噪声：

```yaml
targets:
  - name: target0
    corner_noise:
      frame: target
      matrix:
        - [0.0, 0.0, 0.0]
```

如果不写 `matrix` 或 `path`，默认所有角点噪声为 0。`path` 可以指向 CSV/YAML/NPY 矩阵文件，shape 必须是 `num_corners x 3`。

## 已实现的误差源

| 类别 | 已支持内容 |
|---|---|
| 标定板 | 多 AprilGrid、每板固定 SE(3)、每角点 3D noise matrix、target pose perturbation |
| Camera | 多 camera、不同 fps、T_cam_body、pinhole/omni/eucm/double-sphere 归一化投影、none/radtan/equidistant/fov 畸变、像素高斯噪声、漏检、外点、边界裁剪、rolling shutter 行时间、曝光窗口角点均值、运动模糊代理噪声、位置相关角点协方差 |
| IMU | 多 IMU、不同 fps、T_imu_body、lever arm、gyro/accel bias、white noise density、bias random walk、scale/misalignment matrix、gyro g-sensitivity、gyro sensing rotation、accel size-effect 轴偏移、量化、饱和 |
| 时间 | sensor clock offset、drift ppm、timestamp jitter |
| 输出 | Ceres-compatible corner/IMU CSV、camchain/imu/target YAML、ground truth、sensor pose samples、trajectory keyframes |

## 运行

生成三组示例：

```bash
python3 simulation/scripts/generate_examples.py
```

直接运行某个配置：

```bash
python3 -m simulation.camimu_sim /ABS/path/to/sim_config.yaml
```

检查数据是否能被工程读取：

```bash
build/check_dataset --cam simulation/generated/one_cam_one_imu/camchain.yaml \
  --imu simulation/generated/one_cam_one_imu/imu0.yaml \
  --target simulation/generated/one_cam_one_imu/aprilgrid.yaml \
  --imu-data simulation/generated/one_cam_one_imu/imu0.csv \
  --corners simulation/generated/one_cam_one_imu/cam0_corners.csv
```

每个例子目录都有 `run_calibration.sh`。脚本默认使用全量数据，也支持追加参数：

```bash
simulation/generated/two_cams_two_imus/run_calibration.sh --dry-run
simulation/generated/two_cams_two_imus/run_calibration.sh --max-frames 80 --max-imu-residuals 1200
```

## Camera 时序和角点噪声模型

这些字段都在每个 camera 的 `observation` 下，默认关闭或退化成普通全局快门高斯像素噪声：

```yaml
observation:
  pixel_noise_std_px: 0.15
  rolling_shutter:
    readout_time_s: 0.0015
    direction: top_to_bottom
    reference_row_fraction: 0.5
  exposure:
    exposure_time_s: 0.001
    samples: 3
    blur_noise_scale: 0.15
    blur_dropout_threshold_px: 4.0
  position_dependent_covariance:
    enabled: true
    edge_multiplier: 1.4
    radial_power: 2.0
    u_scale: 1.0
    v_scale: 1.1
    correlation: 0.05
```

实现方式是角点级仿真：rolling shutter 用初始投影行号给每个角点加行读出时间；曝光在窗口内多次投影后取均值；模糊长度用于增加像素噪声或丢点；位置相关协方差按离图像中心的归一化半径缩放。`cam*_corners.csv` 会额外输出 `corner_time_offset_s/sigma_u_px/sigma_v_px/cov_uv_px2/blur_length_px`，当前 Ceres reader 只读前 7 列，所以兼容旧标定流程。

## 已生成的三组例子

| 目录 | 内容 |
|---|---|
| `simulation/generated/one_cam_one_imu` | 1 camera + 1 IMU |
| `simulation/generated/one_cam_four_imus` | 1 camera + 4 IMUs |
| `simulation/generated/two_cams_two_imus` | 2 cameras + 2 IMUs |

每个目录都包含：

```text
sim_config.yaml
manifest.json
ground_truth.yaml
target_points.csv
camchain.yaml
aprilgrid.yaml
cam*_corners.csv
cam*_corner_poses.csv
imu*.csv
imu*.yaml
sensor_poses.csv
trajectory_keyframes.csv
run_calibration.sh
```

## RotationSpline、Slerp/Squad 和 Lie-group cumulative B-spline 的差异

仿真生成端默认使用 `RotationSpline`，不是简单 Slerp。选择理由：

- Slerp 是两帧之间的球面线性插值，角速度分段常量，关键帧处角速度不连续，不适合生成 IMU。
- Squad 可以做 C1 连续四元数插值，但需要自己维护控制四元数和边界条件。
- SciPy `RotationSpline` 是 SO(3) cubic rotation spline，角速度和角加速度连续，并直接提供 body-frame angular rate/acceleration，当前最适合仿真端。

需要区分：仿真生成端已经避免了 rotation-vector 欧式插值的主要问题；Ceres 优化端仍是本工程原有的 `Vec6 pose curve`，不是 Kalibr 的完整 cumulative SO(3)/SE(3) B-spline。因此当前仿真适合验证数据链路、外参、time shift 和噪声模型；若目标是逐项复刻 Kalibr 的轨迹先验和连续时间数学，还需要新增 solver 侧 Lie-group cumulative B-spline。

## 当前限制

- 仿真生成端支持 SO(3) `RotationSpline`；Ceres 优化端仍是 6D pose curve，不是完整 Lie-group cumulative B-spline。
- 当前 rolling shutter/曝光/模糊是角点级代理模型，不生成真实图像，因此不能验证真实 AprilTag 检测器在模糊、曝光、噪声纹理下的行为。
- Kalibr 的 patched `--corner_file` 路径一次只能表达一个 AprilGrid target；当前 multi-camera corner-file 入口还会对所有 camera 复用第一个 corner 文件。公平对比 Kalibr 时应筛选 `target0` 并优先跑单相机数据，双相机对比需要真实图像/ROS bag 或继续改 Kalibr reader。
- `aprilgrid.yaml` 仍是单 target 文件；多标定板通过 corner CSV 中的唯一 `corner_id` 和 `target_x/y/z` 进入优化器，当前工程 residual 会直接使用 CSV 里的 3D 点。
- 生成的标定脚本优先验证数据链路和问题构建；精度评估应继续用 `ground_truth.yaml` 对比 result，并按需要调整初始化、gravity 约定和优化选项。
