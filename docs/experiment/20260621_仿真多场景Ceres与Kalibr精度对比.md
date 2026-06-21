# 仿真多场景 Ceres 与 Kalibr 精度对比

## 背景

当前项目同时维护 Ceres 版 camera-IMU 标定器和 Kalibr Docker 跑法。为了判断 Ceres 版在不同标定场景下的精度和耗时，需要一组可控仿真数据：真值外参、time shift、IMU 相对位姿都已知，视觉角点和 IMU 测量由同一条轨迹生成。

这篇记录的重点不是说明某个代码修复，而是比较 Ceres 与 Kalibr 在这些仿真场景上的标定效果。仿真端的 IMU 运动学一致性和 `RotationSpline` 只作为实验前置条件：如果生成的 gyro 与相机 pose 不一致，后面的 Ceres/Kalibr 精度对比没有意义。

## 比较问题

本次对比回答三个问题：

1. 在单相机单 IMU、单相机多 IMU、双相机多 IMU 场景下，Ceres 标定出的 camera 外参和 time shift 能否回到仿真真值。
2. 多 IMU 场景下，Ceres 与 Kalibr 对非 reference IMU 的相对旋转和平移估计是否在同一精度量级。
3. Ceres 的耗时优势是否伴随明显精度损失，哪些差异来自算法结果，哪些只是评估口径或运行入口限制。

## 数据集与运行范围

仿真数据由 `simulation/scripts/generate_examples.py` 生成，轨迹旋转后端使用：

```yaml
trajectory:
  rotation_interpolation: rotation_spline
```

参与比较的数据集：

| 数据集 | 场景 | Ceres 覆盖 | Kalibr 覆盖 |
|---|---|---|---|
| `one_cam_one_imu` | 单相机、单 IMU | 完整运行 | arm64 Docker，target0 |
| `one_cam_four_imus` | 单相机、4 IMU | 完整运行 | arm64 Docker，target0 |
| `two_cams_two_imus` | 双相机、2 IMU | 完整运行 | 未公平比较 |

Kalibr 当前使用 patched `--corner_file` 路径。这个路径可以公平跑单 camera、单 target0；双相机或多 target 仍需要真实图像/ROS bag，或者继续扩展 Kalibr reader。因此 `two_cams_two_imus` 只列 Ceres 结果，不把 Kalibr 当作同条件对照。

## 指标含义

| 指标 | 含义 |
|---|---|
| `camera rot err` | 标定 `T_cam_body` 与仿真真值的旋转夹角，单位 deg。越小表示 camera-to-body/reference-IMU 旋转越准。 |
| `camera trans err` | `T_cam_body` 平移向量与真值的欧氏距离，单位 mm。 |
| `time shift err` | 估计 camera-to-IMU time shift 减去仿真真值，单位 ms；本批数据真值为 0。 |
| `IMU rotation err` | 非 reference IMU 相对 body/reference-IMU 的旋转误差，单位 deg。 |
| `IMU lever/translation err` | 非 reference IMU 原点相对 body/reference-IMU 的平移误差。Ceres 结果表用 body frame lever arm `r_b` 比较；Kalibr 表用 `T_i_b` 平移比较，二者都反映传感器原点偏移误差，但 YAML 表示不同。 |
| `reprojection` | 视觉重投影残差。Ceres 表为 RMS，Kalibr 表为 report mean，不能逐项等价，只用于同一量级 sanity check。 |
| `gyro/accel residual` | IMU 残差。Ceres 表为 RMS，Kalibr 表为 report mean，也只做量级参考。 |
| `wall time` | Ceres 是本机 binary 运行时间；Kalibr 是 Docker arm64 运行时间。该列反映当前工作流耗时，不是严格同硬件同实现 benchmark。 |

## 评估口径修正

Ceres C++ SO(3) 工具使用 `rotationVectorToMatrix(r)=Exp(-r)` 约定，而 SciPy 使用标准 `Exp(+r)`。Ceres 结果 YAML 中的 `r_i_b` 需要按 Ceres 约定转换成矩阵后再和仿真真值比较。

评估脚本已同步修正：

```python
def _ceres_rotvec_to_matrix(rotvec):
    # Ceres core/so3.h stores rotation vectors with Exp(-r) convention.
    return Rotation.from_rotvec(-np.asarray(rotvec, dtype=float)).as_matrix()
```

这点会直接影响多 IMU 相对旋转结论。修正前，Ceres 4-IMU 的非 reference IMU rotation error 被误报为约 2 度；修正后是 `0.04-0.14 deg`。因此之前“Ceres 多 IMU 旋转明显差”的判断不成立。

## 前置一致性检查

仿真端早期出现过 camera rotation error 约 159 度、但 reprojection residual 很低的现象。这不是标定器精度差异，而是仿真 IMU gyro 与 camera pose 运动学不一致。

Python 仿真使用 SciPy 标准 Rodrigues 约定 `R = Exp(r)`。在这个约定下，body angular velocity 应使用：

```text
omega_body = J_left(-r) * r_dot
```

而不是 C++ 内部相反 rotvec 约定下的：

```text
omega_body = -J_left(r) * r_dot
```

一致性检查结果：

| 公式 | 相对有限差分 body gyro 的 Wahba 角 | 直接 RMS |
|---|---:|---:|
| `-J_left(r) r_dot` | 160.329951 deg | 0.362427 rad/s |
| `J_left(-r) r_dot` | 0.000124 deg | 0.0000197 rad/s |

这个检查只说明仿真数据现在适合做标定器对比；不是本报告的主要结论。

## 运行命令

生成数据：

```bash
python3 -B simulation/scripts/generate_examples.py
```

Ceres 三组标定：

```bash
/usr/bin/time -p simulation/generated/one_cam_one_imu/run_calibration.sh
/usr/bin/time -p simulation/generated/one_cam_four_imus/run_calibration.sh
/usr/bin/time -p simulation/generated/two_cams_two_imus/run_calibration.sh
```

Kalibr arm64 target0：

```bash
python3 -B tools/run_kalibr_docker.py --dataset simulation/generated/one_cam_one_imu --image kalibr-camera-calibration:20.04-arm64 --platform linux/arm64 --corner-file kalibr_cam0_target0_corners.pkl --image-timestamp-file kalibr_cam0_target0_timestamps.txt --imu-data-file imu0.csv --target aprilgrid.yaml --cams camchain.yaml --imu imu0.yaml --trim-imu-edge-count 0 --max-iter 30 --out-root simulation/generated/one_cam_one_imu/kalibr_runs --run-name arm64_target0_rotation_spline_iter30
python3 -B tools/run_kalibr_docker.py --dataset simulation/generated/one_cam_four_imus --image kalibr-camera-calibration:20.04-arm64 --platform linux/arm64 --corner-file kalibr_cam0_target0_corners.pkl --image-timestamp-file kalibr_cam0_target0_timestamps.txt --imu-data-file imu0.csv imu1.csv imu2.csv imu3.csv --target aprilgrid.yaml --cams camchain.yaml --imu imu0.yaml imu1.yaml imu2.yaml imu3.yaml --trim-imu-edge-count 0 --max-iter 30 --timeoffset-padding 0.12 --out-root simulation/generated/one_cam_four_imus/kalibr_runs --run-name arm64_target0_rotation_spline_iter30_pad012
```

## Ceres 结果

### Camera 外参与 time shift

| 数据集 | wall time | camera | rot err | trans err | time shift err | reproj RMS | gyro RMS | accel RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `one_cam_one_imu` | 1.45 s | cam0 | 0.0119 deg | 0.111 mm | -0.013 ms | 0.229 px | 0.00525 rad/s | 0.0159 m/s^2 |
| `one_cam_four_imus` | 5.20 s | cam0 | 0.0312 deg | 0.700 mm | -0.114 ms | 0.244 px | 0.00373 rad/s | 0.0578 m/s^2 |
| `two_cams_two_imus` | 3.37 s | cam0 | 0.0051 deg | 0.070 mm | -0.0099 ms | 0.232 px | 0.00366 rad/s | 0.0525 m/s^2 |
| `two_cams_two_imus` | 3.37 s | cam1 | 0.0047 deg | 0.070 mm | -0.0382 ms | 0.232 px | 0.00366 rad/s | 0.0525 m/s^2 |

Ceres 在三类场景里的 camera 外参都回到亚 0.1 度、毫米以内。双相机场景由于 camchain baseline 通过 `--fix-camera-chain-extrinsics` 固定，表中的 cam1 更像是验证多相机链路和 time shift，而不是独立估计双目 baseline。

### 多 IMU 相对外参

| 数据集 | IMU | rotation err | lever err | time offset err |
|---|---:|---:|---:|---:|
| `one_cam_four_imus` | imu1 | 0.0398 deg | 8.28 mm | 0.047 ms |
| `one_cam_four_imus` | imu2 | 0.0934 deg | 10.21 mm | 0.037 ms |
| `one_cam_four_imus` | imu3 | 0.1419 deg | 11.26 mm | 0.125 ms |
| `two_cams_two_imus` | imu1 | 0.0448 deg | 7.64 mm | 0.0585 ms |

多 IMU 相对旋转已经是 `0.04-0.14 deg`。平移 lever arm 仍在 `7-11 mm`，主要受加速度项、轨迹激励、早停策略和 lever 初始化影响。

## Kalibr arm64 结果

### Camera 外参与 time shift

| 数据集 | wall time | camera | rot err | trans err | time shift err | reproj mean | gyro mean | accel mean |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `one_cam_one_imu` target0 | 6.18 s | cam0 | 0.0740 deg | 0.752 mm | -0.199 ms | 0.199 px | 0.00139 rad/s | 0.0137 m/s^2 |
| `one_cam_four_imus` target0 | 16.57 s | cam0 | 0.0786 deg | 3.34 mm | -0.0627 ms | 0.203 px | 0.00176 rad/s | 0.0183 m/s^2 |

Kalibr 在 target0 单相机路径下也能回到合理精度。视觉 reprojection mean 比 Ceres RMS 小不能直接说明更优，因为统计口径不同；camera 外参误差和 time shift 更适合做跨工具比较。

### 多 IMU 相对外参

| 数据集 | IMU | rotation err | translation err |
|---|---:|---:|---:|
| `one_cam_four_imus` target0 | imu1 | 0.0782 deg | 17.89 mm |
| `one_cam_four_imus` target0 | imu2 | 0.0531 deg | 12.22 mm |
| `one_cam_four_imus` target0 | imu3 | 0.0687 deg | 12.29 mm |

Kalibr 多 IMU 相对旋转在 `0.05-0.08 deg`，和修正评估口径后的 Ceres 处于同一量级。Kalibr 本次输出的多 IMU time offset 为 0，未作为有效对比项使用。

## 参数对照

默认 Ceres 运行来自 `--corner-defaults`，它包含面向速度的生产停止条件：

```text
absolute_cost_change_tolerance=0.05
absolute_parameter_tolerance=0.01
```

在 `one_cam_four_imus` 上，默认运行第 3 次迭代后触发 parameter tolerance 停止；关闭绝对早停后，cost 在 30 次迭代内仍持续下降。严格收敛对照让 residual 下降，也能改善部分 lever arm：

| 配置 | imu1 lever | imu2 lever | imu3 lever | 说明 |
|---|---:|---:|---:|---|
| 默认 | 8.28 mm | 10.21 mm | 11.26 mm | 4 次迭代，速度优先 |
| `--estimate-imu-chain-lever-prior` | 5.37 mm | 7.89 mm | 10.47 mm | lever 初值更好，但仍早停 |
| 关闭绝对早停 | 5.92 mm | 9.29 mm | 11.07 mm | residual 下降，IMU1 改善明显 |
| lever prior + 关闭绝对早停 | 5.85 mm | 9.26 mm | 11.02 mm | 与关闭早停接近 |

结论是：如果比较 Ceres 的极限精度，建议额外跑 accuracy preset；如果比较当前生产默认工作流，默认参数是合理记录。`--estimate-imu-chain-lever-prior` 对 lever 有帮助，但不能单独把所有 IMU 平移误差降到几毫米内。

推荐 accuracy 对照参数：

```bash
--estimate-imu-chain-lever-prior \
--solver-absolute-cost-change-tolerance -1 \
--solver-absolute-parameter-tolerance -1 \
--solver-min-relative-decrease 1e-9
```

## 结论

Ceres 和 Kalibr 在当前仿真数据上都能恢复合理的 camera-IMU 标定结果。Ceres 本机运行更快：单 IMU 约 `1.45 s`，4-IMU 约 `5.20 s`；Kalibr arm64 Docker 对应约 `6.18 s` 和 `16.57 s`。由于运行环境不同，耗时只能代表当前工作流，不是严格算法速度上限。

Camera 外参方面，Ceres 在三组数据上达到 `0.005-0.031 deg`、`0.07-0.70 mm`；Kalibr target0 达到 `0.074-0.079 deg`、`0.75-3.34 mm`。在这批仿真 CSV 上，Ceres camera 外参误差更小，但 Kalibr 覆盖范围受 patched corner-file 路径限制，不能外推到所有真实数据。

Time shift 方面，两者都在亚毫秒内。Ceres camera time shift 误差约 `0.01-0.11 ms`，Kalibr target0 约 `0.06-0.20 ms`。多 IMU time offset 在 Ceres 中也维持在 `0.04-0.13 ms`，这批数据真值为 0，因此这些值可理解为噪声、模型和优化耦合下的估计漂移。

多 IMU 相对旋转方面，Ceres 修正评估口径后是 `0.04-0.14 deg`，Kalibr 是 `0.05-0.08 deg`。两者是同一量级，不存在 Ceres 真实差 2 度的问题；此前 2 度来自 Ceres `r_i_b` 旋转向量符号约定读错。

多 IMU 平移方面，两者都还在毫米到厘米量级。Ceres 默认 lever error 是 `7-11 mm`，Kalibr target0 translation error 是 `12-18 mm`；Ceres 打开 lever prior 可把部分 IMU 降到 `5-8 mm`。这说明当前多 IMU 平移估计还需要更系统的轨迹激励、初始化和停止策略 ablation，不能只靠一个参数判断优劣。

## 限制

- Kalibr patched corner-file 路径只公平覆盖单 camera、target0；双相机和多 target 对比还不完整。
- Ceres residual 使用 RMS，Kalibr report 使用 mean，残差表不能直接排序，只能做量级检查。
- Ceres wall time 和 Kalibr Docker arm64 wall time 不是严格同环境 benchmark。
- 仿真轨迹后端已使用 SO(3) `RotationSpline`，但 Ceres 优化端仍是 6D pose curve，不是完整 Lie-group cumulative B-spline。
- corner covariance、rolling shutter、曝光/模糊等仿真因素已经可生成或部分建模，但本次表格主要比较基础外参/time shift，不展开这些因素的系统 ablation。

## 下一步

1. 为 Ceres 建一个明确的 accuracy preset，和生产默认 `--corner-defaults` 分开汇报。
2. 扩展 Kalibr 数据入口，支持同条件多 camera / 多 target 对比，避免 target0 单相机路径限制结论。
3. 对多 IMU 平移误差做 ablation：关闭/打开 lever prior、不同轨迹激励、不同 IMU 噪声、不同停止策略、是否固定 IMU intrinsics。
4. 增加多 seed 统计，用均值、标准差和最差值替代单次运行结论。
