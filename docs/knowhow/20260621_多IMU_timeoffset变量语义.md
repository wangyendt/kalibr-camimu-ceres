# 多 IMU Time Offset 变量语义

## 场景

排查 `1cam+1imu` 与 `1cam+多imu` 标定结果时，容易把两个不同概念混在一起：

- camera 到参考 IMU 的 time shift。
- 非参考 IMU 相对 IMU0 的 time offset。

这两个量都会改变查询 pose/bias spline 的时间，但在 Kalibr 和当前 Ceres 实现里的地位不同。判断多 IMU 标定是否可信前，先要分清哪个量是优化变量，哪个只是初始化后固定的数值修正。

## 现象

`1cam+1imu` 时，time shift 是优化变量；所以直觉上会认为 `1cam+4imu` 时每路 IMU 也应该各自有一个 time offset 优化变量。

023e1e6 之前和 Kalibr 的默认多 IMU 语义接近；2026-06-21 后，Ceres 又把非参考 IMU 的 offset 升级成了可优化变量：

| 实现 | camera 到参考 IMU time shift | 非参考 IMU 相对 IMU0 time offset |
|---|---|---|
| Kalibr `1cam+1imu` | 优化变量 | 不存在 |
| Kalibr `1cam+多imu` | 优化变量 | `--imu-delay-by-correlation` 估计后固定 |
| 当前 Ceres `1cam+1imu` | 优化变量 | 不存在 |
| 当前 Ceres `1cam+多imu` | 优化变量 | gyro correlation 给初值后默认固定；显式 `--optimize-imu-time-offsets` 时 imu1..N-1 作为 Ceres 变量小窗口优化 |

所以问题不是“多 IMU 关掉了 `1cam+1imu` 里的 time shift 优化”，而是要分清两类时间变量：camera-to-reference-IMU time shift 一直是 Ceres 变量；non-reference IMU 相对 IMU0 的 offset 在 Kalibr 主线里是固定 correction，Ceres 只在显式 `--optimize-imu-time-offsets` 时把它升级为变量。

## 源码对应

Kalibr 的 camera time shift 是 design variable。`IccCamera.addDesignVariables()` 创建 `cameraTimeToImuTimeDv`，并由 `noTimeCalibration` 控制 active 状态：

```text
aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py
cameraTimeToImuTimeDv = aopt.Scalar(0.0)
cameraTimeToImuTimeDv.setActive(not noTimeCalibration)
problem.addDesignVariable(cameraTimeToImuTimeDv, CALIBRATION_GROUP_ID)
```

camera residual 查询时间使用：

```text
frameTime = cameraTimeToImuTimeDv + obs.time + timeshiftCamToImuPrior
```

这里 `timeshiftCamToImuPrior` 是互相关得到的固定初值，`cameraTimeToImuTimeDv` 才是优化里的增量变量。最终输出的 camera-to-reference-IMU time shift 是两者相加。

多 IMU 的相对 offset 在 Kalibr 里是另一个量。`IccImu.timeOffset` 初始化为 `0`；如果打开 `--imu-delay-by-correlation`，非参考 IMU 用 gyro correlation/refine 得到 `self.timeOffset`。IMU residual 查询时间使用：

```text
tk = im.stamp.toSec() + self.timeOffset
```

这个 `self.timeOffset` 不是 expression，也没有 `problem.addDesignVariable(...)`。因此它不是 bundle optimization 里的优化变量。

023e1e6 的 Ceres 与这个 Kalibr 固定 correction 语义对齐：

- `CalibrationState.camera_time_shift_s` 是 Ceres 变量。
- `CalibrationState.imu_time_offsets_s` 是多 IMU 的固定时间修正常量。
- multi-IMU residual 用 `sample.timestamp_s + imu_time_offsets_s[i]` 查询 spline。
- `imu_time_offsets_s` 不增加 Ceres parameter block，不增加 residual，也没有 Jacobian。

2026-06-21 后的 Ceres 曾把该扩展设为默认；2026-06-22 对齐 Kalibr 主优化后，当前实现改成：

- `CalibrationState.camera_time_shift_s` 仍是 camera 到参考 IMU 的 Ceres 变量。
- `CalibrationState.imu_time_offsets_s[0]` 固定为 `0`，定义参考 IMU 时间轴。
- `CalibrationState.imu_time_offsets_s[1..N-1]` 默认作为固定 delay correction 常量使用，和 Kalibr `IccImu.timeOffset` 语义一致。
- 传 `--optimize-imu-time-offsets` 时，非参考 IMU offset 在多 IMU、`calibrated` IMU 模型下加入 Ceres problem，并在初值附近 `±imu_time_offset_bound_s` 内优化。
- `--fix-imu-time-offsets` 是显式固定模式；`--imu-time-offset-bound-s` 只影响显式优化模式。

## Book 状态

`docs/books/kalibr_cam_imu_from_equations_to_ceres` 里已经推了“如果 IMU time offset 也作为变量”的 Jacobian。这个方向现在作为显式扩展落地在 `calibrated` IMU 模型里，但不是 Kalibr-aligned 默认路径；实现方式比书里的公式更工程化：offset Jacobian 先采用局部数值差分，其他参数仍走解析 Jacobian。

关键语义是：

- 公式层面可以把 IMU time offset 作为可选 design variable。
- 当前实现层面，per-IMU time offset 已在 `calibrated` IMU 模型下作为显式 Ceres 变量接入。
- 第 13 章记录的多 IMU 新增主变量主要是非参考 IMU 的外参旋转和杠杆臂；per-IMU time offset 是后续补进来的变量。

因此读 book 时要区分“可推导的扩展模型”和“Kalibr 对齐默认变量集合”。公式上推到的内容未必逐项采用同一种实现；当前默认保留固定 correction，只有显式实验才把 offset 方向的导数实现为局部数值差分并接进 problem。

## 当前实现目标

当前 Ceres 的多 IMU 时间模型是：

```text
camera_time_shift_s: Ceres 变量
imu_time_offsets_s[0]: 固定 0，定义参考 IMU 时间轴
imu_time_offsets_s[1..N-1]: 默认固定 correction；显式 --optimize-imu-time-offsets 时才是 Ceres 变量
所有 camera / IMU / bias / pose / extrinsic residual 在同一个 Ceres problem 中联合优化
```

每路 IMU 的有效 camera-to-IMU time shift 仍按下面组合：

```text
timeshift(cam0 -> imui) = camera_time_shift_s - imu_time_offsets_s[i]
```

固定 correction 口径仍然是一气呵成的 joint optimization，不需要多 stage。显式优化 per-IMU time offset 时，stage 也只是可选求解策略，不应该成为表达该变量的必要条件。

## 实现注意

把 `imu_time_offsets_s` 从固定常量升级为 Ceres 变量时，不能只把 `std::vector<double>` 改成 parameter block。关键风险在 spline 查询：

- 现在 build problem 时，IMU sample 的 timestamp 是固定数值，能提前确定 spline segment 和参与的控制点。
- time offset 变成变量后，`t + offset` 会在优化中变化，basis 权重也会变化。
- 如果 offset 跨过 spline segment 边界，当前 residual 绑定的控制点集合可能不再正确。

可选实现策略：

| 策略 | 做法 | 风险 |
|---|---|---|
| 固定 support，小范围优化 | 用 correlation 初值确定 segment/support，在 cost functor 内按 `t + delta` 计算 basis；给 offset 加 prior/bounds，保证不跨 segment | 实现简单，但需要严格边界约束 |
| 增大 support | residual 绑定更宽的控制点窗口，允许小范围跨 segment | parameter block 更多，代码更复杂 |
| 迭代重建 problem | 优化一段后按新 offset 重建 residual/support | 不是“一气呵成”，调度复杂 |

当前实现采用第一种的 bounded refinement 变体：

- problem build 时按 `sample.timestamp_s + initial_offset` 和 `imu_time_offset_bound_s` 生成 pose/gyro-bias/accel-bias segment buffer。
- residual 运行时按当前 `sample.timestamp_s + imu_time_offsets_s[i]` 动态选择 buffer 内的 active segment。
- offset 本身是 Ceres `1` 维 parameter block，并对 `initial_offset ± bound` 设置 box bound。
- residual 对 offset 的 Jacobian 使用局部数值差分；对 extrinsic、pose control、bias control 仍沿用解析 Jacobian。
- 为避免优化过程中越界，靠近 spline 边界、无法覆盖 `t + offset ± bound` 的 IMU sample 会在建图时跳过。

生产数据里当前最大问题是几十毫秒量级的 IMU 间 delay；correlation 负责把初值拉到毫秒级附近，Ceres 变量负责在几毫秒窗口内精修。

## 验证方法

实现 per-IMU time offset 变量后，至少检查这些信号：

| 检查项 | 期望 |
|---|---|
| problem size | `parameter_blocks` 和 `active_parameter_blocks` 比固定 offset 版本增加 `N-1` |
| YAML | 顶层 `imu_time_offsets_s` 和每路 `camera0_effective_time_shift_s` 都写出优化后的值 |
| residual | gyro/accel residual 不应比固定 offset 版本变差 |
| 对单 IMU 对齐 | 组合后的 effective time shift 相对 4 次单 IMU 仍保持毫秒级 |
| 平移 | 单独观察 `T_c_b` 是否改善；不要把 time offset 成功和 translation 成功混为一谈 |

2026-06-21 的显式优化 smoke 验证使用 `2025_03_14_00_34_14`、4IMU、小样本 `max_frames=20`、每路 `max_imu_residuals=100`、`max_iterations=2`：

| 口径 | offset 优化 | parameter_blocks | active_parameter_blocks | offsets |
|---|---|---:|---:|---|
| `--optimize-imu-time-offsets` | enabled | `24154` | `24153` | `0; -0.0192207; -0.0981117; -0.1197944` |
| fixed correction dry-run | disabled | `24151` | `24150` | 固定为 correlation 初值 |

差值正好是 `N-1=3`，证明 imu1..imu3 的 time offset 在显式模式下作为独立 Ceres 变量进入了同一个 problem。YAML 顶层 `imu_time_offsets_s` 已写出优化后的值。

相关实验记录见 `docs/experiment/20260620_4IMU时间偏移与结果可信度定位.md`。该实验已经证明固定 correction 能把 effective time shift 拉到约 `1ms`，但 reference IMU 的 `T_c_b` 平移仍需单独定位。

## 结论

当前 Ceres 的多 IMU delay correction 已经分成两层：gyro correlation 给初值，默认作为固定 correction 使用；显式 `--optimize-imu-time-offsets` 时，Ceres 才在同一个 problem 里对非参考 IMU offset 做小窗口联合优化。这个显式扩展比 Kalibr `--imu-delay-by-correlation` 更进一步，因为 Kalibr 的 non-reference IMU offset 仍是固定 correction。

后续如果要扩大优化窗口或支持 scale/misalignment IMU 模型，仍必须继续处理 spline support、offset bounds 和扩展模型 residual 的 Jacobian，否则很容易出现“变量加了，但 residual 查询时间和控制点集合不一致”的隐性错误。
