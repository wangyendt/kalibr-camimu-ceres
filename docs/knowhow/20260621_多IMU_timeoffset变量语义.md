# 多 IMU Time Offset 变量语义

## 场景

排查 `1cam+1imu` 与 `1cam+多imu` 标定结果时，容易把两个不同概念混在一起：

- camera 到参考 IMU 的 time shift。
- 非参考 IMU 相对 IMU0 的 time offset。

这两个量都会改变查询 pose/bias spline 的时间，但在 Kalibr 和当前 Ceres 实现里的地位不同。判断多 IMU 标定是否可信前，先要分清哪个量是优化变量，哪个只是初始化后固定的数值修正。

## 现象

`1cam+1imu` 时，time shift 是优化变量；所以直觉上会认为 `1cam+4imu` 时每路 IMU 也应该各自有一个 time offset 优化变量。

但当前实现不是这样：

| 实现 | camera 到参考 IMU time shift | 非参考 IMU 相对 IMU0 time offset |
|---|---|---|
| Kalibr `1cam+1imu` | 优化变量 | 不存在 |
| Kalibr `1cam+多imu` | 优化变量 | `--imu-delay-by-correlation` 估计后固定 |
| 当前 Ceres `1cam+1imu` | 优化变量 | 不存在 |
| 当前 Ceres `1cam+多imu` | 优化变量 | gyro correlation 估计后固定 |

所以问题不是“多 IMU 关掉了 `1cam+1imu` 里的 time shift 优化”，而是“多 IMU 多出来的 IMU 间相对 time offset 目前没有作为 bundle 变量优化”。

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

当前 Ceres 与这个 Kalibr 语义对齐：

- `CalibrationState.camera_time_shift_s` 是 Ceres 变量。
- `CalibrationState.imu_time_offsets_s` 是多 IMU 的固定时间修正常量。
- multi-IMU residual 用 `sample.timestamp_s + imu_time_offsets_s[i]` 查询 spline。
- `imu_time_offsets_s` 不增加 Ceres parameter block，不增加 residual，也没有 Jacobian。

## Book 状态

`docs/books/kalibr_cam_imu_from_equations_to_ceres` 里已经推了“如果 IMU time offset 也作为变量”的 Jacobian，但这不是当前实现状态。

关键语义是：

- 公式层面可以把 IMU time offset 作为可选 design variable。
- 当前实现层面，per-IMU time offset 是 residual 查询时间里的常量。
- 第 13 章记录的多 IMU 新增主变量主要是非参考 IMU 的外参旋转和杠杆臂；per-IMU time offset 当时还不是 bundle 变量。

因此读 book 时要区分“可推导的扩展模型”和“当前已经接进优化器的变量集合”。公式上有 Jacobian，不代表当前 Ceres problem 里已经加了对应 parameter block。

## 正确的下一版目标

更完整的多 IMU 时间模型应该是：

```text
camera_time_shift_s: Ceres 变量
imu_time_offsets_s[0]: 固定 0，定义参考 IMU 时间轴
imu_time_offsets_s[1..N-1]: Ceres 变量
所有 camera / IMU / bias / pose / extrinsic residual 在同一个 Ceres problem 中联合优化
```

每路 IMU 的有效 camera-to-IMU time shift 仍按下面组合：

```text
timeshift(cam0 -> imui) = camera_time_shift_s - imu_time_offsets_s[i]
```

这样仍然是一气呵成的 joint optimization，不需要多 stage。stage 只应该是可选求解策略，不应该是表达 per-IMU time offset 的必要条件。

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

优先建议第一种：先用 gyro correlation 提供接近正确的初值，再给每个非参考 IMU offset 加强 prior 或 box bounds，使其只在几毫秒范围内修正。生产数据里当前最大问题是几十毫秒量级的 IMU 间 delay；correlation 已经能把它拉到约 `1ms` 级，bundle 变量更适合做小范围精修。

## 验证方法

实现 per-IMU time offset 变量后，至少检查这些信号：

| 检查项 | 期望 |
|---|---|
| problem size | `parameter_blocks` 和 `active_parameter_blocks` 比固定 offset 版本增加 `N-1` |
| YAML | 顶层 `imu_time_offsets_s` 和每路 `camera0_effective_time_shift_s` 都写出优化后的值 |
| residual | gyro/accel residual 不应比固定 offset 版本变差 |
| 对单 IMU 对齐 | 组合后的 effective time shift 相对 4 次单 IMU 仍保持毫秒级 |
| 平移 | 单独观察 `T_c_b` 是否改善；不要把 time offset 成功和 translation 成功混为一谈 |

相关实验记录见 `docs/experiment/20260620_4IMU时间偏移与结果可信度定位.md`。该实验已经证明固定 correction 能把 effective time shift 拉到约 `1ms`，但 reference IMU 的 `T_c_b` 平移仍需单独定位。

## 结论

当前 Ceres 的多 IMU delay correction 是必要且合理的第一步，但它只是对齐 Kalibr `--imu-delay-by-correlation` 的固定 correction 语义。下一版如果要更符合直觉和完整模型，应该把 `imu_time_offsets_s[1..N-1]` 升级为 Ceres 变量，并且仍保持单个 Ceres problem 联合优化。

实现时必须处理 spline support、offset prior/bounds 和 YAML/compare 工具，否则很容易出现“变量加了，但 residual 查询时间和控制点集合不一致”的隐性错误。
