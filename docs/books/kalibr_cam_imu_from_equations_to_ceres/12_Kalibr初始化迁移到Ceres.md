# Kalibr 初始化迁移到 Ceres

## 场景

当 `ceres_cam_imu` 和 Docker Kalibr 的最终精度还有差距时，第一步不能直接怀疑优化器。Kalibr 在正式构建 cam-IMU 优化问题前会先做一组初始化：time shift prior、camera-to-IMU 旋转先验、重力方向、pose spline、bias spline。只要其中一个初值不同，后续残差和 staged optimizer 即使一致，也可能进入不同的局部路径。

本章记录当前需要迁移的初始化链路、已经对齐的部分、仍未迁移的部分，以及验证命令。

## 三种初始化入口：暖启动 vs 独立

读这一章前必须先分清 `ceres_cam_imu` 的三种初值来源，否则会把“暖启动一致性”误当成“从零独立标定”。

**`--init-from-kalibr`（暖启动）只从 Kalibr 结果文件 seed 三个量。** 源码 `apps/calibrate_cam_imu.cpp` 的处理就三行（`KalibrResult` 结构体见 `core/types.h`）：

```cpp
options.initial_camera_time_shift_s = kalibr.timeshift_cam_to_imu_s;  // ① camera→IMU time shift
initial_T_c_b = matrixToPose6(kalibr.T_ci);                            // ② camera-IMU 外参
initial_gravity = kalibr.gravity;                                      // ③ 重力
```

它**不读** Kalibr 的 pose spline、bias spline 或任何中间状态。`--init-from-camchain` 类似，但只从 `*-camchain-imucam.yaml` 读 `T_cam_imu` 和 `timeshift_cam_imu`（2 个量，没有 gravity）。完全独立（两个开关都不给）则一个 Kalibr 标定产物都不读。

三种入口下各变量的初值来源：

| 变量 | `--init-from-kalibr` | `--init-from-camchain` | 完全独立 |
|---|---|---|---|
| camera-IMU 外参 `T_c_b` | Kalibr 结果 `T_ci` | camchain `T_cam_imu` | 零平移 + 单位旋转，旋转由 Wahba/Kabsch 闭式估计 |
| camera→IMU time shift | Kalibr 结果 | camchain `timeshift` | gyro-norm 互相关（`--estimate-time-shift-prior`） |
| 重力 `g_w` | Kalibr 结果 | （无，需另给） | `--estimate-orientation-gravity-prior` 均值方向 |
| pose spline 控制点 | **独立**：对导出的 camera target pose 做 B 样条最小二乘 | 同左 | 同左 |
| gyro / accel bias | **独立**：常值零（gyro 可由 orientation prior 注入） | 同左 | 同左 |

这里有一个容易混的点：pose spline 的初值来自 `corner_poses.csv`，那是 Kalibr 在**角点提取阶段**算的单帧 PnP 几何（相机单独算的），**不是** Kalibr cam-imu 联合优化的输出。所以即使用 `--init-from-kalibr`，真正“从 Kalibr 标定结果暖启动”的也只有外参、time-shift、gravity 这三个标定产物。

由此引出三条必须分清的边界：

1. **暖启动一致性 ≠ 从零独立。** `--init-from-kalibr` 把 Kalibr 的三个标定产物当初值，再放开外参 + pose + bias（time-shift 被紧软先验钉住、gravity 常配 `--fix-gravity` 固定）。它验证的命题是“在 Kalibr 解附近，Ceres 会不会稳稳停住并复现残差”——多数据集实测外参只动亚毫米，说明 Ceres 的前向模型 / 残差方向 / 手写 Jacobian / 权重口径在 Kalibr 解处正确自洽。它不回答“不给 Kalibr 任何标定产物能否独立收敛到同一点”。
2. **完全独立尚未等价。** 去掉 `--init-from-kalibr` 后，外参平移差仍约 `3.29 cm`，靠 `independent-capped-pe-full`（只给最后 pose+extrinsic 阶段限 trust-region 半径 + staged 解冻）能压到约 `2.4 mm`，但未达 Kalibr。瓶颈是外参平移可观性 + 动态段 pose 二阶导 + 主优化调度，不是缺某个初始化公式。
3. **time-shift 仍靠紧软先验锚定。** 暖启动里 time-shift 初值 = Kalibr 值，再加 `--time-shift-prior-sigma 0.0001`（0.1 ms）把它钉在附近，所以实测 time-shift delta < 1 ms 是“被先验锚住”的结果，不是自由 refine 出来的。完全放开会漂（曾记录到几十毫秒级），尚未复刻 Kalibr Optimizer2 的最终 time-shift refinement。

## 强激励数据中的诊断边界

强三轴激励数据把初始化差异放大得很明显。这个数据集的作用不是替代通用回归，而是把“残差模型能不能在 Kalibr 解附近工作”和“冷启动能不能进入同一个 basin”分开。

同一份 `one_cam_four_imus_rich` 数据上，Kalibr 命令使用：

```text
--pose-knots-per-second 100
--bias-knots-per-second 50
--timeoffset-padding 0.12
--trim-imu-edge-count 0
--max-iter 30
```

Kalibr 日志还给出两个容易漏掉的约束：最终 pose spline 是 `1248` 个有效时间段，跨度 `12.48 s`；构建问题时 `Do pose motion regularization: False`。因此 Ceres 侧要比较主优化行为时，至少需要让 spline 时间域和额外 pose motion prior 保持同一语义：

| 项 | Kalibr 配置语义 | Ceres 对齐方式 |
|---|---|---|
| pose/bias knot rate | `100 / 50` | `--corner-defaults` 已覆盖 |
| time offset padding | 每侧扩 `2 * 0.12 s`，总跨度多 `0.48 s` | `--time-padding 0.12`，pose 控制点数为 `1248 + 6 - 1 = 1253` |
| pose motion prior | 主优化里关闭 | 不传 `--pose-motion-prior` |
| early stop | 跑满 `--max-iter 30` 或按 Kalibr Optimizer2 语义停止 | 诊断时禁用 Ceres absolute stop：三个 `--solver-absolute-*-tolerance -1` |
| IMU robust loss | 本数据集输出 `Model: calibrated`，对应 `IccImu` 基类，gyro/accel 用 `CauchyMEstimator(mSigma)` | `--corner-defaults` 的 Cauchy 已对齐；Huber 只属于 `scale-misalignment` 扩展分支 |
| step accept | Optimizer2 更接近只接受改善后的状态更新 | Ceres 默认允许非单调步；诊断时可额外试 `--no-solver-use-nonmonotonic-steps` |

这组 Kalibr `Model: calibrated` 的 IMU 残差模型是 base `IccImu`，不是扩展 intrinsic 模型。对应公式为：

$$
\hat{\boldsymbol\omega}_i
=
\mathbf C_{ib}\boldsymbol\omega_b+\mathbf b_g,
$$

$$
\hat{\mathbf a}_i
=
\mathbf C_{ib}
\left(
  \mathbf C_{bw}(\mathbf a_w-\mathbf g_w)
  +\dot{\boldsymbol\omega}_b\times\mathbf r_b
  +\boldsymbol\omega_b\times(\boldsymbol\omega_b\times\mathbf r_b)
\right)+\mathbf b_a.
$$

`IccSensors.py::IccImu.addGyroscopeErrorTerms()` 和 `addAccelerometerErrorTerms()` 对这两类 residual 都挂 `CauchyMEstimator(mSigma)`；`IccScaledMisalignedImu` 才把模型扩展为 $\mathbf M_g,\mathbf A_g,\mathbf C_{g i},\mathbf M_a$ 并改用 `HuberMEstimator(mSigma)`。

在这个对齐设置下，`--init-from-kalibr` 的结果具有一个明确解释：它只说明 Ceres residual 和手写 Jacobian 在 Kalibr 外参、time shift、gravity 附近是可工作的。由于 Kalibr txt 不包含最终 pose spline 和 bias spline，Ceres 仍会独立拟合 pose 控制点并以自己的 bias spline 初值进入优化；所以零迭代 IMU residual 不应被拿来和 Kalibr 最终 residual 直接比较。

更有判别力的实验是让 Ceres 从 Kalibr 外参启动并迭代 30 次。该实验可把 reprojection 降到约 `0.27 px` RMS，gyro 降到约 `0.019 rad/s` RMS，camera rotation error 降到约 `0.026 deg`。这说明生产 residual 的前向模型、残差方向和解析 Jacobian 不是当前强激励失败的主因。

冷启动失败不能再简单归因到 orientation/gravity prior。强激励数据上，Ceres cold start 的 prior 日志给出 gravity 约 `[-0.2730, -0.0121, -9.8027]`、gyro bias 约 `[0.01277, -0.00044, -0.00241]`，而 Kalibr 初始化日志对应为 `[-0.2687, -0.0040, -9.8029]` 和 `[0.01284, -0.00029, -0.00237]`。这说明两边的 orientation prior 链路已经同量级；Kalibr 自己也从一个距最终外参数度的 rotation prior 进入主优化。剩余差距更应放在主优化路径上排查：非单调步、staged release、IMU extrinsic/time-offset 自由度以及 Optimizer2 的状态接受语义。

一组 from-Kalibr 诊断能说明这个边界。只固定 per-IMU time offset 基本不改变 residual，也没有明显改善非参考 IMU lever。误把 gyro/accel loss 切到 Huber 后，reprojection 和 gyro RMS 会改善，但非参考 IMU lever 反而变差到厘米级，说明它不是本数据集的 Kalibr 对齐方向。保持 Cauchy 并关闭 Ceres 非单调步后，camera-to-Kalibr translation delta 可降到 `0.477 mm`，但相对真值的 camera translation 仍为 `3.20 mm`，非参考 IMU lever 为 `14.0 / 8.7 / 18.8 mm`。因此，下一步不是把 residual 改成 AutoDiff，也不是替换为累积 SO(3) spline，而是继续对齐主优化调度和变量释放方式。

非参考 IMU extrinsic 也需要和读取问题分开。`--init-from-kalibr --max-iterations 0` 时，Ceres 输出的 IMU chain 就是 Kalibr seed，truth 对比的 lever error 为 `7.1 / 4.6 / 6.9 mm`，与 Kalibr 量级一致。默认放开非参考 IMU extrinsics 跑 30 次后，lever error 变为 `14.2 / 7.4 / 18.2 mm`；如果加 `--fix-imu-extrinsics`，30 次后 lever 仍保持 `7.1 / 4.6 / 6.9 mm`，而 residual 只从 `0.26457 px / 0.01892 rad/s / 0.33578 m/s^2` 变为 `0.27110 px / 0.01930 rad/s / 0.33626 m/s^2`。但是 Kalibr 源码 `IccImu.addDesignVariables()` 明确会把非参考 IMU 的 `q_i_b_Dv` 和 `r_b_Dv` 设为 active；`--fix-imu-extrinsics` 只能作为诊断，不能当作 Kalibr 对齐默认。把 `i` 延后释放也只能部分缓解：`--stage-free pbet,pbeti --stage-iterations 20,10` 给出 `11.3 / 8.0 / 13.5 mm` 的 lever error，同时 reprojection RMS 升到 `0.28210 px`。这说明非参考 IMU lever gap 不是 Kalibr result 读取、`r_b` 符号或坐标转换错误，而是主优化里该自由度对 cost 的约束较弱：它能换来很小的 residual 收益，却明显远离仿真 truth。后续 staged release、trust-region 或先验设计应把这一点当成可观性问题处理。

Kalibr 的 LM 初值也不能机械照搬成 Ceres 的 trust-region radius。`IccCalibrator.optimize()` 设置的是 `LevenbergMarquardtTrustRegionPolicy(1.0)`、`convergenceDeltaX=1e-2`、`convergenceDeltaJ=1`；其 policy 在失败或 `rho<=0` 时增大内部 `lambda`，成功步才 rebuild system，并由 Optimizer2 回滚 regression step。Ceres iteration summary 中 `mu=1/radius`，语义不完全相同。强行用 `--solver-initial-trust-region-radius 1 --no-solver-use-nonmonotonic-steps` 后，camera-to-Kalibr translation delta 虽只有 `0.175 mm`，但 30 次迭代后的 cost 仍为 `4.03e4`，reprojection/gyro/accel RMS 为 `0.62487 px / 0.05243 rad/s / 0.42731 m/s^2`，说明这只是把外参留在 Kalibr 附近，没有复现 Kalibr residual。

更有用的方向是把 solver 半径作为 staged release 的局部调度参数。`--stage-free pbet,pbeti --stage-iterations 20,10 --stage-solver-initial-trust-region-radii 10000,1 --stage-solver-max-trust-region-radii 1e16,1000 --no-solver-use-nonmonotonic-steps` 把非参考 IMU lever 改到 `9.8 / 7.0 / 11.5 mm`，优于默认全放开和普通晚释放；但 reprojection RMS 变为 `0.28909 px`，camera translation error 为 `3.48 mm`。这说明小半径释放 `i` 是缓解策略，不是最终修复。当前证据更支持把问题定义为非参考 IMU extrinsic 的可观性/正则化取舍，而不是残差或 Jacobian 方向错误。

另一个需要先排除的是 IMU time offset 的符号问题。仿真器现在把 sensor 时钟偏移和标定 offset 分开记录：若 IMU0 的 `clock.offset_s=0`，imu1 的 `clock.offset_s=+0.018`，则 imu1 观测时间比参考时钟晚 18 ms，标定语义下应输出 `time_offset_s=-0.018`。非零 offset 探针中，Ceres dry-run 的多 IMU 链路先验为 `-0.0177778`，短优化后结果为 `-0.0184561`，相对 truth 的误差约 `-0.456 ms`。这说明 Ceres/Kalibr 查询时刻约定和仿真 truth 比较链路在符号上是一致的；剩余的非参考 IMU lever gap 不应优先按 time-offset sign bug 处理。

状态接受语义也需要和残差模型分开看。Ceres 侧提供 `--solver-restore-best-state` 后，每个 solve 可以在 accepted step 上记录当前最低 cost 的 `CalibrationState`，并在结束时把状态恢复到该 solve 内部的 best accepted state。`two_cams_two_imus` 的 8 次 smoke、4 次 rejected-step probe，以及 `one_cam_four_imus_rich` 从 Kalibr 结果暖启动的 30 次诊断都没有触发恢复；强激励 from-Kalibr run 的 cost 单调下降，rejected-step probe 中 Ceres 也已经返回 best accepted state。这说明“最终状态没有回到 best accepted state”不是当前 rich 数据差距的主因。该开关的价值是保护单次 solve 的状态一致性，不能替代 Kalibr Optimizer2 的 staged release、变量释放顺序和外参可观性调度。

## Kalibr 的初始化顺序

Kalibr 的主入口在 `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py`。初始化顺序是：

1. 对每个 camera 调 `findTimeshiftCameraImuPrior(...)`，用角速度模长互相关估计 `timeshiftCamToImuPrior`。
2. 对主 camera 调 `findOrientationPriorCameraToImu(...)`，用 camera pose spline 预测角速度，优化 camera-to-IMU 旋转和常值 gyro bias。
3. 用同一段 pose spline 和旋转先验，把 IMU 比力转到 world，取均值方向初始化重力。
4. 调 `initializePoseSplineFromCameraChain(...)`，使用主 camera 的 target pose 初始化 `T_w_b` pose spline。
5. 对每个 IMU 调 `initBiasSplines(...)`，gyro bias 用前一步估计的 `GyroBiasPrior` 常值初始化，accel bias 用零初始化。
6. 把 pose spline、bias spline、外参、time delay 和 gravity 加成 design variables，再添加 camera/gyro/accel/bias-motion 等 error terms。

这说明 time shift 不是一个普通优化变量的默认值。它先作为 `timeshiftCamToImuPrior` 改变相机观测时间，再让 `cameraTimeToImuTimeDv` 从 `0` 开始优化。Kalibr 最终输出的是：

```text
t_imu = t_cam + timeshiftCamToImuPrior + cameraTimeToImuTimeDv
```

cam0 的 camera-to-IMU 外参平移也不是单独估计出来的。`IccCamera` 构造时把 `T_extrinsic` 设为 `sm.Transformation()`，也就是单位旋转和零平移；`findOrientationPriorCameraToImu(...)` 只覆盖外参旋转，并保留这个平移初值。平移随后依赖主联合优化从 camera residual 和 IMU residual 的耦合中恢复。

## Time Shift 初值

Kalibr 的 `IccSensors.py::findTimeshiftCameraImuPrior` 做法很直接：

1. 调 `initPoseSplineFromCamera(timeOffsetPadding=0.0)`。
2. 该函数默认 `poseKnotsPerSecond=100`，并调用 `pose.initPoseSplineSparse(times, curve, knots, 1e-4)`。
3. 在 pose spline 时间范围内遍历 IMU 样本。
4. 记录 `||omega_measured||` 和 `||omega_predicted||`。预测量来自 `poseSpline.angularVelocityBodyFrame(tk)`。
5. 用 `np.correlate(omega_predicted_norm, omega_measured_norm, "full")` 找峰值。
6. 计算 `discrete_shift = argmax - (len(measured)-1)`，再用 `shift = -discrete_shift * mean(diff(imu_times))`。

`ceres_cam_imu` 对应模块是：

```text
include/ceres_cam_imu/initialization/time_shift_initializer.h
src/initialization/time_shift_initializer.cpp
```

当前迁移状态：

| 项目 | Kalibr | Ceres 当前状态 |
|---|---:|---|
| time-shift pose spline kps | `100` | 默认已改为 `100` |
| timeOffsetPadding | `0.0` | `makeSplineForTimes(..., padding=0.0)` |
| sparse fit lambda | `1e-4` | `TimeShiftPriorOptions::pose_fit_regularization=1e-4` |
| rotation vector unwrap | 有 | `PoseSplineFitOptions::unwrap_rotation_vectors=true` |
| angular velocity | `angularVelocityBodyFrame(t)` | Ceres 用解析 spline 导数计算 body angular velocity |
| correlation | `np.correlate(predicted, measured, "full")` | C++ full correlation loop |
| shift 符号 | `-discrete_shift*dT` | 相同 |

验证命令：

```bash
build/calibrate_cam_imu --kalibr-corner-defaults --cam /ABS/cam_imu/cam0-camchain-640x400.yaml --imu /ABS/cam_imu/imu.yaml --target /ABS/cam_imu/aprilgrid.yaml --imu-data /ABS/cam_imu/data1.csv --corners /ABS/cam_imu/cam0_640x400_corners.csv --corner-poses /ABS/cam_imu/cam0_640x400_corner_poses.csv --kalibr-result /ABS/cam_imu/cam0_640x400_corners-1-results-imucam.txt --init-from-kalibr --estimate-time-shift-prior --dry-run --max-frames 20 --imu-stride 1000 --max-imu-residuals 5
```

关键输出：

```text
estimated time shift prior: shift_s=-0.55052438939288217 pose_kps=100 fit_lambda=0.0001 discrete_shift_samples=278 sample_dt_s=0.001980303558967202 samples=22929
```

Docker Kalibr 零迭代基线的 time shift 是：

```text
-0.5505243893928822 s
```

两者在当前日志精度下对齐。旧的 Ceres 默认相当于显式传入：

```bash
--time-shift-pose-kps 20 --time-shift-fit-lambda 1e-9
```

同一裁剪数据上会得到：

```text
shift_s=-0.55448499651081662
```

这比 Kalibr 风格初值多偏约 `3.96 ms`。因此，之前 time-shift 初值差异主要来自初始化 spline 频率和平滑 lambda 没有对齐。

## timeoffset-padding 在 Ceres 里改了什么

`--timeoffset-padding S`（兼容别名，等价 `--time-padding S`）只落到一个数：`CalibrationOptions::time_padding_s`（默认 `0.04`）。它**不改任何测量数据**，只改 spline 的**时间域宽度**，从而间接改控制点数。

机制在 `src/trajectory/uniform_bspline.cpp::makeSplineForTimes`：

```cpp
const double kalibr_side_padding_s = 2.0 * padding_s;          // 每侧扩 2*S
const double t_min = first_time_s - kalibr_side_padding_s;     // 共扩 4*S
const double t_max = last_time_s  + kalibr_side_padding_s;
const double seconds  = t_max - t_min;                          // = 数据跨度 + 4*S
const int    segments = round(seconds * knots_per_second);      // 固定密度下段数
return UniformBSpline(dimension, order, t_min, t_max, segments);
```

控制点数（即设计变量块数）由段数推出：`numCoefficients() = num_segments + order - 1`（pose order=6）。所以链条是：

```text
--timeoffset-padding S  →  样条时间域两端各扩 2S（共 4S）
                        →  段数 = round((数据跨度 + 4S) × kps)
                        →  控制点数 = 段数 + order - 1
                        →  pose 样条 + 两条 bias 样条的控制点都变多
                        →  parameter_blocks / tangent_params / motion-prior 数都变多
```

它**只影响轨迹表示的分辨率与维度，不影响 residual 数**：camera/gyro/accel residual 数完全不变。某个匿名样例（pose-kps=100、bias-kps=50、数据跨度 48.5345 s）的 dry-run 实测：

| timeoffset-padding | pose 控制点 | bias prior(=bias 段) | parameter_blocks | tangent_params | camera/gyro/accel residual |
|---:|---:|---:|---:|---:|---|
| 0.0 | 4858 | 2427 | 9726 | 43749 | 647266 / 22929 / 22929 |
| 0.01 | 4862 | 2429 | 9734 | 43785 | 647266 / 22929 / 22929 |
| 0.04 | 4874 | 2435 | 9758 | 43893 | 647266 / 22929 / 22929 |
| 0.08 | 4890 | 2443 | 9790 | 44037 | 647266 / 22929 / 22929 |

公式可逐项核对：每增加 `0.01 s` padding → pose 段数加 `4×0.01×100 = 4`（4858→4862→…），每条 bias 段数加 `4×0.01×50 = 2`。padding `0.04` 相对 `0` 共多 16 个 pose 控制点、各 8 个 bias 控制点，于是 `parameter_blocks` 多 `16+8+8 = 32`、`tangent_params` 多 `16×6+8×3+8×3 = 144`。这正好对上 Docker Kalibr 零迭代报告的 `4869` pose segments / `2435` bias segments。

**为什么要 padding。** Kalibr 把 `timeoffset-padding` 当作“标定中 time-shift 可能达到的最大偏移”。camera residual 的查询时刻是 `t = t_cam + timeshift`，当 timeshift 在优化中移动时，查询时刻会越过原始数据的首尾。把样条时间域两端各扩 `2×padding`（留双倍 margin），可以保证移动后的查询时刻仍落在样条支撑内，不会在边界外取值。所以 padding 大一点更安全（更宽的 time-shift 余量），代价是多几十个控制点；它不是精度旋钮，而是“时间边界余量”。

## 其他初始化模块

| 模块 | Kalibr 做法 | Ceres 当前状态 | 结论 |
|---|---|---|---|
| pose spline 初始化 | 主 camera pose，`poseKnotsPerSecond=100`，时间两端加 `2*timeOffsetPadding`，首尾 pose 各复制一次，`initPoseSplineSparse(..., 1e-4)` | 已有 `initialization/pose_spline_fit.*`，支持旋转向量 unwrap、motion lambda 和边界 anchor；`independent-full` 已默认启用 `--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors` | 独立初始化路径已迁移；当前数据上 accel mean 改善，但外参平移差几乎不变 |
| bias spline 初始化 | gyro bias 用 `GyroBiasPrior` 常值，accel bias 用零；bias kps 默认 `50` | bias spline 常值初始化；`CalibrationOptions` 已支持 `initial_gyro_bias_rad_s` 和 `initial_accel_bias_m_s2`；`--kalibr-corner-defaults` 已设 `bias_kps=50`；bias motion prior 已实现 | 频率、accel 零初始化、gyro bias prior 注入路径已对齐 |
| camera-to-IMU 旋转先验 | 用 camera pose spline 预测角速度，优化 `R_i_c` 和常值 gyro bias，最多 50 次 | 已新增 `initialization/orientation_gravity_initializer.*` 和 CLI `--estimate-orientation-gravity-prior`；先用 Wahba/Kabsch 闭式解求 `gyro_imu ~= R_i_c * omega_camera + bias`，再用只含 `r_i_c` 和 `b_g` 的小 Ceres problem 手写 Jacobian refine | 结构已迁移到 Kalibr 的“小优化器”形式；当前数据上闭式解已最优，refine 只执行 1 次且数值不变 |
| 重力初始化 | 用旋转先验把 `-accel` 转到 world，取均值并归一到 `9.80655` | 新 initializer 同时输出 gravity；优化时默认用固定范数球面流形 | 独立 gravity 初值已迁移 |
| camera-to-IMU 平移初值 | cam0 使用 `sm.Transformation()` 的零平移；没有单独平移初始化公式 | `CameraExtrinsicBlock` 默认平移已改为 `[0, 0, 0]`；旧默认 `z=0.1m` 会让独立初始化卡入错误盆地 | 初值已按 Kalibr 对齐；剩余平移差属于主优化路径问题 |
| camchain 结果复用 | Kalibr 输出 `*-camchain-imucam.yaml`，包含 `T_cam_imu` 和 `timeshift_cam_imu` | 已新增 `readCamchainImuPrior(...)` 和 CLI `--init-from-camchain`，从 `--cam` YAML 读取外参和时间偏移 | 这是数据读取层能力，不是独立估计算法；camchain 不含 gravity |
| 多 IMU 初值 | 对非参考 IMU 估计 temporal offset 和相对旋转 | 当前范围是单 camera + 单 IMU | 不在当前实现范围内 |
| camera chain 基线 | 多 camera 时用 chain 外参和主 camera 初始化 pose spline | 当前范围是单 camera | 不在当前实现范围内 |

## 判断方法

如果问题是“Ceres 和 Docker Kalibr 当前精度是否对齐”，应分两层判断：

1. 初始化层：time-shift 初值现在已经和 Docker Kalibr 零迭代对齐，关键证据是 `-0.55052438939288217 s` 对 `-0.5505243893928822 s`。
2. orientation/gravity 初值层：`--estimate-orientation-gravity-prior` 已能在匿名样例上不依赖 Kalibr 初始旋转启动。当前 dry-run 默认 `boundary_anchors=1`、`ceres_refine=1`，旋转先验距 Kalibr 最终结果约 `0.175954 deg`，重力差约 `0.014182 m/s^2`，gyro bias prior 为 `[-0.0011955, -0.0013163, 0.0065238] rad/s`。`refine_iterations=1` 说明小 Ceres refinement 只确认闭式解，没有改变数值。
3. pose spline 初始化层：`independent-full` 现在启用 Kalibr 风格 `--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors`。相对旧 pose-fit，accelerometer mean 从 `0.898892 m/s^2` 降到 `0.880795 m/s^2`，translation delta 从 `0.032903 m` 微降到 `0.0328967 m`，reprojection mean 从 `0.197435 px` 升到 `0.201851 px`。这说明 pose spline 初始化差异已经补齐，但不是 3 cm 级平移差的主因。
4. 外参平移初值层：Kalibr 的 cam0 平移初值是零。`ceres_cam_imu` 已把 `CameraExtrinsicBlock` 默认平移从旧的 `[0, 0, 0.1]` 改为 `[0, 0, 0]`。同一独立初始化 staged 全量命令下，translation delta 从约 `0.101754 m` 降到约 `0.032903 m`，证明旧默认值确实是一个初始化差异。
5. camchain 读取层：`--init-from-camchain` 能从 `cam0_640x400_corners-1-camchain-imucam.yaml` 读取 `T_cam_imu` 和 `timeshift_cam_imu`。dry-run 验证输出 `kalibr_translation_delta_m=6.84e-09`、`kalibr_time_delta_s=0`。这个入口方便直接复用 Docker/camchain YAML，但它不提供 gravity。
6. 最终优化层：`--init-from-kalibr` 的当前全量基线仍约有 `0.846 ms` 的最终 time-shift delta，残差均值约为 `0.196977 px / 0.167393 rad/s / 0.863287 m/s^2`，外参平移差约 `0.052 mm`。这不是 time-shift 初值算法差异；完全替代 `--init-from-kalibr` 后的剩余平移差，主要来自主优化路径和外参平移可观性。

## Sweep preset

`tools/run_ceres_sweep.py` 已经把上述判断拆成可复跑 preset：

| preset | 复查层级 | 预期用途 |
|---|---|---|
| `kalibr-dry-run` | Kalibr txt 初值读取 | 确认 `--init-from-kalibr` 的零迭代状态 |
| `current-full` | 当前 Ceres 全量回归 | 复用 Kalibr txt 初值，判断 Ceres 主优化和 Docker/Kalibr 最终结果的距离 |
| `independent-full` | 独立初始化全量回归 | 不读 Kalibr txt，用已迁移的 time-shift、orientation、gravity、gyro-bias 和 Kalibr 风格 pose-fit 初值启动 |
| `independent-legacy-posefit-full` | 旧 pose-fit 独立初始化回归 | 不启用 `--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors`，用于确认 pose-fit 迁移的影响 |
| `independent-final-pe-full` | 外参平移可观性诊断 | 独立初始化后最后只联合释放 pose 和 extrinsic，固定 bias、time shift 和 gravity，观察外参能否脱离零平移盆地 |
| `independent-capped-pe-full` | trust-region 受限的外参平移诊断 | 在 `independent-final-pe-full` 基础上把最后 `pe` 阶段改为 10 次，并通过 `--stage-solver-max-trust-region-radii 1e16,1e16,1e16,10000000` 只限制最后阶段，用于复现当前独立初始化最接近 Kalibr 的路径 |
| `camchain-dry-run` | camchain YAML 读取 | 确认 `T_cam_imu` 和 `timeshift_cam_imu` 能直接复用 Kalibr/camchain YAML |
| `camchain-full` | camchain YAML 初值全量回归 | 用 camchain 外参/time shift 加独立 orientation/gravity prior 进入 staged optimizer |

推荐先跑：

```bash
python3 tools/run_ceres_sweep.py --preset camchain-dry-run --preset independent-full --run-name preset_probe_camchain_independent --stop-on-failure
```

`summary.csv` 里重点看 `camchain_init_time_shift_s`、`camchain_init_translation_x_m`、`camchain_init_translation_y_m`、`camchain_init_translation_z_m`、`camchain_init_kalibr_translation_delta_m`、`camchain_init_kalibr_time_delta_s`，以及 `independent-full` 的 `time_shift_init_*`、`orientation_init_*`、`pose_init_*` 和 `kalibr_delta_translation_m`。前者回答“读取层是否完全对齐”；`time_shift_init_*`、`orientation_init_*` 和 `pose_init_*` 回答“独立初始化是否按 Kalibr 风格启用”；最终 `kalibr_delta_translation_m` 回答“不依赖 Kalibr txt 后主优化还能不能收敛到同一外参”。

`independent-final-pe-full` 的实测结果提供了一个新的判断边界：最后阶段使用 `--stage-free e,bt,pbt,pe --stage-iterations 0,1,4,8`，只释放 pose 和 camera-to-IMU extrinsic，固定 bias、time shift 和 gravity。匿名样例上 translation delta 从 conservative 独立路径的 `0.0328967 m` 降到 `0.00606277 m`，rotation delta 从 `0.17413 deg` 降到 `0.0943398 deg`；同时 residual mean 变为 `0.208920 px / 0.172327 rad/s / 0.885812 m/s^2`，比 conservative 路径略差。继续把 `pe` 阶段加到 10 或 12 次会让 translation 分别漂到 `0.0470606 m` 和 `0.0807533 m`。因此这个 preset 证明外参平移可通过 joint pose/extrinsic 释放显著改善，但停止策略和主优化调度仍未完全对齐 Kalibr。

为避免只看最终结果，Ceres 侧新增了 `--trace-iteration-state`。该开关通过 Ceres `IterationCallback` 在每次 accepted step 后打印当前 `T_c_b`、time shift、gravity，以及它们相对 `--kalibr-result` 的差值；`run_ceres_sweep.py --extra-arg=--trace-iteration-state` 会把关键字段汇总到 `summary.csv`。`independent_final_pe_trace_summary_20260616` 的证据显示，最后 `pe` 阶段从零平移出发，reference translation delta 按 `0.03297 -> 0.03276 -> 0.03269 -> 0.03254 -> 0.03217 -> 0.03118 -> 0.02843 -> 0.02129 -> 0.00606 m` 下降，第 `8` 次迭代达到当前 preset 的最小值。旧 sweep 中把 `pe` 继续加到 10 或 12 次会继续降 cost，但平移开始远离 Kalibr，所以问题不是 time-shift 初值，而是 joint pose/extrinsic 释放后的停止策略和可观性取舍。

2026-06-16 继续对照 Kalibr 主优化器。`IccCalibrator.optimize()` 并没有使用 `Optimizer2Options` 的默认 LM 初值，而是设置 `LevenbergMarquardtTrustRegionPolicy(1.0)`、`convergenceDeltaX=1e-2`、`convergenceDeltaJ=1` 和 `BlockCholeskyLinearSystemSolver()`。源码 `Optimizer2::applyStateUpdate()` 进一步说明，Kalibr 的 `deltaX` 是最小维度更新向量的最大绝对分量，`deltaJ` 是前后 cost 的绝对下降量。`LevenbergMarquardtTrustRegionPolicy` 的内部状态是 `lambda/gamma/beta/p/mu`：第一步 build system；失败步或 `rho<=0` 只增大 `lambda`；成功步 rebuild system，并按 `rho` 缩小 `lambda`。Ceres 侧已经把 `--solver-initial-trust-region-radius`、`--solver-max-trust-region-radius`、`--solver-min-trust-region-radius`、`--solver-min-relative-decrease`、`--solver-linear-solver`、`--solver-num-threads`、非单调步配置、`--solver-absolute-cost-change-tolerance`、`--solver-absolute-step-tolerance` 和 `--solver-absolute-parameter-tolerance` 暴露为命令行参数；summary 会记录 `solver_*` 字段。staged 模式进一步支持 `--stage-solver-initial-trust-region-radii`、`--stage-solver-max-trust-region-radii`、`--stage-solver-min-trust-region-radii`、`--stage-solver-min-relative-decreases`、`--stage-solver-absolute-cost-change-tolerances`、`--stage-solver-absolute-step-tolerances` 和 `--stage-solver-absolute-parameter-tolerances`，用于只覆盖某些阶段的 solver 选项。Ceres 的 iteration summary 明确 `mu=1/radius`，所以这些参数不能机械等同于 Kalibr 的 lambda；`optimizer/parameter_delta_tracker` 现在能在 accepted step 后统计活跃参数块的最大实际系数变化，并让 `--trace-iteration-state` 逐轮打印 `parameter_delta`。它比 Ceres `step_norm` 更接近 Kalibr `deltaX`，但统计的是 ambient parameter block 的实际变化，不是 Kalibr 最小维 tangent update 的 max-coeff。

阶段状态更新现在也有了独立的保护层。`optimizer/state_snapshot` 在每个 staged solve 前复制 `CalibrationState` 中真正会被 Ceres 改写的变量块：pose controls、gyro/accel bias controls、camera/IMU extrinsic、gravity 和 time shift；spline 元数据不进入快照，因为它描述的是时间结构，不是一次 solve 的参数更新。stage 结束后，`decideCalibrationStageStateUpdate(...)` 只根据同一个 stage problem 的 Ceres summary 做判断：solver failure、非有限 cost、或者 final cost 比 initial cost 更高时恢复 stage 前状态；正常下降和零迭代评估保留更新。CLI 和 sweep 会记录 `stage state [name]: decision=... restored=... initial_cost=... final_cost=...` 以及 `stage_<name>_state_*` 字段。

这个机制的位置需要分清。它不是用 Kalibr 结果作为 oracle 选择“最接近 Kalibr 的一步”，也不会解决独立初始化下外参平移可观性不足的问题；它解决的是 Optimizer2 风格更新循环里的基本安全性：某个阶段如果产生不可用或变坏的参数状态，后续阶段不会在这个坏状态上继续线性化。当前 `current-full` 回归四个 stage 都是 `accepted/restored=0`，说明该保护层不改变已验证的 Kalibr 初值基线。

staged release 还必须把“时间变量”定义完整。Kalibr 的 camera residual 查询时间可以写成

$$
t_b
=
t_c+\Delta_{c}^{\rm prior}+\delta_c,
$$

其中 $\Delta_{c}^{\rm prior}$ 是互相关初始化得到的 camera-to-IMU prior，$\delta_c$ 是 `cameraTimeToImuTimeDv`。Ceres 没有把这两项拆成两个参数块，而是把总和存在 camera time-shift block 里。多 IMU residual 还有非参考 IMU 的时钟对齐量

$$
t_b=t_i+\delta_i,\qquad \delta_0=0,\quad i>0.
$$

Kalibr 主优化里，$\delta_i$ 不是 design variable。`IccImu.findOrientationPrior()` 或 `--imu-delay-by-correlation` 会先估出 `self.timeOffset`，随后 IMU residual 用 `tk = stamp + self.timeOffset` 查询 spline；`IccImu.addDesignVariables()` 只加入 bias spline、`q_i_b` 和 `r_b`，没有把 `timeOffset` 加入 problem。因此 Kalibr-aligned 默认应把非参考 IMU time offset 当固定 correction，而不是主优化变量。Ceres 仍保留显式扩展：只有传 `--optimize-imu-time-offsets` 时，$\delta_i$ 才进入 Ceres problem。

因此 `stage-free` 里的 `t` 首先表示 camera time shift；在显式开启 per-IMU time-offset optimization 时，它也表示所有会改变 IMU 测量查询时刻的变量。Ceres staged options 现在按这个 active set 构造：

$$
\mathcal A_t =
\{s_c\}_{c=0}^{N_c-1}
\cup
\{\delta_i\mid i>0,\ {\tt optimize\_imu\_time\_offsets}=1\}.
$$

当某个 stage 没有释放 `t` 时，`fix_time_shift=true`，并且非参考 IMU time-offset optimization 被关掉；释放 `t` 时，camera time-shift block 会进入 tangent space，显式开启 `--optimize-imu-time-offsets` 时非参考 IMU time-offset block 也一起进入。`none` 和 `-` 是空 active set，不再按普通字符串解析成含有字符 `e` 的 mask，也不会被 legacy IMU-extrinsic always-free 规则覆盖。`two_cams_two_imus` 的 staged dry-run 验证了这个边界：`stage-free none,t --stage-iterations 0,0` 下，`none` stage 的 `parameter_blocks=3080, active_parameter_blocks=0, tangent_params=0`；默认 `t` stage 是 `parameter_blocks=3080, active_parameter_blocks=3, tangent_params=8`，对应两个 camera time shifts 和 legacy 非参考 IMU extrinsic。显式加 `--optimize-imu-time-offsets` 后，`t` stage 变为 `parameter_blocks=3081, active_parameter_blocks=4, tangent_params=9`；多出的 parameter block 正是非参考 IMU time offset。默认 Kalibr-aligned 口径不加 `--optimize-imu-time-offsets` 时，rich 数据的 from-Kalibr dry-run 为 `parameter_blocks=6292, active_parameter_blocks=6291, tangent_params=22641`，与 Kalibr summary 的 `jacobian=129526x22641` 对齐。`stage-free none,i --stage-iterations 0,0` 进一步验证显式 `i` 调度：`none` 仍是 0 active，`i` stage 为 `active_parameter_blocks=1, tangent_params=6`。

新的半径 sweep 给出了当前最有用的调度线索。保持独立初始化、最后 `pe` 阶段和初始半径 `10000` 不变，`pe` 只跑 8 次时，`max_trust_region_radius=10/100/1000/10000` 对应最终 translation delta 约为 `32.84 mm / 32.46 mm / 29.17 mm / 6.06 mm`。这说明较大的 early trust region 对外参平移脱离零平移盆地是必要的。继续把 `pe` 跑到 12 次时，默认 `max=1e16` 会漂到 `80.75 mm`；限制 `max=6.561e7` 后漂到 `56.39 mm`；限制 `max=1e7` 时第 10 次达到最小 `2.433 mm`，第 12 次又回到 `12.54 mm`。因此 `independent-capped-pe-full` 固化为 `--stage-iterations 0,1,4,10 --stage-solver-max-trust-region-radii 1e16,1e16,1e16,10000000`，只给最后 pose/extrinsic 阶段加 cap。2026-06-16 复验输出在 `out/ceres_sweeps/abs_parameter_default_regression_20260616/`：`current-full` 仍保持 translation delta `5.17357e-05 m`、time-shift delta `+0.00084628 s`；`independent-capped-pe-full` 的外参平移差 `0.00243325 m`、rotation delta `0.0886821 deg`、time-shift delta `-0.00252163 s`、residual mean `0.209027 px / 0.172274 rad/s / 0.880630 m/s^2`。同时 `absolute_stop_smoke_20260616`、`stage_absolute_stop_smoke_20260616` 和 `parameter_stop_smoke_20260616` 验证了全局、per-stage 以及 parameter-delta absolute stop callback 都会打印 `absolute_stop` 并进入 `summary.csv`。`independent_kalibr_stop_pe_20260616` 给出 step-norm 负例：最后 `pe` 阶段使用近似 Kalibr 阈值 `deltaJ=1`、`step_norm=0.01`，20 次内没有触发，最终平移差漂到 `0.0424538 m`。新增的 `independent_final_pe_parameter_trace_20260616` 则给出 parameter-delta 负例：final `pe` 的平移差在第 `10` 次达到最小 `0.00243325 m`，而 `parameter_delta` 在第 `20` 次才最小，为 `0.0028641`，此时平移差已经是 `0.0424538 m`。因此 parameter-delta callback 是必要的观测工具，但单个 `deltaX` 阈值仍不能替代当前固定 `pe=10` 的 best-so-far 调度。这个 preset 不是默认推荐解，而是证明剩余差距已经进入主优化调度、trust-region 路径、停止条件和外参可观性取舍问题。

## 下一步

下一阶段若要继续减少对 Kalibr 结果文件的依赖，应把新初始化模块接入全量 staged 评测，而不是只看 dry-run：

1. 继续保留 `--estimate-time-shift-prior --estimate-orientation-gravity-prior` 作为独立初始化入口，但不要引入 accelerometer-only 平移初值。实测数据验证表明这种平移拟合容易被 accel bias、pose 二阶导和动态段噪声混淆。
2. 针对零平移起点设计更接近 Kalibr 的主优化调度。当前 conservative staged 路径会让 pose 先吸收相机约束，最后释放外参时平移仍停在零附近；`independent-final-pe-full` 和 `independent-capped-pe-full` 证明 pose+extrinsic 联合释放与 trust-region 上限能显著改善平移，但迭代数过多仍会漂移。
3. 利用 `optimizer/state_snapshot`、`--solver-restore-best-state` 和完整的 stage active set 继续靠近 Optimizer2 的状态更新语义。当前已经有阶段失败/非有限 cost/cost 上升回滚，也有单次 solve 内部 best accepted state 恢复保护；`stage-free` 的 `t` 已覆盖 camera time shift，并在显式 `--optimize-imu-time-offsets` 时覆盖非参考 IMU time offset。后续若要继续改进接受策略，应只使用当前 stage 内部 cost、鲁棒 residual 统计或稳定性指标，避免把 Kalibr 结果当成默认路径 oracle。
4. Kalibr 的 orientation prior 小优化器结构和 pose spline 初始化正则已经迁移；当前证据显示它们不是剩余平移差的主因。下一轮优先做非参考 IMU extrinsic 的可观性评分、弱先验/正则化实验，并继续比较 Kalibr 主优化的 Optimizer2 线性求解细节和 pose/extrinsic 联合释放阶段的停止策略。
