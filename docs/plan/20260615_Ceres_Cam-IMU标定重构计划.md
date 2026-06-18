# Ceres Cam-IMU 标定重构计划

## 背景

`ceres-cam-imu` 的重构目标是把 cam-IMU 标定从 Kalibr/ROS/catkin 的耦合实现中拆出来，形成一条**可独立运行、可生产部署、可解释速度和精度边界**的 C++/Ceres 链路。Kalibr 仍然是强基线，但它在新链路里的角色必须限制为**评测 oracle**和**历史输入转换环境**，不能成为生产独立标定的初始化依赖。

当前问题已经从“能不能跑通”推进到“默认流程应该是什么”。早期 TUM 诊断曾显示二阶段能把 IMU residual 拉回 Kalibr 同量级，但进一步查 Kalibr 源码和 smoke 实验后，结论需要更新：Kalibr 主优化本身并不是显式分阶段释放变量，Ceres 也不必默认二阶段。当前更合理的生产默认是**Ceres 独立初始化 + 100/50 kps single-stage joint 优化**。

## 目标状态

- **独立求解**：生产命令不传 `--kalibr-result`，不读取 Kalibr 外参、time-shift、gravity 或 IMU intrinsic 作为初值。
- **一键入口**：`prepare_ceres_inputs.py --run-calibration` 转换后直接运行 production single-stage preset；`--run-two-stage` 保留为诊断入口。
- **模型覆盖**：相机模型覆盖 Kalibr 常见 pinhole/omni/eucm/ds 组合；IMU 覆盖 ordinary、scale-misalignment、scale-misalignment-size-effect。
- **生产速度**：camera/gyro/accel/扩展 IMU residual 使用手写解析 Jacobian，避免生产路径上的 per-residual 数值差分。
- **可复现实验**：production、TUM、扩展 IMU、多 camera 的结果都能从 `out/` 日志和实验文档追溯。

## 默认流程

生产默认流程固定为 single-stage joint：

```text
--corner-defaults
--estimate-time-shift-prior
--estimate-orientation-gravity-prior
--pose-fit-motion-lambda 0.0001
--pose-fit-boundary-anchors
--time-shift-prior-sigma 0.0001
--pose-motion-prior
--pose-motion-translation-variance 10
--pose-motion-rotation-variance 1
--max-iterations 150
--solver-max-trust-region-radius 10000000
```

这个 preset 已接入 `prepare_ceres_inputs.py --run-calibration`。如果要完全手工控制参数，可加 `--no-default-calibration-args`；如果只想覆盖数值参数，可在 `--` 后传对应参数，脚本不会重复补默认值。

`run_ceres_sweep.py` 也已收紧独立口径：只有包含 `--init-from-kalibr` 的热启动 variant 才会把 `--kalibr-result` 传给标定二进制；独立 variant 只在求解后调用 `compare_kalibr_result` 做离线 delta。

## Kalibr 做法

Kalibr 的 cam-IMU 主链路不是“强 pose prior + 分阶段释放变量”的显式调度。源码里 `kalibr_calibrate_imu_camera` 调用 `IccCalibrator.buildProblem(...)` 后直接 `optimize(...)`；主问题参数里 `doPoseMotionError=False`。

Kalibr 稳定性的关键在前置初始化和后端数值行为：

| 环节 | Kalibr 行为 | 对 Ceres 的启发 |
|---|---|---|
| time shift | 先估 camera-to-IMU shift prior | Ceres 已有 gyro-norm time-shift prior |
| orientation/gravity | 先解 orientation/gravity/gyro bias prior | Ceres 已有 orientation/gravity initializer |
| pose spline | 用 cam0 target poses 初始化 order-6 pose spline，`initPoseSplineSparse(..., 1e-4)` | Ceres 使用 pose fit motion lambda 和 boundary anchors |
| bias spline | 常值初始化，再加 bias motion terms | Ceres 已加 gyro/accel bias motion prior |
| 主优化 | 一次高 kps joint optimization，LM + BlockCholesky，`convergenceDeltaX=1e-2`、`convergenceDeltaJ=1` | Ceres single-stage joint 与它更接近 |
| 扩展 IMU | `M_a/M_g/A_g/C_g` 在主问题中直接 active | Ceres 扩展 IMU 也应作为 joint 变量，但需守住初始化和尺度 |

因此，Ceres 的默认方向不是复刻一个虚构的 Kalibr 多阶段流程，而是把**初始化、变量尺度、停止条件、trust-region 上限和解析 Jacobian**调到能支撑一次 joint 优化。

## 当前架构

| 模块 | 职责 | 当前状态 |
|---|---|---|
| `io/` | camchain/IMU/target YAML、IMU CSV、corner CSV、Ceres result、Kalibr result parser | 已可用 |
| `processing/` | IMU edge trim、frame limit、corner count、time range | 已拆出 |
| `camera/` | 多相机模型投影和 Jacobian | 已覆盖主要模型并有差分测试 |
| `trajectory/` | order-6 pose/bias B-spline、导数查询 | 已用于全量标定 |
| `initialization/` | pose fit、gyro-norm time shift prior、orientation/gravity prior | 已用于独立标定 |
| `residuals/` | camera、gyro、accel、bias prior、pose prior、time prior、扩展 IMU | 核心 residual 已解析 Jacobian |
| `optimizer/` | problem build、solver、summary/statistics、staged 诊断 | single-stage 为默认；staged 保留 |
| `tools/` | 输入转换、Kalibr Docker wrapper、Ceres sweep、two-stage 诊断 | production preset 已接入 `--run-calibration` |

## 证据一：Production 12 组 single-stage

2026-06-17 重新跑了 12 组 production 数据，命令不含 `--kalibr-result`，结果位于 `out/ceres_sweeps/production_single_stage_20260617/<session>/`。全部 `CONVERGENCE`。

| 数据集 | iter | Ceres 时间 | 外参平移差 | 外参旋转差 | time-shift 差 | Ceres residual mean |
|---|---:|---:|---:|---:|---:|---|
| `2025_03_14_00_10_18` | 110 | 117.7 s | 3.62 mm | 0.0302 deg | -0.988 ms | `0.18027 px / 0.01690 rad/s / 0.11464 m/s^2` |
| `2025_03_14_00_34_14` | 111 | 117.4 s | 4.66 mm | 0.0570 deg | -0.800 ms | `0.18026 px / 0.01710 rad/s / 0.11247 m/s^2` |
| `2025_03_14_00_50_37` | 115 | 122.0 s | 1.96 mm | 0.0052 deg | -0.783 ms | `0.18030 px / 0.01702 rad/s / 0.11921 m/s^2` |
| `2025_03_14_02_13_45` | 113 | 119.3 s | 2.35 mm | 0.0070 deg | -1.391 ms | `0.17971 px / 0.01652 rad/s / 0.11634 m/s^2` |
| `2025_03_14_02_21_41` | 121 | 129.4 s | 2.24 mm | 0.0016 deg | -0.575 ms | `0.17919 px / 0.01757 rad/s / 0.11466 m/s^2` |
| `2025_03_14_10_23_35` | 106 | 115.0 s | 2.38 mm | 0.0036 deg | -1.121 ms | `0.17719 px / 0.01662 rad/s / 0.12377 m/s^2` |
| `2025_04_19_18_43_05` | 79 | 85.1 s | 1.40 mm | 0.0051 deg | -3.199 ms | `0.17081 px / 0.01254 rad/s / 0.09587 m/s^2` |
| `2025_04_19_19_03_03` | 77 | 82.5 s | 3.05 mm | 0.0136 deg | -6.267 ms | `0.17237 px / 0.01316 rad/s / 0.11483 m/s^2` |
| `2025_04_19_19_20_46` | 77 | 82.1 s | 1.48 mm | 0.0000 deg | -0.700 ms | `0.17044 px / 0.01330 rad/s / 0.09613 m/s^2` |
| `2025_04_19_19_35_04` | 75 | 82.1 s | 1.31 mm | 0.0000 deg | +1.339 ms | `0.17189 px / 0.01440 rad/s / 0.08411 m/s^2` |
| `2025_04_19_19_55_25` | 84 | 91.6 s | 1.68 mm | 0.0073 deg | +1.420 ms | `0.17284 px / 0.01661 rad/s / 0.08659 m/s^2` |
| `2025_04_19_20_21_09` | 86 | 90.3 s | 2.49 mm | 0.0113 deg | -6.189 ms | `0.17220 px / 0.01248 rad/s / 0.14118 m/s^2` |

结论：single-stage production preset 可作为默认路径。12 组外参平移差 `1.31-4.66 mm`，旋转差最大 `0.057 deg`，Ceres reprojection mean 稳定在 `0.170-0.180 px`。两个 March session 的离线 Kalibr reprojection baseline 与前次 Docker 表不完全一致，因此 residual delta 不作为默认切换的主判断；默认切换依据是独立收敛、外参/time-shift delta 和 Ceres residual 自身稳定性。

## 证据二：TUM single-stage 最终结果

TUM 前期有过二阶段和旧 hotstart 诊断，其中一次高 kps 诊断把 `M_a/M_g` 拉到异常值。该结果来自旧诊断组合，不再作为当前决策依据；当前保留的最终口径是**不读 Kalibr result 的 single-stage 100/50 kps joint 优化**。

| TUM 数据 | Kalibr residual mean | Ceres single-stage residual mean | Ceres solver | 结论 |
|---|---|---|---|---|
| `dataset-calib-imu1_512_16` | `0.10646-0.10737 px / 0.001196 rad/s / 0.021465 m/s^2` | `0.10352 px / 0.001185 rad/s / 0.021219 m/s^2` | 27 iter, 15.9 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |
| `dataset-calib-imu2_512_16` | `0.10708-0.10702 px / 0.001169 rad/s / 0.021348 m/s^2` | `0.10321 px / 0.001215 rad/s / 0.021175 m/s^2` | 28 iter, 17.3 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |

TUM 证明第一阶段低 kps 轨迹频率不足会放大 IMU residual，但这不是数据读取、多 camera、扩展 IMU 前向公式或 `M_a/M_g` 的问题。直接使用足够的 pose/bias knot rate 后，single-stage 已能达到 Kalibr 量级，并且 Ceres solver 时间明显低于 Kalibr Docker 的 optimize 时间。

## 模型与扩展 IMU 状态

- 相机侧已覆盖 `pinhole+radtan/equidistant/fov/none`、`omni+radtan/none`、`eucm`、`ds`，并通过 projection Jacobian 差分测试。
- IMU 侧支持 `calibrated`、`scale-misalignment`、`scale-misalignment-size-effect`。
- 扩展 IMU 的 gyro、scale/misaligned accel、size-effect accel 已从数值差分切到手写 `SizedCostFunction`；`M_a/M_g/A_g/C_g` 与 Kalibr 的 production/TUM 对比在 `1e-3` 量级。
- Kalibr 和 Ceres 都支持 shared camchain + 多 camera joint 优化；Ceres staged multi-camera 仍是后续增强，不再阻塞当前默认流程。

## 决策

- **默认生产路径改为 single-stage joint**：production 12 组和 TUM 双目 smoke 都支持这个选择。
- **二阶段降级为诊断工具**：保留 `run_ceres_two_stage.py`，用于定位轨迹频率、全局块保护或局部数据病态，不作为默认入口。
- **`--kalibr-result` 只允许在热启动/compare 中出现**：独立 sweep 和 `--run-calibration` 默认不传该参数。
- **扩展 IMU 继续按解析 Jacobian 推进**：生产速度目标要求扩展模型不能回退到 per-residual 数值差分。

## 剩余工作

| 优先级 | 工作 | 目标 |
|---|---|---|
| P0 | 原生 AprilTag/输入转换方案 | 让 pkl/bag/euroc 之外的生产输入不再依赖 Kalibr Docker/ROS 做角点导出 |
| P0 | single-stage 默认回归集 | 把 12 组 production 和 2 组 TUM single-stage 纳入固定回归命令 |
| P1 | native-vs-native 速度评测 | 在同一 Linux native 环境重跑 Ceres 与 Kalibr，给出可信速度倍率 |
| P1 | 外参弱方向毫米级差异分析 | 对齐 LM、停止条件、变量缩放，判断是否值得继续追 sub-mm |
| P1 | 多 camera staged optimizer | 作为诊断能力补齐，不阻塞当前 multi-camera joint 默认路径 |
| P2 | 输入转换边界继续下沉 | 把 observation selection、运行配置描述从 CLI 里继续抽出 |

## 验收线

生产独立路径的验收标准是：不传 `--kalibr-result`，Ceres 自己完成 time/gravity/pose 初始化并 single-stage 收敛；输出结构化 result；必要时再用 `compare_kalibr_result` 做离线评测。实验路径可以读取 Kalibr result，但必须在命令和文档中标明“热启动/compare”，不能和生产默认命令混写。
