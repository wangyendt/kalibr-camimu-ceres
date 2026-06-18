# Kalibr IRLS 鲁棒核对齐

## 背景

多 camera / 多 IMU 支持完成后，TUM 双目已经能与 Kalibr 对齐，但 benchmark 的 1 camera + 4 IMU joint 仍出现异常：四路 IMU 分别做单 IMU cam-IMU 标定都正常，合成 4-IMU joint 后 camera residual 会被 IMU cost 拉坏，reprojection mean 曾达到 `54.758 px`。

这个现象容易误判为多 IMU 建图、`T_ib` 解析、time shift 初始化或 camera residual 坐标系问题。逐项拆开后发现：当 camera extrinsic 和 camera-to-IMU time shift 固定在 Kalibr 热启动值，只优化 pose / bias / gravity / IMU chain 时，Ceres residual 能回到 Kalibr 同量级；一旦释放 camera 或 time shift，结果才漂移。

## 根因

Kalibr 当前 `corner_file` 路径会启用 M-estimator：

- camera reprojection 使用 `CauchyMEstimator(10)`。
- calibrated IMU 的 gyro / accel 也使用 M-estimator。

关键不是宽度参数，而是线性化语义。Kalibr/aslam_backend 把 M-estimator 当作 IRLS 权重使用：每轮根据 raw squared error 计算 `weight`，再用 `sqrt(weight)` 同时缩放 residual 和 Jacobian。它没有把权重函数导数写入 Hessian。

Ceres 标准 `ceres::CauchyLoss` / `ceres::HuberLoss` 不是这个语义。Ceres 会按 loss function 的 `rho'` / `rho''` 参与鲁棒化线性化，和 Kalibr 的 IRLS Hessian 不一致。多 IMU joint 中 IMU residual 数量大、camera/time 又是全局强耦合变量，这个差异会被放大，表现为释放 camera/time 后 drift。

## 修复方式

本项目新增 Kalibr-style M-estimator loss：

- Cauchy 权重：`weight = 1 / (1 + s / width)`，其中 `s` 是 squared residual norm。
- Huber 权重：`weight = 1` when `s < width^2`，否则 `weight = width / sqrt(s)`。
- Ceres loss 输出：`rho[0] = weight * s`、`rho[1] = weight`、`rho[2] = 0`。

这样 Ceres 在构造正规方程时等价于 residual/Jacobian 同乘 `sqrt(weight)`，并避免引入 Kalibr 没有的权重导数项。实现位置是 `src/optimizer/calibration_problem.cpp` 的 `KalibrMEstimatorLoss`。

## 验证证据

使用 `2025_03_14_00_10_18` benchmark，输入为 1 camera corners + 4 路 IMU CSV，Kalibr joint 结果作为全量热启动。staged 口径为：

1. `pbg`：先优化 pose / bias / gravity / IMU chain，固定 camera extrinsic 和 time shift。
2. `pbegt`：继续释放 camera extrinsic 和 time shift。

修复后的 Ceres 与 Kalibr 对比：

| 指标 | Kalibr | Ceres | 差异 |
|---|---:|---:|---:|
| reproj mean | `0.236473 px` | `0.229823 px` | `-0.006651 px` |
| camera rotation | - | - | `0.004655 deg` |
| camera translation | - | - | `0.0755 mm` |
| camera time shift | `-0.0842046 s` | `-0.0837319 s` | `+0.4727 ms` |
| gyro mean | `0.075923 rad/s` | `0.106322 rad/s` | `+0.030398 rad/s` |
| accel mean | `0.339126 m/s^2` | `0.596738 m/s^2` | `+0.257612 m/s^2` |

对照旧实现：同样热启动并释放 `pbg,pbegt` 时，Ceres 标准 Cauchy 会漂到 `2.304 px`，time shift 差约 `+11.12 ms`。修复后 camera/time 可释放，且外参和 time shift 与 Kalibr 保持同量级。

## 使用边界

这个结论说明多 IMU residual 连接、Kalibr `T_ib` 解析、staged 执行路径和 camera/time 释放在 Kalibr 热启动 benchmark 口径下已经对齐。它不等价于证明冷启动全自由多 IMU joint 已经稳定。后续若验证冷启动，需要单独记录初始化、先验和停止条件，不要复用这份热启动结论。

多 camera 的状态独立于这次鲁棒核修复：TUM 双目失败根因是多 camera 未从 camchain 初始化 cam1 外参；补 `--init-from-camchain` 后，TUM 两组双目数据已与 Kalibr 在 `0.06 deg / 1 mm` 内对齐。
