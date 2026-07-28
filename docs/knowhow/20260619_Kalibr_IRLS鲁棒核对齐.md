# Kalibr IRLS 鲁棒核对齐

## 背景

多 camera / 多 IMU 支持完成后，TUM 双目已经能与 Kalibr 对齐，但 benchmark 的 1 camera + 4 IMU joint 仍出现异常：四路 IMU 分别做单 IMU cam-IMU 标定都正常，合成 4-IMU joint 后 camera residual 会被 IMU cost 拉坏，reprojection mean 曾达到 `54.758 px`。

这个现象容易误判为多 IMU 建图、`T_ib` 解析、time shift 初始化或 camera residual 坐标系问题。逐项拆开后发现：当 camera extrinsic 和 camera-to-IMU time shift 固定在 Kalibr 热启动值，只优化 pose / bias / gravity / IMU chain 时，Ceres residual 能回到 Kalibr 同量级；一旦释放 camera 或 time shift，结果才漂移。

## 根因

Kalibr 当前 `corner_file` 路径会启用 M-estimator：

- camera reprojection 使用 `CauchyMEstimator(10)`。
- calibrated IMU 的 gyro / accel 也使用 M-estimator。

关键不是宽度参数，而是线性化语义。Kalibr/aslam_backend 把 M-estimator 当作 IRLS 权重使用：每轮根据 raw squared error 计算 `weight`，再用 `sqrt(weight)` 同时缩放 residual 和 Jacobian。它没有把权重函数导数写入 Hessian。

> **以下这段根因判断已被 20260728 的对照实验推翻，保留原文仅供追溯，不要再引用。** 更正见文末「根因更正（20260728）」。
>
> ~~Ceres 标准 `ceres::CauchyLoss` / `ceres::HuberLoss` 不是这个语义。Ceres 会按 loss function 的 `rho'` / `rho''` 参与鲁棒化线性化，和 Kalibr 的 IRLS Hessian 不一致。多 IMU joint 中 IMU residual 数量大、camera/time 又是全局强耦合变量，这个差异会被放大，表现为释放 camera/time 后 drift。~~

## 修复方式

本项目新增 Kalibr-style M-estimator loss：

- Cauchy 权重：`weight = 1 / (1 + s / width)`，其中 `s` 是 squared residual norm。
- Huber 权重：`weight = 1` when `s < width^2`，否则 `weight = width / sqrt(s)`。
- Ceres loss 输出：~~`rho[0] = weight * s`~~、`rho[1] = weight`、`rho[2] = 0`。
  - 20260728 起 `rho[0]` 改为权重的原函数 $\rho_0(s)=\int_0^s w(u)\,\mathrm du$：Cauchy 为 `width * log1p(s / width)`，Huber 为 `s`（$s\le k^2$）/ `2k*sqrt(s) - k^2`（$s>k^2$）。原来的 `weight * s` 与声明的 `rho[1]` 不自洽（$\frac{\mathrm d}{\mathrm ds}[w(s)s]\ne w(s)$），会让 Ceres 的 trust-region 实际下降量与模型预测下降量对不上。

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

~~对照旧实现：同样热启动并释放 `pbg,pbegt` 时，Ceres 标准 Cauchy 会漂到 `2.304 px`，time shift 差约 `+11.12 ms`。~~（这条对照的归因已被推翻，见下节。）修复后 camera/time 可释放，且外参和 time shift 与 Kalibr 保持同量级。

## 根因更正（20260728）

上面「Ceres 会用 `rho''` 做鲁棒化线性化，所以和 Kalibr IRLS 不一致」的判断是错的。三条证据。

**证据一：Ceres 对凹损失根本不做 Triggs 修正。** `ceres::internal::Corrector` 的构造函数里有一条短路：

```
if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
  residual_scaling_ = sqrt_rho1_;
  alpha_sq_norm_ = 0.0;
  return;
}
```

Cauchy 和 Huber 都是凹的，$\rho''\le0$ 恒成立（Cauchy 严格为负；Huber 在阈值内 $\rho_0(s)=s$ 是仿射的，$\rho''$ 恰为零，同样满足 `rho[2] <= 0.0`），所以标准 `CauchyLoss` / `HuberLoss` 走的**也是**纯 IRLS 路径——`rho''` 被丢弃，只留 $\sqrt{\rho'}$ 缩放。把 `rho[2]` 显式置零并不会改变任何东西。本机 Ceres 2.1 上用一个一维最小算例实测：`CauchyLoss(3.0)`（真 `rho2`）与同 `rho0/rho1` 但 `rho2=0` 的自定义 loss，收敛点 `x=5.0000000000035181`、迭代数 4，**逐位相同**。

注意这条证据的边界：短路是 Ceres 的实现细节，不是它对外承诺的 `LossFunction` 语义，而且本仓库的 CMake 没有钉死 Ceres 版本。上面的实测只覆盖本机 Ceres 2.1。

**证据二：Kalibr 与我们的宽度约定本来就一致。** `MEstimatorPolicies.cpp:46` 的 `CauchyMEstimator::getWeight(error)` 是 `1.0 / (1.0 + error/_sigma2)`，`error` 已是平方误差，所以 `width == sigma2`。旧实现用的是 `ceres::CauchyLoss(std::sqrt(width))`，换算正确，不存在宽度错配。

**证据三：直接对照实验。** `2025_03_14_00_10_18`、Kalibr joint 全量热启动、staged `pbg,pbegt`、30+30 迭代，只换 loss 实现，其余完全相同：

| loss 实现 | reproj mean px | gyro mean rad/s | accel mean m/s² | time shift s | cam0 平移差 |
|---|---:|---:|---:|---:|---:|
| Kalibr（参照） | `0.210966` | `0.016290` | `0.117208` | `-0.1168684` | — |
| 旧 `rho0 = w·s` | `0.321385` | `0.027148` | `0.247940` | `-0.1168807` | `8.65 μm` |
| 新 `rho0 = ∫w` | `0.309146` | `0.026477` | `0.233393` | `-0.1168789` | `7.76 μm` |
| 标准 `CauchyLoss(√width)` / `HuberLoss(width)` | `0.309146` | `0.026477` | `0.233393` | `-0.1168789` | `7.76 μm` |

两个结论。第一，新 `rho0` 与 Ceres 标准 loss 的 reprojection 均值到小数点后 17 位完全相同（`0.30914556219354439`）——因为 $\rho_0,\rho_1$ 解析相等，而 $\rho_2$ 如证据一所述被短路丢弃，二者本就是同一个 loss。第二，标准 Cauchy 在这个口径下给出 `0.309 px`，**不是 `2.304 px`**。

> **关于绝对量级。** 上表三个 Ceres 行都在 `0.31 px` 附近，而本文档「验证证据」一节记录的是 `0.229823 px`。二者相差不是回归：这次复跑用的是当前默认的 `--corner-defaults` 和 `--ceres-multi-imu-candidate-preset none` 单候选路径，与当时的命令行不完全相同。本节要说明的是**同一条命令下只换 loss** 的相对差异，四行之间是严格可比的；跨文档比绝对值则不可比。判断有无回归请看表内三行的相对关系：新 `rho0` 在三项残差上都优于旧 `rho0`，外参和 time shift 差也更小。

所以当年那次 `2.304 px` 不可能由鲁棒核语义造成。引入 `KalibrMEstimatorLoss` 的那个 commit（`b0486dd`）同时重写了多 IMU joint 的大量逻辑，`2.304 px` 更可能来自那批改动里的其它部分，当时被一并记到了鲁棒核头上。

**这份文档的结论该怎么用。** "Kalibr 是 IRLS 语义、不把权重导数写进 Hessian" 这个描述本身仍然正确，也仍然是理解 `KalibrMEstimatorLoss` 的正确框架；不成立的是"因此必须自定义 loss，否则会漂"这一步推论。就 Cauchy / Huber 而言，自定义 loss 与标准 loss 在 Ceres 2.1 上数值等价，保留它的价值在于显式表达意图、以及为将来接入 Ceres 没有内置的权重函数留出位置。

引用这个等价性时请连着它的两个前提一起引用：$\rho_0,\rho_1$ 解析相同（`tests/test_math.cpp` 已对着 `ceres::CauchyLoss` / `ceres::HuberLoss` 逐点比对锁住），以及 `Corrector` 对 $\rho''\le0$ 短路（Ceres 实现细节，只在本机 2.1 上验证，构建未钉版本）。单测锁住的是前者，不是后者。一旦引入凸的 $\rho$，或 Ceres 改掉短路，$\rho_2$ 的取值就会重新产生数值后果。

## 使用边界

这个结论说明多 IMU residual 连接、Kalibr `T_ib` 解析、staged 执行路径和 camera/time 释放在 Kalibr 热启动 benchmark 口径下已经对齐。它不等价于证明冷启动全自由多 IMU joint 已经稳定。后续若验证冷启动，需要单独记录初始化、先验和停止条件，不要复用这份热启动结论。

多 camera 的状态独立于这次鲁棒核修复：TUM 双目失败根因是多 camera 未从 camchain 初始化 cam1 外参；补 `--init-from-camchain` 后，TUM 两组双目数据已与 Kalibr 在 `0.06 deg / 1 mm` 内对齐。
