# Kalibr-style 多相机多 IMU 冷启动：迁移的是初始化算法，不是标定结果

## 结论

`ceres_cam_imu` 的 Kalibr-style 冷启动迁移的是 Kalibr **进入联合优化前的求初值顺序、时间约定和零值约定**，不是把 Kalibr 的最终外参、时间偏移、重力或优化状态读进来。

生产默认路径只运行一次确定性的初始化，再运行一次 Ceres joint solve：

```text
各 camera 对参考 IMU 估时
        ↓
cam0 对参考 IMU 估旋转、重力和参考 gyro bias
        ↓
各非参考 IMU 对参考 IMU 估时间偏移和旋转
        ↓
先把所有时间戳映射到参考 IMU 时间域
        ↓
初始化 pose / bias splines
        ↓
构建并求解一个 Ceres 联合问题
```

这里的 “Kalibr-style” 只表示结构和约定对齐。C++ 实现为了支持独立时间原点，使用了至少保留短序列 `50%` 的归一化全范围相关和粗到细搜索；这不是逐行翻译 Kalibr 的 Python，也不保证 Ceres 与 Kalibr 会进入完全相同的优化轨迹。

## 先分清三类数据

| 数据 | 是否用于 no-Kalibr 冷启动 | 含义 |
|---|---:|---|
| `corner_poses.csv` | 是 | 每帧 AprilGrid PnP 位姿，是相机观测的几何预处理，不是 cam-IMU 联合优化结果 |
| camchain 的相机间基线 | 多相机时需要一种来源 | 优先保留 camchain 几何；缺失项可由同步观测回退估计，但进入初始化器前必须得到完整相机链。cam0 的绝对 cam-IMU 位姿会被重新初始化 |
| Kalibr `results-imucam.txt` 中的外参、time shift、gravity | 否 | 只有显式 `--init-from-kalibr` 的暖启动入口才读取 |

因此，使用 `corner_poses.csv` 或相机链基线不等于“依赖 Kalibr 标定后的值”。真正需要排除的是 `--init-from-kalibr`、`--init-from-result`，以及从 camchain 沿用已有 cam-IMU time shift。当前生产冷启动路径会忽略 camchain time shift，只保留相机间几何；若 camchain 和同步观测都无法给出完整基线，则明确报错，而不是把缺失基线默认为零。

## 时间变量和符号

以 IMU0 为参考时间域。第 $c$ 个相机和第 $m$ 个 IMU 的查询时间分别写成

$$
t^{\mathrm{ref}}=t^{\mathrm{cam}_c}+\Delta_c,
\qquad
t^{\mathrm{ref}}=t^{\mathrm{imu}_m}+\delta_m,
\qquad
\delta_0=0.
$$

其中：

| 符号 | 含义 | 写入位置 |
|---|---|---|
| $\Delta_c$ | camera $c$ 到参考 IMU 的 applied shift | `initial_camera_time_shifts_s[c]` |
| $\delta_m$ | IMU $m$ 到 IMU0 的 applied offset | `initial_imu_time_offsets_s[m]` |
| $\delta_0$ | 参考 IMU 自身偏移 | 恒为 0 |

相机—IMU 和 IMU—IMU 的相关 lag 符号不能互抄。

设相机预测信号和 IMU 测量信号的起点分别为 $t_{p,0}$、$t_{i,0}$，相关实现最大化

$$
\sum_n p[n],i[n-k].
$$

相机 applied shift 为

$$
\Delta=t_{i,0}-t_{p,0}-k\,\Delta t.
$$

当两路时间原点相同时，它退化成 Kalibr camera—IMU 路径的 $-k\Delta t$。IMU—IMU 路径使用另一套索引方向；当前实现对应

$$
\delta=t_{\mathrm{ref},0}-t_{\mathrm{target},0}+k\,\Delta t.
$$

这正是源码里反复强调“camera 路径有 lag 取反，而 IMU chain 路径不能照抄该负号”的原因。

诊断字段还满足

$$
\text{shift\_s}
=\text{discrete\_shift\_samples}\times\text{sample\_dt\_s}
+\text{discrete\_shift\_residual\_s}.
$$

最后一项保留非整数采样周期的时间原点差。不能为了让日志中的 sample 数好看而把真正使用的秒值重新量化。

## 为什么不同时间原点仍能做互相关

“全范围”不是从 `-500 s` 到 `+500 s` 每隔一个 IMU 周期盲扫。实现先分别把两路信号采样到各自从零计数的均匀序列，lag 搜索只与两段信号的长度有关；两个原始时间戳起点之差最后由上面的解析式一次性加回。

假设 camera 从 `0 s` 开始，IMU 从 `500 s` 开始，只要两者记录了足够长的同一段运动：

1. 相关搜索比较的是两段运动波形的相对位置；
2. `500 s` 的原点差不扩大相关循环的长度；
3. 最终输出的 $\Delta$ 同时包含波形 lag 和精确原点差。

默认要求每个候选至少保留较短序列的 `50%`。这不是说原始时间戳必须有 `50%` 交集，而是说对齐后的共同运动样本不能少于短序列的一半。低于这个比例的边缘峰很容易由短片段偶然相关产生。

为了控制成本，搜索分两层：

1. 用 `10 ms` 左右的均匀粗网格比较全部合法 lag；
2. 在粗峰附近按原生 IMU 周期细搜；
3. 若最优点碰到局部细搜窗口，就扩窗，直到峰离开局部边界或触及全局重叠边界。

全局边界峰表示“证据仍想往搜索域外走”，不能解释为零偏移。camera 路径可保留调用者给出的可信初值；没有可信值时不得建立紧先验。非参考 IMU 的时间映射是 joint residual 的必要条件，生产 Kalibr-style 路径遇到边界峰会直接拒绝继续。

## Kalibr 初始化顺序如何落到 Ceres

### 1. 每个 camera 独立估计 $\Delta_c$

`estimateCameraImuTimeShiftPrior(...)` 先用相机 PnP 位姿拟合 order-6 pose spline，再比较

$$
\lVert\boldsymbol\omega_{\mathrm{camera}}(t)\rVert
\quad\text{与}\quad
\lVert\boldsymbol\omega_{\mathrm{imu0}}(t)\rVert.
$$

角速度模长不依赖未知的 camera—IMU 旋转，所以能先估时间，再估旋转。多相机不能共用 cam0 的 shift；每个 camera 都需要自己的 pose 流和自己的 $\Delta_c$。

### 2. 用 cam0 和 IMU0 估计旋转、重力与参考 gyro bias

时间对齐后，`estimateOrientationGravityAndGyroBiasPrior(...)` 求解

$$
\boldsymbol\omega_{i}
\simeq
\mathbf R_{ic}\boldsymbol\omega_c+\mathbf b_g.
$$

实现先用 Wahba/Kabsch 得到闭式旋转和常值 bias，再用只包含旋转、bias 的小 Ceres 问题细化。随后用该旋转把比力转到 world，取均值方向并归一到 $9.80655\,\mathrm{m/s^2}$，得到重力初值。

和 Kalibr 一样，cam0 平移此时不从加速度差单独求解，而是置零，留给后面的联合优化恢复。

### 3. 保留多相机相对基线

cam0 的绝对旋转更新后，其他相机不是各自生成一个互不相干的 body 外参，而是保持初始相机链中的

$$
\mathbf T_{c_i c_0}
=\mathbf T_{c_i b}^{\mathrm{init}}
\left(\mathbf T_{c_0 b}^{\mathrm{init}}\right)^{-1},
$$

再令

$$
\mathbf T_{c_i b}^{\mathrm{seed}}
=\mathbf T_{c_i c_0}\mathbf T_{c_0 b}^{\mathrm{seed}}.
$$

这一步只继承相机之间的几何，不继承 camchain 中已有的 cam-IMU time shift，也不保留 cam0 的旧绝对 cam-IMU 位姿。

### 4. 每个非参考 IMU 独立估计 $\delta_m$ 和 $\mathbf R_{i_m b}$

`estimateImuChainPrior(...)` 逐个把 IMU $m$ 与 IMU0 配对：

1. 对两路 gyro norm 做全范围粗到细相关，求 $\delta_m$；
2. 按 $t^{\mathrm{ref}}=t^{\mathrm{imu}_m}+\delta_m$ 插值配对三轴陀螺；
3. 用 Wahba 和小 Ceres 问题估相对旋转；
4. 所有 IMU 平移置零；非参考 gyro bias 也从零开始。

这里刻意不使用旧的 accel lever-arm 初值和 accel refine 候选。它们是历史诊断分支，不属于最终的单一 Kalibr-style 生产路径。

### 5. 先对齐时间域，再创建 spline

只把 offset 写进 residual 还不够。若 camera、IMU0、IMU1 分别从 `0 s`、`500 s`、`1000 s` 开始，而 spline 仍按原始最小/最大时间创建，就会生成跨度近 `1000 s` 的巨大空样条。

`alignedTimeSpan(...)` 在创建 pose/bias splines 前先计算

```text
camera timestamp + per-camera shift
IMU timestamp    + per-IMU offset
```

再从已对齐时间的最小值和最大值确定样条范围。这个步骤是“支持独立时钟”的端到端条件，不只是互相关函数自己的能力。

### 6. 初始化 Ceres 状态并只求解一次

初始化器输出被写入：

| 初值 | Ceres 状态 |
|---|---|
| cam0 旋转、零平移 | `state.T_c_b` / `state.camera_extrinsics[0]` |
| 每个 camera 的 shift | `state.camera_time_shifts[c]` |
| 重力 | `state.gravity` |
| 参考 gyro bias | 参考 IMU gyro bias spline 的全部控制点 |
| 非参考 IMU 旋转、零平移 | `state.imu_extrinsics[m]` |
| 非参考 IMU offset | 固定时间修正；只有显式开启时才作为优化变量 |

之后走原有 Ceres camera/gyro/accel/bias residual 装配和 joint solve。初始化阶段没有运行 Kalibr optimizer，也没有用 Kalibr 最终值挑候选。

## 代码分层：为什么改动看起来不少

核心编排器位于：

```text
include/ceres_cam_imu/initialization/kalibr_style_multi_imu_initializer.h
src/initialization/kalibr_style_multi_imu_initializer.cpp
```

它主要负责顺序、零值约定、相机链传播和结果组装。真正占代码量的是三个必须贯通的基础能力：

| 层 | 主要文件 | 必要性 |
|---|---|---|
| camera—IMU 全范围估时 | `time_shift_initializer.*` | 支持不同时间原点、50% 重叠、粗到细、边界拒绝和精确秒值 |
| IMU—IMU 全范围估时与旋转 | `multi_imu_initializer.*` | 每个非参考 IMU 都有独立时钟和独立旋转 |
| 共用相关内核 | `src/initialization/uniform_signal_correlation.h` | 两条路径共享归一化相关、重叠域和粗到细搜索，但显式保留相反的 applied-shift 符号约定 |
| 已对齐时间域的状态构造 | `calibration_problem.*` | 防止独立 epoch 生成巨大 spline，并把每路 shift/offset 用到 residual |
| CLI 编排 | `apps/calibrate_cam_imu.cpp` | 明确优先级、阻止 Kalibr time shift 泄漏、处理多 camera pose 流和失败语义 |
| 端到端回归 | `tests/test_kalibr_style_*`、`tests/test_cli_kalibr_style_*` | 锁住 1cam+Nimu、Mcam+Nimu、显式时间优先级、独立 epoch 和生产单路径 |

所以不能只留下一个“调用三个函数”的薄 wrapper，而删掉时间域与状态构造改动；那样单元测试可能仍能估出 shift，真实 joint problem 却会在错误或巨大的时间域上建模。

### 2026-07-30 收口时保留与删除的边界

保留的是生产路径必须贯通的能力：单次 Kalibr-style 编排、两类全范围相关、每路独立时间量、对齐后的 spline 时间域、多相机独立 prior center，以及覆盖这些约定的回归测试。benchmark 仍保留显式 `--ceres-multi-imu-init kalibr`，但它只是比较用的暖启动诊断，缺少 Kalibr result 时会报错，绝不会静默回退成另一种生产初始化。

删除的是本轮演化过程中留下、但已经不再代表产品行为的 runner 六候选 selector、adaptive short/long、四次 single-result seed 拼装，以及重复的 camera/IMU 相关搜索实现。初始化结果中与 `cameras[0]` 重复的别名字段也已删除，避免两份状态日后不一致。

仓库原有的 translation/lever-arm 等显式诊断开关没有在这次迁移中一并删除，因为它们还有独立实验和非生产调用者；Kalibr-style 生产入口不会使用它们，同时传入时会打印忽略警告。这里的“单路径”指生产默认没有运行时选候选或失败后换算法，不表示整个研究仓库只能保留一个诊断函数。

## 与 Kalibr 一致和不一致的边界

| 项 | 当前关系 |
|---|---|
| 初始化顺序 | 对齐 Kalibr 的先时间、后旋转/重力、再进 joint 的结构 |
| camera/IMU 平移初值 | 对齐为零 |
| camera—IMU lag 符号 | 对齐 Kalibr applied shift 约定 |
| 非参考 IMU offset | 默认作为固定时间修正，符合 Kalibr 主问题语义 |
| 独立时间原点 | Ceres 扩展能力；Kalibr 原始实现通常依赖已有近似共同时间域 |
| 相关评分与搜索 | Ceres 使用归一化、最小重叠和粗到细，不是 `np.correlate` 的逐行翻译 |
| 主优化器与停止策略 | 仍是 Ceres，不等同于 Kalibr Optimizer2 |
| 最终 joint 解 | 不承诺逐位相同；需要按外参、time shift、residual 和 wall time 实测 |

## 必须保留的回归条件

代码调整后至少检查以下不变量：

1. camera shift 的正、负和独立 epoch 用例都满足 applied-shift 符号。
2. IMU offset 的正、负和独立 epoch 用例都满足自己的符号约定，不能复用 camera 的负号。
3. 非整数采样周期的 epoch 差保留 `discrete_shift_residual_s`。
4. 重复时间戳只在相关输入中去重；倒序时间戳拒绝。
5. 全局边界峰不被当成零偏移的可信估计。
6. 2cam+Nimu 中两个 camera 的 shift 独立，且相机间基线保持不变。
7. 显式 camera0 time shift 的优先级高于估计器。
8. 独立 epoch 的 joint spline 段数仍与真实记录时长同量级，而不是与 epoch 差同量级。
9. benchmark 默认不出现 `--init-from-kalibr`、`--init-from-result`、历史多候选或 staged fallback。
10. Release 和 Debug 都真实执行断言并通过。

## 证据边界

全量实测结果见：

- `docs/experiment/20260729_Ceres与KalibrDocker多数据集速度精度全量复核.md`
- `docs/experiment/20260729_相机IMU全范围时间偏移初始化复核.md`

当前实测覆盖 12 组 `1cam+1imu`、48 个逐 IMU single、12 个 `1cam+4imu` joint 和 2 个 TUM `2cam+1imu`；另外用合成 CLI 回归覆盖 `2cam+2imu` 和独立 epoch。仓库仍没有一组真实 `Mcam+Nimu` 数据，因此“真实多相机多 IMU”仍应视为尚待补齐的外部验证，而不是由合成测试替代。
