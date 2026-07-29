# Ceres Native 与 Kalibr Docker 多数据集速度精度全量复核

## 结论先行

本页记录 2026-07-29 在当前未提交工作树上的一次完整复跑。Ceres native 串行执行、每例只跑一次、solver 使用 4 线程；Kalibr Docker 使用 arm64 已有结果，不重复计算。Suite B joint 使用新的单路径 Kalibr-style 冷启动，Ceres 运行时不读取 Kalibr 优化结果，也不再跑六候选。

```bash
caffeinate -dimsu python3 tools/run_docker_benchmark.py --suite benchmark-single --benchmark-root /Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/review_20260729/suite_a_single --out-root out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_a_single

caffeinate -dimsu python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset single --benchmark-root /Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/review_20260729/suite_b_single --out-root out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_b_single

caffeinate -dimsu python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --benchmark-root /Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/review_20260729/suite_b_joint --out-root out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_b_joint

caffeinate -dimsu python3 tools/run_docker_benchmark.py --suite tum --tum-root /Users/wayne/Documents/work/data/TUM --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/review_20260729/suite_c_tum --out-root out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_c_tum
```

| Suite | 状态 | Ceres topology | 当前读数 |
|---|---|---|---|
| `benchmark-single` | 12/12 成功 | `1cam+1imu` | C/K 平移差均/最大 `2.27 / 3.12 mm`，time-shift `0.109 / 0.227 ms` |
| Suite B single | 48/48 成功 | 4 次独立 `1cam+1imu` | C/K 平移差均/最大 `3.48 / 16.07 mm`，48/48 的 reprojection mean 都小于 `1 px` |
| Suite B joint | 12/12 流程成功 | 一次 `1cam+4imu` | C/K effective-chain 平移差均/最大 `5.91 / 26.35 mm`；6 例 `USER_SUCCESS`，6 例到达 150 次上限 |
| `tum` | 2/2 成功 | `2cam+1imu` | C/K 平移差均/最大 `0.70 / 1.06 mm`，loop error 最大 `0.0346 deg / 0.245 mm` |

核心判断：

- 当前修改对“真正脱离 Kalibr 的初始化”是正向的。Suite B 的 48 个 single 不再出现旧六候选基线中 9 个边界峰失败；joint 从六候选实际搜索的平均 `1458.2 s` 降为单路径 `548.7 s`，C/K effective-chain 平移差也从 `18.38 / 60.78 mm` 降到 `5.91 / 26.35 mm`。
- 它不是“各项都已超过 Kalibr”。与旧 Kalibr 优化结果热启动 + tight bound 口径的 `2.9 / 5.2 mm` 相比，当前纯 Ceres 冷启动 joint 的平移 tail 更大；但后者使用了 Kalibr 优化后的值，不是可部署的 no-Kalibr 对照。
- 当前 Ceres joint 速度仍需收口。串行口径下，每个 session 四次独立 single 的 wall 平均总和是 `377.9 s`，一次 joint 却是 `548.7 s`，慢 `45.2%`；前 6 例全部用完 150 次迭代预算。
- 独立 single 与 joint 的一致性不能只看 C/K。Ceres single-joint 平移差为 `22.18 / 15.70 / 85.65 mm`，反而略好于 Kalibr 自身的 `25.53 / 18.42 / 96.02 mm`。这说明后半组数据存在 single/joint 估计盆差，不能把与 Kalibr joint 的毫米差全部解读为 Ceres 错误。
- 现有真实数据覆盖了 `1cam+1imu`、`1cam+4imu` 和 `2cam+1imu`，但没有真实 `Mcam+Nimu` 数据。`2cam+2imu` 的多相机多 IMU 组合目前由 CLI/单元 fixture 覆盖，不应写成已有真实数据验证。

## 配置口径

`--suite` 是 `tools/run_docker_benchmark.py` 的实验选择参数，不是 Ceres native 标定器参数。Ceres 标定器使用 `--corner-defaults` 按输入数量选择 topology。

| 项目 | 本次口径 |
|---|---|
| Ceres | native Release `build/calibrate_cam_imu`，solver `num_threads=4` |
| Ceres 代码 | 基于 HEAD `be8c0ddb0d6c` 的未提交工作树；commit 只是基座，不能单独复现本结果 |
| Kalibr Docker repo | `5fe9a59928ea`，`linux/arm64`，复用 2026-07-29 已有结果 |
| benchmark 数据根 | `/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data` |
| TUM 数据根 | `/Users/wayne/Documents/work/data/TUM` |
| 执行 | 四个 suite 串行，每例一次，`caffeinate -dimsu` 保持唤醒 |
| common defaults | `pose_kps=100`, `bias_kps=50`, `time_padding_s=0.04`, `camera_time_offset_buffer_s=0`，camera/gyro/accel loss 为 `cauchy:10` |
| single stop policy | `max_iterations=150`，absolute cost stop `0.005`，absolute step stop `0.02` |
| joint stop policy | `max_iterations=150`，absolute cost stop 关闭，absolute step stop `0.02` |
| 时间偏移初值 | 相机—每路 IMU 先粗后细互相关，默认不被 `±200 ms` 裁剪，候选至少保留较短信号 `50%` 重叠 |
| joint 初值 | 一次 deterministic Kalibr-style initializer；不读 Kalibr result，不跑 single seed，不跑六候选 |

summary 中 74 条 Ceres 记录都没有 `ceres_init_kalibr_result`，并且 `ceres_multi_imu_candidate_preset=none`；72 条 benchmark 记录还显式标为 `ceres_candidate=default`，TUM 两条没有 candidate 字段。Kalibr 结果只在 Ceres 完成后用作评估基线。

## 指标口径

| 指标 | 单位 | 含义 | 注意事项 |
|---|---:|---|---|
| success | count | Kalibr、Ceres、compare return code 均为 0 | `NO_CONVERGENCE` 在本文表示用完迭代上限，不是 subprocess 失败 |
| C/K translation | mm | Ceres 与 Kalibr 对应外参平移欧氏距离 | 与 Kalibr 比，不是真值误差 |
| C/K rotation | deg | SO(3) 相对旋转角 | 按 Ceres `Exp(-r)` 约定复算 |
| C/K time-shift | ms | Ceres time shift 减 Kalibr time shift | 聚合表使用绝对值 |
| residual mean | px / rad/s / m/s² | reprojection、gyro、accel mean residual | 不同单位不可横向比较 |
| optimize time | s | solver 内部优化耗时 | Ceres 与 Kalibr 内部统计口径不完全相同 |
| wall time | s | runner 端到端耗时 | Kalibr 本轮为复用值，与 Ceres 不是同时复跑 |
| Suite B effective chain | `T_c_i` | Ceres: `T_c_b inv(T_i_b)`；Kalibr: `T_ci inv(Tib)` | joint summary 的 camera0 列不能代表全部 4 路 IMU |
| TUM loop error | deg / mm | `T_cam1_cam0` 与 `T_cam1_imu T_imu_cam0` 的闭环差 | 只适用于多 camera |

## 结果总览

| Suite | Case 数 | success | 平均/最大平移差 | 平均/最大旋转差 | 平均/最大 abs time-shift | Ceres/Kalibr wall mean | Ceres/Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| `benchmark-single` vs Kalibr | 12 | 12/12 | `2.27 / 3.12 mm` | `0.0028 / 0.0067 deg` | `0.109 / 0.227 ms` | `87.8 / 127.5 s` | `85.5 / 75.6 s` |
| Suite B single vs Kalibr | 48 | 48/48 | `3.48 / 16.07 mm` | `0.0365 / 0.8816 deg` | `0.186 / 0.561 ms` | `94.5 / 123.7 s` | `92.2 / 73.2 s` |
| Suite B joint effective vs Kalibr | 12 joint / 48 chain | 12/12 | `5.91 / 26.35 mm` | `0.011 / 0.066 deg` | `0.388 / 1.210 ms` | `548.7 / 325.2 s` | `545.8 / 225.9 s` |
| `tum` vs Kalibr | 2 | 2/2 | `0.70 / 1.06 mm` | `0.0323 / 0.0402 deg` | `0.048 / 0.075 ms` | `71.3 / 64.7 s` | `70.0 / 23.5 s` |

## Suite A：benchmark-single

### 范围与聚合

12 个生产 benchmark session，每个 session 只使用 `data1.csv`。Ceres 是 `1cam+1imu`，12 例均为 `USER_SUCCESS`。

| 行数 | success | 平均/中位/最大平移差 | 平均/中位/最大旋转差 | 平均/中位/最大 abs time-shift | Ceres/Kalibr wall mean/total | Ceres iter mean/max |
|---:|---:|---:|---:|---:|---:|---:|
| 12 | 12/12 | `2.27 / 2.28 / 3.12 mm` | `0.0028 / 0.0024 / 0.0067 deg` | `0.109 / 0.104 / 0.227 ms` | `87.8 / 1053.5 s` / `127.5 / 1530.2 s` | `85.0 / 115` |

### 分数据集结果

`C/K` 表示 Ceres native / Kalibr arm64。

| 数据集 | 平移差 mm | 旋转差 deg | time-shift 差 ms | reproj px C/K | accel m/s² C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---:|---:|---:|---|---|---:|---:|---:|
| b01 | 2.05 | 0.0048 | +0.115 | 0.180173 / 0.179774 | 0.107558 / 0.108088 | 105.0 / 56.0 | 107.4 / 115.6 | 104 |
| b02 | 2.00 | 0.0032 | +0.024 | 0.180229 / 0.179743 | 0.107188 / 0.107294 | 105.0 / 81.4 | 106.8 / 138.6 | 105 |
| b03 | 1.89 | 0.0045 | -0.018 | 0.180156 / 0.180087 | 0.114465 / 0.114536 | 110.0 / 40.2 | 112.3 / 93.2 | 109 |
| b04 | 1.95 | 0.0045 | +0.042 | 0.179604 / 0.179124 | 0.107056 / 0.107415 | 105.0 / 79.8 | 106.6 / 130.3 | 106 |
| b05 | 2.23 | 0.0014 | +0.029 | 0.179073 / 0.178664 | 0.110898 / 0.111014 | 115.0 / 107.1 | 117.0 / 157.9 | 115 |
| b06 | 2.05 | 0.0017 | +0.064 | 0.177005 / 0.176607 | 0.116533 / 0.116744 | 100.0 / 53.3 | 102.7 / 104.1 | 99 |
| b07 | 2.33 | 0.0016 | +0.091 | 0.171694 / 0.171469 | 0.080700 / 0.080972 | 60.7 / 84.9 | 63.2 / 134.9 | 61 |
| b08 | 2.39 | 0.0000 | +0.227 | 0.171486 / 0.171172 | 0.084695 / 0.085099 | 62.8 / 101.7 | 65.2 / 151.7 | 62 |
| b09 | 2.60 | 0.0010 | +0.199 | 0.170900 / 0.170591 | 0.091989 / 0.093327 | 60.0 / 72.8 | 62.4 / 123.2 | 60 |
| b10 | 2.37 | 0.0000 | +0.184 | 0.171155 / 0.170795 | 0.088368 / 0.089701 | 61.1 / 61.1 | 63.6 / 111.5 | 61 |
| b11 | 3.12 | 0.0067 | +0.157 | 0.172019 / 0.171706 | 0.092370 / 0.092505 | 68.6 / 60.8 | 71.2 / 111.0 | 67 |
| b12 | 2.33 | 0.0040 | +0.156 | 0.171529 / 0.171249 | 0.116588 / 0.116552 | 72.5 / 108.6 | 74.9 / 158.2 | 71 |

读数：与 20260616 旧文档的 `2.05 / 4.03 mm`、`2.06 / 6.27 ms` 相比，外参仍是毫米级，time-shift 改善到亚 `0.23 ms`。Ceres wall 平均从 `110.2 s` 降到 `87.8 s`，但本轮只跑一次，不应把全部差异都归因于代码。

## Suite B：benchmark-multi-imu

### 范围与读法

每个 session 各跑四个独立 `single_imu1..4` 和一个 `joint_4imu`。joint 是实际部署的一次单路径：它不读前面四个 single 结果，四个 single 只用于实验对照。

| 对比 | 含义 | 用途 |
|---|---|---|
| Ceres single-joint | Ceres 独立 single vs Ceres joint effective chain | 看 Ceres joint 是否保持独立标定一致性 |
| Kalibr single-joint | Kalibr 独立 single vs Kalibr joint effective chain | 看数据/约束自身的 single-joint 差异 |
| C/K single | Ceres single vs Kalibr single | 看 Ceres 单 IMU 是否贴近 Kalibr |
| C/K joint | Ceres joint effective chain vs Kalibr joint effective chain | 看 joint 整条 IMU chain 是否贴近 Kalibr |

### 48 路独立 single

48 路全部为 `USER_SUCCESS`，初始化器 0 次边界拒绝，峰值相关系数最小 `0.9936`，reprojection mean 最大 `0.1977 px`。因此当前已不是“48 路中有 9 路初始化失败”的状态。

| 数量 | C/K 平移 mean/median/max | C/K 旋转 mean/median/max | abs time-shift mean/median/max | reproj mean max |
|---:|---:|---:|---:|---:|
| 48 | `3.48 / 2.46 / 16.07 mm` | `0.0365 / 0.0111 / 0.8816 deg` | `0.186 / 0.135 / 0.561 ms` | `0.1977 px` |

C/K single 平移差的前 10 项：

| 排名 | case | IMU | Δt mm | ΔR deg | Δτ ms |
|---:|---|---:|---:|---:|---:|
| 1 | b09 | 3 | 16.07 | 0.882 | -0.507 |
| 2 | b10 | 4 | 11.20 | 0.098 | -0.514 |
| 3 | b11 | 3 | 8.19 | 0.081 | -0.468 |
| 4 | b11 | 4 | 7.60 | 0.025 | +0.109 |
| 5 | b12 | 3 | 7.51 | 0.063 | -0.240 |
| 6 | b10 | 3 | 7.21 | 0.059 | -0.391 |
| 7 | b09 | 4 | 5.23 | 0.103 | +0.486 |
| 8 | b09 | 2 | 5.16 | 0.069 | +0.017 |
| 9 | b08 | 3 | 4.94 | 0.034 | +0.227 |
| 10 | b11 | 1 | 3.12 | 0.0067 | +0.157 |

### 48 条 single-joint / C-K effective chain

每个单元格依次为 translation、rotation、abs time-shift 的 `mean / median / max`。

| 分组 | Ceres single-joint | Kalibr single-joint | C/K single | C/K joint effective |
|---|---|---|---|---|
| all 48 | `22.18 / 15.70 / 85.65 mm`, `0.166 / 0.082 / 1.701 deg`, `1.24 / 1.26 / 2.57 ms` | `25.53 / 18.42 / 96.02 mm`, `0.147 / 0.096 / 0.887 deg`, `1.27 / 1.10 / 2.87 ms` | `3.48 / 2.46 / 16.07 mm`, `0.036 / 0.011 / 0.882 deg`, `0.19 / 0.14 / 0.56 ms` | `5.91 / 3.71 / 26.35 mm`, `0.011 / 0.005 / 0.066 deg`, `0.39 / 0.35 / 1.21 ms` |
| b01-b06 | `7.20 / 6.81 / 11.96 mm`, `0.081 / 0.069 / 0.227 deg`, `0.71 / 0.72 / 1.74 ms` | `7.08 / 7.03 / 12.84 mm`, `0.082 / 0.066 / 0.227 deg`, `0.80 / 0.71 / 1.57 ms` | `2.29 / 2.29 / 2.99 mm`, `0.009 / 0.007 / 0.022 deg`, `0.11 / 0.03 / 0.56 ms` | `2.11 / 2.13 / 4.85 mm`, `0.001 / 0.000 / 0.005 deg`, `0.28 / 0.27 / 0.74 ms` |
| b07-b12 | `37.17 / 36.85 / 85.65 mm`, `0.251 / 0.128 / 1.701 deg`, `1.78 / 1.75 / 2.57 ms` | `43.99 / 40.02 / 96.02 mm`, `0.212 / 0.143 / 0.887 deg`, `1.73 / 1.74 / 2.87 ms` | `4.67 / 3.05 / 16.07 mm`, `0.064 / 0.026 / 0.882 deg`, `0.26 / 0.21 / 0.51 ms` | `9.71 / 7.92 / 26.35 mm`, `0.021 / 0.016 / 0.066 deg`, `0.50 / 0.51 / 1.21 ms` |

读数：

- b01-b06 的 C/K joint effective 平移差平均 `2.11 mm`、最大 `4.85 mm`，已与旧 Kalibr-init tight 结果同量级。
- b07-b12 的平移 tail 仍大，主要是 b12 四路。但 Ceres 的 single-joint 差异没有比 Kalibr 更坏，所以这是后半组数据的 joint basin/可观性问题，而不是一个可以由“C/K 不一样”单独证明的 Ceres 错码。
- 旋转与时间偏移已经稳定；当前 joint 主要 tail 在平移。

### Joint 分 case 结果

`Δt/ΔR/abs Δτ` 取该 joint 四条 effective chain 的最大值。

| case | C/K joint max Δt mm | max ΔR deg | max abs Δτ ms | Δreproj px | Δgyro rad/s | Δaccel m/s² | Ceres/Kalibr wall s | Ceres/Kalibr optimize s | Ceres it/termination |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| b01 | 3.36 | 0.000 | 0.478 | +0.00009 | -0.00000 | -0.00034 | 691.2 / 288.4 | 689.0 / 195.7 | 151 / NO_CONVERGENCE |
| b02 | 3.70 | 0.001 | 0.185 | +0.00001 | -0.00000 | +0.00008 | 660.8 / 197.6 | 658.0 / 105.2 | 151 / NO_CONVERGENCE |
| b03 | 4.85 | 0.003 | 0.503 | -0.00002 | -0.00000 | +0.00032 | 651.2 / 207.2 | 649.0 / 111.8 | 151 / NO_CONVERGENCE |
| b04 | 3.72 | 0.003 | 0.339 | -0.00001 | +0.00005 | -0.00010 | 652.7 / 163.5 | 650.0 / 70.0 | 151 / NO_CONVERGENCE |
| b05 | 3.25 | 0.005 | 0.742 | -0.00003 | +0.00006 | +0.00008 | 684.9 / 162.5 | 682.0 / 69.8 | 151 / NO_CONVERGENCE |
| b06 | 3.28 | 0.000 | 0.248 | +0.00005 | -0.00002 | +0.00004 | 639.2 / 221.3 | 637.0 / 126.7 | 151 / NO_CONVERGENCE |
| b07 | 10.43 | 0.023 | 0.842 | +0.00079 | +0.00087 | -0.00927 | 438.6 / 529.6 | 435.0 / 421.5 | 84 / USER_SUCCESS |
| b08 | 7.72 | 0.025 | 1.210 | +0.00032 | +0.00010 | -0.00031 | 430.5 / 597.1 | 427.0 / 491.9 | 83 / USER_SUCCESS |
| b09 | 10.38 | 0.018 | 0.650 | +0.00040 | +0.00039 | -0.00346 | 419.1 / 388.6 | 416.0 / 282.1 | 80 / USER_SUCCESS |
| b10 | 9.53 | 0.020 | 0.911 | +0.00068 | +0.00061 | -0.00455 | 444.5 / 558.1 | 441.0 / 448.8 | 86 / USER_SUCCESS |
| b11 | 8.80 | 0.019 | 0.600 | +0.00018 | +0.00056 | -0.00367 | 397.8 / 339.0 | 394.0 / 240.2 | 83 / USER_SUCCESS |
| b12 | 26.35 | 0.066 | 1.135 | +0.00170 | +0.00309 | -0.02307 | 473.8 / 250.1 | 471.0 / 146.5 | 89 / USER_SUCCESS |

`NO_CONVERGENCE` 的 6 例都有 151 个 successful steps，是达到迭代预算，不是数值失败。它们的 effective-chain 平移差反而全部小于 `4.85 mm`；因此 termination label 不能当作精度排名。

### 速度：一次 joint vs 四次 single

| 路径 | session 数 | Ceres wall mean/total | Kalibr wall mean/total | Ceres optimize mean | Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|
| 四次独立 single 的串行和 | 12 | `377.9 / 4534.8 s` | `494.8 / 5937.7 s` | `368.9 s` | `292.7 s` |
| 一次 joint | 12 | `548.7 / 6584.2 s` | `325.2 / 3903.0 s` | `545.8 s` | `225.9 s` |

Kalibr 的 joint 平均比四次 single 串行和快 `34.3%`；Ceres 相反慢 `45.2%`。新初始化器自身的外围开销只有约 `3 s`，主耗时是 joint solver，尤其是 b01-b06 全部跑满 150 次。因此后续性能优化应聚焦在 joint 收敛/终止策略，不是恢复多候选。

## Suite C：TUM

### 范围

TUM suite 跑两个双目单 IMU case，Ceres topology 为 `2cam+1imu`。loop error 使用仓库脚本从本次 summary 派生：

```bash
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_c_tum/tum/tum_summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_c_tum/tum/tum_loop_error.csv
```

### 分 case 结果

| Case | 平移差 mm | 旋转差 deg | time-shift 差 ms | loop error Ceres deg/mm | loop error Kalibr deg/mm | reproj px C/K | gyro rad/s C/K | accel m/s² C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---:|---:|---:|---:|---:|---|---|---|---:|---:|---:|
| `tum_imu1_bag` | 1.064 | 0.0402 | -0.075 | 0.0294 / 0.213 | 0.0000 / 0.000 | 0.095005 / 0.100287 | 0.001163 / 0.001199 | 0.021010 / 0.021706 | 72.0 / 22.9 | 73.3 / 65.8 | 117 |
| `tum_imu2_euroc_kalibr_export` | 0.340 | 0.0243 | -0.021 | 0.0346 / 0.245 | 0.0044 / 0.000 | 0.093822 / 0.100253 | 0.001122 / 0.001171 | 0.020781 / 0.021605 | 68.0 / 24.0 | 69.3 / 63.6 | 111 |

两例 C/K 平移差都小于 `1.1 mm`，time-shift 差小于 `0.08 ms`；Ceres 三类 residual mean 均略低于 Kalibr。Ceres wall 平均 `71.3 s`，略慢于 Kalibr arm64 的复用记录 `64.7 s`，优化段仍约为 Kalibr 的 3 倍。

## 与 20260616 文档和六候选基线的变化

| 范围 | 20260616 当时口径 | 六候选 no-Kalibr 诊断口径 | 当前单路径 | 判断 |
|---|---:|---:|---:|---|
| Suite A C/K translation mean/max | `2.05 / 4.03 mm` | `2.27 / 3.16 mm` | `2.27 / 3.12 mm` | 稳定在毫米级 |
| Suite A abs time-shift mean/max | `2.06 / 6.27 ms` | `0.109 / 0.222 ms` | `0.109 / 0.227 ms` | 符号修复效果保持 |
| Suite A Ceres wall mean | `110.2 s` | `93.9 s` | `87.8 s` | 本轮更快，但无重复次数支持统计归因 |
| Suite B C/K single mean/max | `5.2 / 41.6 mm` | 全部 `47.9 / 365.8 mm` | `3.48 / 16.07 mm` | 全范围互相关消除 9 个边界峰失败 |
| Suite B C/K joint effective mean/max | Kalibr-init tight `2.9 / 5.2 mm` | `18.38 / 60.78 mm` | `5.91 / 26.35 mm` | 明显优于六候选，但未达使用 Kalibr 优化值热启动的对齐程度 |
| Suite B Ceres joint 实际搜索 wall mean | 单条 Kalibr-init `90.4 s` | 六候选 `1458.2 s` | 单路径 `548.7 s` | 比六候选省 `62.4%`，仍慢于四次 single 串行和 |
| TUM C/K translation mean/max | `0.69 / 1.04 mm` | `0.707 / 1.070 mm` | `0.702 / 1.064 mm` | 基本持平 |
| TUM Ceres wall mean | `61.5 s` | `99.8 s` | `71.3 s` | 介于两次记录之间，单次 wall 波动明显 |

这张表的关键是区分三种不同问题：

1. 时间初始化的符号和搜索范围已修好，48 路 single 是直接证据。
2. 用 Kalibr 优化结果做 joint 初值可以得到更小的 C/K 差，但这不满足独立部署要求。
3. 新 Kalibr-style 迁移让纯 Ceres 从六候选收敛为一条路，但 joint solver 本身的速度与平移 tail 仍有改进空间。

## 边界

- 外参差异是 Ceres 与 Kalibr 的差，不是真值误差；residual 更低也不自动等于外参更真实。
- Kalibr wall 来自复用结果，Ceres wall 来自本轮串行单次复跑。速度表适合看量级和路径差异，不适合解读几个百分点的微小变化。
- Suite B effective-chain 统计由本次 Ceres YAML 与 Kalibr result txt 复算，不使用 summary 中只代表 camera0/IMU0 的单列。
- 当前没有机械/CAD 真值，不能在 b12 上裁定 Ceres joint 或 Kalibr joint 谁更接近真值。
- 本轮没有重跑 Kalibr，也没有重做 stop-policy、线程数、外参 bound 或 robust loss ablation。
- 真实 `Mcam+Nimu` 数据仍是覆盖空缺。

## 证据和产物

| 路径 | 内容 |
|---|---|
| `out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_a_single/summary.csv` | Suite A 12 例汇总 |
| `out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_b_single/summary.csv` | Suite B 48 个独立 single 汇总 |
| `out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_b_joint/summary.csv` | Suite B 12 个单路径 joint 汇总 |
| `out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_c_tum/tum/tum_summary.csv` | TUM 两例汇总 |
| `out/docker_benchmarks/review_20260729/kalibr_style_single_path_full/suite_c_tum/tum/tum_loop_error.csv` | TUM 闭环误差 |
| 各 case 的 `ceres/result.yaml` | Ceres 外参、时间偏移、IMU chain 与 residual |
| 各 case 的 `compare_arm64/compare.clean.log` | C/K 直接对比记录 |
