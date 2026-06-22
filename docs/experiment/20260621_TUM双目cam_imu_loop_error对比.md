# TUM双目cam-imu loop error对比

## 结论先行

上一版把 Kalibr 默认 fixed camera-chain 和 Ceres free camera-chain 混在一起比较，结论不公平。本轮按 fixed 和 free 两个口径重跑后，Ceres 与 Kalibr 的双目 loop consistency 已经是同量级。

| 口径 | Kalibr | Ceres | 结论 |
|---|---|---|---|
| fixed camera-chain | 默认固定 `T_cn_cnm1` | `--fix-camera-chain-extrinsics` tight prior | 两者 loop error 都接近 0 |
| free camera-chain | `--recompute-camera-chain-extrinsics` | 不加 fixed-chain 约束 | 两者 loop error 都在 `0.21-0.24 mm`、`0.03-0.035 deg` |

核心结论：上一版“Kalibr 按 loop error 胜出”只适用于 Kalibr fixed vs Ceres free 的混合默认口径，不应作为 solver 精度结论。

## 指标口径

| 指标 | 含义 |
|---|---|
| `Loop rot deg` | `inv(T_cam1_cam0) * (T_cam1_imu * T_imu_cam0)` 的旋转角 |
| `Loop trans mm` | 同一 loop error 的平移模长 |
| `fixed` | 双目 baseline 固定或强约束，主要验证约束口径是否一致 |
| `free` | 双目 baseline 作为自由变量重估，主要比较同自由度下的 loop consistency |
| `Reproj/Gyro/Accel mean` | 优化后观测残差均值，用于确认不是靠牺牲 residual 换 loop |

## 背景

这次实验重新回答同一个问题：在两组 TUM 双目 + 单 IMU 数据上，`kalibr-docker` arm64 和 Ceres native 的双目 baseline loop consistency 如何比较。

上一版实验把 Kalibr 默认口径和 Ceres 当前口径直接比较。这个比较不公平：Kalibr 默认固定 camera-chain baseline，Ceres 则把每个 camera 的 `T_cam_imu` 作为自由外参优化。因此上一版“Kalibr 按 loop error 胜出”的结论只能描述默认混合口径，不能作为 solver 精度结论。

本轮补齐两个公平口径：

- **fixed camera-chain**：Kalibr 默认固定 `T_cn_cnm1`；Ceres 使用新增 `--fix-camera-chain-extrinsics`，让 cam0-IMU 仍可优化，但对 `T_camN_cam0` 加 tight relative prior。
- **free camera-chain**：Kalibr 使用 `--recompute-camera-chain-extrinsics`；Ceres 不加 fixed-chain 约束。

评测的 loop 是同一个双目 baseline 的两种表达：

```text
T_c1c2 = T_cam1_cam0
T_c1,i * T_i,c2 = T_cam1_imu * T_imu_cam0
loop_error = inv(T_cam1_cam0) * (T_cam1_imu * T_imu_cam0)
```

## 本轮代码改动

- `calibrate_cam_imu --init-from-camchain` 现在不仅读取每个 camera 的 `T_cam_imu`，也读取 `T_cn_cnm1`。如果后续 camera 没有 `T_cam_imu`，但有相邻 baseline，则递推组合出 `T_camN_imu = T_camN_camN-1 * T_camN-1_imu`。
- 如果 camchain 外参不完整，会用黄色 warning 提示 fallback；当存在 cam0 corner poses 时，会尝试基于共同 AprilGrid 观测做 PnP baseline fallback。
- 新增 `--fix-camera-chain-extrinsics`，通过 tight relative prior 固定 `T_camN_cam0`，但不固定 cam0-IMU 外参。
- 修正 `tools/run_docker_benchmark.py`：TUM Kalibr bag 命令现在会透传 `--kalibr-extra-arg`。此前中间目录 `out/docker_benchmarks/20260621_tum_loop_free_chain_arm64_ceres_native` 没有真正传入 `--recompute-camera-chain-extrinsics`，不作为 free-chain 结论依据。

## 配置

| 项目 | 内容 |
|---|---|
| 数据根目录 | `/Users/wayne/Documents/work/data/TUM` |
| 数据集 1 | `dataset-calib-imu1_512_16.bag` |
| 数据集 2 | `dataset-calib-imu2_512_16`，实验中转成 bag |
| Camchain | `dataset-calib-imu2_512_16/dso/camchain.yaml` |
| Target | `april_6x6_80x80cm.yaml` |
| Kalibr 镜像 | `kalibr-camera-calibration:20.04-arm64` |
| Ceres 模式 | native `build/calibrate_cam_imu` |
| Ceres 代码状态 | `e994e6bbc8ca` 之后的本轮 working tree |
| fixed 输出 | `out/docker_benchmarks/20260621_tum_loop_fixed_chain_arm64_ceres_native` |
| free 输出 | `out/docker_benchmarks/20260621_tum_loop_free_chain_recompute_arm64_ceres_native` |

## 命令

fixed camera-chain：

```bash
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/20260621_tum_loop_fixed_chain_arm64_ceres_native --ceres-mode native --ceres-extra-arg=--fix-camera-chain-extrinsics
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/20260621_tum_loop_fixed_chain_arm64_ceres_native/tum/summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/20260621_tum_loop_fixed_chain_arm64_ceres_native/tum/loop_error_summary.csv
```

free camera-chain：

```bash
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/20260621_tum_loop_free_chain_recompute_arm64_ceres_native --ceres-mode native --kalibr-extra-arg=--recompute-camera-chain-extrinsics
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/20260621_tum_loop_free_chain_recompute_arm64_ceres_native/tum/summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/20260621_tum_loop_free_chain_recompute_arm64_ceres_native/tum/loop_error_summary.csv
```

## Loop Error结果

| 模式 | 数据集 | Solver | Loop rot deg | Loop trans mm | Reproj px mean |
|---|---|---|---:|---:|---:|
| fixed | `tum_imu1_bag` | Kalibr arm64 | 0.000000 | 0.000009 | 0.1002865 |
| fixed | `tum_imu1_bag` | Ceres native | 0.000063 | 0.000423 | 0.1000320 |
| fixed | `tum_imu2_euroc_kalibr_export` | Kalibr arm64 | 0.004387 | 0.000008 | 0.1002534 |
| fixed | `tum_imu2_euroc_kalibr_export` | Ceres native | 0.000022 | 0.000489 | 0.0984953 |
| free | `tum_imu1_bag` | Kalibr arm64 | 0.030215 | 0.211928 | 0.0976476 |
| free | `tum_imu1_bag` | Ceres native | 0.029578 | 0.213330 | 0.0956195 |
| free | `tum_imu2_euroc_kalibr_export` | Kalibr arm64 | 0.034187 | 0.242328 | 0.0969439 |
| free | `tum_imu2_euroc_kalibr_export` | Ceres native | 0.034648 | 0.244333 | 0.0944126 |

## Residual对照

| 模式 | 数据集 | Solver | Reproj px mean | Gyro rad/s mean | Accel m/s^2 mean |
|---|---|---|---:|---:|---:|
| fixed | `tum_imu1_bag` | Kalibr arm64 | 0.1002865 | 0.00119866 | 0.0217058 |
| fixed | `tum_imu1_bag` | Ceres native | 0.1000320 | 0.00116715 | 0.0217672 |
| fixed | `tum_imu2_euroc_kalibr_export` | Kalibr arm64 | 0.1002534 | 0.00117094 | 0.0216047 |
| fixed | `tum_imu2_euroc_kalibr_export` | Ceres native | 0.0984953 | 0.00112916 | 0.0212373 |
| free | `tum_imu1_bag` | Kalibr arm64 | 0.0976476 | 0.00119866 | 0.0217058 |
| free | `tum_imu1_bag` | Ceres native | 0.0956195 | 0.00116317 | 0.0210351 |
| free | `tum_imu2_euroc_kalibr_export` | Kalibr arm64 | 0.0969439 | 0.00117095 | 0.0216054 |
| free | `tum_imu2_euroc_kalibr_export` | Ceres native | 0.0944126 | 0.00112743 | 0.0207738 |

## 分析

fixed-chain 口径下，两边都把 loop error 压到了接近零。Kalibr 因为内部固定 camera chain，平移闭环是 `1e-8 m` 量级；其中一组 rotation 仍有 `0.004 deg`，主要来自结果文本输出/解析精度和矩阵舍入，而不是 baseline 作为自由变量漂移。Ceres 通过 tight relative prior 达到 `4e-7 m` 量级，也就是约 `0.0004-0.0005 mm`，rotation 在 `1e-5-1e-4 deg` 量级。

free-chain 口径下，两边的 loop error 都回到 `0.21-0.24 mm`、`0.03-0.035 deg` 量级。Kalibr 和 Ceres 的 loop translation 差别只有约 `0.001-0.002 mm`，已经不是上一版默认混合口径里看到的数量级差距。Ceres 的 reprojection、gyro、accel residual mean 仍整体低于 Kalibr。

这说明上一版的主要现象来自约束口径差异，而不是 Kalibr 在同等自由度下天然把 loop error 压得更低。公平比较时，应明确说明 camera-chain 是 fixed 还是 free。

## 结论

按公平口径重跑后，结论改为：

- fixed camera-chain：Kalibr 和 Ceres 都能把 loop consistency 压到近零；Kalibr 是硬固定 baseline，Ceres 当前是 tight prior。两者的 fixed 结果都已到亚微米级平移闭环误差。
- free camera-chain：Kalibr recompute 与 Ceres free 的 loop error 基本同量级；Ceres residual 更低，loop translation 只比 Kalibr 大约 `1-2 um`。
- 上一版“Kalibr 按 loop error 胜出”只适用于 Kalibr fixed vs Ceres free 的混合默认口径，不应作为 solver 精度结论。

## 未覆盖问题

- 这次只跑 TUM 两组双目数据，未覆盖更多相机拓扑。
- Ceres 的 fixed-chain 仍是 tight relative prior，不是完全硬约束重参数化。
- PnP fallback 没有用专门构造的缺外参 camchain 做单元/集成测试；当前 TUM 输入直接提供了完整 `T_cam_imu`。

## 下一步

如果要把 Ceres fixed-chain 做成和 Kalibr 默认完全一致的硬约束，应把非 cam0 的相机外参重参数化为 `T_camN_cam0_fixed * T_cam0_imu`，而不是通过 tight prior 近似固定。验收时继续使用本实验的 fixed/free 两套命令和 `loop_error_summary.csv`。
