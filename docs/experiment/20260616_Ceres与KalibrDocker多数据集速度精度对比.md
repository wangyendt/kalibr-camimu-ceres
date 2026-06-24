# Ceres Native 与 Kalibr Docker 多数据集速度精度对比

## 结论先行

本页只记录 2026-06-24 当前可复现结果。Suite A/Suite C 来自完整命令；Suite B 的 single 48 路来自完整 `benchmark-multi-imu` run，joint 12 路来自当前默认 tight bound 的 Ceres-only rerun。旧 ablation、旧 commit、旧派生表和已经删除的诊断草稿不再作为本文证据。

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/single_amd64_arm64 --out-root out/docker_benchmarks/single_arm64_ceres_current
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
```

| Suite | 状态 | Ceres topology | Kalibr 平台 | 当前读数 |
|---|---|---|---|---|
| `benchmark-single` | 12/12 通过 | `1cam+1imu` | arm64 reused | C/K 平移差均/最大 `2.05 / 4.03 mm`，旋转 `0.0056 / 0.0148 deg`，Ceres wall 均值 `110.2 s` |
| `benchmark-multi-imu` | single 48/48 + tight joint 12/12 通过 | joint: `1cam+Nimu`; single: `1cam+1imu` | arm64 | Ceres single-joint 平移均/中/最大 `28.0 / 25.0 / 95.7 mm`; C/K joint effective chain 平移均/最大 `2.9 / 5.2 mm` |
| `tum` | 2/2 通过 | `Mcam+1imu` | arm64 | C/K 平移差均/最大 `0.69 / 1.04 mm`，Ceres loop error 最大 `0.0347 deg / 0.245 mm` |

核心判断：

- `benchmark-single` 当前 Ceres-only arm64 复跑仍是毫米级对齐，最大平移差 `b08 = 4.03 mm`；本轮没有复现单 IMU 厘米级外参偏差，Ceres wall 均值从旧 run 的 `114.4 s` 降到 `110.2 s`。
- `benchmark-multi-imu` 的流程成功率已恢复，Kalibr joint 没有失败；当前 tighter non-reference IMU extrinsic bound 将 C/K joint effective chain 最大平移差从旧口径 `163.0 mm` 压到 `5.2 mm`。
- Suite B 的 rotation 必须按 Ceres/Kalibr rotation-vector 符号复算；按源码口径，Ceres single-joint 最大旋转差是 `1.270 deg`，不是十几度级。
- Suite B 仍未完全收口：`b09` joint 的 accel residual delta 仍为 `+0.651 m/s^2`，`b09/imu1` 的 Ceres single-joint 平移差仍有 `95.7 mm`。
- TUM 双目单 IMU 精度贴近 Kalibr，residual mean 全部略低；速度上 Ceres wall 一快一慢，优化段仍慢于 Kalibr arm64。

## 配置口径

`--suite` 是 `tools/run_docker_benchmark.py` 的实验选择参数，不是 Ceres native 标定器参数。Ceres 标定器实际通过 `--corner-defaults` 按输入数量自动选择 topology。

| 项目 | 本次口径 |
|---|---|
| Ceres | native `build/calibrate_cam_imu` |
| Ceres commit | Suite A / Suite B tight joint Ceres binary 为 `d2f88a6`; runner/doc 当前提交为 `5458f68` |
| Kalibr Docker repo commit | `e83ecfc4d6c0` |
| benchmark 数据根 | `/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data` |
| TUM 数据根 | `/Users/wayne/Documents/work/data/TUM` |
| common defaults | `pose_kps=100`, `bias_kps=50`, `time_padding_s=0.04`, `camera_time_offset_buffer_s=0`, camera/gyro/accel loss 为 `cauchy:10` |
| solver defaults | `max_iterations=150`, Ceres function/gradient/parameter tolerance 均为 `0`，absolute step stop `0.02`，absolute cost/parameter stop disabled，nonmonotonic enabled，max consecutive `20` |
| multi-IMU runner 额外项 | joint case 使用 `--staged --stage-free pbg,pbegti --stage-iterations 30,30`，非参考 IMU extrinsic component-wise bound 为 `translation<=0.003 m`、rotation-vector `<=0.005 rad`；Ceres joint 默认用 Kalibr arm64 结果初始化 |

## 指标口径

| 指标 | 单位 | 含义 | 注意事项 |
|---|---:|---|---|
| success | count | Kalibr、Ceres、compare return code 均为 0 | 只说明流程完成 |
| C/K translation | mm | Ceres 与 Kalibr `T_c_i` 或 `T_c_b` 平移欧氏距离 | 与 Kalibr 比，不是真值误差 |
| C/K rotation | deg | SO(3) 相对旋转角 | Ceres rotation-vector 使用 Kalibr 符号，复算时不能用标准 Rodrigues 正号 |
| C/K time-shift | ms | Ceres time shift 减 Kalibr time shift | 表中保留符号，聚合看绝对值 |
| residual mean | px / rad/s / m/s^2 | reprojection、gyro、accel mean residual | 不同单位不能横向比较 |
| optimize time | s | solver 内部优化耗时 | Ceres 与 Kalibr 内部统计口径不同 |
| wall time | s | runner 端到端耗时 | 包含 Docker、读写、报告生成 |
| Suite B effective chain | `T_c_i` | Ceres: `T_c_b * inv(T_i_b)`；Kalibr: `T_ci0 * inv(T_ib)` | joint 的 camera0 不能代表所有 IMU chain |
| TUM loop error | deg / mm | `T_cam1_cam0` 与 `T_cam1_imu * T_imu_cam0` 的闭环差 | 只适用于多 camera |

## 结果总览

| Suite | Case 数 | success | 平均/最大平移差 | 平均/最大旋转差 | 平均/最大 abs time-shift | Ceres/Kalibr wall mean | Ceres/Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| `benchmark-single` vs Kalibr arm64 | 12 | 12/12 | `2.05 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `110.2 / 122.8 s` | `106.8 / 73.7 s` |
| `benchmark-multi-imu` single cases | 48 | 48/48 | C/K single `5.2 / 41.6 mm` | `0.035 / 0.831 deg` | `1.20 / 6.27 ms` | `123.5 / 119.8 s` | `120.4 / 71.2 s` |
| `benchmark-multi-imu` joint cases | 12 | 12/12 | C/K joint effective `2.9 / 5.2 mm` | `0.107 / 0.404 deg` | `0.06 / 0.54 ms` | `90.4 / 313.3 s` | `86.5 / 220.4 s` |
| `tum` vs Kalibr arm64 | 2 | 2/2 | `0.69 / 1.04 mm` | `0.0316 / 0.0398 deg` | `0.024 / 0.045 ms` | `61.5 / 57.6 s` | `60.0 / 21.5 s` |

## Suite A: benchmark-single

### 范围

12 个生产 benchmark session，每个 session 只使用 `data1.csv`。Ceres 是 `1cam+1imu`；Kalibr arm64 结果复用自早前完整 run，只重跑当前 Ceres native。结果来源：

```text
out/docker_benchmarks/single_arm64_ceres_current/benchmark_single/summary.csv
out/docker_benchmarks/single_arm64_ceres_current/summary.csv
```

### 聚合结果

| Kalibr 平台 | 行数 | success | 平均/最大平移差 | 平均/最大旋转差 | 平均/最大 abs time-shift | Ceres/Kalibr wall mean | Ceres/Kalibr optimize mean | Ceres iter mean/max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| arm64 | 12 | 12/12 | `2.05 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `110.2 / 122.8 s` | `106.8 / 73.7 s` | `102.5 / 142` |

读数：当前 Ceres native 相对 Kalibr arm64 Docker 的 wall mean 更快，但优化段仍更慢。与旧 Ceres run 相比，wall 总和从 `1373.2 s` 降到 `1322.7 s`，iter 总和从 `1363` 降到 `1230`；最大平移仍是 `4.03 mm`。

### 分数据集结果

下表以 Kalibr arm64 为逐数据集基线。`C/K` 表示 Ceres native / Kalibr arm64。

| 数据集 | 平移差 mm | 旋转差 deg | time-shift 差 ms | reproj px C/K | accel m/s^2 C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---:|---:|---:|---|---|---:|---:|---:|
| b01 | 1.69 | 0.0060 | -0.99 | 0.180275 / 0.179774 | 0.114557 / 0.108088 | 136.0 / 52.3 | 139.1 / 102.9 | 130 |
| b02 | 1.36 | 0.0040 | -0.80 | 0.180265 / 0.179743 | 0.112463 / 0.107294 | 140.0 / 77.0 | 142.6 / 126.2 | 134 |
| b03 | 1.17 | 0.0050 | -0.78 | 0.180301 / 0.180087 | 0.119200 / 0.114536 | 149.0 / 40.0 | 152.3 / 89.3 | 142 |
| b04 | 2.32 | 0.0070 | -1.39 | 0.179710 / 0.179124 | 0.116340 / 0.107415 | 123.0 / 78.2 | 126.0 / 127.2 | 116 |
| b05 | 2.53 | 0.0016 | -0.57 | 0.179187 / 0.178664 | 0.114687 / 0.111014 | 119.0 / 106.8 | 122.0 / 156.2 | 116 |
| b06 | 1.88 | 0.0034 | -1.12 | 0.177188 / 0.176607 | 0.123734 / 0.116744 | 126.0 / 52.6 | 129.3 / 101.7 | 120 |
| b07 | 0.97 | 0.0046 | -3.20 | 0.170813 / 0.171469 | 0.095860 / 0.080972 | 94.9 / 83.7 | 98.7 / 132.8 | 91 |
| b08 | 4.03 | 0.0148 | -6.27 | 0.172360 / 0.171172 | 0.114861 / 0.085099 | 68.2 / 96.4 | 71.9 / 144.6 | 67 |
| b09 | 1.13 | 0.0000 | -0.70 | 0.170436 / 0.170591 | 0.096126 / 0.093327 | 88.2 / 71.2 | 91.9 / 120.0 | 87 |
| b10 | 0.99 | 0.0000 | +1.34 | 0.171894 / 0.170795 | 0.084100 / 0.089701 | 88.4 / 59.6 | 92.1 / 108.4 | 85 |
| b11 | 3.15 | 0.0090 | +1.42 | 0.172832 / 0.171706 | 0.086650 / 0.092505 | 70.8 / 60.0 | 74.6 / 109.1 | 67 |
| b12 | 3.35 | 0.0123 | -6.19 | 0.172194 / 0.171249 | 0.141210 / 0.116552 | 78.4 / 107.0 | 82.2 / 155.5 | 75 |

读数：最大平移差仍是 `b08`，为 `4.03 mm`；最大 time-shift 差出现在 `b08/b12`，约 `6.2 ms`。`b12` accel residual 仍高于 Kalibr，是单 IMU 后续观察点。

## Suite B: benchmark-multi-imu

### 范围与读法

每个 benchmark 跑 1 个 `joint_4imu` 和 4 个独立单 IMU case：`single_imu1..4`。本节比较四件事：

| 对比 | 含义 | 用途 |
|---|---|---|
| Ceres single-joint | Ceres 独立单 IMU vs Ceres 4IMU joint effective chain | 看 Ceres joint 是否保持独立标定一致性 |
| Kalibr single-joint | Kalibr 独立单 IMU vs Kalibr 4IMU joint effective chain | 看数据/约束自身的 single-joint 差异 |
| C/K single | Ceres single vs Kalibr single | 看 Ceres 单 IMU 是否贴近 Kalibr |
| C/K joint | Ceres joint effective chain vs Kalibr joint effective chain | 看 joint 整条 IMU chain 是否贴近 Kalibr |

结果来源：

```text
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/summary.csv
out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/summary.csv
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/*/*/ceres/result.yaml
out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/*/joint_4imu/ceres/result.yaml
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/*/*/kalibr_arm64/input/*-results-imucam.txt
out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/*/joint_4imu/compare_arm64/compare.clean.log
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_iteration_trace.csv
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_stop_policy_replay.csv
```

single 48 路结果来自完整 `multi_imu_arm64` run；joint 12 路结果使用 2026-06-24 的 Ceres-only tight rerun，并通过 `--reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64` 复用 Kalibr arm64 结果。该 tight rerun 对应当前 runner 默认 multi-IMU bound：`translation<=0.003 m`、rotation-vector `<=0.005 rad`。

### 48 路聚合

每个单元格格式为：translation `mean / median / max`，rotation `mean / median / max`，abs time-shift `mean / median / max`。

| 分组 | Ceres single-joint | Kalibr single-joint | C/K single | C/K joint effective |
|---|---|---|---|---|
| all 48 | `28.0 / 25.0 / 95.7 mm`, `0.218 / 0.137 / 1.270 deg`, `2.09 / 1.74 / 8.24 ms` | `25.5 / 18.4 / 96.0 mm`, `0.147 / 0.096 / 0.887 deg`, `1.27 / 1.10 / 2.87 ms` | `5.2 / 2.6 / 41.6 mm`, `0.035 / 0.007 / 0.831 deg`, `1.20 / 0.87 / 6.27 ms` | `2.9 / 3.7 / 5.2 mm`, `0.107 / 0.040 / 0.404 deg`, `0.06 / 0.02 / 0.54 ms` |
| b01-b06 | `11.5 / 9.6 / 28.3 mm`, `0.099 / 0.077 / 0.231 deg`, `1.82 / 1.75 / 3.04 ms` | `7.1 / 7.0 / 12.8 mm`, `0.082 / 0.066 / 0.227 deg`, `0.80 / 0.71 / 1.57 ms` | `4.6 / 2.4 / 21.7 mm`, `0.008 / 0.005 / 0.030 deg`, `1.04 / 0.90 / 2.32 ms` | `3.0 / 3.7 / 4.5 mm`, `0.040 / 0.033 / 0.191 deg`, `0.02 / 0.02 / 0.03 ms` |
| b07-b12 | `44.4 / 41.3 / 95.7 mm`, `0.338 / 0.264 / 1.270 deg`, `2.36 / 1.68 / 8.24 ms` | `44.0 / 40.0 / 96.0 mm`, `0.212 / 0.143 / 0.887 deg`, `1.73 / 1.74 / 2.87 ms` | `5.9 / 3.1 / 41.6 mm`, `0.063 / 0.023 / 0.831 deg`, `1.35 / 0.78 / 6.27 ms` | `2.9 / 3.6 / 5.2 mm`, `0.175 / 0.204 / 0.404 deg`, `0.11 / 0.03 / 0.54 ms` |

读数：

- Ceres single-joint 平移 tail 已随 tight joint bound 明显下降，但仍大于 Kalibr single-joint，最大来自 `b09/imu1 = 95.7 mm`。
- C/K single 的主 outlier 是 `b09/imu3 = 41.6 mm / 0.831 deg`，说明至少部分异常已经出现在独立单 IMU 标定。
- C/K joint effective 的平移 tail 已压到 `5.2 mm`，不再是当前 Suite B 的主风险；但 `b09` joint accel residual 仍是最大异常。
- `b01-b06` 明显更稳定；`b07-b12` 的 tail 由 `b09/b11/b12` 拉大，后续定位应按异常 session 而不是简单前后六组二分。

### Top outliers

#### Ceres single-joint 按平移排序

| 排名 | case | IMU | Δt mm | ΔR deg | Δτ ms |
|---:|---|---|---:|---:|---:|
| 1 | b09 | imu1 | 95.7 | 0.860 | -3.21 |
| 2 | b12 | imu4 | 66.5 | 0.291 | -0.22 |
| 3 | b11 | imu1 | 63.7 | 0.197 | +0.36 |
| 4 | b12 | imu1 | 61.0 | 0.081 | -7.82 |
| 5 | b12 | imu3 | 54.3 | 0.075 | -1.38 |
| 6 | b11 | imu4 | 50.5 | 0.553 | +2.82 |
| 7 | b10 | imu4 | 49.0 | 0.530 | -1.07 |
| 8 | b07 | imu4 | 47.5 | 0.509 | -0.87 |
| 9 | b11 | imu3 | 44.8 | 0.311 | -1.26 |
| 10 | b07 | imu3 | 44.8 | 0.140 | -1.55 |

#### C/K single 按平移排序

| 排名 | case | IMU | Δt mm | ΔR deg | Δτ ms |
|---:|---|---|---:|---:|---:|
| 1 | b09 | imu3 | 41.6 | 0.831 | -2.17 |
| 2 | b09 | imu4 | 25.5 | 0.069 | -0.93 |
| 3 | b04 | imu3 | 21.7 | 0.001 | -0.79 |
| 4 | b06 | imu3 | 21.4 | 0.030 | -0.70 |
| 5 | b03 | imu3 | 19.9 | 0.004 | -0.93 |
| 6 | b10 | imu4 | 10.6 | 0.100 | -0.33 |
| 7 | b11 | imu4 | 8.3 | 0.018 | +3.54 |
| 8 | b11 | imu3 | 7.9 | 0.081 | -0.36 |
| 9 | b12 | imu3 | 7.8 | 0.062 | +0.15 |
| 10 | b10 | imu3 | 7.4 | 0.058 | -0.32 |

#### C/K joint effective 按平移排序

| 排名 | case | IMU | Δt mm | ΔR deg | Δτ ms |
|---:|---|---|---:|---:|---:|
| 1 | b10 | imu4 | 5.21 | 0.288 | -0.021 |
| 2 | b11 | imu2 | 5.21 | 0.232 | -0.028 |
| 3 | b11 | imu4 | 5.21 | 0.404 | -0.028 |
| 4 | b09 | imu2 | 5.20 | 0.317 | -0.054 |
| 5 | b09 | imu3 | 5.20 | 0.355 | -0.054 |
| 6 | b10 | imu2 | 4.76 | 0.215 | -0.021 |
| 7 | b07 | imu3 | 4.53 | 0.070 | -0.539 |
| 8 | b04 | imu3 | 4.51 | 0.075 | -0.009 |
| 9 | b06 | imu4 | 4.50 | 0.042 | -0.027 |
| 10 | b09 | imu4 | 4.44 | 0.372 | -0.054 |

### 速度

| 分组 | case 数 | Ceres wall mean/total | Kalibr wall mean/total | Ceres optimize mean | Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|
| single cases | 48 | `123.5 / 5928.3 s` | `119.8 / 5748.4 s` | `120.4 s` | `71.2 s` |
| joint cases | 12 | `90.4 / 1084.9 s` | `313.3 / 3759.8 s` | `86.5 s` | `220.4 s` |

#### Joint case 速度明细

Ceres joint 是两阶段 staged solve，表中的 Ceres optimize 为两个 `stage timing` 的 `minimizer_time_s` 之和；runner summary 原始 `ceres_optimize_s` 只反映其中一个 stage，不能直接作为 joint 总优化耗时。

| benchmark | C/K joint effective Δt mm | ΔR deg | abs Δτ ms | Δreproj px | Δgyro rad/s | Δaccel m/s^2 | Ceres/Kalibr wall s | Ceres/Kalibr optimize s | Ceres stage it |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| b01 | 3.95 | 0.047 | 0.012 | +0.110 | +0.0108 | +0.1314 | 25.5 / 279.0 | 21.8 / 193.3 | 3+2 |
| b02 | 4.10 | 0.033 | 0.009 | +0.109 | +0.0104 | +0.1298 | 24.8 / 193.5 | 21.1 / 104.5 | 3+2 |
| b03 | 4.42 | 0.107 | 0.020 | +0.105 | +0.0099 | +0.1341 | 25.1 / 194.7 | 21.3 / 105.9 | 3+2 |
| b04 | 4.51 | 0.075 | 0.009 | +0.100 | +0.0097 | +0.1420 | 25.3 / 157.0 | 21.5 / 69.5 | 3+2 |
| b05 | 4.30 | 0.191 | 0.019 | +0.108 | +0.0096 | +0.1486 | 25.1 / 156.6 | 21.3 / 68.7 | 3+2 |
| b06 | 4.50 | 0.082 | 0.027 | +0.121 | +0.0098 | +0.1368 | 25.4 / 211.4 | 21.5 / 123.7 | 3+2 |
| b07 | 4.53 | 0.341 | 0.539 | -0.005 | +0.0004 | +0.0240 | 307.4 / 510.2 | 303.5 / 409.8 | 16+31 |
| b08 | 3.59 | 0.371 | 0.037 | -0.005 | +0.0006 | +0.0366 | 127.8 / 577.5 | 123.9 / 480.5 | 17+5 |
| b09 | 5.20 | 0.372 | 0.054 | -0.031 | +0.0089 | +0.6509 | 206.5 / 381.0 | 202.4 / 279.4 | 31+5 |
| b10 | 5.21 | 0.288 | 0.021 | +0.012 | +0.0023 | +0.1147 | 54.8 / 539.7 | 50.7 / 438.8 | 4+5 |
| b11 | 5.21 | 0.404 | 0.028 | +0.002 | +0.0022 | +0.1182 | 47.0 / 323.0 | 43.2 / 230.1 | 4+5 |
| b12 | 1.54 | 0.011 | 0.001 | -0.000 | +0.0005 | -0.0024 | 190.2 / 236.3 | 186.3 / 140.6 | 31+2 |

读数：joint staged 路径在 wall/optimize 上仍明显快于 Kalibr arm64 joint，且 tight bound 后 C/K joint effective chain 已进入 `<=5.3 mm`。精度风险从外参链 tail 转移到 residual tail：`b09` 的 accel delta 仍为 `+0.6509 m/s^2`，`b01-b06` 的 reproj/gyro/accel residual 也稳定高于 Kalibr。

## Suite C: TUM

### 范围

TUM suite 跑两个双目单 IMU case，Ceres topology 为 `Mcam+1imu`，Kalibr 只跑 arm64。loop error 使用仓库脚本从本次 summary 派生：

```bash
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/tum_arm64/tum/tum_summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv
```

结果来源：

```text
out/docker_benchmarks/tum_arm64/tum/tum_summary.csv
out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv
```

### 分 case 结果

| Case | 输入来源 | 平移差 mm | 旋转差 deg | time-shift 差 ms | loop error Ceres deg/mm | loop error Kalibr deg/mm | reproj px C/K | gyro rad/s C/K | accel m/s^2 C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---|---:|---:|---:|---:|---:|---|---|---|---:|---:|---:|
| `tum_imu1_bag` | bag | 1.04 | 0.0398 | -0.045 | 0.0295 / 0.213 | 0.0000 / 0.000 | 0.094929 / 0.100287 | 0.001163 / 0.001199 | 0.021031 / 0.021706 | 87.6 / 21.6 | 88.8 / 58.9 | 151 |
| `tum_imu2_euroc_kalibr_export` | euroc | 0.34 | 0.0233 | +0.004 | 0.0347 / 0.245 | 0.0044 / 0.000 | 0.093744 / 0.100253 | 0.001127 / 0.001171 | 0.020772 / 0.021605 | 32.5 / 21.3 | 34.2 / 56.3 | 55 |

读数：两个 TUM case 的 C/K 外参平移差都小于 `1.04 mm`，time-shift 差小于 `0.045 ms`。Ceres residual mean 均低于 Kalibr arm64。Ceres loop error 为 `0.03 deg / 0.2 mm` 量级；Kalibr loop error 接近 0，主要因为直接 baseline 和 Kalibr 外参来自同一 camchain/优化输出链路。

### 速度

| Case | Ceres wall | Kalibr arm64 wall | Ceres optimize | Kalibr arm64 optimize | 结论 |
|---|---:|---:|---:|---:|---|
| `tum_imu1_bag` | `88.8 s` | `58.9 s` | `87.6 s` | `21.6 s` | Ceres wall 更慢 |
| `tum_imu2_euroc_kalibr_export` | `34.2 s` | `56.3 s` | `32.5 s` | `21.3 s` | Ceres wall 更快，优化段仍更慢 |


## 跨 suite 分析

- `1cam+1imu` 当前外参精度稳定，主要剩余观察点是 `b08/b12` 的 time-shift 和 accel residual 差异。
- `1cam+Nimu` 的流程与 Kalibr delay 口径已恢复；tight bound 后当前风险不在 C/K joint effective chain 平移，而在 Ceres single-joint 一致性和 residual tail。
- Suite B 的 C/K single 与 C/K joint 是两个不同问题：single 最大 `41.56 mm`，tight joint effective 最大 `5.21 mm`。当前 C/K joint 外参链已经明显好于旧口径，但 `b09` 的 accel residual 仍说明 pose spline / stage0 basin 需要继续定位。
- `Mcam+1imu` 的 TUM 精度已贴近 Kalibr，loop error 应继续作为多相机稳定性指标保留。
- 速度结论必须带平台：当前 Suite A / TUM 相对 Kalibr arm64 Docker 的优化段仍慢；Suite A wall mean 略快，TUM wall 一快一慢；multi-IMU joint staged wall 明显快。

## 边界

- 外参差异是 Ceres 与 Kalibr 的差，不是真值误差。
- Ceres 与 Kalibr 的 optimize time 不是完全相同的内部统计口径；wall time 包含不同外围开销。
- Suite B 的 effective chain 统计是本文从本次 `result.yaml` 和 Kalibr result txt 派生，不是 runner summary 的原始列。
- Suite B joint 的 Ceres optimize time 使用 staged log 中两个 stage 的 `minimizer_time_s` 求和；runner summary 原始 `ceres_optimize_s` 不用于 joint 总优化耗时结论。
- 本文只用 tight bound 做了 current default joint rerun；没有重新做 stop policy、nonmonotonic、IMU 裁边、time-shift prior 或 fixed IMU extrinsics full ablation。
- TUM 只覆盖 Kalibr arm64，没有 amd64 平台对照；`Mcam+Nimu` 仍未覆盖。

## 复现入口

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/single_amd64_arm64 --out-root out/docker_benchmarks/single_arm64_ceres_current
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/tum_arm64/tum/tum_summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv
```

| 文件 | 用途 |
|---|---|
| `out/docker_benchmarks/single_arm64_ceres_current/summary.csv` | Suite A 当前 Ceres-only arm64 总汇总 |
| `out/docker_benchmarks/single_arm64_ceres_current/benchmark_single/summary.csv` | Suite A 当前 Ceres-only arm64 子目录汇总 |
| `out/docker_benchmarks/single_arm64_ceres_current/ceres_iteration_trace.csv` | Suite A 当前 Ceres 迭代轨迹 |
| `out/docker_benchmarks/single_arm64_ceres_current/ceres_stop_policy_replay.csv` | Suite A 当前 stop policy replay |
| `out/docker_benchmarks/multi_imu_arm64/summary.csv` | Suite B 总汇总 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/summary.csv` | Suite B 子目录汇总 |
| `out/docker_benchmarks/multi_imu_arm64/ceres_iteration_trace.csv` | Suite B Ceres 迭代轨迹 |
| `out/docker_benchmarks/multi_imu_arm64/ceres_stop_policy_replay.csv` | Suite B stop policy replay |
| `out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/summary.csv` | Suite B current-default tight joint Ceres-only rerun |
| `out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/ceres_iteration_trace.csv` | Suite B tight joint Ceres 迭代轨迹 |
| `out/docker_benchmarks/tum_arm64/summary.csv` | Suite C 总汇总 |
| `out/docker_benchmarks/tum_arm64/tum/tum_summary.csv` | Suite C 子目录汇总 |
| `out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv` | Suite C loop error 派生结果 |

## 下一步

1. 盯 `b09` 的 accel residual tail：优先检查 stage0 pose spline basin、pose acceleration spike、IMU residual 时间段和角点轨迹是否一致。
2. 对 Suite B 做小规模 ablation：固定/释放 `T_i_b`、非参考 IMU lever 先验/bound、分阶段小范围释放、IMU time offset 固定/释放；重点看 residual，而不是只看外参链。
3. 对 TUM 做 `fix-camera-chain-extrinsics` 或 camera-chain prior 对照，确认 loop error 与速度/精度的关系。
4. 若要制定默认 stop policy，优先基于本次 `ceres_iteration_trace.csv` 和 `ceres_stop_policy_replay.csv`，不要只看最终迭代数。
