# Ceres Native 与 Kalibr Docker 多数据集速度精度对比

## 结论先行

本页只记录 2026-06-24 这次三条命令的当前结果。旧 ablation、旧 commit、旧派生表和已经删除的诊断草稿不再作为本文证据。

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --out-root out/docker_benchmarks/single_amd64_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
```

| Suite | 状态 | Ceres topology | Kalibr 平台 | 当前读数 |
|---|---|---|---|---|
| `benchmark-single` | 12/12 通过 | `1cam+1imu` | amd64 + arm64 | C/K 平移差均/最大 `1.92 / 4.03 mm`，旋转 `0.0056 / 0.0148 deg`，Ceres wall 均值 `114.4 s` |
| `benchmark-multi-imu` | 60/60 通过 | joint: `1cam+Nimu`; single: `1cam+1imu` | arm64 | Ceres single-joint 平移均/中/最大 `39.5 / 36.5 / 161.8 mm`; C/K joint effective chain 平移均/最大 `19.1 / 163.0 mm` |
| `tum` | 2/2 通过 | `Mcam+1imu` | arm64 | C/K 平移差均/最大 `0.69 / 1.04 mm`，Ceres loop error 最大 `0.0347 deg / 0.245 mm` |

核心判断：

- `benchmark-single` 仍是毫米级对齐，最大平移差 `b08 = 4.03 mm`；本轮没有复现单 IMU 厘米级外参偏差。
- `benchmark-multi-imu` 的流程成功率已恢复，Kalibr joint 没有失败；但 Ceres joint effective chain 仍有明显 tail，集中在 `b09` 非参考 IMU。
- Suite B 的 rotation 必须按 Ceres/Kalibr rotation-vector 符号复算；按源码口径，Ceres single-joint 最大旋转差是 `1.648 deg`，不是十几度级。
- TUM 双目单 IMU 精度贴近 Kalibr，residual mean 全部略低；速度上 Ceres wall 一快一慢，优化段仍慢于 Kalibr arm64。

## 配置口径

`--suite` 是 `tools/run_docker_benchmark.py` 的实验选择参数，不是 Ceres native 标定器参数。Ceres 标定器实际通过 `--corner-defaults` 按输入数量自动选择 topology。

| 项目 | 本次口径 |
|---|---|
| Ceres | native `build/calibrate_cam_imu` |
| Ceres commit | `3f644ba614ce` |
| Kalibr Docker repo commit | `e83ecfc4d6c0` |
| benchmark 数据根 | `/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data` |
| TUM 数据根 | `/Users/wayne/Documents/work/data/TUM` |
| common defaults | `pose_kps=100`, `bias_kps=50`, `time_padding_s=0.04`, `camera_time_offset_buffer_s=0`, camera/gyro/accel loss 为 `cauchy:10` |
| solver defaults | `max_iterations=150`, Ceres function/gradient/parameter tolerance 均为 `0`，absolute step stop `0.02`，absolute cost/parameter stop disabled，nonmonotonic enabled，max consecutive `20` |
| multi-IMU runner 额外项 | joint case 使用 `--staged --stage-free pbg,pbegt --stage-iterations 30,30`；Ceres joint 默认用 Kalibr arm64 结果初始化 |

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
| `benchmark-single` vs Kalibr amd64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.4 / 192.2 s` | `111.3 / 119.4 s` |
| `benchmark-single` vs Kalibr arm64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.4 / 122.8 s` | `111.3 / 73.7 s` |
| `benchmark-multi-imu` single cases | 48 | 48/48 | C/K single `5.2 / 41.6 mm` | `0.035 / 0.831 deg` | `1.20 / 6.27 ms` | `123.5 / 119.8 s` | `120.4 / 71.2 s` |
| `benchmark-multi-imu` joint cases | 12 | 12/12 | C/K joint effective `19.1 / 163.0 mm` | `0.267 / 2.573 deg` | `0.24 / 2.65 ms` | `83.5 / 313.3 s` | `79.5 / 220.4 s` |
| `tum` vs Kalibr arm64 | 2 | 2/2 | `0.69 / 1.04 mm` | `0.0316 / 0.0398 deg` | `0.024 / 0.045 ms` | `61.5 / 57.6 s` | `60.0 / 21.5 s` |

## Suite A: benchmark-single

### 范围

12 个生产 benchmark session，每个 session 只使用 `data1.csv`。Ceres 是 `1cam+1imu`；Kalibr 默认同时跑 amd64 和 arm64。结果来源：

```text
out/docker_benchmarks/single_amd64_arm64/benchmark_single/summary.csv
out/docker_benchmarks/single_amd64_arm64/summary.csv
```

### 聚合结果

| Kalibr 平台 | 行数 | success | 平均/最大平移差 | 平均/最大旋转差 | 平均/最大 abs time-shift | Ceres/Kalibr wall mean | Ceres/Kalibr optimize mean | Ceres iter mean/max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| amd64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.4 / 192.2 s` | `111.3 / 119.4 s` | `113.6 / 151` |
| arm64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.4 / 122.8 s` | `111.3 / 73.7 s` | `113.6 / 151` |

读数：amd64 和 arm64 的 Kalibr 外参结果在本文精度下相同；差别主要体现在 Docker wall/optimize time。Ceres native 相对 Kalibr amd64 Docker 更快；相对 arm64 Docker，wall 略快但优化段更慢。

### 分数据集结果

下表以 Kalibr amd64 为逐数据集基线。`C/K` 表示 Ceres native / Kalibr amd64。

| 数据集 | 平移差 mm | 旋转差 deg | time-shift 差 ms | reproj px C/K | accel m/s^2 C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---:|---:|---:|---|---|---:|---:|---:|
| b01 | 1.69 | 0.0060 | -0.99 | 0.180275 / 0.179774 | 0.114557 / 0.108088 | 129.0 / 84.3 | 131.4 / 161.3 | 130 |
| b02 | 1.08 | 0.0039 | -0.80 | 0.180266 / 0.179743 | 0.112463 / 0.107294 | 149.0 / 126.3 | 151.7 / 198.6 | 151 |
| b03 | 1.02 | 0.0050 | -0.78 | 0.180301 / 0.180087 | 0.119199 / 0.114536 | 149.0 / 63.0 | 151.6 / 136.0 | 151 |
| b04 | 2.32 | 0.0070 | -1.39 | 0.179710 / 0.179124 | 0.116340 / 0.107415 | 114.0 / 127.0 | 116.7 / 199.8 | 116 |
| b05 | 2.53 | 0.0016 | -0.57 | 0.179187 / 0.178664 | 0.114687 / 0.111014 | 114.0 / 170.4 | 116.5 / 242.9 | 116 |
| b06 | 1.88 | 0.0034 | -1.12 | 0.177188 / 0.176607 | 0.123734 / 0.116744 | 118.0 / 83.9 | 120.7 / 156.9 | 120 |
| b07 | 0.97 | 0.0046 | -3.20 | 0.170813 / 0.171469 | 0.095860 / 0.080972 | 88.4 / 136.3 | 92.1 / 209.0 | 91 |
| b08 | 4.03 | 0.0148 | -6.27 | 0.172360 / 0.171172 | 0.114861 / 0.085099 | 65.3 / 155.6 | 69.0 / 227.7 | 67 |
| b09 | 0.56 | 0.0000 | -0.70 | 0.170437 / 0.170591 | 0.096120 / 0.093327 | 124.0 / 115.6 | 128.0 / 188.5 | 128 |
| b10 | 0.43 | 0.0000 | +1.34 | 0.171897 / 0.170795 | 0.084087 / 0.089701 | 146.0 / 98.4 | 149.4 / 169.8 | 151 |
| b11 | 3.15 | 0.0090 | +1.42 | 0.172832 / 0.171706 | 0.086650 / 0.092505 | 65.8 / 97.3 | 69.6 / 169.4 | 67 |
| b12 | 3.35 | 0.0123 | -6.19 | 0.172194 / 0.171249 | 0.141210 / 0.116552 | 72.6 / 174.3 | 76.3 / 245.9 | 75 |

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
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/*/*/ceres/result.yaml
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/*/*/kalibr_arm64/input/*-results-imucam.txt
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_iteration_trace.csv
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_stop_policy_replay.csv
```

### 48 路聚合

每个单元格格式为：translation `mean / median / max`，rotation `mean / median / max`，abs time-shift `mean / median / max`。

| 分组 | Ceres single-joint | Kalibr single-joint | C/K single | C/K joint effective |
|---|---|---|---|---|
| all 48 | `39.5 / 36.5 / 161.8 mm`, `0.284 / 0.139 / 1.648 deg`, `1.92 / 1.63 / 8.25 ms` | `25.5 / 18.4 / 96.0 mm`, `0.147 / 0.096 / 0.887 deg`, `1.27 / 1.10 / 2.87 ms` | `5.2 / 2.6 / 41.6 mm`, `0.035 / 0.007 / 0.831 deg`, `1.20 / 0.87 / 6.27 ms` | `19.1 / 12.3 / 163.0 mm`, `0.267 / 0.069 / 2.573 deg`, `0.24 / 0.02 / 2.65 ms` |
| b01-b06 | `21.1 / 21.7 / 43.0 mm`, `0.125 / 0.096 / 0.339 deg`, `1.82 / 1.75 / 3.04 ms` | `7.1 / 7.0 / 12.8 mm`, `0.082 / 0.066 / 0.227 deg`, `0.80 / 0.71 / 1.57 ms` | `4.6 / 2.4 / 21.7 mm`, `0.008 / 0.005 / 0.030 deg`, `1.04 / 0.90 / 2.32 ms` | `12.9 / 16.7 / 23.6 mm`, `0.077 / 0.067 / 0.385 deg`, `0.02 / 0.02 / 0.03 ms` |
| b07-b12 | `58.0 / 46.1 / 161.8 mm`, `0.443 / 0.285 / 1.648 deg`, `2.03 / 1.48 / 8.25 ms` | `44.0 / 40.0 / 96.0 mm`, `0.212 / 0.143 / 0.887 deg`, `1.73 / 1.74 / 2.87 ms` | `5.9 / 3.1 / 41.6 mm`, `0.063 / 0.023 / 0.831 deg`, `1.35 / 0.78 / 6.27 ms` | `25.2 / 4.8 / 163.0 mm`, `0.456 / 0.076 / 2.573 deg`, `0.46 / 0.03 / 2.65 ms` |

读数：

- Ceres single-joint 平移 tail 仍明显大于 Kalibr single-joint，最大来自 `b09/imu4 = 161.8 mm`。
- C/K single 的主 outlier 是 `b09/imu3 = 41.6 mm / 0.831 deg`，说明至少部分异常已经出现在独立单 IMU 标定。
- C/K joint effective 的最大平移和最大旋转都集中在 `b09` 非参考 IMU chain；camera0 接近 Kalibr 不能代表整条 joint chain 接近。
- `b01-b06` 明显更稳定；`b07-b12` 的 tail 由 `b09/b11/b12` 拉大，后续定位应按异常 session 而不是简单前后六组二分。

### Top outliers

#### Ceres single-joint 按平移排序

| 排名 | case | IMU | Δt mm | ΔR deg | Δτ ms |
|---:|---|---|---:|---:|---:|
| 1 | b09 | imu4 | 161.8 | 1.648 | -1.14 |
| 2 | b09 | imu3 | 132.7 | 1.431 | -1.61 |
| 3 | b09 | imu1 | 95.1 | 0.339 | -0.61 |
| 4 | b09 | imu2 | 93.7 | 0.806 | -0.44 |
| 5 | b11 | imu4 | 73.0 | 1.302 | +2.82 |
| 6 | b11 | imu1 | 63.7 | 0.197 | +0.35 |
| 7 | b12 | imu1 | 61.0 | 0.081 | -7.82 |
| 8 | b11 | imu3 | 56.1 | 0.391 | -1.27 |
| 9 | b07 | imu4 | 49.1 | 0.523 | -1.39 |
| 10 | b12 | imu3 | 49.0 | 0.066 | -1.38 |

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
| 1 | b09 | imu4 | 163.0 | 1.696 | -2.65 |
| 2 | b09 | imu3 | 149.7 | 2.573 | -2.65 |
| 3 | b09 | imu2 | 77.9 | 1.353 | -2.65 |
| 4 | b11 | imu4 | 56.3 | 1.167 | -0.02 |
| 5 | b11 | imu2 | 36.2 | 0.299 | -0.02 |
| 6 | b11 | imu3 | 29.7 | 0.360 | -0.02 |
| 7 | b03 | imu2 | 23.6 | 0.139 | -0.02 |
| 8 | b12 | imu4 | 23.0 | 0.043 | +0.00 |
| 9 | b03 | imu4 | 22.8 | 0.097 | -0.02 |
| 10 | b03 | imu3 | 21.6 | 0.247 | -0.02 |

### 速度

| 分组 | case 数 | Ceres wall mean/total | Kalibr wall mean/total | Ceres optimize mean | Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|
| single cases | 48 | `123.5 / 5928.3 s` | `119.8 / 5748.4 s` | `120.4 s` | `71.2 s` |
| joint cases | 12 | `83.5 / 1002.1 s` | `313.3 / 3759.8 s` | `79.5 s` | `220.4 s` |

#### Joint case 速度明细

Ceres joint 是两阶段 staged solve，表中的 Ceres optimize 为两个 `stage timing` 的 `minimizer_time_s` 之和；runner summary 原始 `ceres_optimize_s` 只反映其中一个 stage，不能直接作为 joint 总优化耗时。

| benchmark | Ceres joint wall s | Kalibr joint wall s | Ceres joint optimize s | Kalibr joint optimize s | Ceres stage it |
|---|---:|---:|---:|---:|---:|
| b01 | 23.4 | 279.0 | 19.3 | 193.3 | 3+2 |
| b02 | 22.6 | 193.5 | 18.7 | 104.5 | 3+2 |
| b03 | 26.6 | 194.7 | 22.8 | 105.9 | 4+2 |
| b04 | 23.2 | 157.0 | 19.2 | 69.5 | 3+2 |
| b05 | 23.0 | 156.6 | 19.0 | 68.7 | 3+2 |
| b06 | 23.9 | 211.4 | 19.8 | 123.7 | 3+2 |
| b07 | 107.7 | 510.2 | 103.5 | 409.8 | 16+5 |
| b08 | 101.4 | 577.5 | 97.3 | 480.5 | 16+5 |
| b09 | 262.5 | 381.0 | 258.5 | 279.4 | 21+31 |
| b10 | 184.4 | 539.7 | 180.4 | 438.8 | 5+31 |
| b11 | 43.7 | 323.0 | 39.8 | 230.1 | 5+5 |
| b12 | 159.9 | 236.3 | 155.9 | 140.6 | 31+2 |

读数：joint staged 路径在 wall/optimize 上明显快于 Kalibr arm64 joint；但精度结论不能只看速度，必须同时看 effective chain tail。

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
- `1cam+Nimu` 的流程与 Kalibr delay 口径已恢复；当前风险不在 run failure，而在 Ceres joint effective chain 的非参考 IMU tail。
- Suite B 的 C/K single 与 C/K joint 是两个不同问题：single 最大 `41.56 mm`，joint effective 最大 `163.03 mm`。joint 的异常更强，说明多 IMU chain refinement 或 staged 释放策略仍有优化空间。
- `Mcam+1imu` 的 TUM 精度已贴近 Kalibr，loop error 应继续作为多相机稳定性指标保留。
- 速度结论必须带平台：Ceres native 相对 Kalibr amd64 Docker 快；相对 Kalibr arm64 Docker，单 IMU/TUM 优化段仍慢；multi-IMU joint staged wall 明显快。

## 边界

- 外参差异是 Ceres 与 Kalibr 的差，不是真值误差。
- Ceres 与 Kalibr 的 optimize time 不是完全相同的内部统计口径；wall time 包含不同外围开销。
- Suite B 的 effective chain 统计是本文从本次 `result.yaml` 和 Kalibr result txt 派生，不是 runner summary 的原始列。
- Suite B joint 的 Ceres optimize time 使用 staged log 中两个 stage 的 `minimizer_time_s` 求和；runner summary 原始 `ceres_optimize_s` 不用于 joint 总优化耗时结论。
- 本文没有重新做 stop policy、nonmonotonic、IMU 裁边、time-shift prior 或 fixed IMU extrinsics ablation。
- TUM 只覆盖 Kalibr arm64，没有 amd64 平台对照；`Mcam+Nimu` 仍未覆盖。

## 复现入口

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --out-root out/docker_benchmarks/single_amd64_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
python3 tools/evaluate_tum_loop_error.py --summary out/docker_benchmarks/tum_arm64/tum/tum_summary.csv --camchain /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --out out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv
```

| 文件 | 用途 |
|---|---|
| `out/docker_benchmarks/single_amd64_arm64/summary.csv` | Suite A 总汇总 |
| `out/docker_benchmarks/single_amd64_arm64/benchmark_single/summary.csv` | Suite A 子目录汇总 |
| `out/docker_benchmarks/single_amd64_arm64/ceres_iteration_trace.csv` | Suite A Ceres 迭代轨迹 |
| `out/docker_benchmarks/single_amd64_arm64/ceres_stop_policy_replay.csv` | Suite A stop policy replay |
| `out/docker_benchmarks/multi_imu_arm64/summary.csv` | Suite B 总汇总 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/summary.csv` | Suite B 子目录汇总 |
| `out/docker_benchmarks/multi_imu_arm64/ceres_iteration_trace.csv` | Suite B Ceres 迭代轨迹 |
| `out/docker_benchmarks/multi_imu_arm64/ceres_stop_policy_replay.csv` | Suite B stop policy replay |
| `out/docker_benchmarks/tum_arm64/summary.csv` | Suite C 总汇总 |
| `out/docker_benchmarks/tum_arm64/tum/tum_summary.csv` | Suite C 子目录汇总 |
| `out/docker_benchmarks/tum_arm64/tum/tum_loop_error.csv` | Suite C loop error 派生结果 |

## 下一步

1. 盯 `b09/imu2..4` 和 `b11/imu4` 的 Ceres joint effective chain：先检查 Kalibr init 导入后的 `T_i_b`、staged 释放前后平移变化、pose acceleration spike 与 accel residual。
2. 对 Suite B 做小规模 ablation：`fix-imu-extrinsics`、非参考 IMU lever 先验/bound、分阶段小范围释放、IMU time offset 固定/释放。
3. 对 TUM 做 `fix-camera-chain-extrinsics` 或 camera-chain prior 对照，确认 loop error 与速度/精度的关系。
4. 若要制定默认 stop policy，优先基于本次 `ceres_iteration_trace.csv` 和 `ceres_stop_policy_replay.csv`，不要只看最终迭代数。
