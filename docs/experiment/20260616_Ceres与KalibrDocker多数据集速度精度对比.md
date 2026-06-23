# Ceres Native 与 Kalibr Docker 多数据集速度精度对比

## 结论先行

当前文档只保留最新可复用结果，不混入旧实验表。三组 suite 均已按当前 runner 完成；`benchmark-multi-imu` 已使用修正后的 Kalibr 多 IMU delay 口径重跑。

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --out-root out/docker_benchmarks/single_amd64_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
```

| Suite | 状态 | Ceres topology | Kalibr 平台 | 关键结论 |
|---|---|---|---|---|
| `benchmark-single` | 已完成 | `1cam+1imu` | amd64 + arm64 | `12/12` 通过；平均平移差 `1.92 mm`，最大 `4.03 mm`；Ceres wall 均值 `114.0 s` |
| `benchmark-multi-imu` | 已完成 | joint 为 `1cam+Nimu`，single 子项为 `1cam+1imu` | arm64 | `60/60` 通过；Kalibr joint `12/12` 成功；Ceres joint-vs-single 平移均值 `39.5 mm`、最大 `161.8 mm`；C/K 有效链差 single 均值 `5.2 mm`、joint 均值 `19.1 mm` |
| `tum` | 已完成 | `Mcam+1imu` | arm64 | `2/2` 通过；平均平移差 `0.69 mm`，最大 `1.04 mm`；Ceres loop error 最大 `0.0347 deg / 0.249 mm` |

核心判断：

- `benchmark-single` 已经恢复到毫米级外参差异，没有本轮厘米级异常。
- `benchmark-multi-imu` 的 Kalibr 失败主因已确认是旧 runner 漏传 `--imu-delay-by-correlation`；修正后 Kalibr joint 恢复 `12/12`。Suite B rotation 已改用 SO(3) 相对旋转重算，旧的 `16.98 deg` 级 Ceres single-joint rotation 结论作废；当前主要异常收敛到 Ceres joint effective chain 的平移 tail。
- `benchmark-multi-imu` 的 C/K 对比要分两层看：single Ceres vs single Kalibr 平移差均值 `5.2 mm`，joint Ceres vs joint Kalibr 有效链平移差均值 `19.1 mm`；joint 的 camera0 接近不代表全部 IMU chain 都接近。single 最大值 `41.56 mm / 0.831 deg` 来自 b09/single_imu3，定点 ablation 见 `docs/experiment/20260623_b09_single_joint_outlier_ablation.md`。
- TUM 双目单 IMU 精度已对齐 Kalibr：外参差小于 `1.1 mm`，time-shift 差小于 `0.05 ms`。
- TUM 的 camera-chain loop error 显示 Ceres 两路 camera-to-IMU 外参和固定双目链之间仍有 `0.03 deg / 0.2 mm` 级闭环误差；这不是失败，但应该作为 `Mcam+1imu` 的稳定性指标保留。
- 速度不能写成 Ceres 全面更快：Ceres native 相对 Kalibr amd64 Docker 明显更快；相对 Kalibr arm64 Docker，优化段仍慢。

## 配置口径

`--suite` 是 `tools/run_docker_benchmark.py` 的实验选择参数，不是 Ceres native 标定器参数。Ceres running 口径由 `build/calibrate_cam_imu --corner-defaults` 按输入数量分发：

| 输入规模 | Topology | 当前 running |
|---|---|---|
| 1 个 `--corners` + 1 个 `--imu-data` | `1cam+1imu` | production solver defaults |
| 1 个 `--corners` + 多个 `--imu-data` | `1cam+Nimu` | production solver defaults；runner 对 joint 多 IMU 额外加 staged |
| 多个 `--corners` + 1 个 `--imu-data` | `Mcam+1imu` | production solver defaults |
| 多个 `--corners` + 多个 `--imu-data` | `Mcam+Nimu` | production solver defaults；当前未覆盖 |

production solver defaults：

| 参数 | 值 |
|---|---:|
| `pose_kps / bias_kps` | `100 / 50` |
| `time_padding_s` | `0.04` |
| `camera_time_offset_buffer_s` | `0` |
| robust loss | camera / gyro / accel 均为 `cauchy:10` |
| `max_iterations` | `150` |
| Ceres `function/gradient/parameter tolerance` | `0 / 0 / 0` |
| `solver_max_trust_region_radius` | `1e7` |
| absolute stop | cost `-1`，step `0.02`，parameter `-1` |
| nonmonotonic | enabled，max steps `20` |

实验元信息：

| 项目 | 内容 |
|---|---|
| Ceres | native `build/calibrate_cam_imu` |
| Ceres commit | `71a23e65b752` |
| Kalibr Docker repo commit | `e83ecfc4d6c0` |
| benchmark 数据根 | `/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data` |
| TUM 数据根 | `/Users/wayne/Documents/work/data/TUM` |

## 指标口径

| 指标 | 单位 | 含义 | 注意事项 |
|---|---:|---|---|
| success | count | Ceres、Kalibr、compare return code 是否为 0 | 只说明流程完成 |
| rotation diff | deg | Ceres 与 Kalibr `T_c_b` 旋转差 | 与 Kalibr 比，不是真值误差 |
| translation diff | mm | Ceres 与 Kalibr `T_c_b` 平移欧氏差 | 与 Kalibr 比，不是真值误差 |
| time-shift diff | ms | Ceres time shift 减 Kalibr time shift | 保留正负号，聚合看绝对值 |
| residual mean | px / rad/s / m/s^2 | 重投影、gyro、accel residual mean | 不同 residual 类型不能跨单位比较 |
| optimize time | s | 后端优化耗时 | Ceres 与 Kalibr 内部统计口径不同 |
| wall time | s | runner 端到端耗时 | 包含 Docker、读写、报告生成 |
| loop error | deg / mm | 双目链闭环误差 | 只适用于多 camera |
| joint-vs-single `T_c_i` diff | mm / deg / ms | 同一 IMU 在独立标定和 joint 标定中的有效 camera-to-IMU 外参差 | 只适用于 multi-IMU；Ceres result 中 joint 有效链按 `T_c_b * T_b_i`，Kalibr result 中按 `T_ci0 * inv(T_ib)`；rotation 必须用 SO(3) 相对旋转，不能用两个绝对旋转角相减 |

TUM loop error 的定义如下。Kalibr camchain 给出 `T_c1_c0 = cam1.T_cn_cnm1`，本文使用 `T_c0_c1 = inv(T_c1_c0)` 作为固定双目链；优化外参给出 `T_c0_i` 和 `T_c1_i`，于是经 IMU 闭合得到：

```text
T_c0_c1_via_imu = T_c0_i * T_i_c1
loop_error = inv(T_c0_c1) * T_c0_c1_via_imu
```

这与 `T_c1c2` 对比 `T_c1i * T_ic2` 是同一个检查，只是这里显式写成 cam0/cam1，避免 0-index 和 1-index 混淆。

## 结果总览

| Suite | Case 数 | success | 平均/最大平移差 | 平均/最大旋转差 | 平均/最大 abs time-shift | Ceres/Kalibr wall mean | Ceres/Kalibr optimize mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| `benchmark-single` vs Kalibr amd64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.0 / 192.0 s` | `110.8 / 119.4 s` |
| `benchmark-single` vs Kalibr arm64 | 12 | 12/12 | `1.92 / 4.03 mm` | `0.0056 / 0.0148 deg` | `2.06 / 6.27 ms` | `114.0 / 122.7 s` | `110.8 / 73.7 s` |
| `benchmark-multi-imu` Ceres joint-vs-single | 48 对 | 60/60 run OK | `39.5 / 161.8 mm` | `0.284 / 1.648 deg` | `1.92 / 8.25 ms` | single `125.8 / 123.1 s`; joint `85.9 / 321.1 s` | single `122.1 / 73.2 s`; joint `81.8 / 225.7 s` |
| `tum` vs Kalibr arm64 | 2 | 2/2 | `0.69 / 1.04 mm` | `0.0316 / 0.0398 deg` | `0.024 / 0.045 ms` | `61.7 / 58.1 s` | `60.2 / 21.8 s` |

读数：`benchmark-single` 精度已经稳定，但 `benchmark_08/12` 的 time-shift 和 accel residual 仍是后续定位重点。TUM 精度更贴近 Kalibr，但 Ceres 优化段仍明显慢于 Kalibr arm64。

`benchmark-multi-imu` 按 Ceres 内部一致性检查汇总：比较每个 `dataN.csv` 独立标定得到的 `T_c_i`，和同一 session 的 joint 4IMU 标定中第 N 个 IMU 的有效 `T_c_i`。staged joint 的 Ceres optimize time 采用两个 stage timing 求和。

Suite B 还单独统计 C/K 差异：single Ceres vs single Kalibr 的有效链平移差均/最大为 `5.23 / 41.56 mm`，joint Ceres vs joint Kalibr 的有效链平移差均/最大为 `19.07 / 163.03 mm`。这两个数回答的是“Ceres 与 Kalibr 是否一致”，与上表的 “single 与 joint 是否一致” 是两个不同问题。

## Suite A: benchmark-single

### 范围

`benchmark-single` 跑 12 个生产 benchmark session，每个 session 只使用 `data1.csv`。Ceres 是 `1cam+1imu` topology；Kalibr 默认同时跑 amd64 和 arm64。

结果来源：

```text
out/docker_benchmarks/single_amd64_arm64/benchmark_single/summary.csv
```

### 聚合结果

| Kalibr 平台 | 行数 | success | 平均平移差 | 最大平移差 | 平均旋转差 | 最大旋转差 | 平均 abs time-shift | 最大 abs time-shift |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `linux/amd64` | 12 | 12/12 | `1.92 mm` | `4.03 mm` | `0.0056 deg` | `0.0148 deg` | `2.06 ms` | `6.27 ms` |
| `linux/arm64` | 12 | 12/12 | `1.92 mm` | `4.03 mm` | `0.0056 deg` | `0.0148 deg` | `2.06 ms` | `6.27 ms` |

### 分数据集结果

下表以 Kalibr `linux/amd64` 为精度和逐数据集耗时基线；`C/K` 表示 Ceres native / Kalibr amd64。

| 数据集 | 平移差 mm | 旋转差 deg | time-shift 差 ms | reproj px C/K | accel m/s^2 C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---:|---:|---:|---|---|---:|---:|---:|
| b01 | 1.69 | 0.0060 | -0.99 | 0.180275 / 0.179774 | 0.114557 / 0.108088 | 127.0 / 85.2 | 129.9 / 161.5 | 130 |
| b02 | 1.08 | 0.0039 | -0.80 | 0.180266 / 0.179743 | 0.112463 / 0.107294 | 148.0 / 125.1 | 150.5 / 196.1 | 151 |
| b03 | 1.02 | 0.0050 | -0.78 | 0.180301 / 0.180087 | 0.119199 / 0.114536 | 146.0 / 62.1 | 148.4 / 133.6 | 151 |
| b04 | 2.32 | 0.0070 | -1.39 | 0.179710 / 0.179124 | 0.116340 / 0.107415 | 114.0 / 125.6 | 116.6 / 197.6 | 116 |
| b05 | 2.53 | 0.0016 | -0.57 | 0.179187 / 0.178664 | 0.114687 / 0.111014 | 115.0 / 171.7 | 118.3 / 245.8 | 116 |
| b06 | 1.88 | 0.0034 | -1.12 | 0.177188 / 0.176607 | 0.123734 / 0.116744 | 118.0 / 83.7 | 121.0 / 155.4 | 120 |
| b07 | 0.97 | 0.0046 | -3.20 | 0.170813 / 0.171469 | 0.095860 / 0.080972 | 88.5 / 137.1 | 92.2 / 209.3 | 91 |
| b08 | 4.03 | 0.0148 | -6.27 | 0.172360 / 0.171172 | 0.114861 / 0.085099 | 66.0 / 156.4 | 69.7 / 228.2 | 67 |
| b09 | 0.56 | 0.0000 | -0.70 | 0.170437 / 0.170591 | 0.096120 / 0.093327 | 124.0 / 116.4 | 127.5 / 189.0 | 128 |
| b10 | 0.43 | 0.0000 | +1.34 | 0.171897 / 0.170795 | 0.084087 / 0.089701 | 145.0 / 98.5 | 148.4 / 172.2 | 151 |
| b11 | 3.15 | 0.0090 | +1.42 | 0.172832 / 0.171706 | 0.086650 / 0.092505 | 65.2 / 97.5 | 69.0 / 170.0 | 67 |
| b12 | 3.35 | 0.0123 | -6.19 | 0.172194 / 0.171249 | 0.141210 / 0.116552 | 72.5 / 173.9 | 76.3 / 245.7 | 75 |

读数：`b08` 是最大平移差、最大旋转差和最大 time-shift 差，但平移差仍为 `4.03 mm`。`b12` 当前 Ceres 在第 75 次迭代停止，平移差 `3.35 mm`，主要剩余差异是 time shift `-6.19 ms` 和 accel residual 偏高。

### 速度读数

| 对比平台 | Ceres wall mean | Kalibr wall mean | Ceres optimize mean | Kalibr optimize mean | 结论 |
|---|---:|---:|---:|---:|---|
| Kalibr amd64 | `114.0 s` | `192.0 s` | `110.8 s` | `119.4 s` | Ceres native 更快 |
| Kalibr arm64 | `114.0 s` | `122.7 s` | `110.8 s` | `73.7 s` | Ceres wall 略快，优化段更慢 |

## Suite B: benchmark-multi-imu

### 读法

Suite B 跑 12 个 benchmark，每个 benchmark 有 1 个 `joint_4imu` 和 4 个独立单 IMU 标定：`single_imu1..4`。本节只保留四类核心对比：

| 对比 | 含义 | 用途 |
|---|---|---|
| Ceres single-joint | 同一 IMU 的 Ceres 独立标定 vs Ceres 4IMU joint 有效链 | 看 Ceres joint 是否保持独立标定一致性 |
| Kalibr single-joint | 同一 IMU 的 Kalibr 独立标定 vs Kalibr 4IMU joint 有效链 | 看数据/约束本身的 single-joint 差异 |
| C/K single | Ceres single vs Kalibr single | 看单 IMU native Ceres 是否贴近 Kalibr |
| C/K joint | Ceres joint effective chain vs Kalibr joint effective chain | 看 joint 整条 IMU chain 是否贴近 Kalibr |

表格里的 `T/R/t` 统一表示：translation `mm` / rotation `deg` / time-shift `ms`。`wall C/K` 表示 Ceres wall time / Kalibr wall time，单位为秒。joint 的有效外参使用 `T_c_i = T_c_b * T_b_i`；joint 的有效 time-shift 使用 `camera_time_shift_s - imu_time_offset_s`。

### 聚合对比

| 分组 | Ceres single-joint T/R/t 均/中/最大 | Kalibr single-joint T/R/t 均/中/最大 | C/K single T/R/t 均/中/最大 | C/K joint T/R/t 均/中/最大 |
|---|---:|---:|---:|---:|
| all 48 | `39.5/36.5/161.8 mm`, `0.284/0.139/1.648 deg`, `1.92/1.63/8.25 ms` | `25.5/18.4/96.0 mm`, `0.147/0.096/0.887 deg`, `1.27/1.10/2.87 ms` | `5.2/2.6/41.6 mm`, `0.035/0.007/0.831 deg`, `1.20/0.87/6.27 ms` | `19.1/12.3/163.0 mm`, `0.267/0.069/2.573 deg`, `0.24/0.02/2.65 ms` |
| b01-b06 | `21.1/21.7/43.0 mm`, `0.125/0.096/0.339 deg`, `1.82/1.75/3.04 ms` | `7.1/7.0/12.8 mm`, `0.082/0.066/0.227 deg`, `0.80/0.71/1.57 ms` | `4.6/2.4/21.7 mm`, `0.008/0.005/0.030 deg`, `1.04/0.90/2.32 ms` | `12.9/16.7/23.6 mm`, `0.077/0.067/0.385 deg`, `0.02/0.02/0.03 ms` |
| b07-b12 | `58.0/46.1/161.8 mm`, `0.443/0.285/1.648 deg`, `2.03/1.48/8.25 ms` | `44.0/40.0/96.0 mm`, `0.212/0.143/0.887 deg`, `1.73/1.74/2.87 ms` | `5.9/3.1/41.6 mm`, `0.063/0.023/0.831 deg`, `1.35/0.78/6.27 ms` | `25.2/4.8/163.0 mm`, `0.456/0.076/2.573 deg`, `0.46/0.03/2.65 ms` |

速度：48 个 single case 的 wall time 为 Ceres/Kalibr `125.8 / 123.1 s` 均值，total `6036.6 / 5908.0 s`；12 个 joint case 的 wall time 为 Ceres/Kalibr `85.9 / 321.1 s` 均值，total `1030.7 / 3853.6 s`。joint 速度优势来自 Ceres staged 路径，但精度仍要看有效 IMU chain。

读数：

- single C/K 大多数很近，最大 `41.6 mm / 0.831 deg` 集中在 `b09/imu3`，这个 case 已单独做 ablation。
- joint C/K 最大平移 `163.0 mm`、最大旋转 `2.573 deg` 都集中在 `b09` 的非参考 IMU chain。
- Ceres single-joint rotation 经 SO(3) 相对旋转重算后不再有十几度尾部：最大为 `b09/imu4 1.648 deg`，`b08/imu2` 从旧口径的 `16.978 deg` 修正为 `0.381 deg`。当前主要问题是 joint effective chain 的平移 tail。
- b01-b06 的 C/K single 基本是毫米级到 2cm；b07-b12 的 C/K tail 由 b09、b11、b12 拉大。

### 48 路明细

#### 列与单位

| 列 | 含义 | 单位/符号 |
|---|---|---|
| `benchmark` | benchmark 编号，`b01..b12` | 无 |
| `IMU` | 对应 `single_imu1..4`，joint 表示同一 IMU 的 effective chain | 无 |
| `Δt` | 两个结果之间的外参平移距离 | `mm`，非负，越小越一致 |
| `ΔR` | 两个结果之间的 SO(3) 相对旋转角 | `deg`，非负，越小越一致；按 `R_delta = R_a * R_b^T` 计算 |
| `Δτ` | time-shift 差值 | `ms`，有符号；single-joint 为 `single - joint`，C/K 为 `Ceres - Kalibr` |
| `wall` | runner 记录的端到端墙钟耗时 | `s`，越小越快 |
| single-joint | 同一 solver 内独立单 IMU 标定与 4IMU joint effective chain 的差 | 检查 joint 是否保持 single 一致性 |
| C/K single | Ceres single 与 Kalibr single 的差 | 检查 native 单 IMU 标定是否贴近 Kalibr |
| C/K joint | Ceres joint effective chain 与 Kalibr joint effective chain 的差 | 检查整条 joint IMU chain 是否贴近 Kalibr |

joint 的有效外参使用 `T_c_i = T_c_b * T_b_i`；joint 的有效 time-shift 使用 `camera_time_shift_s - imu_time_offset_s`。joint wall time 是一次 4IMU joint 标定的耗时，所以每个 benchmark 只有一条，不再在四个 IMU 行里重复。

#### A. Ceres single-joint

同一 IMU 的 Ceres 独立标定减 Ceres 4IMU joint effective chain。该表用于直接观察 Ceres joint 是否放大某个 IMU chain 的外参或 time-shift 差异。

| benchmark | IMU | Δt mm | ΔR deg | Δτ ms |
|---|---|---:|---:|---:|
| b01 | imu1 | 8.8 | 0.231 | -1.64 |
| b01 | imu2 | 21.5 | 0.128 | -1.28 |
| b01 | imu3 | 23.7 | 0.243 | -2.22 |
| b01 | imu4 | 22.8 | 0.215 | -2.59 |
| b02 | imu1 | 7.8 | 0.072 | -1.46 |
| b02 | imu2 | 21.1 | 0.084 | -1.25 |
| b02 | imu3 | 23.6 | 0.120 | -1.53 |
| b02 | imu4 | 21.9 | 0.079 | -1.49 |
| b03 | imu1 | 10.4 | 0.172 | -1.27 |
| b03 | imu2 | 28.3 | 0.136 | -1.61 |
| b03 | imu3 | 43.0 | 0.191 | -2.02 |
| b03 | imu4 | 26.3 | 0.076 | -3.04 |
| b04 | imu1 | 9.0 | 0.061 | -2.02 |
| b04 | imu2 | 18.7 | 0.076 | -1.16 |
| b04 | imu3 | 36.9 | 0.153 | -2.17 |
| b04 | imu4 | 16.6 | 0.071 | -2.84 |
| b05 | imu1 | 9.4 | 0.107 | -1.19 |
| b05 | imu2 | 24.1 | 0.047 | -1.13 |
| b05 | imu3 | 23.0 | 0.339 | -2.03 |
| b05 | imu4 | 22.3 | 0.086 | -2.40 |
| b06 | imu1 | 14.6 | 0.067 | -1.76 |
| b06 | imu2 | 19.4 | 0.154 | -1.74 |
| b06 | imu3 | 36.9 | 0.059 | -2.12 |
| b06 | imu4 | 16.2 | 0.030 | -1.78 |
| b07 | imu1 | 41.7 | 0.051 | -4.72 |
| b07 | imu2 | 36.1 | 0.170 | -0.75 |
| b07 | imu3 | 46.7 | 0.143 | -2.07 |
| b07 | imu4 | 49.1 | 0.523 | -1.39 |
| b08 | imu1 | 40.8 | 0.135 | -8.25 |
| b08 | imu2 | 41.6 | 0.381 | -2.27 |
| b08 | imu3 | 39.4 | 0.558 | -1.74 |
| b08 | imu4 | 38.9 | 1.050 | -0.60 |
| b09 | imu1 | 95.1 | 0.339 | -0.61 |
| b09 | imu2 | 93.7 | 0.806 | -0.44 |
| b09 | imu3 | 132.7 | 1.431 | -1.61 |
| b09 | imu4 | 161.8 | 1.648 | -1.14 |
| b10 | imu1 | 34.6 | 0.055 | -0.73 |
| b10 | imu2 | 38.5 | 0.111 | -2.01 |
| b10 | imu3 | 36.8 | 0.049 | -2.08 |
| b10 | imu4 | 46.2 | 0.244 | -1.13 |
| b11 | imu1 | 63.7 | 0.197 | +0.35 |
| b11 | imu2 | 42.5 | 0.447 | -1.64 |
| b11 | imu3 | 56.1 | 0.391 | -1.27 |
| b11 | imu4 | 73.0 | 1.302 | +2.82 |
| b12 | imu1 | 61.0 | 0.081 | -7.82 |
| b12 | imu2 | 26.9 | 0.121 | -1.58 |
| b12 | imu3 | 49.0 | 0.066 | -1.38 |
| b12 | imu4 | 45.9 | 0.326 | -0.22 |

#### B. Kalibr single-joint

同一 IMU 的 Kalibr 独立标定减 Kalibr 4IMU joint effective chain。该表是数据/约束本身 single-joint 差异的参考基线。

| benchmark | IMU | Δt mm | ΔR deg | Δτ ms |
|---|---|---:|---:|---:|
| b01 | imu1 | 7.6 | 0.227 | -0.66 |
| b01 | imu2 | 7.2 | 0.133 | -0.42 |
| b01 | imu3 | 6.4 | 0.180 | -1.57 |
| b01 | imu4 | 4.8 | 0.144 | -0.90 |
| b02 | imu1 | 7.1 | 0.069 | -0.67 |
| b02 | imu2 | 6.1 | 0.019 | -0.34 |
| b02 | imu3 | 7.1 | 0.134 | -1.05 |
| b02 | imu4 | 5.8 | 0.062 | -0.87 |
| b03 | imu1 | 9.8 | 0.174 | -0.51 |
| b03 | imu2 | 7.1 | 0.093 | -0.55 |
| b03 | imu3 | 7.0 | 0.058 | -1.11 |
| b03 | imu4 | 4.9 | 0.095 | -0.74 |
| b04 | imu1 | 7.5 | 0.057 | -0.64 |
| b04 | imu2 | 8.0 | 0.023 | -0.41 |
| b04 | imu3 | 7.7 | 0.026 | -1.38 |
| b04 | imu4 | 5.4 | 0.038 | -0.74 |
| b05 | imu1 | 7.5 | 0.106 | -0.64 |
| b05 | imu2 | 9.5 | 0.034 | -0.32 |
| b05 | imu3 | 7.0 | 0.033 | -1.48 |
| b05 | imu4 | 5.1 | 0.027 | -0.85 |
| b06 | imu1 | 12.8 | 0.069 | -0.66 |
| b06 | imu2 | 6.6 | 0.056 | -0.43 |
| b06 | imu3 | 5.4 | 0.013 | -1.45 |
| b06 | imu4 | 6.5 | 0.095 | -0.87 |
| b07 | imu1 | 42.1 | 0.048 | -1.54 |
| b07 | imu2 | 39.9 | 0.098 | -0.85 |
| b07 | imu3 | 40.2 | 0.044 | -2.41 |
| b07 | imu4 | 46.8 | 0.161 | -2.28 |
| b08 | imu1 | 41.7 | 0.124 | -2.01 |
| b08 | imu2 | 39.7 | 0.153 | -2.34 |
| b08 | imu3 | 33.6 | 0.035 | -1.92 |
| b08 | imu4 | 40.0 | 0.414 | -1.81 |
| b09 | imu1 | 96.0 | 0.858 | -2.56 |
| b09 | imu2 | 31.4 | 0.559 | -1.76 |
| b09 | imu3 | 25.8 | 0.887 | -2.09 |
| b09 | imu4 | 24.0 | 0.125 | -2.87 |
| b10 | imu1 | 34.9 | 0.052 | -2.04 |
| b10 | imu2 | 40.0 | 0.114 | -1.81 |
| b10 | imu3 | 33.0 | 0.007 | -1.72 |
| b10 | imu4 | 39.4 | 0.155 | -0.76 |
| b11 | imu1 | 64.3 | 0.199 | -1.09 |
| b11 | imu2 | 44.2 | 0.200 | -1.68 |
| b11 | imu3 | 34.9 | 0.133 | -0.93 |
| b11 | imu4 | 45.9 | 0.190 | -0.75 |
| b12 | imu1 | 62.4 | 0.071 | -1.63 |
| b12 | imu2 | 38.7 | 0.165 | -1.56 |
| b12 | imu3 | 49.1 | 0.035 | -1.53 |
| b12 | imu4 | 67.7 | 0.253 | -1.59 |

#### C. C/K single

Ceres single 减 Kalibr single。该表用于检查 48 个独立单 IMU 标定中 Ceres native 与 Kalibr 的外参和 time-shift 一致性。

| benchmark | IMU | Δt mm | ΔR deg | Δτ ms |
|---|---|---:|---:|---:|
| b01 | imu1 | 1.7 | 0.006 | -0.99 |
| b01 | imu2 | 1.2 | 0.004 | -0.87 |
| b01 | imu3 | 2.8 | 0.019 | -0.66 |
| b01 | imu4 | 3.3 | 0.007 | -1.71 |
| b02 | imu1 | 1.1 | 0.004 | -0.80 |
| b02 | imu2 | 1.3 | 0.002 | -0.92 |
| b02 | imu3 | 1.4 | 0.014 | -0.49 |
| b02 | imu4 | 1.5 | 0.011 | -0.63 |
| b03 | imu1 | 1.0 | 0.005 | -0.78 |
| b03 | imu2 | 2.2 | 0.006 | -1.08 |
| b03 | imu3 | 19.9 | 0.004 | -0.93 |
| b03 | imu4 | 3.7 | 0.005 | -2.32 |
| b04 | imu1 | 2.3 | 0.007 | -1.39 |
| b04 | imu2 | 1.1 | 0.003 | -0.76 |
| b04 | imu3 | 21.7 | 0.001 | -0.79 |
| b04 | imu4 | 3.5 | 0.004 | -2.11 |
| b05 | imu1 | 2.5 | 0.002 | -0.57 |
| b05 | imu2 | 2.6 | 0.006 | -0.83 |
| b05 | imu3 | 4.1 | 0.023 | -0.57 |
| b05 | imu4 | 3.3 | 0.008 | -1.57 |
| b06 | imu1 | 1.9 | 0.003 | -1.12 |
| b06 | imu2 | 2.6 | 0.005 | -1.34 |
| b06 | imu3 | 21.4 | 0.030 | -0.70 |
| b06 | imu4 | 1.9 | 0.009 | -0.94 |
| b07 | imu1 | 1.0 | 0.005 | -3.20 |
| b07 | imu2 | 0.4 | 0.000 | +0.08 |
| b07 | imu3 | 0.8 | 0.028 | +0.32 |
| b07 | imu4 | 0.7 | 0.029 | +0.87 |
| b08 | imu1 | 4.0 | 0.015 | -6.27 |
| b08 | imu2 | 0.5 | 0.006 | +0.04 |
| b08 | imu3 | 4.9 | 0.033 | +0.15 |
| b08 | imu4 | 0.7 | 0.027 | +1.18 |
| b09 | imu1 | 0.6 | 0.000 | -0.70 |
| b09 | imu2 | 7.3 | 0.091 | -1.33 |
| b09 | imu3 | 41.6 | 0.831 | -2.17 |
| b09 | imu4 | 25.5 | 0.069 | -0.93 |
| b10 | imu1 | 0.4 | 0.000 | +1.34 |
| b10 | imu2 | 0.4 | 0.000 | -0.17 |
| b10 | imu3 | 7.4 | 0.058 | -0.32 |
| b10 | imu4 | 10.6 | 0.100 | -0.33 |
| b11 | imu1 | 3.1 | 0.009 | +1.42 |
| b11 | imu2 | 3.1 | 0.000 | +0.02 |
| b11 | imu3 | 7.9 | 0.081 | -0.36 |
| b11 | imu4 | 8.3 | 0.018 | +3.54 |
| b12 | imu1 | 3.3 | 0.012 | -6.19 |
| b12 | imu2 | 0.4 | 0.002 | -0.02 |
| b12 | imu3 | 7.8 | 0.062 | +0.15 |
| b12 | imu4 | 0.5 | 0.031 | +1.37 |

#### D. C/K joint

Ceres joint effective chain 减 Kalibr joint effective chain。该表用于检查 12 个 joint 标定展开到 48 条 IMU chain 后的 Ceres/Kalibr 一致性。

| benchmark | IMU | Δt mm | ΔR deg | Δτ ms |
|---|---|---:|---:|---:|
| b01 | imu1 | 0.0 | 0.000 | -0.01 |
| b01 | imu2 | 16.6 | 0.041 | -0.01 |
| b01 | imu3 | 17.4 | 0.083 | -0.01 |
| b01 | imu4 | 18.5 | 0.072 | -0.01 |
| b02 | imu1 | 0.0 | 0.000 | -0.01 |
| b02 | imu2 | 17.9 | 0.068 | -0.01 |
| b02 | imu3 | 17.8 | 0.020 | -0.01 |
| b02 | imu4 | 18.3 | 0.009 | -0.01 |
| b03 | imu1 | 0.0 | 0.001 | -0.02 |
| b03 | imu2 | 23.6 | 0.139 | -0.02 |
| b03 | imu3 | 21.6 | 0.247 | -0.02 |
| b03 | imu4 | 22.8 | 0.097 | -0.02 |
| b04 | imu1 | 0.0 | 0.003 | -0.01 |
| b04 | imu2 | 10.9 | 0.065 | -0.01 |
| b04 | imu3 | 13.8 | 0.148 | -0.01 |
| b04 | imu4 | 9.9 | 0.037 | -0.01 |
| b05 | imu1 | 0.0 | 0.000 | -0.02 |
| b05 | imu2 | 16.0 | 0.076 | -0.02 |
| b05 | imu3 | 17.6 | 0.385 | -0.02 |
| b05 | imu4 | 16.3 | 0.057 | -0.02 |
| b06 | imu1 | 0.3 | 0.008 | -0.03 |
| b06 | imu2 | 17.0 | 0.122 | -0.03 |
| b06 | imu3 | 16.8 | 0.071 | -0.03 |
| b06 | imu4 | 17.4 | 0.103 | -0.03 |
| b07 | imu1 | 0.0 | 0.004 | -0.02 |
| b07 | imu2 | 4.2 | 0.080 | -0.02 |
| b07 | imu3 | 6.7 | 0.073 | -0.02 |
| b07 | imu4 | 9.1 | 0.355 | -0.02 |
| b08 | imu1 | 0.0 | 0.000 | -0.03 |
| b08 | imu2 | 4.8 | 0.520 | -0.03 |
| b08 | imu3 | 4.6 | 0.558 | -0.03 |
| b08 | imu4 | 4.8 | 0.620 | -0.03 |
| b09 | imu1 | 0.8 | 1.143 | -2.65 |
| b09 | imu2 | 77.9 | 1.353 | -2.65 |
| b09 | imu3 | 149.7 | 2.573 | -2.65 |
| b09 | imu4 | 163.0 | 1.696 | -2.65 |
| b10 | imu1 | 0.8 | 0.006 | +0.04 |
| b10 | imu2 | 1.8 | 0.005 | +0.04 |
| b10 | imu3 | 1.4 | 0.005 | +0.04 |
| b10 | imu4 | 2.8 | 0.000 | +0.04 |
| b11 | imu1 | 0.0 | 0.003 | -0.02 |
| b11 | imu2 | 36.2 | 0.299 | -0.02 |
| b11 | imu3 | 29.7 | 0.360 | -0.02 |
| b11 | imu4 | 56.3 | 1.167 | -0.02 |
| b12 | imu1 | 0.0 | 0.001 | +0.00 |
| b12 | imu2 | 16.7 | 0.046 | +0.00 |
| b12 | imu3 | 10.5 | 0.033 | +0.00 |
| b12 | imu4 | 23.0 | 0.043 | +0.00 |

#### E. Single wall time

每一行是一条独立单 IMU 标定，单位为秒。

| benchmark | IMU | Ceres single wall s | Kalibr single wall s |
|---|---|---:|---:|
| b01 | imu1 | 131.0 | 101.6 |
| b01 | imu2 | 150.3 | 141.1 |
| b01 | imu3 | 128.3 | 88.7 |
| b01 | imu4 | 115.5 | 156.2 |
| b02 | imu1 | 150.1 | 125.4 |
| b02 | imu2 | 150.0 | 138.7 |
| b02 | imu3 | 151.0 | 113.5 |
| b02 | imu4 | 153.9 | 167.6 |
| b03 | imu1 | 152.0 | 90.0 |
| b03 | imu2 | 121.8 | 103.8 |
| b03 | imu3 | 151.9 | 107.0 |
| b03 | imu4 | 118.7 | 171.1 |
| b04 | imu1 | 116.4 | 128.8 |
| b04 | imu2 | 152.5 | 141.5 |
| b04 | imu3 | 152.9 | 93.5 |
| b04 | imu4 | 116.4 | 156.0 |
| b05 | imu1 | 120.2 | 157.8 |
| b05 | imu2 | 104.3 | 91.6 |
| b05 | imu3 | 141.3 | 90.4 |
| b05 | imu4 | 117.9 | 170.9 |
| b06 | imu1 | 122.8 | 104.1 |
| b06 | imu2 | 113.6 | 128.5 |
| b06 | imu3 | 149.3 | 90.8 |
| b06 | imu4 | 150.3 | 116.9 |
| b07 | imu1 | 92.3 | 138.7 |
| b07 | imu2 | 149.3 | 133.7 |
| b07 | imu3 | 101.6 | 101.3 |
| b07 | imu4 | 152.9 | 111.8 |
| b08 | imu1 | 69.4 | 152.2 |
| b08 | imu2 | 148.7 | 132.8 |
| b08 | imu3 | 100.5 | 86.3 |
| b08 | imu4 | 155.7 | 122.5 |
| b09 | imu1 | 132.8 | 121.5 |
| b09 | imu2 | 150.6 | 108.1 |
| b09 | imu3 | 73.6 | 289.6 |
| b09 | imu4 | 77.0 | 97.3 |
| b10 | imu1 | 153.5 | 115.7 |
| b10 | imu2 | 152.4 | 136.2 |
| b10 | imu3 | 118.7 | 77.7 |
| b10 | imu4 | 150.9 | 77.2 |
| b11 | imu1 | 70.5 | 111.9 |
| b11 | imu2 | 78.8 | 111.4 |
| b11 | imu3 | 84.6 | 73.6 |
| b11 | imu4 | 96.0 | 137.9 |
| b12 | imu1 | 79.8 | 171.2 |
| b12 | imu2 | 150.9 | 122.3 |
| b12 | imu3 | 111.1 | 75.8 |
| b12 | imu4 | 152.6 | 126.1 |

#### F. Joint wall time

每一行是一条 4IMU joint 标定，单位为秒。旧宽表中该值会在同一 benchmark 的四个 IMU 行里重复，这里去重后展示。

| benchmark | Ceres joint wall s | Kalibr joint wall s |
|---|---:|---:|
| b01 | 23.4 | 289.4 |
| b02 | 22.3 | 192.2 |
| b03 | 27.6 | 197.8 |
| b04 | 23.5 | 156.5 |
| b05 | 23.6 | 161.4 |
| b06 | 24.0 | 212.9 |
| b07 | 110.2 | 529.7 |
| b08 | 105.5 | 586.8 |
| b09 | 267.7 | 393.5 |
| b10 | 193.0 | 555.9 |
| b11 | 45.4 | 330.1 |
| b12 | 164.5 | 247.4 |

### 同工位一致性

下表比较“同一个 IMU 编号跨 benchmark 的 `T_ci` 是否一致”。数值是组内两两最大差异，格式为 translation `mm` / rotation `deg`。`all12` 不稳定时，再看 `b01-b06` 和 `b07-b12`。

| system/mode | IMU | all12 max T/R | b01-b06 max T/R | b07-b12 max T/R |
|---|---|---:|---:|---:|
| ceres single | imu1 | 51.1 / 9.424 | 9.6 / 1.748 | 51.1 / 9.424 |
| ceres single | imu2 | 97.4 / 3.703 | 4.2 / 1.848 | 97.4 / 3.703 |
| ceres single | imu3 | 120.7 / 7.683 | 20.6 / 1.598 | 120.7 / 6.662 |
| ceres single | imu4 | 81.4 / 3.988 | 8.5 / 1.122 | 81.4 / 3.443 |
| ceres joint | imu1 | 91.6 / 9.443 | 7.0 / 1.715 | 63.5 / 9.443 |
| ceres joint | imu2 | 98.4 / 4.043 | 14.1 / 1.764 | 98.4 / 4.043 |
| ceres joint | imu3 | 160.8 / 7.721 | 11.3 / 1.722 | 140.2 / 6.690 |
| ceres joint | imu4 | 136.9 / 4.814 | 12.8 / 1.195 | 132.3 / 4.496 |
| kalibr single | imu1 | 52.0 / 9.414 | 10.1 / 1.748 | 52.0 / 9.414 |
| kalibr single | imu2 | 93.8 / 3.703 | 4.3 / 1.846 | 93.8 / 3.703 |
| kalibr single | imu3 | 136.2 / 7.687 | 14.2 / 1.594 | 136.2 / 6.653 |
| kalibr single | imu4 | 107.5 / 3.992 | 7.8 / 1.121 | 107.5 / 3.435 |
| kalibr joint | imu1 | 92.3 / 9.440 | 7.0 / 1.715 | 64.1 / 9.440 |
| kalibr joint | imu2 | 120.4 / 3.718 | 5.4 / 1.788 | 120.4 / 3.718 |
| kalibr joint | imu3 | 125.0 / 7.658 | 10.0 / 1.627 | 125.0 / 6.671 |
| kalibr joint | imu4 | 129.3 / 4.024 | 7.0 / 1.092 | 129.3 / 3.511 |

读数：

- b01-b06 是稳定批次：同一 IMU 的 `T_ci` 在 Ceres/Kalibr、single/joint 中基本保持在毫米到 `1-2 cm`，rotation 约 `1-2 deg` 以内。
- b07-b12 不能整体视作一个稳定工位批次：b09、b11、b12 会把同 IMU 一致性拉到 `5-16 cm` 和数度级。
- b09 的异常同时出现在 Ceres 和 Kalibr 的同工位一致性里，但 Ceres joint/C-K tail 更重；这说明 b09 本身数据/工况就不稳定，Ceres joint 又进一步放大了 effective chain 差异。
- 后续不应只按“前 6 / 后 6”二分判断夹具；更合理的是先固定 b01-b06 作为稳定参考批次，再把 b09、b11、b12 作为异常 session 单独定位。

### 定位补充：Ceres joint 平移异常

本轮修正后，旧表里的 `b08/imu2 16.978 deg`、`b11/imu2 15.782 deg` 不再作为证据；它们来自派生统计把绝对旋转角差当成 relative SO(3) rotation。正确口径下 Ceres single-joint rotation tail 降到 `1.648 deg`。因此后续定位不再优先追 rotation，而是盯 Ceres joint effective chain 的平移 tail：`b09/imu4 161.8 mm`、`b09/imu3 132.7 mm`、`b09/imu2 93.7 mm`，以及 C/K joint 中 `b09/imu4 163.0 mm`、`b09/imu3 149.7 mm`。

正式仿真实验记录在 `docs/experiment/20260621_仿真多场景Ceres与Kalibr精度对比.md`。该文档的 `1cam+4imu` 真值结果显示 Ceres camera translation 误差 `0.766 mm`，非 reference IMU lever error 为 `6.88 / 9.68 / 11.75 mm`，结论是精度可用但多 IMU running 耗时偏高。这说明当前仿真基线没有复现生产 b09 的 `10-16 cm` 级平移异常。

| 仿真依据 | camera T/R/t 误差 | imu1/2/3 lever error | imu1/2/3 rotation error | 读数 |
|---|---:|---:|---:|---|
| `docs/experiment/20260621...` 的 `one_cam_four_imus/calibration_result.yaml` | `0.766 mm / 0.0834 deg / -0.152 ms` | `6.88 / 9.68 / 11.75 mm` | `0.0387 / 0.0853 / 0.1329 deg` | Ceres 仿真精度接近金标，没有复现 b09 tail |

已追加 b09-like posecsv simulation：用生产 b09 `cam0_640x400_corner_poses.csv` 驱动仿真轨迹，并用 b09 Kalibr joint 输出的 `T_cam_imu` / `T_i_b` 作为真值。结果显示 single 对真值保持 `1.6-6.0 mm`，但 joint 的非参考 IMU effective chain 对真值达到 `94-169 mm`；按生产 b09 最近帧 `(timestamp, corner_id)` mask 过滤角点后仍复现，且 joint 最大平移为 `168.7 mm`。给 joint 加 `--fix-imu-extrinsics` 后，非参考 IMU 平移误差降到 `0.01 mm` 内，但 accel RMS 从 `3.75-3.92` 升到 `4.03-4.08 m/s^2`。因此当前更可信的定位是：b09-like 强二阶轨迹下，Ceres joint 会让非参考 IMU lever arm 漂移以吸收 pose acceleration / accel residual 不一致；问题不来自角点过多，也不是单纯缺少金标。

| b09-like 仿真 | 角点 | single max T | joint non-ref mean/max T | fix IMU extrinsics 后 | 读数 |
|---|---:|---:|---:|---:|---|
| `focus_35_55_unmasked` | `359870` | `5.98 mm` | `99.27 / 103.87 mm` | `0.00 / 0.00 mm` | 轨迹足以复现 joint tail |
| `focus_35_55_masked` | `134060` | `5.71 mm` | `162.31 / 168.70 mm` | `0.01 / 0.01 mm` | 生产角点 mask 后仍复现 |

### 数据入口

```text
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_focus_comparison.csv  # rotation 已按 SO(3) 相对旋转重算
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_focus_comparison.md
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_split_tables.md
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_fixture_consistency.csv
out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_fixture_consistency_by_case.csv
out/ablations/20260623_b09_posecsv_sim_overview/summary.csv
out/ablations/20260623_b09_posecsv_sim_overview/summary.md
```

## Suite C: TUM

### 范围

TUM suite 跑两个双目单 IMU case，Ceres topology 为 `Mcam+1imu`，Kalibr 只跑 arm64。

结果来源：

```text
out/docker_benchmarks/tum_arm64/tum/tum_summary.csv
```

### 分 case 结果

| Case | 输入来源 | 平移差 mm | 旋转差 deg | time-shift 差 ms | loop error Ceres deg/mm | loop error Kalibr deg/mm | reproj px C/K | gyro rad/s C/K | accel m/s^2 C/K | 优化 s C/K | 墙钟 s C/K | Ceres it |
|---|---|---:|---:|---:|---:|---:|---|---|---|---:|---:|---:|
| `tum_imu1_bag` | bag | 1.04 | 0.0398 | -0.045 | 0.0295 / 0.216 | 0.0036 / 0.000 | 0.094929 / 0.100287 | 0.001163 / 0.001199 | 0.021031 / 0.021706 | 88.4 / 21.8 | 89.7 / 60.2 | 151 |
| `tum_imu2_euroc_kalibr_export` | euroc | 0.34 | 0.0233 | +0.004 | 0.0347 / 0.249 | 0.0000 / 0.000 | 0.093744 / 0.100253 | 0.001127 / 0.001171 | 0.020772 / 0.021605 | 32.0 / 21.8 | 33.8 / 56.1 | 55 |

读数：两个 TUM case 的 Ceres 与 Kalibr 外参差都小于 `1.1 mm`，time-shift 差小于 `0.05 ms`。Ceres residual mean 全部略低于 Kalibr arm64。Ceres loop error 为 `0.03 deg / 0.2 mm` 量级，说明两路 camera-to-IMU 外参和固定双目链之间还有可量化闭环差异；Kalibr loop error 接近 0，主要因为 baseline 来自同一 camchain，表中非零旋转来自文本矩阵精度截断。

### 速度读数

| Case | Ceres wall | Kalibr arm64 wall | Ceres optimize | Kalibr arm64 optimize | 结论 |
|---|---:|---:|---:|---:|---|
| `tum_imu1_bag` | `89.7 s` | `60.2 s` | `88.4 s` | `21.8 s` | Ceres 更慢 |
| `tum_imu2_euroc_kalibr_export` | `33.8 s` | `56.1 s` | `32.0 s` | `21.8 s` | wall 更快，优化段更慢 |

读数：TUM 精度风险不高，速度问题更明确。`Mcam+1imu` 后续应拆 solver 迭代、scale-misalignment IMU intrinsic residual、多相机 residual 构建成本，以及 loop error 与 camera-chain prior/fix 策略的关系。

## 跨 suite 分析

- `1cam+1imu` 当前主风险不是外参漂移，而是 `b08/b12` 的 time-shift 和 accel residual 差异。
- multi suite 的 Kalibr delay 问题已修复并重跑；`1cam+Nimu` 的时间对齐已恢复。SO(3) rotation 口径修正后，Ceres joint 的主要剩余风险是 b09/b11 的 IMU chain 平移 outlier。
- `1cam+Nimu` joint staged 在速度上明显快于 Kalibr joint，但精度结论必须和 joint-vs-single、有效 `T_c_i` 一起看。
- `Mcam+1imu` 的 TUM 外参与 residual 已对齐，但 camera-chain loop error 应进入后续常规指标。
- 速度结论必须带 Kalibr 平台：相对 Kalibr amd64 Docker，Ceres native 快；相对 Kalibr arm64 Docker，Ceres 优化段仍慢。
- staged joint 的速度不能和 Kalibr 直接相减得出“Ceres 完整 pipeline 更快”；当前可说的是修正后 joint Ceres staged wall/optimize 均值低于 Kalibr arm64 joint，但仍有精度 outlier。

## 边界

- 外参差异是 Ceres 与 Kalibr 的差，不是真值误差。
- Ceres 与 Kalibr 的 optimize time 不是完全同一内部统计口径，wall time 也包含不同外围开销。
- multi joint 的历史旧输出缺 Kalibr `--imu-delay-by-correlation`，不能再引用；本文 Suite B 数值来自修正后的重跑。
- multi joint 的 Ceres optimize time 按 Ceres stage timing 求和；summary 原始字段可能只反映最后一个 stage，不应用于 staged 总耗时结论。
- multi joint 的 C/K camera0 精度不能代表全部 IMU chain；必须使用 joint-vs-single 有效 `T_c_i` 检查。
- TUM 只跑 Kalibr arm64，没有 amd64 平台对照。
- loop error 当前只在 TUM 双目上记录；`Mcam+Nimu` 尚未覆盖。
- 本文没有做全 suite 级 stopping policy、nonmonotonic、time-shift prior、IMU 裁边 ablation；b09/single_imu3 与 b09 joint 尾部的定点 ablation 已单独记录在 `docs/experiment/20260623_b09_single_joint_outlier_ablation.md`。

## 复现入口

已完成结果入口：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --out-root out/docker_benchmarks/single_amd64_arm64
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/multi_imu_arm64
python3 tools/run_docker_benchmark.py --suite tum --kalibr-platform linux/arm64 --out-root out/docker_benchmarks/tum_arm64
```

当前文件：

| 文件 | 用途 |
|---|---|
| `out/docker_benchmarks/single_amd64_arm64/benchmark_single/summary.csv` | benchmark-single 主汇总 |
| `out/docker_benchmarks/single_amd64_arm64/summary.csv` | benchmark-single suite 汇总 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/summary.csv` | benchmark-multi-imu 主汇总 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/joint_vs_single_imu_extrinsics.csv` | Suite B 96 行 Ceres/Kalibr single-vs-joint 明细；旧 `rotation_delta_deg` 不作为 SO(3) 相对旋转结论使用 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/joint_vs_single_case_summary.csv` | Suite B 24 行 case 摘要 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/joint_vs_single_time_summary.csv` | Suite B single/joint 速度摘要 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_vs_kalibr_effective_chain_delta.csv` | Suite B Ceres/Kalibr 有效 IMU chain 差异 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/ceres_vs_kalibr_effective_chain_summary.csv` | Suite B Ceres/Kalibr single 与 joint 有效链统计 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_focus_comparison.csv` | Suite B 48 路核心表；rotation 已按 SO(3) 相对旋转重算 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_focus_comparison.md` | Suite B 48 路核心表 Markdown；rotation 已按 SO(3) 相对旋转重算 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_48_split_tables.md` | Suite B 48 路拆分明细表 Markdown；rotation 已按 SO(3) 相对旋转重算 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_fixture_consistency.csv` | Suite B 同 IMU 跨 benchmark 一致性统计 |
| `out/docker_benchmarks/multi_imu_arm64/benchmark_multi_imu/suite_b_fixture_consistency_by_case.csv` | Suite B 同工位一致性逐 case 偏差 |
| `out/docker_benchmarks/tum_arm64/tum/tum_summary.csv` | TUM 主汇总 |
| `out/docker_benchmarks/tum_arm64/tum/tum_imu2_euroc_kalibr_export/ceres/result.yaml` | 用户贴出的 TUM euroc Ceres result |
| `out/docker_benchmarks/tum_arm64/tum/tum_imu2_euroc_kalibr_export/compare_arm64/compare.clean.log` | TUM euroc Ceres/Kalibr delta |

## 下一步

1. 定位 b09/imu2/imu3/imu4、b11/imu2/imu4 的 joint 平移 outlier：b09/single_imu3 已排除 stop policy 主因，下一步优先看 pose acceleration spike、`T_i_b` 初始化、`r_b/r_i_b` 释放策略、reference IMU 固定、timeoffset 共享/约束和 staged mask。
2. 对 joint effective chain 做 Ceres/Kalibr per-IMU 对照，确认 outlier 来自 Ceres chain refinement、Kalibr init 导入、还是 staged 后变量释放。
3. 对 TUM 做 `fix-camera-chain-extrinsics` / camera-chain prior 对照，确认 loop error 与速度、精度之间的关系。
4. 做 Ceres 速度拆分和 stop policy ablation：固定迭代数、cost plateau、monotonic/nonmonotonic、IMU residual/Jacobian 成本。
