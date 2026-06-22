# Ceres 与 Kalibr Docker 多数据集速度精度对比

## 阅读定位

这篇是 Ceres/Kalibr 长周期 benchmark 的主索引，不是单一实验记录。它同时覆盖匿名基准、Kalibr 热启动、速度口径、扩展 IMU、多 camera/TUM 和多 IMU joint 定位。阅读时先看本节和“结论”，再按结果地图进入具体实验。

核心口径：

- 匿名基准独立标定：Ceres 不读取 Kalibr 结果，直接从本项目初始化链路求解。
- 热启动诊断：Ceres 读取 Kalibr 标定结果，只用于判断两套优化问题的固有距离。
- 速度对比：正式主线是 macOS native Ceres vs Kalibr Docker；Docker parity 只用于解释环境差异。
- TUM 和多 IMU：后续修正较多，历史失败保留为定位证据，不能脱离对应小节直接引用旧结论。

## 结论先行

| 对比项 | 当前结论 | 关键数字 | 结论边界 |
|---|---|---|---|
| 匿名基准独立标定 | Ceres 全部收敛，reprojection 与 Kalibr 基本一致 | 平均 reproj 差 `0.00068 px`；旋转最大 `0.0145°`；平移 `2.08-3.77 mm` | IMU 使用 Kalibr-compatible 裁边口径，不是 raw 全量 IMU |
| Kalibr 热启动 | 两套优化问题不是逐位相同 | 从 Kalibr 解出发仍漂 `0.19-2.63 mm` | 这是诊断口径，不代表默认部署路径 |
| 速度 | Ceres native 平均优化段快于 Kalibr Docker，但不是每组都快 | Ceres `89.3 s` vs Kalibr `128.2 s`，平均 `1.57x` | 墙钟包含 Docker 与非优化开销 |
| 扩展 IMU | 模型不再只是 smoke，已到全量数据证据 | `M_a/M_g` 相对差约 `1e-3` | accel residual 仍略高，平移弱可观 |
| TUM 双目 | 修正 camchain 初始化后通过 | Ceres 与 Kalibr 在 `0.06° / 1 mm` 内 | 早期失败来自 cam1 初值缺失 |
| 多 IMU joint | Kalibr 热启动 staged 口径已通过，冷启动仍有限制 | reproj `0.229823 px`，camera 差 `0.004655° / 0.0755 mm` | 不能外推为全自由冷启动已完成 |

## 结果地图

| 小节 | 回答的问题 | 应引用的结论 |
|---|---|---|
| 实验一：匿名基准独立标定 | 不读 Kalibr 时 Ceres 是否稳定 | 12 组全部收敛，外参和 reprojection 与 Kalibr 接近 |
| 实验二：热启动一致性 | 从同一 Kalibr 解出发是否保持不变 | 不保持，说明两套优化问题存在固有距离 |
| 实验三：速度结构 | Ceres 快在哪里，哪些速度口径不能混用 | native vs Docker 是当前正式口径；优化耗时和墙钟要分开 |
| 实验四：模型覆盖与扩展 IMU | 相机模型和扩展 IMU 是否有全量证据 | 扩展 IMU intrinsic 与 Kalibr 达到 `1e-3` 量级 |
| 实验五：多 camera 与 TUM single-stage | TUM 双目和 4IMU joint 的当前状态 | TUM 已通过；4IMU 旧失败已定位并有热启动通过口径 |
| 结论 / 复现命令 | 可复用的最终摘要和命令入口 | 优先引用这里，不直接引用历史失败段落 |

## 背景

`ceres-cam-imu` 的目标不是做一个依附 Kalibr 输出的后处理器，而是形成一条**可独立运行、可部署、可解释速度和精度边界**的 Ceres cam-IMU 标定链路。Kalibr 在这份文档里只承担两个角色：第一，作为已知强基线，帮助量化 Ceres 的精度差；第二，在部分输入格式还没有原生检测器时，提供角点导出/ROS bag 转换环境。

因此本文把三种口径明确拆开：

| 口径 | 用途 | 是否读取 Kalibr 结果参与求解 |
|---|---|---|
| **Ceres 独立标定** | 默认独立路径，从 Ceres 自己的 time/gravity/pose 初始化开始求解 | **否** |
| **Ceres 热启动标定** | 实验诊断，检查两套优化问题从同一点出发会漂多远 | 是，且显式使用 `--init-from-kalibr` |
| **Kalibr Docker 标定** | 精度/速度基线与扩展 IMU oracle | Kalibr 自身求解 |

## 目标

这份实验文档要回答四个问题。

1. **独立精度**：不读 Kalibr 结果时，匿名基准数据集能否全部收敛，外参、time-shift、reprojection 与 Kalibr 相差多少。
2. **优化问题一致性**：从 Kalibr 解热启动后，Ceres 会不会停在原地；若不会，固有差异有多大。
3. **模型覆盖**：新增相机模型、扩展 IMU 的 `M_a/M_g/A_g/C_g`、多 camera 链路是否有全量数据证据，而不只是单测。
4. **TUM multi-camera 真值对比**：TUM 双目数据上，Ceres 能否把 gyro/accel residual 提升到 Kalibr 同量级，并且外参、time-shift、gravity、IMU intrinsic 等全局设计变量离 TUM calibrated 真值有多远。

## 数据与口径

| 项 | 内容 |
|---|---|
| 匿名基准数据 | `<DATASET_DIR>/cam_imu`，统一使用同一类 YAML/CSV 输入模板 |
| TUM 数据 | `/ABS/TUM` 下两组 `dataset-calib-imu*_512_16` 双目 cam-IMU |
| Ceres | 当前正式评测使用 macOS 原生 `build/calibrate_cam_imu` Release；Ceres Docker 只保留为历史 parity probe，不作为后续主线速度/精度证据 |
| Kalibr | 当前正式评测使用 DockerHub `wang121ye/kalibr-camera-calibration:20.04` |
| 默认 Ceres 口径 | `--corner-defaults`，显式覆盖参数优先级更高 |
| 评测指标 | 外参平移差、外参旋转差、time-shift 差、reprojection/gyro/accel residual mean、墙钟 |

差异均为 Ceres 减 Kalibr。`T_c_b` 表示把点从 IMU/body frame 变到 camera frame 的外参；平移差为 `||t_C - t_K||`，旋转差为 `R_C R_K^T` 的测地角，time-shift 定义为 `t_imu = t_cam + tau`。

### IMU 裁边口径

匿名基准数据和扩展 IMU 表格使用的是 **Kalibr-compatible 有效 IMU 区间**，不是 raw `data1.csv` 的全量行数。原因是 Kalibr 的 `--imu_data_file` 路径默认 `trim_imu_edge_count=1000`，本仓库的 Kalibr Docker wrapper 也显式传入 `--trim-imu-edge-count 1000`；Ceres 的 `--corner-defaults` 对齐该口径，默认设置 `--imu-trim-edge-count 1000`。

具体规则是裁掉首尾边缘 IMU 样本，保留索引 `[1000, num_messages - 1000]`。例如某个匿名样例的 raw IMU 是 `24859` 行，参与 Kalibr/Ceres 优化与 residual 统计的是 `22860` 行。这样做是为了避免 spline 边界处缺少稳定支撑的 IMU 点进入优化；若要评估 raw IMU 全量口径，需要 Kalibr 和 Ceres 都显式设置 `--trim-imu-edge-count 0` 并作为新的实验表格单独报告。

## 实验一：匿名基准独立标定

这一组是默认独立路径：Ceres 不读取 Kalibr 的外参、time-shift、gravity 或 IMU intrinsic 作为初值。初始化来自 Ceres 自己的 gyro-norm time-shift prior、orientation/gravity prior、pose spline fit、bias zero init 和 joint 优化。表格中的 delta 在评测阶段与 Kalibr 结果文件离线比较得到。

本表的 IMU 约束与 residual 统计均采用上一节的裁边口径。相机角点不做对应裁边，仍按 corner CSV 中的全部有效观测统计 reprojection。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 优化/墙钟 | Kalibr 优化/墙钟 |
|---|---:|---:|---:|---|---:|---:|
| benchmark_01 | 3.37 mm | 0.0065° | -0.989 ms | 0.180273 / 0.179774 | 95.2s / 97.9s | 87.0s / 163.4s |
| benchmark_02 | 3.07 mm | 0.0046° | -0.803 ms | 0.180264 / 0.179743 | 97.1s / 99.9s | 131.6s / 212.0s |
| benchmark_03 | 2.96 mm | 0.0056° | -0.783 ms | 0.180307 / 0.180087 | 101.0s / 103.4s | 66.1s / 143.8s |
| benchmark_04 | 3.35 mm | 0.0074° | -1.391 ms | 0.179707 / 0.179124 | 98.6s / 101.4s | 131.5s / 206.1s |
| benchmark_05 | 3.38 mm | 0.0018° | -0.575 ms | 0.179185 / 0.178664 | 107.0s / 109.9s | 176.5s / 253.5s |
| benchmark_06 | 3.39 mm | 0.0039° | -1.122 ms | 0.177193 / 0.176607 | 155.0s / 159.5s | 87.4s / 184.1s |
| benchmark_07 | 2.23 mm | 0.0061° | -3.199 ms | 0.170813 / 0.171469 | 69.3s / 73.4s | 165.1s / 268.2s |
| benchmark_08 | 3.77 mm | 0.0145° | -6.267 ms | 0.172362 / 0.171172 | 67.6s / 71.6s | 163.9s / 244.1s |
| benchmark_09 | 2.30 mm | 0.0018° | -0.700 ms | 0.170432 / 0.170591 | 65.2s / 69.0s | 141.3s / 225.0s |
| benchmark_10 | 2.08 mm | 0.0000° | +1.339 ms | 0.171887 / 0.170795 | 63.9s / 67.8s | 100.7s / 176.7s |
| benchmark_11 | 2.45 mm | 0.0082° | +1.420 ms | 0.172836 / 0.171706 | 75.0s / 78.8s | 102.1s / 178.9s |
| benchmark_12 | 3.17 mm | 0.0121° | -6.189 ms | 0.172195 / 0.171249 | 76.4s / 80.3s | 185.5s / 261.7s |

结果来源：`out/docker_benchmarks/20260618_step4_native_ceres_no_param_stop/benchmark_single/summary.csv`。本次复跑显式追加 `--solver-absolute-parameter-tolerance -1`，避免 Ceres 被参数变化阈值提前停止。

**结论**：匿名基准集独立标定全部收敛；reprojection 与 Kalibr 基本一致，平均差 `0.00068 px`，最大差 `0.00119 px`。旋转差平均 `0.0060°`、最大 `0.0145°`；外参平移差平均 `2.96 mm`、最大 `3.77 mm`；time-shift 绝对差平均 `2.06 ms`，`benchmark_08/12` 约 `6.2 ms`，需要后续单独确认时钟先验或停止条件。

## 实验二：热启动一致性

热启动不是默认独立路径。它只回答一个诊断问题：如果把 Kalibr 的外参、time-shift、gravity 当初值，再用与实验一相同的 Ceres joint 优化器放开求解，最终会离 Kalibr 多远。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 墙钟 |
|---|---:|---:|---:|---|---:|
| benchmark_01 | 1.82 mm | 0.0275° | +0.071 ms | 0.1802 / 0.1798 | 36 s (32 it) |
| benchmark_02 | 2.63 mm | 0.0555° | +0.042 ms | 0.1802 / 0.1797 | 57 s (53 it) |
| benchmark_03 | 1.43 mm | 0.0045° | -0.009 ms | 0.1802 / 0.1801 | 59 s (55 it) |
| benchmark_04 | 1.14 mm | 0.0044° | +0.029 ms | 0.1795 / 0.1791 | 36 s (32 it) |
| benchmark_05 | 1.06 mm | 0.0019° | +0.001 ms | 0.1791 / 0.1787 | 39 s (35 it) |
| benchmark_06 | 0.30 mm | 0.0018° | +0.062 ms | 0.1770 / 0.1766 | 49 s (45 it) |
| benchmark_07 | 0.19 mm | 0.0014° | +0.030 ms | 0.1717 / 0.1715 | 53 s (51 it) |
| benchmark_08 | 0.46 mm | 0.0000° | +0.082 ms | 0.1715 / 0.1712 | 43 s (40 it) |
| benchmark_09 | 0.49 mm | 0.0000° | +0.071 ms | 0.1708 / 0.1706 | 47 s (44 it) |
| benchmark_10 | 0.70 mm | 0.0000° | +0.037 ms | 0.1711 / 0.1708 | 48 s (45 it) |
| benchmark_11 | 0.55 mm | 0.0043° | +0.036 ms | 0.1719 / 0.1717 | 91 s (89 it) |
| benchmark_12 | 0.43 mm | 0.0041° | +0.055 ms | 0.1715 / 0.1712 | 69 s (66 it) |

**结论**：Ceres 与 Kalibr 不是逐位相同的优化问题。从同一个 Kalibr 解出发，Ceres 仍会漂到 `0.19-2.63 mm` 后收敛；这就是两套实现的固有距离。实验一当前复跑的 `2.08-3.77 mm` 可以理解为“固有距离 + 独立初始化/收敛路径的额外代价”。

## 实验三：速度结构

这一节必须分清速度口径，否则会得出错误结论。后续 benchmark 至少记录两个时间：

| 时间 | 定义 | 记录方式 | 是否包含数据转换/读取 |
|---|---|---|---|
| 墙钟 | 外层命令从启动到退出的总耗时 | `run_docker_benchmark.py` 里的 `kalibr_elapsed_s` / `ceres_elapsed_s` | 包含 Docker 启动、文件复制/挂载、读取、建 problem、优化、写结果和报告 |
| 优化耗时 | 后端优化循环耗时 | Kalibr 日志 `Optimizing elapsed`；Ceres iteration table 最后一行 `total_time` | 不包含外层准备、比较脚本和输入格式转换；Ceres 也不包含 problem 构造前的读取 |

benchmark CSV 口径中，Ceres 原生二进制直接读取已经准备好的 YAML/CSV 输入，不会在求解命令内部调用 Kalibr Docker。`.bag`、`.pkl`、EuRoC 到 CSV/角点 CSV 的转换属于准备阶段，后续单独记录为 `prepare_elapsed_s`，不计入优化耗时。

| 口径 | Ceres 环境 | Kalibr 环境 | 这份文档原表是否属于该口径 | 结论边界 |
|---|---|---|---|---|
| 原生求解口径 | macOS arm64 Release 二进制 | linux/amd64 Docker | 是 | 只能说明当前本机部署时 Ceres 原生二进制快于 Kalibr Docker 模拟 |
| Docker parity 口径 | `kalibr-camimu-ceres-solver:20.04`，基于 Kalibr 镜像 | `wang121ye/kalibr-camera-calibration:20.04` | 否，2026-06-18 开始新增 | 用来比较同 Docker/OpenCV 环境下的速度与精度，不能复用原生表格的速度结论 |

原生求解口径下，Ceres 墙钟为 `81-131 s / 75-121 it`，Kalibr Docker 为 `145-261 s / 3-12 it`。这个速度对比不能解释为算法原生倍率，因为 Kalibr 是 amd64 Docker 模拟，而 Ceres 是 arm64 原生。

2026-06-18 曾新增 Docker parity probe，用来解释“为什么 Ceres Docker 没有复现旧表提速”。该 probe 现在降级为历史诊断，不作为后续实验主口径。用户已明确后续实验统一使用 **macOS 原生 Ceres vs Kalibr Docker**。

| 口径 | Kalibr 墙钟 | Kalibr 优化耗时 | Ceres 墙钟 | Ceres 优化耗时 | 迭代/停止 | 外参差 | time-shift 差 | reproj px (Ceres/Kalibr) |
|---|---:|---:|---:|---:|---|---:|---:|---|
| Docker parity probe | `178.21 s` | `92.010 s` | `162.31 s` | `154 s` | Ceres `94 it`，Kalibr `5 it` | `0.0065°`, `3.38 mm` | `-0.991 ms` | `0.18029 / 0.17977` |
| host-native Ceres probe | `178.21 s` | `92.010 s` | `100.34 s` | `97.4 s` | Ceres `94 it` | 见原生 sweep | 见原生 sweep | `0.18027 / 0.17977` |

这组数据把旧结论拆开了：

- Docker parity 口径下，Ceres Docker 的墙钟只比 Kalibr Docker 快约 `9%`，但优化耗时是 `154 s`，慢于 Kalibr 的 `92.010 s`。如果默认不统计读取/准备，Ceres Docker 当前不能复现实验文档里“明显更快”的结论。
- host-native Ceres 口径下，单组墙钟 `100.34 s`，相对同组 Kalibr Docker `178.21 s` 能复现旧表里 Ceres 明显更快的现象；但优化耗时 `97.4 s` 与 Kalibr `92.010 s` 同量级且略慢。因此这条提速主要来自原生执行和更低的非优化开销，不应表述为 Ceres 优化循环本身更快。
- Ceres Docker 为了和 Kalibr 对齐 OpenCV/系统环境，当前继承 Kalibr 的 linux/amd64 基镜像，在 Apple Silicon 上同样走 amd64 模拟；同时 Ceres 独立求解需要约 `94` 次迭代，Kalibr 同组约 `5` 次迭代。Ceres 单步便宜的优势被迭代次数和模拟环境抵消。

因此后续速度表的正式主线固定为：**Ceres 原生 macOS Release 二进制**记录 Ceres 墙钟/优化耗时，**Kalibr DockerHub 镜像**记录 Kalibr 墙钟/优化耗时。Docker parity 只在需要解释环境差异时作为附录数据。

本次 12 组匿名基准复跑中，Kalibr 平均优化耗时 `128.2 s`，Ceres 平均优化耗时 `89.3 s`，平均速度比 `Kalibr/Ceres = 1.57x`。这个速度优势不均匀：`benchmark_01/03/06` 的 Ceres 优化段慢于 Kalibr，`benchmark_07/08/09/12` 的 Ceres 优化段约 `2.2-2.4x` 快于 Kalibr。墙钟口径下，Ceres 原生二进制仍明显短于 Kalibr Docker，但这包含 Docker 启动、读取、报告生成和 Rosetta/amd64 模拟等非优化因素。

更有价值的是结构差异：Ceres 单步便宜但迭代多，墙钟主要跟 problem size 和停止条件走；Kalibr 单步贵但迭代少，墙钟主要受绝对停止条件和模拟环境波动影响。较大问题规模样例的 camera residual 约 `662k`，原生 Ceres 用时 `111-131 s`；较小问题规模样例约 `582k`，原生 Ceres 用时 `81-89 s`，组内稳定、组间清晰。

### 停止条件口径

Kalibr 的 cam-IMU 优化使用三类停止条件：最大迭代数、`deltaX <= 1e-2`、`|deltaJ| <= 1`。后端循环的逻辑等价于三者满足其一即可退出；例如 `benchmark_01` 的扩展 IMU run 在第 8 次迭代 `dJ=0.303 < 1` 时停止，当时 `deltaX=0.027` 仍大于 `1e-2`。

Ceres 也补了三类停止入口，但只对齐**停止结构**，不逐值复用 Kalibr 的 `J/dJ/deltaX`。原因有三点：

- `J/dJ` 是优化器内部目标值。Kalibr 使用 aslam backend 的目标统计，Ceres 报告的是自身 loss convention 下的 cost；两边的 robust loss、先验项和归一化不逐项相同。
- `deltaX` 也不是同一个量。Kalibr 统计最小维度设计变量更新向量的 max coefficient；Ceres 当前 callback 统计 active parameter block 的实际最大系数变化，只是更接近 `deltaX` 的工程代理量。
- residual parity 不要求内部 `J` parity。实验四说明扩展 IMU 的 reprojection/gyro/accel residual 和 `M_a/M_g` 已到同量级，但这不能推出两边 `dJ=1` 代表相同收敛程度。

`benchmark_01` 扩展 IMU 的 smoke 结果说明，Ceres 若直接照搬 Kalibr 的 `dJ=1` 会提前退出：

| 口径 | 停止点 | 墙钟 | 结果判断 |
|---|---|---:|---|
| Kalibr 扩展 IMU | 8 it，`dJ=0.303 < 1`，`deltaX=0.027` | optimize `160 s` | 作为 residual 与扩展 IMU intrinsic 基线 |
| Ceres 直接用 `dJ=1` | 49 it，`cost_change=0.951` | `60 s` | 过早；外参旋转差 `0.31°`，平移差 `12.8 mm`，gravity 差 `0.092` |
| Ceres 用参数变化 `1e-2` | 107 it，`parameter_delta=0.00987` | `122 s` | residual 同量级，`M_a/M_g` 相对差 `1e-3` 量级 |
| Ceres 跑满 150 it | max-iteration reference | `168 s` | delta 最稳，但时间增加 |

因此当前 `--corner-defaults` 采用默认 preset：保留最大迭代数、绝对 cost change 和参数变化三类停止条件，但把 cost change 收紧到 `5e-2`，并保留已验证的参数变化阈值 `1e-2`。这解释了为什么 Ceres 和 Kalibr 的 residual 可以对齐，而日志里的 `J/dJ/deltaX` 不应该逐数值比较。

## 实验四：模型覆盖与扩展 IMU

相机侧已补齐 `pinhole+radtan/equidistant/fov/none`、`omni+radtan/none`、`eucm`、`ds` 的读取和投影 Jacobian；`tests/test_math.cpp` 对 `projectWithJacobian()` 做中心差分验证，`check_dataset` 会打印实际读取到的 camera/distortion model，避免 YAML 被静默当成默认模型。

IMU 侧新增两类模型：

| Ceres 参数 | 对应含义 |
|---|---|
| `--imu-model scale-misalignment` | `M_a/M_g/A_g`，对应 scale/misalignment 与 gyro sensing rotation |
| `--imu-model scale-misalignment-size-effect` | 在上面基础上增加 size-effect 的 accelerometer sensing-axis offset `C_g` |

扩展 gyro、scale/misaligned accel 和 size-effect accel 均已从数值差分切到手写 `SizedCostFunction`。`ctest --test-dir build --output-on-failure` 覆盖普通 IMU residual、扩展 IMU Kalibr 源码公式等价、扩展 IMU 解析 Jacobian 与中心差分复核。

三组匿名基准数据的全量 `scale-misalignment` 对比：

这里的“全量”指相机角点全量与 Kalibr-compatible 有效 IMU 区间全量；IMU 不是 raw `data1.csv` 全量行数。

| 数据集 | residual mean Ceres/Kalibr | 外参差 | time-shift 差 | `M_a` rel | `M_g` rel | `A_g` fro | `C_g` fro |
|---|---|---:|---:|---:|---:|---:|---:|
| `benchmark_01` | `0.18006/0.18081 px`, `0.01669/0.01746 rad/s`, `0.11228/0.10409 m/s^2` | `0.082°`, `2.05 mm` | `-1.26 ms` | `8.54e-4` | `4.94e-4` | `4.71e-4` | `2.03e-3` |
| `benchmark_06` | `0.17663/0.17722 px`, `0.01596/0.01673 rad/s`, `0.11395/0.10608 m/s^2` | `0.069°`, `1.85 mm` | `-1.20 ms` | `7.19e-4` | `5.26e-4` | `4.62e-4` | `1.75e-3` |
| `benchmark_10` | `0.17208/0.18885 px`, `0.01385/0.01412 rad/s`, `0.07172/0.06555 m/s^2` | `0.053°`, `6.13 mm` | `-1.31 ms` | `6.09e-4` | `1.45e-3` | `9.22e-4` | `2.05e-3` |

**结论**：扩展 IMU 不再只是模型级 smoke。`M_a/M_g` 相对差在 `1e-3` 量级，`A_g/C_g` Frobenius 也在 `1e-3` 量级；reprojection 和 gyro 基本追平 Kalibr，accelerometer residual 仍略高，弱可观外参平移在毫米级漂移。

## 实验五：多 camera 与 TUM single-stage

> 2026-06-18 定位：TUM 双目失败根因是 native Ceres runner 没有给多 camera 传 `--init-from-camchain`，导致 cam1 从默认 identity 外参启动并收敛到 180 度翻转。已在应用层和 runner 中修复。后续 4-IMU joint 失败继续定位到 Kalibr IRLS M-estimator 与 Ceres 标准 robust loss 的线性化差异，2026-06-19 benchmark 热启动口径已通过，见下文。

### 旧多 IMU 失败记录

使用 `2025_03_14_00_10_18` 同一次机械臂数据，输入为 1 路 camera corners 和 4 路 IMU CSV。结果来源：`out/docker_benchmarks/20260618_step4_native_ceres_no_param_stop/benchmark_multi_imu/summary.csv`。

| 模式 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 优化 | Kalibr 优化 | 判断 |
|---|---:|---:|---:|---|---:|---:|---|
| joint_4imu | 156.72 mm | 0.0785° | -11.726 ms | 54.758200 / 0.236473 | 835.0s | 155.1s | 失败 |
| single_imu1 | 3.37 mm | 0.0065° | -0.989 ms | 0.180273 / 0.179774 | 94.7s | 86.0s | 通过 |
| single_imu2 | 3.32 mm | 0.0048° | -0.873 ms | 0.180560 / 0.180073 | 88.0s | 150.9s | 通过 |
| single_imu3 | 4.12 mm | 0.0184° | -0.665 ms | 0.179557 / 0.179053 | 104.0s | 65.4s | 通过 |
| single_imu4 | 4.05 mm | 0.0062° | -1.707 ms | 0.179983 / 0.179159 | 103.0s | 173.0s | 通过 |

这组记录保留为失败证据：4 路 IMU 独立 cam-IMU 标定均与 Kalibr 对齐，说明各路 IMU 数据本身可用；但当时 native Ceres 的 4-IMU joint 标定没有收敛到正确解，reprojection 均值达到 `54.76 px`，平移差 `15.7 cm`。后续定位表明，这不是多 IMU 数据不可用，而是 joint 释放 camera/time 时 robust loss 线性化与 Kalibr 不一致。

### 当前 TUM 双目复跑结果

TUM 使用两组数据：`dataset-calib-imu1_512_16.bag` 和 `dataset-calib-imu2_512_16` EuRoC。Kalibr 输入已修正为 `/out/input/*.bag` 可写路径，保证结果文件能写出。Ceres 结果为在同一批 Kalibr 结果上手工加 `--init-from-camchain` 后复跑：

- `out/ceres_sweeps/20260618_tum_imu1_init_from_camchain/result.yaml`
- `out/ceres_sweeps/20260618_tum_imu2_init_from_camchain/result.yaml`

| 数据 | Kalibr optimize | Ceres optimize | cam0 差 | cam1 差 | time-shift 差 | reproj px (Ceres/Kalibr) | 判断 |
|---|---:|---:|---:|---:|---:|---|---|
| tum_imu1_bag | 38.9s | 14.1s | 0.0397° / 0.858 mm | 0.0540° / 0.857 mm | -0.046 ms / -0.054 ms | 0.09494 / 0.10028 | 通过 |
| tum_imu2_euroc_kalibr_export | 39.7s | 13.2s | 0.0241° / 0.345 mm | 0.0318° / 0.149 mm | +0.003 ms / +0.008 ms | 0.09374 / 0.10024 | 通过 |

结论：TUM 双目通过。旧失败结果中 cam1 接近 `180° / 1.2 m` 的翻转不是 corner 导出或投影 residual 坐标系问题，而是多 camera 初值缺失。修正后两个 TUM 数据集的双目外参都与 Kalibr 在 `0.06° / 1 mm` 内对齐，reprojection 与 Kalibr 同量级且略低。

对应代码修正：

- `calibrate_cam_imu`：多 camera 且未从完整 prior 初始化时，自动从 camchain 读取每个 camera 的 `T_cam_imu`。
- `tools/run_docker_benchmark.py`：TUM Ceres 命令显式传 `--init-from-camchain`。
- `tools/prepare_ceres_inputs.py`：多 camera `--run-calibration` / `--run-two-stage` 自动追加 `--init-from-camchain`。

### 当前 4-IMU joint 定位

4-IMU joint 原始失败不是数据不可用：四路单 IMU 独立标定均通过。2026-06-18 进一步把问题拆成三类变量：IMU chain 初值、camera extrinsic、camera-to-IMU time shift。源码复核后确认，当前 kalibr-docker 的 `corner_file` 路径会设置 `CauchyMEstimator(10)`；Kalibr/aslam_backend 的 M-estimator 是 IRLS 口径：用 `sqrt(weight)` 同时缩放 residual 和 Jacobian，不把权重导数写入 Hessian。Ceres 标准 `CauchyLoss`/`HuberLoss` 不是这个线性化语义。

关键修正是：本项目把 Cauchy/Huber 换成 Kalibr-style M-estimator loss，`rho' = weight`、`rho'' = 0`。这样 Ceres 的正规方程线性化与 Kalibr IRLS 更一致；旧的 “Cauchy 宽度已经一致，所以 robust 不是主因” 结论被这次复核推翻。

诊断结果：

| Ceres 对照 | reproj px | 外参差 | time-shift 差 | 结论 |
|---|---:|---:|---:|---|
| 原始 runner：Cauchy + time prior + pose motion prior + identity IMU chain | 54.758 | 156.7 mm / 0.0785° | -11.73 ms | 失败 |
| Kalibr 全量热启动 + Cauchy + 固定 camera/time + 30 iter | 0.230 | 0.0 mm / 0.0034° | 0 ms | 相机模型、投影 residual、Kalibr result 解析基本对齐 |
| Kalibr 全量热启动 + Ceres 标准 Cauchy + staged `pbg,pbegt` | 2.304 | 1.36 mm / 0.0780° | +11.12 ms | 释放 camera/time 后 time shift 漂移 |
| Kalibr 全量热启动 + Cauchy + staged `pbg,pbge` | 12.850 | 33.28 mm / 0.0936° | 0 ms | 只释放 camera extrinsic 也会牺牲相机 residual |
| Kalibr 全量热启动 + all no-loss + staged `pbg,pbgt` | 2.499 | 0.0 mm / 0.0034° | +15.09 ms | 只释放 time shift 会漂移 |
| Kalibr 全量热启动 + all no-loss + 非 staged 全自由 | 15.396 | 0.342 mm / 0.0299° | +17.32 ms | 即使无 robust，直接全自由仍不稳定 |
| Kalibr 全量热启动 + Kalibr-style M-estimator + staged `pbg,pbegt` | 0.229823 | 0.0755 mm / 0.00465° | +0.4727 ms | 通过，camera/time 可释放 |

当前 runner 已恢复为多 IMU joint benchmark 口径：Kalibr 先跑出 reference result，Ceres joint 使用 `--kalibr-result --init-from-kalibr --staged --stage-free pbg,pbegt --stage-iterations 30,30`，并移除原来的 time prior 和 pose motion prior。该口径的验证结果为：

| 指标 | Kalibr | Ceres staged `pbg,pbegt` | 差异 |
|---|---:|---:|---:|
| reproj mean | 0.236473 px | 0.229823 px | -0.006651 px |
| camera rotation | - | - | 0.004655° |
| camera translation | - | - | 0.0755 mm |
| camera time shift | -0.0842046 s | -0.0837319 s | +0.4727 ms |
| gyro mean | 0.075923 rad/s | 0.106322 rad/s | +0.030398 rad/s |
| accel mean | 0.339126 m/s² | 0.596738 m/s² | +0.257612 m/s² |

这个结果说明多 IMU residual 连接、Kalibr `T_ib` 解析、相机 residual、staged 执行路径和 camera/time 释放已经能在 Kalibr 热启动 benchmark 口径下与 Kalibr 对齐。当前结论仍限定在 benchmark 等价验证：Ceres 使用 Kalibr joint 结果作为全量初值，不等同于证明冷启动全自由多 IMU joint 也已经稳定。

### 历史 smoke 背景

Kalibr cam-IMU 支持多 camera：`kalibr_calibrate_imu_camera --cams <camchain.yaml>` 会通过 `IccCameraChain` 构建 camera chain，并为每个 camera 加重投影误差、time shift 和 camera-chain baseline。Ceres 当前也支持**一个共享 camchain + 多个 `--corners` CSV** 的 joint 优化，结果 YAML 写出 `camera_chain`；staged 分支已接入多 camera/multi-IMU problem build，但正式 TUM 结果仍来自当前验证过的 `--init-from-camchain` single-stage 口径。

TUM 双目使用同一个 `--cam <camchain.yaml>`，并传两个 `--corners`。如果误传两个相同的 `--cam`，两路会都按 `cam0` section 读取，cam1 投影会错误；这属于 CLI 约定问题，不是多相机优化缺失。

早期二阶段诊断说明，20/10 kps 的低频 pose/bias 轨迹承载不了 TUM 高频 IMU；这不是数据读取、multi-camera、扩展 IMU 前向公式或 `M_a/M_g` 的问题。最终保留的实验口径是**不读 Kalibr result 的 100/50 kps single-stage joint 优化**。

### TUM 真值与变量口径

TUM 官方说明 512x512 calibrated/exported 数据已经做过一致时间戳、IMU scaling 和 axis alignment；本实验因此把 calibrated camchain 的 `T_cam_imu` 当作外参真值，把 camera-IMU time shift 真值按 `0 ms` 处理。本地真值文件统一按 `/ABS/TUM/<dataset>/dso/camchain.yaml` 或转换输出中的 staging copy 读取。官方下载入口见 `https://vision.in.tum.de/tumvi/exported/euroc/512_16/`。

gravity 的方向在结果文件里表达于 AprilGrid target 坐标系。除非额外知道 AprilGrid target 相对 MoCap/world 的真值姿态，否则不能把 gravity 向量方向直接和 TUM world 真值比较；这里只把 gravity 模长和标准重力 `9.80665 m/s^2` 比较，并额外报告 Ceres 与 Kalibr 之间的向量差。

本实验使用 `--imu-model scale-misalignment`，所以全局设计变量还包括 `M_a/M_g/A_g/C_g`。TUM calibrated 数据已做 IMU scaling/axis alignment，因此这组变量的真值按 corrected measurement space 的 `M_a=I`、`M_g=I`、`A_g=0`、`C_g=I` 做 sanity 对比；这不同于官方 Raw Data 段落里的 raw sensor correction matrix。

### 变量解释与读数方式

下面几张表混合了三类量：残差质量、全局标定变量对真值的误差、Ceres 与 Kalibr 两个 solver 的直接差异。除最后一张“Ceres 与 Kalibr 的直接差异”表外，所有 `rot/trans/time` 都是 **solver 结果减 TUM calibrated 真值**。

| 字段 | 定义 | 背景与读数方式 |
|---|---|---|
| `Kalibr residual mean` / `Ceres single-stage residual mean` | 重投影、gyro、accel residual 的均值 | 重投影单位是 `px`；gyro 单位是 `rad/s`；accel 单位是 `m/s^2`。它们衡量优化问题最终解释观测的能力，不直接等价于外参真值误差。 |
| `Ceres solver` | Ceres 迭代数、墙钟和终止状态 | 用来确认 single-stage run 是否正常收敛。这里报告的是 Ceres 原生二进制的墙钟，不应和 Kalibr Docker 的墙钟做严格原生算法倍率比较。 |
| `cam0 rot` / `cam1 rot` | 每个 camera 的 `T_cam_imu` 旋转误差 | 使用 `R_est R_truth^T` 的 SO(3) 测地角，单位是度。数值越小，说明该 camera 相对 IMU 的方向越接近 TUM calibrated camchain。 |
| `cam0 trans` / `cam1 trans` | 每个 camera 的 `T_cam_imu` 平移误差 | 使用 `||t_est - t_truth||`，单位是 `mm`。这是 camera 光心相对 IMU/body 的平移差。 |
| `baseline rot` / `baseline trans` | 双目相机之间相对外参的误差 | baseline 定义为 `T_cam1_cam0 = T_cam1_imu * inv(T_cam0_imu)`，也就是 cam0 到 cam1 的 stereo baseline。它检查多 camera chain 内部几何是否保持正确；如果 cam0/cam1 一起相对 IMU 漂移，baseline 仍可能很小。 |
| `cam0 time` / `cam1 time` | 每个 camera 的 camera-to-IMU time shift 误差 | time shift 定义为 `t_imu = t_cam + tau`。TUM calibrated 真值按硬同步 `tau=0` 处理，所以表里的值就是 solver 估计的 `tau`。负值表示对应 IMU 时间比 camera 时间更早。 |
| `gravity` | 优化得到的重力向量 | 该向量在 AprilGrid target 坐标系里表达，用于解释 accel residual。没有 target-to-world 真值姿态时，方向不能直接和 TUM world gravity 对比。 |
| `||g|| - 9.80665` | 重力模长误差 | 只比较模长和标准重力常数的差，单位是 `m/s^2`。本实验里两边都被 gravity-length manifold 约束到几乎相同的模长。 |
| `||M_a-I||_F` | accelerometer scale/misalignment 矩阵离单位阵的 Frobenius 范数 | `M_a` 作用在 accel prediction 外侧。TUM calibrated 数据已经做过 IMU scaling/axis alignment，所以 sanity 真值按单位阵处理。 |
| `||M_g-I||_F` | gyroscope scale/misalignment 矩阵离单位阵的 Frobenius 范数 | `M_g` 作用在 gyro prediction 外侧。数值反映 solver 在 calibrated measurement space 里又估出的剩余 scale/misalignment。 |
| `||A_g||_F` | gyro g-sensitivity 矩阵的 Frobenius 范数 | `A_g` 表示加速度对 gyro 测量的耦合项，真值按零矩阵处理。 |
| `angle(C_g,I)` | gyro sensing rotation 相对单位阵的角度 | `C_g` 是 gyro sensing axis rotation。表里用旋转角表示它偏离单位阵多少。 |
| `gravity delta` | Ceres gravity 与 Kalibr gravity 的向量差 | 这是最后一张直接差异表里的字段，计算 `||g_ceres - g_kalibr||`，不是对 TUM world 真值的误差。 |

历史结果来源：

| 数据 | Kalibr result | Ceres result |
|---|---|---|
| `dataset-calib-imu1_512_16` | `out/kalibr_runs/tum_imu1_ext_sm_writable/input/dataset-calib-imu1_512_16-results-imucam.txt` | `out/ceres_sweeps/tum_single_stage_smoke_20260617/imu1_highkps_all_free_80iter/result.yaml` |
| `dataset-calib-imu2_512_16` | `out/kalibr_runs/tum_imu2_ext_sm_writable/input/dataset-calib-imu2_512_16-results-imucam.txt` | `out/ceres_sweeps/tum_single_stage_smoke_20260617/imu2_highkps_all_free_80iter/result.yaml` |

| TUM 数据 | Kalibr residual mean | Ceres single-stage residual mean | Ceres solver | 判断 |
|---|---|---|---|---|
| `dataset-calib-imu1_512_16` | `0.10646-0.10737 px`, `0.00119649 rad/s`, `0.02146520 m/s^2` | `0.10352 px`, `0.00118534 rad/s`, `0.02121921 m/s^2` | 27 iter, 15.9 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |
| `dataset-calib-imu2_512_16` | `0.10708-0.10702 px`, `0.00116924 rad/s`, `0.02134775 m/s^2` | `0.10321 px`, `0.00121503 rad/s`, `0.02117489 m/s^2` | 28 iter, 17.3 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |

外参、baseline 和 time shift 对真值的差异：

| 数据集 | Solver | cam0 rot | cam0 trans | cam1 rot | cam1 trans | baseline rot | baseline trans | cam0 time | cam1 time |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `dataset-calib-imu1_512_16` | Kalibr | `0.10985°` | `1.667 mm` | `0.10976°` | `1.667 mm` | `0.00393°` | `0.000 mm` | `-0.1663 ms` | `-0.1685 ms` |
| `dataset-calib-imu1_512_16` | Ceres | `0.12423°` | `1.814 mm` | `0.08938°` | `1.597 mm` | `0.03658°` | `0.286 mm` | `-0.1554 ms` | `-0.1555 ms` |
| `dataset-calib-imu2_512_16` | Kalibr | `0.07133°` | `2.098 mm` | `0.07150°` | `2.098 mm` | `0.00000°` | `0.000 mm` | `-0.1919 ms` | `-0.1929 ms` |
| `dataset-calib-imu2_512_16` | Ceres | `0.07093°` | `2.157 mm` | `0.05128°` | `1.961 mm` | `0.04377°` | `0.346 mm` | `-0.1735 ms` | `-0.1716 ms` |

gravity 与扩展 IMU 设计变量对真值的差异：

| 数据集 | Solver | gravity | `||g|| - 9.80665` | `||M_a-I||_F` | `||M_g-I||_F` | `||A_g||_F` | `angle(C_g,I)` |
|---|---|---|---:|---:|---:|---:|---:|
| `dataset-calib-imu1_512_16` | Kalibr | `[0.039184, -9.698127, -1.453690]` | `-0.000100` | `6.30e-3` | `2.83e-3` | `1.33e-3` | `0.1300°` |
| `dataset-calib-imu1_512_16` | Ceres | `[0.038603, -9.698098, -1.453900]` | `-0.000100` | `6.12e-3` | `2.66e-3` | `1.40e-3` | `0.1342°` |
| `dataset-calib-imu2_512_16` | Kalibr | `[0.034087, -9.697015, -1.461220]` | `-0.000100` | `5.95e-3` | `2.67e-3` | `7.91e-4` | `0.1296°` |
| `dataset-calib-imu2_512_16` | Ceres | `[0.034050, -9.696857, -1.462267]` | `-0.000100` | `5.74e-3` | `2.55e-3` | `8.22e-4` | `0.1117°` |

Ceres 与 Kalibr 的直接差异：

| 数据集 | cam0 rot | cam0 trans | cam1 rot | cam1 trans | cam0 time | cam1 time | gravity delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| `dataset-calib-imu1_512_16` | `0.03332°` | `0.286 mm` | `0.03492°` | `0.172 mm` | `+0.0109 ms` | `+0.0130 ms` | `0.000618 m/s^2` |
| `dataset-calib-imu2_512_16` | `0.02060°` | `0.203 mm` | `0.03081°` | `0.209 mm` | `+0.0185 ms` | `+0.0213 ms` | `0.001060 m/s^2` |

**历史结论状态**：2026-06-17 smoke 曾显示 TUM single-stage residual 与 Kalibr 同量级；2026-06-18 正式 runner 初次复跑失败后，已定位为多 camera 未从 camchain 初始化 cam1 外参。补 `--init-from-camchain` 后，两组 TUM 双目数据重新回到与 Kalibr 同量级，旧失败结果只作为定位线索保留。

## 输入格式与独立性

`calibrate_cam_imu` 标定二进制本身读取统一中间格式：camchain/IMU/target YAML、IMU CSV、一个或多个角点 CSV、可选 corner poses CSV。它不需要 Kalibr Docker，也不需要先读取 Kalibr 标定结果。

当前外层转换入口是 `tools/prepare_ceres_inputs.py`，支持：

| 输入 | 当前方式 | 是否依赖 Kalibr Docker/ROS |
|---|---|---|
| `pkl` | `export_kalibr_corners.py` 转角点 CSV | 转换依赖，求解不依赖 |
| `bag` | `export_kalibr_bag_to_ceres.py` 转角点/时间戳/IMU CSV | 转换依赖，求解不依赖 |
| `euroc` / TUM `mav0` | 默认用 cpp_tools AprilTag/camera model 原生导出角点与 IMU CSV；`--euroc-backend kalibr-docker` 保留为基线 | 默认转换不依赖，Kalibr backend 依赖 |

`--run-calibration` 可一键完成转换后单阶段标定，并默认补上 single-stage preset；`--run-two-stage` 保留为 TUM/轨迹频率诊断入口。EuRoC 已经可以走 native 导出；当前仍依赖 Kalibr Docker 的主要是 Kalibr pickle 和 ROS bag 转 CSV。

## 结论

- **Ceres 独立标定已经覆盖匿名基准集**：全部 `CONVERGENCE`，reprojection 与 Kalibr 基本一致，外参旋转最大 `0.0145°`，外参平移 `2.08-3.77 mm`。
- **匿名基准表格采用 Kalibr-compatible IMU 裁边口径**：Kalibr `--imu_data_file` 与 Ceres `--corner-defaults` 均裁掉首尾 `1000` 个 IMU 样本；这些结论不能直接解释为 raw IMU 全量行数实验。
- **热启动证明两套优化问题不是逐位相同**：从 Kalibr 解出发仍会漂 `0.19-2.63 mm`，这解释了独立口径中剩余毫米级差异的下限。
- **停止条件对齐结构，不对齐内部数值**：Ceres 已有 max-iteration、absolute cost change、parameter delta 三类停止条件；`J/dJ/deltaX` 的定义与 Kalibr 不同，所以不能直接照搬 Kalibr 的 `dJ=1`。
- **扩展 IMU 有全量证据**：`M_a/M_g` 相对差约 `1e-3`，`A_g/C_g` Frobenius 约 `1e-3`，并且已切到手写解析 Jacobian。
- **多 IMU joint 在 Kalibr 热启动 benchmark 口径通过**：4 路 IMU 分别与同一 camera 独立标定均能贴近 Kalibr；原始全自由 4-IMU joint reprojection 达 `54.76 px`。定位后确认主因是 Ceres 标准 robust loss 与 Kalibr IRLS M-estimator 线性化不等价；改为 Kalibr-style loss 后，`--init-from-kalibr --staged --stage-free pbg,pbegt` 可释放 camera/time，reprojection `0.229823 px`，camera 差 `0.004655° / 0.0755 mm`，time-shift 差 `0.4727 ms`。
- **多 camera/TUM 已通过当前修正口径**：TUM 双目失败根因是缺少 camchain 外参初值；补 `--init-from-camchain` 后，native Ceres reprojection 为 `0.0937-0.0949 px`，cam0/cam1 均与 Kalibr 在 `0.06°/1 mm` 内对齐。
- **速度结论需按当前正式口径解释**：后续主线固定为 macOS native Ceres vs Kalibr Docker；本轮 12 组平均优化耗时为 Ceres `89.3 s`、Kalibr `128.2 s`，但 Ceres 并非每组都更快，墙钟优势也包含 Kalibr Docker 的非优化开销。

## 复现命令

Ceres 独立标定单数据集示例，`<DS>` 替换为具体 `cam_imu` 目录：

```bash
build/calibrate_cam_imu --corner-defaults \
  --cam <DS>/cam0-camchain-640x400.yaml --imu <DS>/imu.yaml --target <DS>/aprilgrid.yaml \
  --imu-data <DS>/data1.csv --corners <DS>/cam0_640x400_corners.csv \
  --corner-poses <DS>/cam0_640x400_corner_poses.csv \
  --estimate-time-shift-prior --estimate-orientation-gravity-prior \
  --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 \
  --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 \
  --max-iterations 150 --solver-max-trust-region-radius 10000000 \
  --output-result /tmp/ceres_independent.yaml
```

`--corner-defaults` 会对齐当前 Kalibr-compatible 口径，其中包含 `--imu-trim-edge-count 1000`。若要跑 raw IMU 全量对照实验，需要显式追加 `--imu-trim-edge-count 0`，并确保 Kalibr 也使用相同设置。

评测 delta 另跑：

```bash
build/compare_kalibr_result \
  --kalibr-result <DS>/cam0_640x400_corners-1-results-imucam.txt \
  --ceres-result /tmp/ceres_independent.yaml
```

热启动诊断只用于实验：

```bash
build/calibrate_cam_imu --corner-defaults \
  --cam <DS>/cam0-camchain-640x400.yaml --imu <DS>/imu.yaml --target <DS>/aprilgrid.yaml \
  --imu-data <DS>/data1.csv --corners <DS>/cam0_640x400_corners.csv \
  --corner-poses <DS>/cam0_640x400_corner_poses.csv \
  --kalibr-result <DS>/cam0_640x400_corners-1-results-imucam.txt --init-from-kalibr \
  --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 \
  --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 \
  --max-iterations 150 --solver-max-trust-region-radius 10000000
```

批量本地数据集的单行命令见 `docs/常用命令.txt` 的占位符模板。
