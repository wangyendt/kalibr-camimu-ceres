# Docker 速度口径与 benchmark 暂停记录

## 背景

2026-06-18 开始把 Ceres solver 封装进 `docker/Dockerfile`，基于 `wang121ye/kalibr-camera-calibration:20.04`，并用 `gettool ceres -v 2.2.0` 构建 Ceres。这个镜像的目标是让公开仓库用户在没有本地 Kalibr 工程的情况下复现实验，同时保证 Ceres 与 Kalibr 使用接近的系统/OpenCV 环境。

用户随后明确：后续实验主口径改为 **macOS 原生 Ceres vs Kalibr Docker**。因此 Docker parity probe 只保留为解释环境差异的历史诊断，不再作为正式 benchmark 主线。

## 当前状态

| 状态 | 内容 | 证据 |
|---|---|---|
| done | Ceres solver Docker 构建成功 | `docker/Dockerfile`，镜像 `kalibr-camimu-ceres-solver:20.04` |
| done | 第一组完整 Docker parity probe 跑通 | `out/docker_benchmarks/20260618_step1_probe/summary.csv` |
| done | benchmark runner 已拆出优化耗时字段 | `tools/run_docker_benchmark.py` 输出 `kalibr_optimize_s` / `ceres_optimize_s` |
| done | host-native Ceres 单组 timing probe 跑通 | `out/ceres_sweeps/20260618_native_timing_probe/summary.csv` |
| done | 正式口径已切到 native Ceres vs Kalibr Docker | `tools/run_docker_benchmark.py --ceres-mode native` |
| done | 12 组 benchmark single-IMU 已完成 | `out/docker_benchmarks/20260618_step4_native_ceres_no_param_stop/benchmark_single/summary.csv` |
| done | benchmark multi-IMU 一次数据已完成 | `out/docker_benchmarks/20260618_step4_native_ceres_no_param_stop/benchmark_multi_imu/summary.csv` |
| done | TUM retry 已完成，Kalibr 可写输出路径已修复 | `out/docker_benchmarks/20260618_step5_tum_retry_native_ceres_vs_kalibr_docker/tum/summary.csv` |
| invalid | `out/docker_benchmarks/20260618_step1_docker/benchmark_single/benchmark_single_summary.csv` 是中断残缺结果 | 第 1/2 组 return code 为 `137`，不能写入正式实验结论 |

## 速度口径

| 口径 | 说明 | 当前判断 |
|---|---|---|
| 墙钟 | 外层命令总耗时，包含 Docker 启动、读取、建 problem、优化、写结果和报告 | 用于描述用户实际等多久 |
| 优化耗时 | Kalibr `Optimizing elapsed` 或 Ceres iteration table 最后一行 `total_time` | 用于排除输入转换、读取和报告开销后比较 solver 优化段 |
| 原生求解口径 | Ceres 作为本机 Release 二进制运行，Kalibr 仍在 amd64 Docker 中运行 | 旧文档表格属于这个口径；可以说明当前机器上 Ceres 原生部署有墙钟收益 |
| Docker parity 口径 | Ceres Docker 与 Kalibr Docker 都在 linux/amd64 Docker 环境下运行 | 2026-06-18 probe 属于这个口径；第一组 Ceres Docker 只保留轻微墙钟优势，优化段反而慢 |

当前正式结论：

| 项 | 结果 |
|---|---|
| benchmark single-IMU | 12/12 收敛；平均旋转差 `0.0060°`，平均平移差 `2.96 mm`，平均 reproj delta `0.00068 px` |
| benchmark single-IMU 速度 | Ceres 平均优化 `89.3 s`，Kalibr 平均优化 `128.2 s`；Ceres 并非每组都更快 |
| benchmark 4-IMU joint | Kalibr 热启动 benchmark 口径已通过；Kalibr-style M-estimator + staged `pbg,pbegt` 后 reproj `0.229823 px`，camera 差 `0.004655° / 0.0755 mm`，time-shift 差 `0.4727 ms` |
| 4 路 IMU 独立 cam-IMU | 均通过；平移差 `3.3-4.1 mm`，reproj delta `<0.001 px` |
| TUM 双目 | 已定位并修复；补 `--init-from-camchain` 后两组 TUM reproj `0.0937-0.0949 px`，cam0/cam1 均与 Kalibr 在 `0.06°/1 mm` 内对齐 |
| TUM Kalibr retry | 通过；Kalibr 结果文件写出到 `/out/input/*-results-imucam.txt` |

## 最新定位记录

| 项 | 结论 | 证据 |
|---|---|---|
| TUM multi-camera | 根因是 native Ceres 多 camera 未从 camchain 初始化 cam1 外参；修复后通过 | `out/ceres_sweeps/20260618_tum_imu1_init_from_camchain/result.yaml`，`out/ceres_sweeps/20260618_tum_imu2_init_from_camchain/result.yaml` |
| Kalibr IMU chain parser | 已能解析 `T_ib (imu0 to imuN)`，`--init-from-kalibr` 可初始化 Ceres multi-IMU extrinsics | `src/io/kalibr_result_parser.cpp`，`apps/calibrate_cam_imu.cpp` |
| 4-IMU joint 原始失败 | 根因不是数据不可用；全自由释放 camera/time 时会被 IMU cost 吸走，导致 reproj `54.76 px` | `out/docker_benchmarks/20260618_step4_native_ceres_no_param_stop/benchmark_multi_imu/summary.csv` |
| 4-IMU staged-pbg 安全口径 | `--init-from-kalibr --staged --stage-free pbg` 固定 camera/time，reproj `0.230 px`，证明 residual 接线和 Kalibr result 解析正确 | `out/ceres_sweeps/20260618_joint4imu_runner_safe_pbg_cauchy_30/result.yaml` |
| 4-IMU joint 通过口径 | 根因是 Ceres 标准 robust loss 与 Kalibr IRLS M-estimator 线性化不等价；改为 Kalibr-style loss 后 `pbg,pbegt` 可释放 camera/time | `out/ceres_sweeps/20260619_joint4imu_kalibr_mestimator_staged_pbg_pbegt_30_30/result.yaml` |

## 单组证据

| 口径 | Kalibr 墙钟 | Kalibr 优化耗时 | Ceres 墙钟 | Ceres 优化耗时 | 判断 |
|---|---:|---:|---:|---:|---|
| Docker parity，`benchmark_01` | `178.21 s` | `92.010 s` | `162.31 s` | `154 s` | Ceres Docker 不复现旧表的大幅提速；排除读取/准备后优化段慢于 Kalibr |
| host-native Ceres，`benchmark_01` | `178.21 s` | `92.010 s` | `100.34 s` | `97.4 s` | 复现 Ceres 墙钟明显更快；优化段与 Kalibr 同量级且略慢 |

结论：旧实验文档中的速度收益不能直接迁移到“双 Docker 同环境”口径。当前正式实验主线已经切到 host-native Ceres vs Kalibr Docker；Docker parity 只作为公开可复现和环境差异诊断的附录口径。

## 下一步

| 状态 | 优先级 | 内容 | 最小动作 |
|---|---|---|---|
| done | P0 | 先决定 speed benchmark 的主口径 | 实验文档已把旧 native 表和新 Docker parity 表拆开 |
| done | P0 | 重新跑 12 组 benchmark | 已按 native Ceres vs Kalibr Docker 完成 |
| done | P0 | 定位 multi-camera 失败 | 缺 `--init-from-camchain`；已修复应用层、benchmark runner 和 prepare 脚本 |
| done | P0 | 定位 4-IMU joint 失败 | 已定位为 Kalibr IRLS M-estimator 与 Ceres 标准 robust loss 线性化差异；已改为 Kalibr-style loss 并通过热启动 benchmark |
| open | P1 | 找回 Ceres 的实际提速路径 | 当前 native Ceres 平均优化快，但存在 benchmark_01/03/06 慢于 Kalibr 的样本 |
| open | P2 | 针对 Ceres Docker 做速度 ablation | 仅作为公开可复现/环境差异诊断，不作为当前正式主线 |
