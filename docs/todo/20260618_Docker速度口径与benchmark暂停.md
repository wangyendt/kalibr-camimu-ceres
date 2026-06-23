# Docker 速度口径与 benchmark 当前待办

## 背景

当前正式 benchmark 口径固定为 **macOS native Ceres vs Kalibr Docker**。Ceres Docker parity 只用于公开复现和环境差异诊断，不再作为速度主结论。

最新单 IMU 全量结果已更新到实验文档：`docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md`。本 todo 只保留仍需要行动的状态，不重复主表。

## 当前状态

| 状态 | 内容 | 证据 |
|---|---|---|
| done | benchmark runner 已记录 wall time 与 optimize time | `tools/run_docker_benchmark.py` 输出 `kalibr_elapsed_s`、`ceres_elapsed_s`、`kalibr_optimize_s`、`ceres_optimize_s` |
| done | benchmark-single 最新 12 组复跑完成 | `out/docker_benchmarks/single_amd64_arm64/benchmark_single/summary.csv` |
| done | Ceres benchmark-single solver 口径已固化为非单调步 + step stop | summary 记录 `ceres_solver_use_nonmonotonic_steps=1`、`ceres_solver_max_consecutive_nonmonotonic_steps=20`、`ceres_solver_absolute_step_tolerance=0.02` |
| done | 单目单 IMU running 默认使用 camera time-offset fixed-segment fast path | `--corner-defaults`、`tools/run_docker_benchmark.py`、`tools/prepare_ceres_inputs.py`、`tools/run_wallclock_sweep.py` 和单目单 IMU仿真脚本均设置 `camera_time_offset_buffer_s=0` |
| done | `benchmark_05` 平移异常已定位 | 根因是 nonmonotonic + Ceres 标准 `function_tolerance` 在第 32 次过早收敛;禁用标准 stop、使用 `absolute_step_tolerance=0.02` 且关闭 camera time-offset 动态 buffer 后 b05 平移差 `2.53 mm`、优化 `115.1 s` |
| open | Ceres 当前速度路径待优化 | Ceres/Kalibr amd64 优化均值 `209.5 s / 121.9 s`，wall 均值 `213.7 s / 196.0 s` |

## 当前结论

| 项 | 最新结果 | 判断 |
|---|---|---|
| single-IMU 成功率 | Ceres、Kalibr、compare 均 `12/12` return code 0 | 流程稳定 |
| 精度 | 重投影平均 delta `+0.00055 px`，旋转最大 `0.0149°` | 相机残差和方向稳定 |
| 平移 | 旧 summary 中 `benchmark_05` 为 `27.99 mm`;当前单例修复验证为 `2.53 mm` | 需复跑 12 组刷新主表 |
| 速度 | Ceres optimize 平均 `209.5 s`，Kalibr amd64 `121.9 s`，Kalibr arm64 `75.3 s` | 当前不能宣称 Ceres 更快 |

## 当前待办

| 状态 | 类型 | 优先级 | 内容 | 来源 | 下一步 |
|---|---|---|---|---|---|
| done | bug | P0 | 定位 `benchmark_05` 平移异常 | `/tmp/b05_buffer0_full.yaml` | runner 已改为禁用 Ceres 标准 stop、使用 absolute step stop，并将 benchmark-single 的 camera time-offset buffer 设为 0;后续复跑全 suite 刷新 summary |
| done | config | P0 | 固化单目单 IMU 实测/仿真一致的 running 配置 | `simulation/generated/one_cam_one_imu*/run_calibration.sh` | 单目单 IMU 仿真不再强制 `--init-from-camchain`,并使用同 benchmark-single 的 solver stop 与 `camera-time-offset-buffer 0`;因现有仿真仅 10s,保留 `--imu-trim-edge-count 0` 避免裁空 IMU |
| open | perf | P1 | 找回 Ceres 实际提速路径 | 最新非单调步口径下 Ceres 平均优化慢于 Kalibr amd64 | 做 monotonic/nonmonotonic、固定迭代数、固定停止条件的 timing ablation |
| open | accuracy | P1 | 检查 `benchmark_08/12` 的 `6 ms` 级 time-shift 差 | 最新主表 | 对比 time-shift prior、IMU 裁边、accel outlier 与 Kalibr result |
| open | maintenance | P2 | 后续修复后替换主实验表 | awesome-docs 规则要求实验文档保留最新结果 | 复跑同一 suite 后只更新 `docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md` |

## 已处理

| 日期 | 原事项 | 结果 | 去向 |
|---|---|---|---|
| 2026-06-22 | 固化 benchmark-single solver 口径 | `calibrate_cam_imu` 与 runner 记录非单调步口径，summary 可直接审计实际 solver 设置 | 主实验文档的“范围与配置” |
| 2026-06-22 | 更新 single-IMU 最新结果 | 主实验文档已改为 `71a23e65b752` 最新快照，不再混放历史表 | `docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md` |
| 2026-06-23 | 定位并修复 b05 平移异常与耗时膨胀 | 现 runner 的 b05 在第 32 次 `CONVERGENCE` 停在 `T_c_b.z=0.05063 m`,平移差 `27.99 mm`;禁用标准 stop 且 `absolute_step_tolerance=0.02` 后若保留 camera time-offset 动态 buffer 会到 `318.5 s`;将 benchmark-single 的 `camera_time_offset_buffer_s=0` 后第 115 次停止,`T_c_b.z=0.07594 m`,平移差 `2.53 mm`,优化 `115.1 s` | `tools/run_docker_benchmark.py` |
