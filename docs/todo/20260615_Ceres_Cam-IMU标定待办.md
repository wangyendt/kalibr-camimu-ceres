# Ceres Cam-IMU 标定待办

## 当前状态

`ceres_cam_imu/` 已经从原型进入可评测状态：普通 IMU、扩展 IMU、主要相机模型、结构化结果、批量 sweep、TUM two-stage 和匿名基准独立评测都已跑通。当前要守住的边界是：**默认独立标定不读取 Kalibr result**；Kalibr 只用于评测基线、热启动诊断或历史格式转换。

最新结论写入 `docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md`：

- 匿名基准独立标定 2026-06-18 旧表全部 `CONVERGENCE`，外参平移差为毫米级，reprojection 基本追平 Kalibr。
- 2026-06-22 `54a2517` 复跑仍全部收敛且 reprojection delta 平均 `0.00072 px`，但外参平移平均 `14.08 mm`、最大 `28.32 mm`；已定位为 solver 口径差异：旧表依赖 `--solver-use-nonmonotonic-steps`，当时 runner 未启用该选项。当前代码把非单调步设为二进制默认，最大连续非单调步默认 `20`，`benchmark_12` 从 `19.26 mm / 43 it` 恢复到 `3.05 mm / 78 it`。
- 热启动仍漂 `0.19-2.63 mm`，说明两套优化问题本身不是逐位同解。
- 扩展 IMU 的 `M_a/M_g/A_g/C_g` 已有匿名基准/TUM 全量对比，量级可控。
- TUM 双目通过二阶段把 gyro/accel residual 拉到 Kalibr 同量级。

## 当前待办

| 优先级 | 状态 | 事项 | 下一步 |
|---|---|---|---|
| P0 | open | 原生输入链路 | 评估/实现 Ceres 侧 AprilTag 检测、bag/euroc parser，减少转换阶段对 Kalibr Docker/ROS 的依赖 |
| P0 | open | 多 camera staged | 把 single-camera staged 调度推广到 shared camchain + 多 `--corners`，解除当前 multi-camera joint-only 限制 |
| P1 | open | TUM two-stage preset 产品化 | 固化参数、失败诊断和 summary 字段，让 TUM parity 流程从实验脚本变成稳定入口 |
| P1 | open | native-vs-native 速度评测 | 在同一 Linux native 环境重跑 Kalibr 与 Ceres，替代当前 arm64 Ceres vs amd64 Docker 模拟对比 |
| P1 | open | 毫米级弱方向差异分析 | 对齐 LM 策略、停止条件、变量缩放/预条件，判断外参平移是否能继续收敛到 sub-mm |
| P2 | open | 输入配置抽象 | 把 observation selection、运行配置和 sweep dataset 描述从 CLI 继续下沉到 processing/config 层 |

## 风险与观察点

| 风险 | 当前判断 | 处理方式 |
|---|---|---|
| 独立标定被误认为依赖 Kalibr | `--kalibr-result` 曾出现在 independent 复现命令里，容易误导 | 独立命令去掉该参数；对比改用 `compare_kalibr_result --ceres-result` 离线完成 |
| 转换阶段仍依赖 Kalibr Docker/ROS | pkl/bag/euroc 目前可一键转并标定，但不是完全原生 | 明确“求解不依赖、转换仍依赖”；后续补原生检测/读取 |
| 多 camera staged 缺失 | TUM 当前用 joint + two-stage 固定全局块验证 | 后续实现 staged multi-camera，避免双目默认路径只能走 joint |
| 外参平移剩余毫米级差异 | 热启动也会漂，说明不是单纯初始化问题 | 把它作为优化问题等价性研究，而不是继续盲目调初始化 |
| `54a2517` 复跑出现厘米级平移差 | 已定位为 solver 口径差异，不是手写 Jacobian 或初始化退化；新 runner 单调步在 `benchmark_12` 停在 `19.26 mm / 43 it`，补非单调步后当前代码恢复 `3.05 mm / 78 it` | 固化 benchmark-single 是否启用非单调步；旧毫米级表只能标注为 nonmonotonic solver 口径 |
| TUM 低 kps IMU residual 偏高 | 已证明是 pose/bias 轨迹频率问题 | 二阶段固定全局块后用 100/50 kps refine pose/bias |

## 已处理里程碑

| 事项 | 结果 |
|---|---|
| Ceres 子工程骨架 | `io/camera/trajectory/residuals/optimizer/tools/apps/tests` 已成型，CMake/ctest 可跑 |
| 样例数据读取 | camchain/IMU/target YAML、IMU CSV、corner CSV、corner poses、Kalibr/Ceres result 均已读通 |
| 普通 residual 解析 Jacobian | camera、gyro、accel、bias prior、pose/time prior 已有单测和样例数据 smoke |
| 扩展 IMU 解析 Jacobian | `scale-misalignment`、`scale-misalignment-size-effect` 已从差分切到手写 `SizedCostFunction` |
| Kalibr 口径对齐 | IMU trim、time padding、gravity manifold、Cauchy loss、problem-size summary 已复核 |
| 结构化输出与对比 | `--output-result`、`compare_kalibr_result --ceres-result`、residual statistics、top outlier 已接入 |
| 批量评测 | `run_ceres_sweep.py`、`run_kalibr_docker.py`、`run_ceres_two_stage.py` 已支撑当前实验 |
| time-shift 初始化同步 bug | 已同步 `initial_camera_time_shift_s` 与 `initial_camera_time_shifts[0]`，匿名基准恢复收敛 |
| 参数命名去 Kalibr 化 | 正式入口改为 `--corner-defaults`，旧 `--kalibr-corner-defaults` 仅兼容历史脚本 |
| `54a2517` benchmark-single 平移差定位 | 根因是 2026-06-18 旧表启用了 Ceres 非单调步，而 2026-06-22 当时 runner 未启用；当前代码把非单调步设为默认后，`benchmark_12` 恢复到 `3.05 mm` |
| benchmark-single solver 口径固化 | `calibrate_cam_imu` 默认启用 `use_nonmonotonic_steps=1`、`max_consecutive_nonmonotonic_steps=20`；`tools/run_docker_benchmark.py --suite benchmark-single` 不再显式追加这两个参数，summary 同时记录 runner 默认和 Ceres 日志解析出的实际 solver 口径；需要复现单调步时传 `--no-ceres-nonmonotonic-steps` |

## 推荐复现入口

- 默认独立：`calibrate_cam_imu --corner-defaults ... --estimate-time-shift-prior --estimate-orientation-gravity-prior ...`
- 评测 delta：`compare_kalibr_result --kalibr-result ... --ceres-result ...`
- 热启动诊断：`calibrate_cam_imu --corner-defaults ... --kalibr-result ... --init-from-kalibr ...`
- TUM parity：`python3 tools/run_ceres_two_stage.py ...`
