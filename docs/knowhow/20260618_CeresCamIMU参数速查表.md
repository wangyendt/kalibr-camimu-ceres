# Ceres Cam-IMU 参数速查表

## 场景

这个仓库的主入口是 CMake 生成的 `build/calibrate_cam_imu`。它读取中立格式：camchain YAML、IMU YAML、AprilGrid YAML、IMU CSV、角点 CSV、可选 corner poses CSV。Kalibr Docker 只用于基线、热启动诊断和部分格式转换。

## 二进制入口

| 二进制 | 用途 | 典型命令 |
|---|---|---|
| `build/calibrate_cam_imu` | 主标定器 | `build/calibrate_cam_imu --help` |
| `build/check_dataset` | 检查 YAML/CSV 读取和数据规模 | `build/check_dataset --cam ... --imu ... --target ... --imu-data ... --corners ...` |
| `build/compare_kalibr_result` | 离线对比 Ceres result 与 Kalibr result | `build/compare_kalibr_result --kalibr-result ... --ceres-result ...` |

## `calibrate_cam_imu` 必填输入

| 参数 | 背景 | 影响与限制 |
|---|---|---|
| `--cam` | Kalibr camchain YAML；可传一个共享 camchain，也可按相机重复传 | 多相机推荐一个共享 camchain + 多个 `--corners` |
| `--imu` | Kalibr IMU 噪声 YAML | 决定 gyro/accel residual 权重 |
| `--target` | AprilGrid YAML | 必须和角点 target 坐标一致 |
| `--imu-data` | CSV: `timestamp_ns,gx,gy,gz,ax,ay,az` | 时间单位和坐标方向必须和 camchain/IMU 约定一致 |
| `--corners` | CSV: `timestamp_ns,corner_id,pixel_x,pixel_y,target_x,target_y,target_z` | 多相机重复传；顺序对应 camchain 中 `cam0/cam1/...` |
| `--corner-poses` | CSV: `timestamp_ns,T_t_c_00...T_t_c_33` | time shift、orientation/gravity、pose fit 初始化需要 |

## 初始化与生产 preset

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--corner-defaults` | 关闭 | production corner-file 标定 preset | 设置 100/50 kps、30 iter、0.04 padding、IMU 裁边 1000、Cauchy width 10、生产停止阈值 |
| `--kalibr-corner-defaults` | 关闭 | 旧别名 | 兼容旧脚本，建议改用 `--corner-defaults` |
| `--init-from-kalibr` | 关闭 | 从 Kalibr result seed 外参/time/gravity | 只用于热启动诊断，生产独立标定不要用 |
| `--kalibr-result` | 空 | Kalibr result 文件 | `--init-from-kalibr` 或 compare/trace 需要 |
| `--init-from-camchain` | 关闭 | 从 camchain 读取 `T_cam_imu` 和 `timeshift_cam_imu` | 适合已有外参的 warm start |
| `--init-from-result` | 空 | 从 Ceres result YAML 继续 | 常用于两阶段或恢复运行 |
| `--estimate-time-shift-prior` | 关闭 | gyro-norm cross-correlation 估 time shift | 需要 `--corner-poses` |
| `--time-shift-pose-kps` | `100` | time-shift 初始化用 pose spline kps | 太低会损失高频角速度 |
| `--time-shift-fit-lambda` | `1e-4` | time-shift 初始化 pose fit 正则 | 越大越平滑 |
| `--estimate-orientation-gravity-prior` | 关闭 | 估 camera-IMU 旋转、gyro bias、gravity | 需要 `--corner-poses` |
| `--orientation-prior-pose-kps` | `100` | orientation/gravity 初始化 pose kps | 影响角速度拟合 |
| `--orientation-prior-fit-lambda` | `1e-4` | orientation/gravity 初始化正则 | 越大越平滑 |
| `--no-orientation-prior-boundary-anchors` | 关闭 | 去掉边界 pose anchors | 可能改变边界段拟合 |
| `--no-orientation-prior-ceres-refine` | 关闭 | 跳过小 Ceres refine | 只保留 closed-form 初值 |

## 轨迹、数据裁剪与问题规模

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--pose-kps` | `20` | pose spline knots per second | 越大轨迹更灵活，变量更多 |
| `--bias-kps` | `10` | bias spline knots per second | 越大 bias 更灵活，变量更多 |
| `--time-padding` / `--timeoffset-padding` | `0.04` | spline 时间边界 padding | 两侧实际 padding 为 `2*S` |
| `--max-frames` | `0` | 限制相机帧数 | `0` 表示不限，调试时可减小 |
| `--imu-stride` | `1` | IMU 下采样步长 | 大于 1 可加速 smoke，但改变优化问题 |
| `--max-imu-residuals` | `0` | 最多 IMU residual | `0` 表示不限 |
| `--imu-trim-edge-count` | `0` 或 preset `1000` | 裁掉首尾 IMU 样本 | 对齐 Kalibr production 时用 `1000` |
| `--dry-run` | 关闭 | 只构建问题不求解 | 用于规模和 residual 检查 |
| `--top-residuals` | `5` | 打印最大 residual 数 | 排查坏点 |
| `--inspect-time` | 空 | 检查某个时间附近 residual | 可重复传 |
| `--inspect-times` | 空 | 逗号分隔多个检查时间 | 与 `--inspect-window` 配合 |
| `--inspect-window` | `0.02` | 检查时间窗口秒数 | 越大输出越多 |

## 固定变量与模型

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--fix-poses` | 关闭 | 固定 pose spline | 用于 residual 或局部诊断 |
| `--fix-biases` | 关闭 | 固定 gyro/accel bias spline | 用于 residual 或局部诊断 |
| `--fix-camera-extrinsic` | 关闭 | 固定 camera-to-IMU 外参 | 两阶段 stage2 常用 |
| `--fix-time-shift` | 关闭 | 固定 time shift | 两阶段 stage2 常用 |
| `--fix-gravity` | 关闭 | 固定 gravity | 控制全局自由度 |
| `--estimate-gravity-length` | 关闭 | gravity 用 3D 欧氏向量 | 默认是固定模长方向 manifold |
| `--imu-model` | `calibrated` | `calibrated/scale-misalignment/scale-misalignment-size-effect` | 扩展模型会增加 IMU intrinsic 参数块 |
| `--fix-imu-intrinsics` | 关闭 | 固定扩展 IMU 参数 | 验证前向模型或防止过拟合 |

支持相机模型：`pinhole+radtan/equidistant/fov/none`、`omni+radtan/none`、`eucm`、`ds/double-sphere`。相机内参当前固定，不参与 Ceres 优化。

## robust loss、先验和 motion prior

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--camera-loss` | `cauchy` | `cauchy/huber/none` | 相机 residual robust kernel |
| `--camera-loss-width` | `10` | camera robust width | 越小越强鲁棒 |
| `--gyro-loss` | `cauchy` | gyro robust kernel | 同上 |
| `--gyro-loss-width` | `10` | gyro robust width | 同上 |
| `--accel-loss` | `cauchy` | accel robust kernel | 同上 |
| `--accel-loss-width` | `10` | accel robust width | 同上 |
| `--time-shift-prior-sigma` | `0` | time-shift prior 标准差 | `0` 表示不加 prior |
| `--pose-motion-prior` | 关闭 | 添加 pose motion prior | 改善弱可观漂移但会影响收敛路径 |
| `--pose-motion-all-segments` | 关闭 | 对全 spline 加 prior | 更接近 Kalibr BSplineMotionError 范围 |
| `--pose-motion-order` | `2` | pose motion derivative 阶数 | 必须在 `[1, spline_order)` |
| `--pose-motion-translation-variance` | `1e6` | 平移 prior 方差 | 越小约束越强 |
| `--pose-motion-rotation-variance` | `1e5` | 旋转 prior 方差 | 越小约束越强 |
| `--pose-fit-diagonal-lambda` | `1e-9` | pose fit 对角正则 | 稳定线性系统 |
| `--pose-fit-motion-lambda` | `0` | pose fit motion 正则 | production 常用 `0.0001` |
| `--pose-fit-boundary-anchors` | 关闭 | 边界重复 anchors | 对齐 Kalibr 初始化 |
| `--pose-motion-local-center` | 空 | 局部 prior 中心时间 | 与 local window 配合 |
| `--pose-motion-local-half-window` | `0` | 局部 prior 半窗口 | `0` 关闭局部缩放 |
| `--pose-motion-local-translation-scale` | `1` | 局部平移方差缩放 | 小于 1 加强局部约束 |
| `--pose-motion-local-rotation-scale` | `1` | 局部旋转方差缩放 | 小于 1 加强局部约束 |

## Ceres solver

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--max-iterations` | `10` 或 preset `30` | 最大迭代数 | production 独立常覆盖到 `150` |
| `--solver-function-tolerance` | `1e-6` | Ceres function tolerance | 内部停止条件 |
| `--solver-gradient-tolerance` | `1e-10` | Ceres gradient tolerance | 内部停止条件 |
| `--solver-parameter-tolerance` | `1e-8` | Ceres parameter tolerance | 内部停止条件 |
| `--solver-initial-trust-region-radius` | `1e4` | 初始 trust region | 影响早期步长 |
| `--solver-max-trust-region-radius` | `1e16` | 最大 trust region | production 常用 `10000000` 限制漂移 |
| `--solver-min-trust-region-radius` | `1e-32` | 最小 trust region | 太小可能耗时 |
| `--solver-min-relative-decrease` | `1e-3` | Ceres 相对下降阈值 | 影响接受步 |
| `--solver-absolute-cost-change-tolerance` | `-1` | 绝对 cost change 停止 | 负值关闭；preset 用 `5e-2` |
| `--solver-absolute-step-tolerance` | `-1` | 绝对 step 停止 | 负值关闭 |
| `--solver-absolute-parameter-tolerance` | `-1` | active 参数最大变化停止 | 负值关闭；preset 用 `1e-2` |
| `--solver-linear-solver` | `SPARSE_NORMAL_CHOLESKY` | Ceres 线性求解器 | 可选 `DENSE_QR/CGNR/SPARSE_SCHUR/...` |
| `--solver-num-threads` | `4` | Ceres 线程数 | 影响速度与资源 |
| `--solver-use-nonmonotonic-steps` | 关闭 | 非单调步 | 可能跳出局部路径 |
| `--solver-max-consecutive-nonmonotonic-steps` | `5` | 非单调连续步上限 | 与上项配合 |
| `--trace-iteration-state` | 关闭 | 打印迭代状态 | 有 Kalibr result 时打印 delta |

## staged 优化

| 参数 | 背景 | 影响 |
|---|---|---|
| `--staged` | 开启保守 staged preset | 当前多相机不支持 staged |
| `--stage-iterations N0,N1,...` | 每阶段迭代数 | 必须和阶段数一致 |
| `--stage-free MASK[,MASK...]` | 每阶段自由变量，`p/b/e/t/g`，`-` 表示只评估 | 覆盖默认阶段策略 |
| `--stop-on-stage-failure` | 某阶段失败立即停止 | 适合批量回归 |
| `--stage-pose-translation-variances` | 每阶段平移 prior 方差 | 与 `--pose-motion-prior` 配合 |
| `--stage-pose-rotation-variances` | 每阶段旋转 prior 方差 | 与上项配合 |
| `--stage-pose-motion-orders` | 每阶段 motion prior 阶数 | 需合法 |
| `--stage-time-shift-prior-sigmas` | 每阶段 time-shift prior sigma | `0` 关闭该阶段 prior |
| `--stage-solver-initial-trust-region-radii` | 每阶段初始半径 | 覆盖全局 solver |
| `--stage-solver-max-trust-region-radii` | 每阶段最大半径 | production final PE cap 用过 |
| `--stage-solver-min-trust-region-radii` | 每阶段最小半径 | 覆盖全局 solver |
| `--stage-solver-min-relative-decreases` | 每阶段相对下降 | 覆盖全局 solver |
| `--stage-solver-absolute-cost-change-tolerances` | 每阶段绝对 cost stop | `-1` 关闭 |
| `--stage-solver-absolute-step-tolerances` | 每阶段绝对 step stop | `-1` 关闭 |
| `--stage-solver-absolute-parameter-tolerances` | 每阶段参数变化 stop | `-1` 关闭 |

## 输出

| 参数 | 用途 | 影响 |
|---|---|---|
| `--output-result` | 写 Ceres result YAML | 后续 compare、init-from-result、生产落盘都需要 |
| `--export-spline-controls` | 导出 spline 控制点 | 用于调试轨迹 |
| `--export-imu-diagnostics` | 导出 IMU 诊断 CSV | 用于 residual 时间序列分析 |

## Python 工具

| 工具 | 关键参数 | 背景 |
|---|---|---|
| `tools/prepare_ceres_inputs.py` | `--source-type pkl|bag|euroc --out-dir ... --run-calibration -- ...` | 把 Kalibr pkl、ROS bag、EuRoC/TUM 转成 Ceres CSV，可顺手跑标定 |
| `tools/run_kalibr_docker.py` | `--dataset --run-name --max-iter --trim-imu-edge-count --export-poses --extra-arg` | 用 Docker 跑 Kalibr 基线 |
| `tools/run_ceres_sweep.py` | `--dataset --preset --variant --base-arg --extra-arg` | 批量跑 Ceres 变体并汇总 CSV |
| `tools/run_ceres_two_stage.py` | `--stage1-* --stage2-* --imu-model --out-dir` | TUM/轨迹频率诊断，不是默认生产主路径 |

## 当前限制

- 主求解器不依赖 Kalibr result，但 pkl/bag/euroc 转换阶段仍可能依赖 Kalibr Docker/ROS。
- 多相机支持 shared camchain + 多 `--corners` 的非 staged joint 优化；staged multi-camera 还未实现。
- 相机内参目前固定，不参与 Ceres 优化。
- 速度结论来自当前机器和当前 Docker/native 组合，跨平台评估必须重新跑实验。
