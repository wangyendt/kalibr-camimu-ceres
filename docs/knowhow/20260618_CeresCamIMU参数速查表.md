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

## 初始化与默认 preset

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--corner-defaults` | 关闭 | corner-file production preset | 按 `--corners` 数量和 `--imu-data` 数量识别 topology；公共设置包括 100/50 kps、0.04 padding、camera time-offset fixed-segment buffer、Cauchy width 10、production solver stop |
| `--kalibr-corner-defaults` | 关闭 | 旧别名 | 兼容旧脚本，建议改用 `--corner-defaults` |
| `--init-from-kalibr` | 关闭 | 从 Kalibr result seed 外参/time/gravity | 只用于热启动诊断，默认独立标定不要用 |
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

## `--corner-defaults` topology 分发

`--corner-defaults` 是当前唯一的默认 running preset 入口。它不看数据集名，也不看 `run_docker_benchmark.py --suite` 名称，只按输入规模分发：`camera_count = --corners` 个数，`imu_count = --imu-data` 个数。显式命令行参数仍然覆盖 preset。

| Topology | 识别条件 | 当前默认配置 | 备注 |
|---|---|---|---|
| `1cam+1imu` | 1 个 `--corners`，1 个 `--imu-data` | 公共 corner defaults + production solver defaults + `absolute_cost_change_tolerance=0.005` | benchmark-single、benchmark-multi-imu 的 `single_imu1..4` 子项都会落到这里；保留 `step=0.02`，用 cost plateau 省掉长尾迭代 |
| `1cam+Nimu` | 1 个 `--corners`，多个 `--imu-data` | 公共 corner defaults + production solver defaults | runner 当前对 joint 多 IMU 额外走 staged 和非参考 IMU extrinsic 小范围释放，这是按 IMU 数量触发，不按数据集名触发 |
| `Mcam+1imu` | 多个 `--corners`，1 个 `--imu-data` | 公共 corner defaults + production solver defaults | TUM 双目单 IMU 落到这里 |
| `Mcam+Nimu` | 多个 `--corners`，多个 `--imu-data` | 公共 corner defaults + production solver defaults | 预留多相机多 IMU；当前没有单独数值 |

公共 corner defaults：

| 参数 | 值 | 说明 |
|---|---:|---|
| `pose_knots_per_second` | `100` | `--pose-kps` |
| `bias_knots_per_second` | `50` | `--bias-kps` |
| `time_padding_s` | `0.04` | `--time-padding` / `--timeoffset-padding` |
| `camera_time_offset_buffer_s` | `0` | camera time-shift residual 固定 segment 快路径，避免把每个角点 residual 扩到过多 pose controls |
| `imu_trim_edge_count` | `1000` | `--imu-trim-edge-count` |
| camera / gyro / accel loss | `cauchy:10` | 对齐当前 Kalibr calibrated IMU 主线 |

多 IMU topology 专用补充：

| 参数 | 适用 topology | 值 | 说明 |
|---|---|---:|---|
| `imu_chain_prior_offset_search` | `1cam+Nimu` / `Mcam+Nimu` | `full-overlap` | 不传 `--imu-chain-prior-max-offset-s` 时启用；按当前两路 IMU 原始时间轴重叠序列长度做 gyro correlation delay 搜索，对齐 Kalibr |
| `imu_chain_prior_offset_search` | 单 IMU topology | `bounded:0.2` | 保持 `ImuChainInitializerOptions` 基础默认；单 IMU 不会实际估 IMU chain prior |

production solver defaults：

| 参数 | 值 |
|---|---:|
| `max_iterations` | `150` |
| `solver_function_tolerance` | `0` |
| `solver_gradient_tolerance` | `0` |
| `solver_parameter_tolerance` | `0` |
| `solver_max_trust_region_radius` | `1e7` |
| `solver_absolute_cost_change_tolerance` | 公共默认 `-1`；`1cam+1imu` 为 `0.005` |
| `solver_absolute_step_tolerance` | `0.02` |
| `solver_absolute_parameter_tolerance` | `-1` |
| `solver_use_nonmonotonic_steps` | `true` |
| `solver_max_consecutive_nonmonotonic_steps` | `20` |

## 常用 benchmark 命令到 topology 的映射

这里的 `benchmark-single`、`benchmark-multi-imu`、`tum` 只是实验 suite 名，用于选择数据、Kalibr Docker 平台和输出目录；它们不再是 Ceres running preset 名。

| 命令 | Ceres 输入规模 | Topology | runner 额外做什么 | Kalibr 平台 |
|---|---|---|---|---|
| `--suite benchmark-single` | 1 个 corners + `data1.csv` | `1cam+1imu` | 加 time-shift/orientation 初始化、pose fit boundary anchors、time-shift prior、pose motion prior；solver 默认来自 `--corner-defaults` | 不传 `--kalibr-platform` 时跑 amd64 + arm64 |
| `--suite benchmark-multi-imu` | joint: 1 个 corners + `data1..4.csv`；single 子项: 1 个 corners + 单路 IMU CSV | joint 为 `1cam+Nimu`；single 子项为 `1cam+1imu` | joint 多 IMU按 topology 触发 staged：`pbg,pbegti`，默认 `30,30`，非参考 IMU extrinsic 先固定后以 component-wise `translation<=0.003 m`、rotation-vector `<=0.005 rad` 小范围释放；single 子项同 `1cam+1imu` 初始化/先验 | 用户命令传 `--kalibr-platform linux/arm64` 时只跑 arm64 |
| `--suite tum` | 2 个 corners + 1 个 IMU CSV | `Mcam+1imu` | 加 `--init-from-camchain`、time-shift/orientation 初始化、pose motion prior、`--imu-model scale-misalignment`；solver 默认来自 `--corner-defaults` | 用户命令传 `--kalibr-platform linux/arm64` 时只跑 arm64 |

只想验证当前 Ceres native 改动时，不要用 `benchmark-single` 默认双平台口径重跑 Kalibr。传 `--kalibr-platform linux/arm64 --reuse-kalibr-from <旧out-root>` 后，runner 会复用旧目录中的 Kalibr arm64 result/log/summary，只重新执行 Ceres 和 compare。

### `run_docker_benchmark.py` 常用调试参数

这些参数只属于 runner，不会直接透传给 Ceres native 标定器，除非 runner 在构造 Ceres 命令时显式转换。

| runner 参数 | 默认 | 作用 | 典型用途 |
|---|---:|---|---|
| `--benchmark-multi-subset all|joint|single` | `all` | 控制 `benchmark-multi-imu` 跑 joint、single 或全部 | Ceres-only 验证 joint 配置时用 `joint`，避免重跑 48 路 single |
| `--ceres-multi-imu-stage-free` | `pbg,pbegti` | multi-IMU joint staged 的 free mask 列表 | ablation 单阶段或调整释放顺序 |
| `--ceres-multi-imu-stage-iterations` | `30` | 每个 stage 的最大迭代数 | 控制 joint 每阶段上限；runner 会按 stage 数自动展开 |
| `--ceres-multi-imu-stage-step-tolerances` | 空 | 覆盖每个 stage 的 absolute step tolerance | 验证停止条件对速度/残差的影响 |
| `--ceres-multi-imu-extrinsic-translation-bound-m` | `0.003` | 传给 Ceres 的非参考 IMU extrinsic 平移 component-wise bound | 当前 default tight joint 配置 |
| `--ceres-multi-imu-extrinsic-rotation-bound-rad` | `0.005` | 传给 Ceres 的非参考 IMU extrinsic rotation-vector component-wise bound | 当前 default tight joint 配置 |

## 轨迹、数据裁剪与问题规模

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--pose-kps` | `20` | pose spline knots per second | 越大轨迹更灵活，变量更多 |
| `--bias-kps` | `10` | bias spline knots per second | 越大 bias 更灵活，变量更多 |
| `--time-padding` / `--timeoffset-padding` | `0.04` | spline 时间边界 padding | 两侧实际 padding 为 `2*S` |
| `--camera-time-offset-buffer` | `-1`；`--corner-defaults` 设为 `0` | camera time shift 活跃时相机 residual 的 pose control buffer | 负值复用 `time-padding`，对齐 Kalibr 动态 expression；`0` 使用固定 segment 快路径，当前 topology preset 默认采用 |
| `--max-frames` | `0` | 限制相机帧数 | `0` 表示不限，调试时可减小 |
| `--imu-stride` | `1` | IMU 下采样步长 | 大于 1 可加速 smoke，但改变优化问题 |
| `--max-imu-residuals` | `0` | 最多 IMU residual | `0` 表示不限 |
| `--imu-trim-edge-count` | `0` 或 preset `1000` | 裁掉首尾 IMU 样本 | 对齐 Kalibr-compatible 口径时用 `1000` |
| `--dry-run` | 关闭 | 只构建问题不求解 | 用于规模和 residual 检查 |
| `--top-residuals` | `5` | 打印最大 residual 数 | 排查坏点 |
| `--inspect-time` | 空 | 检查某个时间附近 residual | 可重复传 |
| `--inspect-times` | 空 | 逗号分隔多个检查时间 | 与 `--inspect-window` 配合 |
| `--inspect-window` | `0.02` | 检查时间窗口秒数 | 越大输出越多 |

## 多 IMU、delay correction 与 time offset

多 IMU 入口通过重复传 `--imu` 和 `--imu-data` 打开，第一路 IMU 是 reference IMU。当前默认不是多 stage：camera residual、每路 IMU gyro/accel residual、bias/pose prior、IMU chain extrinsic 和固定 delay correction 都在同一个 Ceres problem 里联合优化。per-IMU time offset 仍是可选扩展变量，只有显式 `--optimize-imu-time-offsets` 时进入 problem。

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--imu-delay-correction auto|on|off` | `auto` | 多 IMU 时用 gyro correlation 估非参考 IMU 相对 IMU0 的 delay | `auto` 在多 IMU 默认启用；单 IMU 不启用 |
| `--no-imu-delay-correction` | 关闭 | 禁用 IMU 间 delay correction | 所有 IMU residual 使用原始 timestamp；生产 4IMU 数据通常不应这么跑 |
| `--fix-imu-time-offsets` | 关闭 | 显式请求 Kalibr-style 固定 correction 语义 | 默认已经固定；该开关主要用于和 `--optimize-imu-time-offsets` 组合时保持不进 parameter block |
| `--optimize-imu-time-offsets` | 关闭 | 显式要求优化非参考 IMU offset | 多 IMU + `calibrated` 模型下才支持；默认不启用,以对齐 Kalibr 主优化 |
| `--imu-time-offset-bound-s` | `0.005` | 每路非参考 IMU offset 的 box bound 半宽 | 仅在 `--optimize-imu-time-offsets` 时生效；太大可能跨 spline support |
| `--imu-extrinsic-translation-bound-m` | `-1`；multi-IMU joint runner 传 `0.003` | 非参考 IMU extrinsic 平移相对当前 problem 构建值的 component-wise box bound 半宽 | `-1` 关闭；必须为正数才启用；当前用于 staged 第二阶段小范围释放；三维平移范数尾部可能到 `sqrt(3)` 倍 component bound |
| `--imu-extrinsic-rotation-bound-rad` | `-1`；multi-IMU joint runner 传 `0.005` | 非参考 IMU extrinsic rotation-vector 相对当前 problem 构建值的 component-wise box bound 半宽 | `-1` 关闭；必须为正数才启用；不要用 `0` 表示固定，固定整块请用 `--fix-imu-extrinsics` |
| `--no-estimate-imu-chain-prior` | 关闭 | 禁用多 IMU chain prior | 会失去非参考 IMU rotation/delay 初值，通常只用于消融 |
| `--imu-chain-prior-max-offset-s` | 基础默认 `0.2`；`--corner-defaults` + 多 IMU topology 不传该参数时使用 raw overlap 全长 | gyro correlation 的 delay 搜索窗口 | 对齐 Kalibr：按当前两路 IMU 原始时间轴重叠序列长度做 full-correlation；显式传该参数才切回固定秒数窗口；只影响多 IMU chain prior 初始化搜索，不扩大 camera time-shift residual 的 pose-control buffer |
| `--imu-chain-prior-stride` | `1` | chain prior 初始化下采样 | 大于 1 加速初始化，但可能影响 correlation 峰值 |
| `--imu-chain-prior-min-samples` | `200` | chain prior 最少匹配样本 | 过少会拒绝估计 |
| `--imu-chain-prior-min-excitation` | `1e-8` | rotation prior 最小激励阈值 | 动作太弱时会拒绝估计 |
| `--no-imu-chain-prior-ceres-refine` | 关闭 | 跳过 IMU chain 小 Ceres refine | 只保留 correlation/closed-form 初值 |
| `--imu-chain-prior-refine-iterations` | `50` | IMU chain 小 refine 迭代数 | 只影响 solve 前的 pair prior refine |
| `--estimate-imu-chain-lever-prior` | 关闭 | 从加速度差估非参考 IMU lever arm 初值 | 实验功能；默认只估 rotation 和 delay |
| `--no-estimate-imu-chain-lever-prior` | 关闭 | 显式关闭 lever prior | 与上项配合 |
| `--imu-chain-prior-min-lever-excitation` | `1e-8` | lever prior 最小激励阈值 | 激励不足时不估 lever |
| `--imu-chain-prior-max-lever-m` | `1.0` | lever prior 最大模长 | 过滤异常 lever 解 |

IMU chain prior 搜索日志：

| 日志字段 | 含义 | 判断方式 |
|---|---|---|
| `imu_chain_prior_offset_search=full-overlap` | 当前使用 Kalibr-style raw overlap 搜索 | `--corner-defaults` + 多 IMU topology 且未显式传 `--imu-chain-prior-max-offset-s` |
| `imu_chain_prior_offset_search=bounded:S` | 当前使用固定秒数窗口 | 显式传了 `--imu-chain-prior-max-offset-s S`，或不走多 IMU corner-defaults |
| `time_offset_search_radius_s` | 实际 correlation 搜索半径 | full-overlap 时约等于两路 IMU 原始时间轴重叠时长；bounded 时等于 `S` |
| `max_search_lag_samples` | 搜索半径折算到目标 IMU sample lag | 约为 `round(time_offset_search_radius_s / sample_dt_s)` |
| `discrete_shift_samples` | 互相关峰值对应的 lag | 如果等于 `±max_search_lag_samples`，说明有界窗口可能截断了真实 delay |

时间变量语义：

| 变量 | 是否优化 | 说明 |
|---|---|---|
| `camera_time_shift_s` | 是 | cam0 到 reference IMU 的 time shift；单 IMU 和多 IMU 都存在 |
| `imu_time_offsets_s[0]` | 否 | 固定 `0`，定义 reference IMU 时间轴 |
| `imu_time_offsets_s[1..N-1]` | 默认否；显式可优化 | 默认是固定 delay correction；传 `--optimize-imu-time-offsets` 后在多 IMU + `calibrated` 模型下作为 Ceres 变量 |

组合每路 camera-to-IMU effective time shift 时用：

```text
timeshift(cam0 -> imui) = camera_time_shift_s - imu_time_offsets_s[i]
```

实测状态：2026-06-21 已在 `2025_03_14_00_34_14` 做 4IMU smoke 验证，限制 `--max-frames 20 --max-imu-residuals 100 --max-iterations 2`。显式 offset 优化时 `parameter_blocks=24154`，固定 correction dry-run 为 `parameter_blocks=24151`，差值正好是 `N-1=3`；输出 YAML 顶层 `imu_time_offsets_s` 写出了优化后的 `0; -0.0192207; -0.0981117; -0.1197944`。这证明变量扩展路径可用。2026-06-22 对齐 Kalibr 主优化后，默认口径回到固定 correction；需要实验该扩展时显式传 `--optimize-imu-time-offsets`。

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
| `--pose-fit-diagonal-lambda` | `0` | pose fit 对角正则 | 默认关闭以匹配 Kalibr `initSplineSparse`；需要数值阻尼时显式开启 |
| `--pose-fit-motion-lambda` | `0` | pose fit motion 正则 | 默认批量评测常用 `0.0001` |
| `--pose-fit-boundary-anchors` | 关闭 | 边界重复 anchors | 对齐 Kalibr 初始化 |
| `--pose-motion-local-center` | 空 | 局部 prior 中心时间 | 与 local window 配合 |
| `--pose-motion-local-half-window` | `0` | 局部 prior 半窗口 | `0` 关闭局部缩放 |
| `--pose-motion-local-translation-scale` | `1` | 局部平移方差缩放 | 小于 1 加强局部约束 |
| `--pose-motion-local-rotation-scale` | `1` | 局部旋转方差缩放 | 小于 1 加强局部约束 |

## Ceres solver

| 参数 | 默认 | 背景 | 影响 |
|---|---:|---|---|
| `--max-iterations` | `10`；`--corner-defaults` 设为 `150` | 最大迭代数 | runner 默认不再按 suite 覆盖；需要实验覆盖时显式传 `--ceres-max-iterations` 或 `--ceres-extra-arg` |
| `--solver-function-tolerance` | `1e-6`；`--corner-defaults` 设为 `0` | Ceres function tolerance | production preset 关闭 Ceres 标准 function stop，避免过早停止 |
| `--solver-gradient-tolerance` | `1e-10`；`--corner-defaults` 设为 `0` | Ceres gradient tolerance | production preset 关闭 Ceres 标准 gradient stop |
| `--solver-parameter-tolerance` | `1e-8`；`--corner-defaults` 设为 `0` | Ceres parameter tolerance | production preset 关闭 Ceres 标准 parameter stop |
| `--solver-initial-trust-region-radius` | `1e4` | 初始 trust region | 影响早期步长 |
| `--solver-max-trust-region-radius` | `1e16`；`--corner-defaults` 设为 `1e7` | 最大 trust region | production preset 用 `1e7` 限制异常步长 |
| `--solver-min-trust-region-radius` | `1e-32` | 最小 trust region | 太小可能耗时 |
| `--solver-min-relative-decrease` | `1e-3` | Ceres 相对下降阈值 | 影响接受步 |
| `--solver-absolute-cost-change-tolerance` | `-1`；`--corner-defaults` 的 `1cam+1imu` 设为 `0.005` | 绝对 cost change 停止 | 只在单相机单 IMU topology 用作保守 cost plateau stop；其他 topology 默认关闭 |
| `--solver-absolute-step-tolerance` | `-1`；`--corner-defaults` 设为 `0.02` | 绝对 step 停止 | 当前 production preset 主要停止条件 |
| `--solver-absolute-parameter-tolerance` | `-1` | active 参数最大变化停止 | production preset 保持关闭 |
| `--solver-linear-solver` | `SPARSE_NORMAL_CHOLESKY` | Ceres 线性求解器 | 可选 `DENSE_QR/CGNR/SPARSE_SCHUR/...` |
| `--solver-num-threads` | `4` | Ceres 线程数 | 影响速度与资源 |
| `--solver-use-nonmonotonic-steps` / `--no-solver-use-nonmonotonic-steps` | 默认开启；`--corner-defaults` 也开启 | 非单调步 | 当前生产默认启用；单调步只作为显式消融，不再由 runner 按 suite 切换 |
| `--solver-max-consecutive-nonmonotonic-steps` | `20` | 非单调连续步上限 | `--corner-defaults` 保持 `20` |
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
| `--stage-solver-max-trust-region-radii` | 每阶段最大半径 | final PE cap 诊断用过 |
| `--stage-solver-min-trust-region-radii` | 每阶段最小半径 | 覆盖全局 solver |
| `--stage-solver-min-relative-decreases` | 每阶段相对下降 | 覆盖全局 solver |
| `--stage-solver-absolute-cost-change-tolerances` | 每阶段绝对 cost stop | `-1` 关闭 |
| `--stage-solver-absolute-step-tolerances` | 每阶段绝对 step stop | `-1` 关闭 |
| `--stage-solver-absolute-parameter-tolerances` | 每阶段参数变化 stop | `-1` 关闭 |

## 输出

| 参数 | 用途 | 影响 |
|---|---|---|
| `--output-result` | 写 Ceres result YAML | 后续 compare、init-from-result、结果落盘都需要 |
| `--export-spline-controls` | 导出 spline 控制点 | 用于调试轨迹 |
| `--export-imu-diagnostics` | 导出 IMU 诊断 CSV | 用于 residual 时间序列分析 |

## Python 工具

| 工具 | 关键参数 | 背景 |
|---|---|---|
| `tools/prepare_ceres_inputs.py` | `--source-type pkl|bag|euroc --out-dir ... --run-calibration -- ...` | 把 Kalibr pkl、ROS bag、EuRoC/TUM 转成 Ceres CSV，可顺手跑标定 |
| `tools/run_kalibr_docker.py` | `--dataset --run-name --max-iter --trim-imu-edge-count --export-poses --extra-arg` | 用 Docker 跑 Kalibr 基线 |
| `tools/run_docker_benchmark.py` | `--suite benchmark-single|benchmark-multi-imu|tum --out-root ...` | suite 只选择数据和 Kalibr 平台；Ceres running 默认来自 `--corner-defaults` topology，`--ceres-max-iterations` 只有显式传入才覆盖 |
| `tools/run_docker_benchmark.py` | `--reuse-kalibr-from <旧out-root或suite目录>` | 跳过 Kalibr Docker，复用旧 Kalibr result/log/summary，适合只验证当前 Ceres native 配置；缺少匹配结果会直接报错，不会静默重跑 Kalibr |
| `tools/run_ceres_sweep.py` | `--dataset --preset --variant --base-arg --extra-arg` | 批量跑 Ceres 变体并汇总 CSV |
| `tools/run_ceres_two_stage.py` | `--stage1-* --stage2-* --imu-model --out-dir` | TUM/轨迹频率诊断，不是默认主路径 |

## 当前限制

- 主求解器不依赖 Kalibr result，但 pkl/bag/euroc 转换阶段仍可能依赖 Kalibr Docker/ROS。
- 多相机支持 shared camchain + 多 `--corners` 的非 staged joint 优化；staged multi-camera 还未实现。
- 多 IMU per-IMU time offset 优化当前只接入 `calibrated` IMU 模型；`scale-misalignment` 和 `scale-misalignment-size-effect` 仍使用固定 delay correction。
- 2026-06-21 的 per-IMU time offset 变量化已通过 4IMU smoke 实测，但尚未完成全量生产数据回归。
- 相机内参目前固定，不参与 Ceres 优化。
- 结果表里的 `benchmark-single`、`benchmark-multi-imu`、`tum` 只能表示数据选择和 Kalibr 平台选择，不能作为 Ceres running 口径；Ceres 口径以 `corner-defaults` topology 日志和实际命令行为准。
- 速度结论来自当前机器和当前 Docker/native 组合，跨平台评估必须重新跑实验。
