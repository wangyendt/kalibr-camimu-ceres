# Ceres/Kalibr 主优化对齐与 IMU 外参可观性待办

## 当前结论

这份 todo 最初假设根因是 "Ceres 使用欧氏 rotvec B 样条, Kalibr 使用累积 SO(3) B 样条"。按本机 Kalibr 源码复核后,这个假设不成立。

本次对齐的 Kalibr 源码路径是:

- `/Users/wayne/Documents/work/code/project/ffalcon/kalibr-docker/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py`
- `/Users/wayne/Documents/work/code/project/ffalcon/kalibr-docker/aslam_nonparametric_estimation/bsplines/src/BSplinePose.cpp`
- `/Users/wayne/Documents/work/code/project/ffalcon/kalibr-docker/Schweizer-Messer/sm_kinematics/src/RotationVector.cpp`

源码事实:

- Kalibr cam-imu 主路径创建的是 `bsplines.BSplinePose(splineOrder, sm.RotationVector())`,不是累积 SO(3) 样条。
- `BSplinePose::transformationAndJacobian()` 先对 6 维欧氏曲线 `evalDAndJacobian()` 求值,再用 `RotationVector::parametersToRotationMatrix()` 映射到旋转矩阵。
- Kalibr 的旋转向量约定是 `R = Exp(-r)`,`rotationMatrixToParameters()` 返回同一符号约定。
- Kalibr 的 body gyro 运动量是

  $$
  \boldsymbol\omega_b
  =
  -R(r)^\top S(r)\dot r.
  $$

  在该约定下有 `R(r)^T S(r) = J_left(r)`,所以 Ceres residual 里的 `-leftJacobianSO3(r) * r_dot` 与 Kalibr 前向模型一致。
- Kalibr 的 `angularAccelerationBodyFrame()` 源码同样使用二阶曲线导数:

  $$
  \boldsymbol\alpha_b^K
  =
  -R(r)^\top S(r)\ddot r,
  $$

  不包含额外的 `Jdot * rdot` 项。Ceres 当前 accelerometer residual 的前向模型与这个接口一致。

因此当前判断:

- **不要把已有生产 residual 改成 AutoDiff。** 之前的手写 Jacobian 方向是对的,应继续用 Kalibr 源码逐项核对解析链式法则。
- **不要把 P0 修复定义为切换到累积 SO(3) 样条。** 累积 SO(3) 可以保留为研究/验证原型,但它不是对齐这份 Kalibr 主路径的必要条件。
- 当前主要问题在 **初始化之后的主优化配置、变量自由度和 running preset**。冷启动 orientation/gravity prior 与 Kalibr 自己的初始化输出已经很接近,不能再把它单独定为 P0 根因。
- 2026-06-23 复核 `benchmark_05` 后确认,单目单 IMU 的厘米级平移异常和耗时膨胀不是手写 Jacobian 问题。根因是 Ceres 标准 `function_tolerance` 过早停止,以及把 `time_padding_s` 同时当成 camera time-shift residual 的动态 buffer,导致每个角点 residual 从 6 个 pose control 扩到约 14 个。当前 production/benchmark-single 默认用 `--camera-time-offset-buffer 0` 保留 spline padding 但回到固定 segment 快路径。

## 已复现实验

数据集: `simulation/generated/one_cam_four_imus_rich`

Kalibr 对照命令记录在:

`simulation/generated/one_cam_four_imus_rich/kalibr_runs/arm64_rich_C/command.txt`

关键 Kalibr 配置:

- `--pose-knots-per-second 100`
- `--bias-knots-per-second 50`
- `--timeoffset-padding 0.12`
- `--max-iter 30`
- `--trim-imu-edge-count 0`
- 日志显示最终 pose spline: `1248` segments over `12.48 s`
- 日志显示 `Do pose motion regularization: False`

### 1. 从 Kalibr 结果初始化,Ceres residual 能接近 Kalibr

命令要点:

- `--init-from-kalibr`
- `--time-padding 0.12`
- 不加 `--pose-motion-prior`
- 禁用 Ceres 额外 absolute early-stop
- 跑 30 iterations

结果:

| 指标 | Ceres from Kalibr init | Kalibr result |
|---|---:|---:|
| reprojection | RMS 0.266 px | mean 0.226 px |
| gyro | RMS 0.0189 rad/s | mean 0.0028-0.0041 rad/s |
| accel | mean 0.044 m/s^2 | mean 0.0187-0.0624 m/s^2 |
| camera rotation error vs truth | 0.026 deg | 0.164 deg |
| IMU lever error vs truth | 8.3-19.4 mm | 4.6-7.1 mm |

这说明 Ceres 生产 residual 的前向模型和手写 Jacobian 没有根本性错误。若 Jacobian 或坐标模型错到不可用,从 Kalibr 外参出发也不会降到这个量级。

### 2. 冷启动仍失败,但 orientation prior 不是单独根因

Kalibr-aligned 配置冷启动:

- `--init-from-camchain`
- `--estimate-time-shift-prior`
- `--estimate-orientation-gravity-prior`
- `--time-padding 0.12`
- 不加 `--pose-motion-prior`
- 禁用 absolute early-stop
- 跑 30 iterations

orientation/gravity prior 日志:

| 项 | Ceres cold start | Kalibr 初始化日志 |
|---|---:|---:|
| gravity | `[-0.2730, -0.0121, -9.8027]` | `[-0.2687, -0.0040, -9.8029]` |
| gyro bias prior | `[0.01277, -0.00044, -0.00241]` | `[0.01284, -0.00029, -0.00237]` |
| vs Kalibr final rotation | `3.33 deg` | 同量级,Kalibr 自己的 prior 也不是最终外参 |
| vs Kalibr final gravity | `0.276 m/s^2` | 同量级 |

最终结果仍偏差大:

| 指标 | Ceres cold start |
|---|---:|
| reprojection RMS | 1.19 px |
| gyro RMS | 0.059 rad/s |
| accel RMS | 0.427 m/s^2 |
| Kalibr rotation delta | 1.39 deg |
| Kalibr translation delta | 4.1 mm |

因此旧判断需要修正: **orientation/gravity prior 不是单独坏先验。** Ceres 和 Kalibr 都会先得到一个约 3 deg 的 camera-IMU rotation prior,Kalibr 随后靠主优化把它拉到最终解。Ceres 当前失败更像是主优化阶段的变量自由度、LM 接受策略、staged release 和停止条件还没与 Kalibr Optimizer2 对齐。

### 3. 配置差异已经确认

| 项 | Kalibr arm64 rich C | 之前 Ceres run script | 影响 |
|---|---:|---:|---|
| time offset padding | 0.12 | 0.04 | spline 时间边界和控制点数不一致 |
| pose motion regularization | off | on | 对 pose spline 施加了 Kalibr 没有的先验 |
| early stop | max iter 30 | absolute parameter tolerance 0.01 会提前停 | 可能在仍下降时停止 |
| IMU robust loss | 本数据集 `Model: calibrated` 对应 `IccImu` 基类,Cauchy width 10 | corner defaults 为 Cauchy width 10 | 已确认主线对齐;Huber 只属于 `scale-misalignment` 扩展模型 |
| Ceres nonmonotonic steps | Kalibr Optimizer2 更接近改善才接受 | Ceres 默认允许非单调步 | 单独关闭后 camera-to-Kalibr delta 改善,但 truth/lever 未解决 |

其中 `time-padding=0.12` 会让 Ceres pose spline 变成 `1253` 个控制点,对应 Kalibr 的 `1248` segments + `order-1`。

### 4. 新增主优化配置实验

| 实验 | 关键变化 | 结论 |
|---|---|---|
| Cauchy + 固定 per-IMU offset | `--fix-imu-time-offsets` | residual 基本不变,lever error 仍为 7.4-18.2 mm,不是主因 |
| Huber gyro/accel 反证 | `--gyro-loss huber --accel-loss huber` | reprojection RMS 降到 0.251 px,gyro RMS 降到 0.0158 rad/s,但非参考 IMU lever 变差到 21-41 mm;不是本数据集 Kalibr 对齐方向 |
| Huber + 固定 per-IMU offset | 再加 `--fix-imu-time-offsets` | camera-to-Kalibr delta 改善,但 lever 仍为 23-42 mm |
| Huber + 固定 offset + monotonic | 再加 `--no-solver-use-nonmonotonic-steps` | camera-to-Kalibr translation delta 降到约 0.48 mm,但 accel/lever 仍不理想 |
| Cauchy + monotonic | `--no-solver-use-nonmonotonic-steps`,当前默认固定非参考 IMU time offset | 保持 Kalibr base IMU loss 口径;camera-to-Kalibr translation delta `0.477 mm`,但 truth camera translation 仍为 `3.20 mm`,非参考 IMU lever 为 `14.0/8.7/18.8 mm`;reprojection/gyro/accel RMS 为 `0.26689/0.01905/0.33588` |
| 非零 IMU time offset 探针 | imu1 `clock.offset_s=+0.018`,IMU0 为 0 | 仿真 truth 写出 imu1 `time_offset_s=-0.018`;Ceres dry-run 链路先验为 `-0.0177778`;短优化结果 `-0.0184561`,truth 对比误差 `-0.456 ms` |
| Kalibr IRLS robust 语义 | Ceres loss 返回 `rho={w*s,w,0}` | 与 Kalibr `MEstimator` 的 residual/Jacobian 同乘 `sqrt(w)` 对齐;新增 `kalibrMEstimatorWeight/Rho` 和 `test_math` 单测 |
| IRLS helper 仿真 smoke | `two_cams_two_imus`,触发 camera-chain prior/IMU time offset/manifold,8 iter 限制 | 输出 `/tmp/ceres_irls_helper_regression.yaml`;camera rotation `0.00563 deg`,translation `<1 um`,reprojection RMS `0.26638 px`,gyro RMS `0.01648 rad/s`,accel RMS `0.15673 m/s^2` |
| best accepted state restore guard | 新增 `--solver-restore-best-state`,在 Ceres accepted step 中记录当前 solve 的最低 cost 状态 | `two_cams_two_imus` 8 iter smoke、4 iter rejected-step probe 和 `one_cam_four_imus_rich` from-Kalibr 30 iter 均未触发恢复;Ceres 在这些 case 已返回 best accepted state,该项不是当前 rich 数据差距主因 |
| staged time active set | `stage-free` 的 `t` 控制 camera time shift,并在显式 `--optimize-imu-time-offsets` 时控制非参考 IMU time offset;修正 `none`/`-` 空 mask 解析 | `two_cams_two_imus` dry-run: `none` stage 为 `parameter_blocks=3080, active_parameter_blocks=0, tangent_params=0`;默认 `t` stage 为 `parameter_blocks=3080, active_parameter_blocks=3, tangent_params=8`;显式 `--optimize-imu-time-offsets` 后 `t` stage 为 `parameter_blocks=3081, active_parameter_blocks=4, tangent_params=9` |
| Kalibr 主优化 IMU time offset 口径 | `IccImu.addDesignVariables()` 没有把 `timeOffset` 加入 problem,只固定用于 `tk = stamp + timeOffset` 查询 | Ceres 默认改为固定 gyro-correlation correction;只有显式 `--optimize-imu-time-offsets` 才把非参考 IMU offset 放入 Ceres problem。rich from-Kalibr dry-run 现在 `tangent_params=22641`,对齐 Kalibr summary `jacobian=129526x22641`;完整 30 iter 得到 reprojection RMS `0.26457 px`,gyro RMS `0.01892 rad/s`,accel RMS `0.33578 m/s^2`,非参考 IMU lever `14.2/7.4/18.2 mm` |
| 非参考 IMU extrinsic 漂移定位 | from-Kalibr 0-iter 与 `--fix-imu-extrinsics` 30-iter 对照 | 0-iter 即 Kalibr seed 时 lever error 为 `7.1/4.6/6.9 mm`;放开优化 30 iter 后变为 `14.2/7.4/18.2 mm`;固定 IMU extrinsics 30 iter 保持 `7.1/4.6/6.9 mm`,而 residual 只从 `0.26457/0.01892/0.33578` 变为 `0.27110/0.01930/0.33626`。说明 lever gap 来自主优化自由度/可观性,不是 Kalibr result 读取或 r_b 符号错误 |
| staged release `i` 负例 | `--stage-free pbet,pbeti --stage-iterations 20,10` | 晚释放非参考 IMU extrinsics 后 lever 为 `11.3/8.0/13.5 mm`,比默认放开好但仍明显差于固定 Kalibr seed;reprojection RMS 变为 `0.28210 px`,camera translation error `3.35 mm`。单靠 release 顺序不足以解决,后续要看 trust-region/先验/可观性指标 |
| Kalibr Optimizer2 LM 口径复核 | `LevenbergMarquardtTrustRegionPolicy(1.0)`,失败回滚,成功步才 rebuild system | Kalibr 主优化设置为 `lambdaInit=1.0`,`convergenceDeltaX=1e-2`,`convergenceDeltaJ=1`,`BlockCholeskyLinearSystemSolver()`;LM policy 中 `rho<=0` 会只增大 `lambda` 并复用线性化。Ceres iteration summary 的 `mu=1/radius`,不能把 Kalibr `lambda` 机械映射成 Ceres radius |
| `initial_trust_region_radius=1` 反例 | `--solver-initial-trust-region-radius 1 --no-solver-use-nonmonotonic-steps` | camera-to-Kalibr translation delta 只有 `0.175 mm`,但 30 iter 后 cost 仍为 `4.03e4`,reprojection/gyro/accel RMS 为 `0.62487/0.05243/0.42731`,非参考 IMU lever 为 `20.6/4.1/21.2 mm`。小半径只是把解留在 Kalibr 外参附近,没有复现 Kalibr residual |
| staged release `i` + 小半径缓解 | `--stage-free pbet,pbeti --stage-iterations 20,10 --stage-solver-initial-trust-region-radii 10000,1 --stage-solver-max-trust-region-radii 1e16,1000 --no-solver-use-nonmonotonic-steps` | lever 改为 `9.8/7.0/11.5 mm`,优于默认放开和普通晚释放,但 reprojection RMS 升到 `0.28909 px`,camera translation error `3.48 mm`;这是调度缓解,不是最终对齐 |
| benchmark-single b05 running preset | `function/gradient/parameter tolerance=0`, `absolute_step_tolerance=0.02`, `camera_time_offset_buffer=0` | b05 从旧的 `27.99 mm / 92.2 s / 32 iter` 恢复到 `2.53 mm / 115.1 s / 116 iter`;动态 buffer 版本虽能继续优化但耗时约 `318.5 s`,不适合作为当前单目单 IMU默认 |

结论: 本数据集 Kalibr 默认 `Model: calibrated` 是 base `IccImu`,gyro/accel robust loss 为 Cauchy。Huber 属于 `scale-misalignment` 扩展模型,不能作为当前对齐目标。robust loss 还必须按 Kalibr IRLS 语义线性化,不能用 Ceres 标准 `CauchyLoss/HuberLoss` 的 `rho''` 语义;当前 native 实现和单测已覆盖这一点。best-state restore 已作为诊断保护项加入,但现有仿真未复现 "final state 不是 best accepted state"。`stage-free` 的 time active set 已按 Kalibr 主线改为默认只释放 camera time shift,非参考 IMU time offset 保留为显式扩展变量,并修正了 `none` 空 mask 被误识别为释放 `e` 的问题。非参考 IMU lever gap 已定位为主优化自由度/可观性问题:固定 Kalibr seed 的 IMU extrinsics 几乎不伤 residual,但 Kalibr 源码确认非参考 IMU 的 `q_i_b/r_b` 默认仍是 active,所以不能把 `--fix-imu-extrinsics` 当作 Kalibr 对齐默认。当前 solver 小矩阵说明,关闭非单调步和限制释放 `i` 的 trust region 只能缓解,不能同时复现 residual 与 truth lever。下一步不应改 residual/Jacobian,而应继续围绕非参考 IMU extrinsic 的可观性指标、弱先验/正则化实验和 Kalibr Optimizer2 线性求解细节做最小对照。

非零 IMU time offset 探针修正了仿真/评估链路的一个盲点: 以前 `ground_truth.yaml` 没有记录每个 sensor 的真实标定 offset,`compare_results_to_ground_truth.py` 默认用 0 比较 IMU offset。现在仿真输出同时记录 `clock_offset_s` 和标定语义的 `time_offset_s`,比较脚本按 truth offset 计算误差。该探针说明 Ceres/Kalibr 的 IMU time offset 符号和读取链路是自洽的;当前非参考 IMU lever gap 不能再优先归因于 time offset sign。

## 为什么不改 AutoDiff

之前的问题不是"手写 Jacobian 写错所以需要 AutoDiff"。

证据:

- Ceres 梯度检查曾显示生产 residual 没有 Jacobian mismatch。
- Kalibr 源码本身也是解析链式法则,不是靠 AutoDiff 替代数学。
- 从 Kalibr 外参初始化后,Ceres 手写 Jacobian 能把成本从 `6.21e4` 降到 `2.28e4`,残差接近 Kalibr 量级。

AutoDiff 可以作为局部验证工具,例如临时写一个对照 cost function 做数值核对,但不应替代生产 residual 的解析实现。

## 当前待办

| 状态 | 优先级 | 内容 | 下一步 |
|---|---|---|---|
| done | P0 | 复核 Kalibr 源码实际旋转表示 | 确认为 `BSplinePose(... RotationVector())` |
| done | P0 | 排除 "必须切换累积 SO(3)" 假设 | 累积 SO(3) 降级为研究原型 |
| done | P0 | 排除 "必须改 AutoDiff" 假设 | 保留手写 Jacobian,继续做解析推导 |
| done | P0 | 修正 Ceres 仿真脚本配置,对齐 Kalibr: `time-padding=0.12`,默认不要加 `pose-motion-prior`,禁用会误伤的 absolute early-stop | 已改 `simulation/scripts/generate_examples.py` 的 `run_calibration.sh` 生成逻辑 |
| done | P0 | 复核 orientation/gravity prior 是否真坏 | Kalibr 日志显示其 gravity/gyro bias prior 与 Ceres 结果接近,不是单独根因 |
| done | P0 | 对齐 Kalibr robust loss 线性化语义 | Native `KalibrMEstimatorLoss` 使用 IRLS `rho={w*s,w,0}`;`test_math` 覆盖 Cauchy/Huber 权重和 `rho[2]=0` |
| done | P0 | 添加 best accepted state restore 诊断开关 | `--solver-restore-best-state` 已实现并完成 smoke;当前三组仿真未触发恢复,说明 rich 差距不是 final state 未回到 best accepted state |
| done | P0 | 对齐 staged time active set | `stage-free t` 默认释放 camera time shift;显式 `--optimize-imu-time-offsets` 时也释放非参考 IMU time offset;`stage-free none/-` 修正为空 active set |
| done | P0 | 对齐 Kalibr 主优化的 IMU time offset 自由度 | 默认固定非参考 IMU delay correction,不再默认增加 `N-1` 个 Ceres time-offset 变量;显式 `--optimize-imu-time-offsets` 保留扩展能力 |
| done | P0 | 定位非参考 IMU extrinsic 是否读错/符号错 | from-Kalibr 0-iter 与 `--fix-imu-extrinsics` 证明 Kalibr seed 本身 lever 正常;Ceres 放开后漂移,问题在主优化可观性/调度 |
| done | P0 | 复核 Kalibr Optimizer2 LM/停止/回滚语义 | 已确认 `lambdaInit=1.0`,`deltaX=1e-2`,`deltaJ=1`,失败回滚;Ceres radius 不能机械映射 Kalibr lambda |
| done | P0 | 跑 from-Kalibr trust-region/monotonic/staged `i` 小矩阵 | monotonic 和 staged 小半径只能缓解,`radius=1` 是 residual 反例;没有发现应修改生产 residual/Jacobian 的证据 |
| done | P0 | 固化单目单 IMU production running preset | `--corner-defaults`、benchmark-single runner、`prepare_ceres_inputs.py`、wallclock sweep 和单目单 IMU仿真脚本均使用 `camera_time_offset_buffer=0`;benchmark-single 额外关闭 Ceres 标准 stop 并用 `absolute_step_tolerance=0.02` |
| done | P0 | 复核 Kalibr camera time-offset expression 控制点集合 | Kalibr `transformationAtTime(time, padding, padding)` 会注册 buffer 范围内所有 coefficients,当前时间只有 6 个非零 Jacobian,其余是零 Jacobian;0.04s/100Hz/order6 时 residual 设计变量约 14 个 |
| open | P0 | 非参考 IMU extrinsic 可观性处理 | 设计弱先验/正则化或可观性评分实验,目标是 residual 不明显退化且 lever 不偏离 Kalibr seed/truth |
| open | P0 | 对齐主优化阶段:staged release、非参考 IMU extrinsic 自由度、停止/接受指标 | 基于 from-Kalibr 结果继续做最小对照,再迁移到 cold start |
| open | P1 | 单独验证 orientation prior 的 `omega_camera` 与 Kalibr 输出是否一致 | 作为 sanity check 保留,但优先级低于主优化路径 |
| open | P1 | 冷启动改进方案:支持从 Kalibr-style gyro-norm time shift 和 rotation prior 分阶段初始化,或允许从 camchain/truth seed 进入 staged solve | 先做最小可复现实验 |
| done | P1 | loss 对齐:确认本数据集 Kalibr `Model: calibrated` 用 Cauchy width 10 | Huber 是 `scale-misalignment` 扩展模型;Ceres `--corner-defaults` 的 Cauchy 主线正确 |
| done | P1 | per-IMU time offset 策略对齐 | `--fix-imu-time-offsets` 单独不改善 lever;非零 offset 探针验证 sign/truth compare 链路正确 |
| done | P1 | 审核 books 第 5/6/7 章的 RotationVector BSpline 主线 | 已确认第 5/6/7 章按 Kalibr `RotationVector` 写法展开;新增第 12 章强激励配置/初始化边界说明 |
| open | P1 | appendix Jacobian 速查表按后续实际代码差异增补 | 当前不改生产 residual Jacobian,暂不需要大改速查表 |
| open | P2 | 用干净强激励数据端到端回归 | 指标目标: reprojection 接近 0.2-0.3 px, gyro/accel 接近 Kalibr 量级 |
| open | P2 | 用含异常值的 spiky 数据回归 robust loss | 仅在 clean 通过后做 |

## 保留的研究分支

`include/ceres_cam_imu/trajectory/spline_eval.h` 中的累积 SO(3) prototype 和 `tests/test_math.cpp` 的有限差分测试可以保留为验证工具,但它不是当前生产路径的 P0 修复。

该分支的合理用途:

- 验证大旋转下欧氏 rotvec spline 与 manifold spline 的差异。
- 为未来替换轨迹表示做准备。
- 作为数学推导的独立 sanity check。

不应把它直接混入当前 Kalibr 对齐修复,否则会同时改变模型、初始化、残差和 Jacobian,难以判断收益来源。

## 复现入口

从 Kalibr 初始化的正向验证:

```bash
build/calibrate_cam_imu \
  --corner-defaults \
  --time-padding 0.12 \
  --cam simulation/generated/one_cam_four_imus_rich/camchain.yaml \
  --target simulation/generated/one_cam_four_imus_rich/aprilgrid.yaml \
  --imu simulation/generated/one_cam_four_imus_rich/imu0.yaml \
  --imu-data simulation/generated/one_cam_four_imus_rich/imu0.csv \
  --imu simulation/generated/one_cam_four_imus_rich/imu1.yaml \
  --imu-data simulation/generated/one_cam_four_imus_rich/imu1.csv \
  --imu simulation/generated/one_cam_four_imus_rich/imu2.yaml \
  --imu-data simulation/generated/one_cam_four_imus_rich/imu2.csv \
  --imu simulation/generated/one_cam_four_imus_rich/imu3.yaml \
  --imu-data simulation/generated/one_cam_four_imus_rich/imu3.csv \
  --corners simulation/generated/one_cam_four_imus_rich/cam0_corners.csv \
  --corner-poses simulation/generated/one_cam_four_imus_rich/cam0_corner_poses.csv \
  --kalibr-result simulation/generated/one_cam_four_imus_rich/kalibr_runs/arm64_rich_C/input/kalibr_cam0_target0_corners-fixture-results-imucam.txt \
  --init-from-kalibr \
  --pose-fit-motion-lambda 0.0001 \
  --pose-fit-boundary-anchors \
  --imu-trim-edge-count 0 \
  --max-iterations 30 \
  --solver-absolute-cost-change-tolerance -1 \
  --solver-absolute-step-tolerance -1 \
  --solver-absolute-parameter-tolerance -1 \
  --top-residuals 0
```

对比真值:

```bash
python3 simulation/scripts/compare_results_to_ground_truth.py \
  --ground-truth simulation/generated/one_cam_four_imus_rich/ground_truth.yaml \
  --result /tmp/ceres_from_kalibr_pad012_no_pose_prior.yaml \
  --label ceres_from_kalibr_pad012_no_pose_prior
```
