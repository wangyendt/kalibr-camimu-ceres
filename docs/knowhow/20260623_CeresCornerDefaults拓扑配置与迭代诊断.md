# Ceres corner-defaults 拓扑配置与迭代诊断

## 触发场景

当命令里使用 `build/calibrate_cam_imu --corner-defaults` 时，Ceres 会按输入数量自动识别当前标定拓扑：

- `1cam+1imu`
- `1cam+Nimu`
- `Mcam+1imu`
- `Mcam+Nimu`

当前没有 `--topology` 这类显式参数，也没有四份独立配置文件。拓扑由输入数量自动推断。四类 topology 共享 production solver defaults 的主体配置，但允许少量 topology 覆写；多 IMU topology 另外把 gyro-correlation delay 搜索改为 Kalibr 口径的 raw overlap full-correlation。显式传 `--imu-chain-prior-max-offset-s` 时才退回固定秒数窗口。

## 自动检测规则

检测发生在 `apps/calibrate_cam_imu.cpp`。规则只看两个参数的数量：

| Topology | 检测条件 | 示例 |
|---|---|---|
| `1cam+1imu` | `--corners` 传 1 个，`--imu-data` 传 1 个 | 单 camera、单 IMU |
| `1cam+Nimu` | `--corners` 传 1 个，`--imu-data` 传多个 | 单 camera、4 IMU joint |
| `Mcam+1imu` | `--corners` 传多个，`--imu-data` 传 1 个 | TUM 双目单 IMU |
| `Mcam+Nimu` | `--corners` 传多个，`--imu-data` 传多个 | 多 camera、多 IMU |

注意点：

- `--cam` 和 `--imu` 可以传多个，但 topology 当前只按 `--corners` 与 `--imu-data` 的数量判断。
- 不传 `--corner-defaults` 时不会应用 topology preset，只使用 `CalibrationOptions` 的基础默认值和显式命令行参数。
- 显式命令行参数优先级最高，例如 `--solver-max-consecutive-nonmonotonic-steps 10` 会覆盖 `--corner-defaults` 里的默认值。

## 当前 running 配置

四类 topology 目前共享以下公共 defaults：

| 参数 | 值 | 说明 |
|---|---:|---|
| `pose_kps` | `100` | pose spline knot rate |
| `bias_kps` | `50` | bias spline knot rate |
| `time_padding_s` | `0.04` | spline time padding |
| `camera_time_offset_buffer_s` | `0` | camera time-shift residual 固定 segment 快路径 |
| camera / gyro / accel loss | `cauchy:10` | 当前 calibrated IMU 主线 |

Topology 专用补充：

| Topology | 参数 | 值 | 说明 |
|---|---|---:|---|
| `1cam+Nimu` / `Mcam+Nimu` | `imu_chain_prior_offset_search` | `full-overlap` | 对齐 Kalibr：按当前两路 IMU 原始时间轴重叠序列长度做 gyro correlation delay 搜索；不影响 camera time-shift residual 的 pose-control buffer |
| 单 IMU topology | `imu_chain_prior_offset_search` | `bounded:0.2` | 保持 `ImuChainInitializerOptions` 基础默认；显式 `--imu-chain-prior-max-offset-s` 可覆盖 |

四类 topology 共享 production solver defaults 的主体配置；少量 stop policy 会按 topology 局部覆写：

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

除 `1cam+Nimu` 的 delay 搜索窗口和 `1cam+1imu` 的 cost plateau stop 外，当前真正的 running 差异来自 runner 额外传入的参数：

| 场景 | Topology | runner 额外参数 |
|---|---|---|
| `benchmark-single` | `1cam+1imu` | time-shift/orientation 初始化、pose fit boundary anchors、time-shift prior、pose motion prior；topology preset 使用 `absolute_cost_change_tolerance=0.005` |
| `benchmark-multi-imu joint_4imu` | `1cam+Nimu` | `--staged --stage-free pbg,pbegti --stage-iterations 30,30 --imu-extrinsic-translation-bound-m 0.003 --imu-extrinsic-rotation-bound-rad 0.005` 等 staged 参数 |
| `benchmark-multi-imu single_imu1..4` | `1cam+1imu` | 同单 IMU 初始化/先验 |
| `tum` | `Mcam+1imu` | `--init-from-camchain`、time-shift/orientation 初始化、pose motion prior、`--imu-model scale-misalignment` |

## 修改非单调步上限的影响

当前 `solver_max_consecutive_nonmonotonic_steps=20` 写在 `applyProductionSolverDefaults()` 里。因为四类 topology 都调用同一个 production solver defaults：

- 如果把代码里的 `20` 改成 `10`，所有使用 `--corner-defaults` 的 topology 都会受影响。
- `1cam+Nimu` 的 staged run 也会受影响，因为 staged options 是从 base options 拷贝出来的，只覆盖阶段自由变量和阶段迭代数等少量字段。
- 如果只想临时实验，不需要改代码，直接传：

```bash
--solver-max-consecutive-nonmonotonic-steps 10
```

这只影响当前命令。配合 runner 时用：

```bash
--ceres-extra-arg=--solver-max-consecutive-nonmonotonic-steps --ceres-extra-arg=10
```

如果未来要让四类 topology 使用不同值，需要在 `applyCornerDefaults()` 里按 `CornerDefaultTopology` 分支设置，而不是改共享的 `applyProductionSolverDefaults()`。

## 150+ 迭代现象

实验文档里出现 `Ceres it=151` 的 case，并不表示 Ceres 做了 151 次有效更新。Ceres report 的 `Iterations: 151` 包含初始状态行；当前 `max_iterations=150` 时，`151` 基本等价于跑满 150 次迭代。

已检查的日志里，以下 case 是 `NO_CONVERGENCE`，不是正常触发 absolute stop：

| Case | 日志结论 | 末端信号 |
|---|---|---|
| `benchmark_02` | `Iterations: 151`, `Termination: NO_CONVERGENCE` | 第 150 行 step 约 `3.48e-02`，大于 `0.02` |
| `benchmark_03` | `Iterations: 151`, `Termination: NO_CONVERGENCE` | 第 150 行 step 约 `6.86e-02`，大于 `0.02` |
| `benchmark_10` | `Iterations: 151`, `Termination: NO_CONVERGENCE` | 第 150 行 step 约 `5.82e-02`，大于 `0.02` |
| `tum_imu1_bag` | `Iterations: 151`, `Termination: NO_CONVERGENCE` | raw step 很大，`absolute_step_tolerance=0.02` 不适合直接触发 |

初步判断：

- `1cam+1imu` 的部分 case 在后期 cost change 已经很小，但 step 仍略高于 `0.02`，所以只靠 absolute step stop 会继续跑到 max iteration。当前代码只对 `1cam+1imu` 增加 `absolute_cost_change_tolerance=0.005`，保留 `step=0.02`，避免 b05 在 `step=0.05/0.07` 下出现 5-7mm 级退化。
- TUM 的 `Mcam+1imu + scale-misalignment` raw step 更不稳定，可能被 IMU intrinsic、camera-chain、多相机变量尺度影响；同一个 `absolute_step_tolerance=0.02` 不一定适合所有 topology。
- 非单调步上限从 `20` 改 `10` 不一定能解决 150 iteration。它会改变可接受的非单调窗口，但当前这些日志主要问题是停止条件没有触发，而不是连续失败步过多；日志里 `num_unsuccessful_steps=0`。

## 定位建议

要兼顾效率和精度，下一步不建议直接全局降低 `max_iterations` 或把非单调步上限从 `20` 改成 `10` 后当作生产结论。更稳的定位顺序是：

1. 在 summary 里记录最终几行 iteration 的 `cost_change`、`step_norm`、`gradient_norm` 和 termination type，先区分“真实还在收敛”和“cost 已平台但 step 不触发”。
2. 对 `benchmark_02/03/10` 做固定迭代数截断实验，例如 `80/100/120/150`，比较外参差、time-shift、reproj/gyro/accel residual，找精度平台点。
3. 对 TUM 单独做 `Mcam+1imu` stop policy 实验，因为 scale-misalignment 和双目链路会让 raw step 与单目单 IMU不可直接共用阈值。
4. 只在实验显示不伤精度后，再把某个 topology 的 running 分支收紧；不要直接改共享 production defaults。

当前最可能的方向：

- `1cam+1imu` 已加入更保守的 cost plateau stop；现有定点验证覆盖 `b05` 的负例和 `b10` 的收益，但仍需要完整 `benchmark-single` 复跑确认 12 组统计。
- `Mcam+1imu` 需要单独的 stop criterion 或变量归一化诊断，不能简单继承单目阈值。
- `1cam+Nimu` 已用现有 Kalibr arm64 结果做 Ceres-only joint 复跑；当前默认 tight bound 已验证能压住 C/K joint effective chain 平移 tail，但不修复 `b09` accel residual。

## 2026-06-24 Ceres-only 预验证

这次预验证不重跑 Kalibr Docker，只复用 `out/docker_benchmarks/*` 里的 Kalibr arm64 结果，重跑当前工作树的 Ceres native。它用于快速筛选 running 配置，不能替代正式三 suite 复测。

| 范围 | 配置 | 旧结果 | 新结果 | 判断 |
|---|---|---:|---:|---|
| Suite A `benchmark-single` 12 组 | `1cam+1imu` 增加 `absolute_cost_change_tolerance=0.005`，保留 `step=0.02` | Ceres wall 总和 `1373.2 s`，iter 总和 `1363`，平移均/最大 `1.92 / 4.03 mm` | Ceres wall 总和 `1295.9 s`，iter 总和 `1230`，平移均/最大 `2.05 / 4.03 mm` | 效率小幅改善，最大平移不变；`b05` 未复现早停退化 |
| Suite B `benchmark-multi-imu` 12 个 joint | `pbg,pbegti`，非参考 IMU extrinsic `translation<=0.005 m`、rotation-vector `<=0.01 rad` | effective chain 平移均/中/最大 `30.7 / 17.9 / 163.0 mm` | case-max 平移均/中/最大 `5.6 / 6.1 / 8.7 mm` | 平移 tail 明显收敛 |
| Suite B `benchmark-multi-imu` 12 个 tight joint | `pbg,pbegti`，非参考 IMU extrinsic component-wise `translation<=0.003 m`、rotation-vector `<=0.005 rad` | case-max 平移均/中/最大 `5.6 / 6.1 / 8.7 mm` | `4.3 / 4.5 / 5.2 mm`；all-chain 平移均/中/最大 `2.9 / 3.7 / 5.2 mm` | 当前默认推荐；进一步压住外参链 tail |
| Suite B tight joint 耗时 | 同上 | 5mm joint wall 总和 `1260.8 s` | tight joint wall 总和 `1084.9 s`，stage timing 总和 `1038.4 s` | 总体未变慢；单点仍有长尾，`b07/b09/b12` 超过 190s |
| Suite B tight joint residual | 同上 | 5mm accel delta 均/中/最大 `0.140 / 0.130 / 0.644 m/s^2` | `0.147 / 0.131 / 0.651 m/s^2` | 外参 tail 改善不修复 residual；`b09` accel delta 仍是主问题 |

预验证产物：

2026-06-24 的 full tight joint 复跑只重跑 Ceres native，并通过 `--reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64` 复用 Kalibr arm64。它证明当前 default tight bound 可以把 C/K joint effective chain 控制到 `<=5.21 mm / <=0.405 deg`，但 `b09` accel residual delta 仍为 `+0.651 m/s^2`，后续应围绕 pose spline / stage0 basin 定位，而不是继续只收紧外参 bound。

```text
/private/tmp/ceres_native_validation_current/summary.csv
/private/tmp/ceres_native_validation_suiteb_5mm/summary.csv
out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight/benchmark_multi_imu/summary.csv
```

正式做 Ceres-only 复验时使用 `--reuse-kalibr-from`，只复用既有 Kalibr arm64 结果并重跑当前 Ceres native。不要在这个场景使用未指定平台的 `benchmark-single` 默认命令，否则会重新启动 Kalibr amd64 + arm64：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/single_amd64_arm64 --out-root out/docker_benchmarks/single_arm64_ceres_current
```

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_ceres_current_joint_tight
```

## 验证命令

查看某次运行实际 topology 和 solver defaults：

```bash
rg -n "corner defaults active|solver options|Ceres Solver Report|absolute_stop" out/docker_benchmarks -S
```

临时测试非单调步上限 10：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --session 2025_03_14_00_34_14 --out-root /tmp/bench_nonmono10 --ceres-extra-arg=--solver-max-consecutive-nonmonotonic-steps --ceres-extra-arg=10
```

临时测试固定迭代上限：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-single --session 2025_03_14_00_34_14 --out-root /tmp/bench_iter100 --ceres-max-iterations 100
```
