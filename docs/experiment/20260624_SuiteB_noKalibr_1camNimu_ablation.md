# SuiteB no-Kalibr 1cam+Nimu Ablation

> **历史实验状态（2026-07-30）：** 本文记录的是 2026-06-24 至 06-27
> 用于定位优化 basin 的 adaptive / single-seed / 多候选 ablation，文中的“当前
> 默认”和复现命令只对当时 revision 成立。当前生产 runner 已收敛为
> `--ceres-multi-imu-init kalibr-style`：一次确定性初始化加一次 joint solve，旧
> selector 参数已经删除。本文的数值仍作为历史对照保留，当前结果见
> `20260729_Ceres与KalibrDocker多数据集速度精度全量复核.md`。

## 结论先行

这轮先定位 `b09 / 2025_04_19_19_20_46`，再补跑 12 组 full adaptive joint，并用 `simulation/generated/one_cam_four_imus` 做 ground-truth sanity check。结论是：当前还没有找到可以在不使用 Kalibr result 的情况下，把 benchmark 后半组的 1cam+4imu joint 拉到现有 Kalibr-init tight joint 同量级的配置。

### 2026-06-27 收尾决策

`1cam+Nimu` 的 runner 默认配置收尾为
`--ceres-multi-imu-candidate-preset no-kalibr-accuracy-trimmed`。这个默认只按
topology 触发：1 个 camera corners + 多个 IMU data 时启用，不按 benchmark
编号、采集时间或数据集名称分支。它不使用 Kalibr result 初始化，也不使用
Kalibr result 做候选选择；Kalibr 只在 benchmark harness 的最后 compare。

当前 trimmed 六候选：

```text
single_short
single_long
chain_multi_t_wide_tight
chain_multi_ograv_t_wide_tight
chain_accel_refine_t_wide_tight
chain_long_single_time_t_wide_tight
```

当前最优视图用 b01-b06 的 full no-Kalibr preset 结果，加 b07-b12 最新
六候选 focused 结果合并统计：

| 指标 | 当前 no-Kalibr `1cam+Nimu` |
|---|---:|
| C/K camera0 平移均/中/最大 | `13.1 / 9.2 / 37.4 mm` |
| C/K camera0 旋转均/中/最大 | `0.130 / 0.069 / 0.562 deg` |
| time offset 绝对误差均/最大 | `0.875 / 1.985 ms` |
| wall time 总/均值 | `12850 s / 1071 s` |

这不是 Kalibr-init tight joint 的完成态；它是当前 no-Kalibr 可落地的默认
折中。b09 仍是平移 tail，b11 仍是旋转 tail。仿真 `one_cam_four_imus`
上同一 selector 会回退 `single_long`，camera GT 为
`0.205 mm / 0.0028 deg / -0.097 ms`，说明 health gate 对可控金标场景仍能
保护 camera 外参。

几个负结果比较明确：

- 只扩大 IMU offset 搜索并显式优化非参考 IMU offset，不能解决 b09 joint 外参偏差。
- 用 Ceres single 结果生成 joint 初值是可落地的 no-Kalibr 方向，但必须按 Ceres/Kalibr rotation-vector 约定写入 `r_i_b`。runner 已修正该符号问题；修正后非参考 IMU chain 不再由符号错误主导。
- 符号修正后，b09 的 Ceres-single seed 仍停在 single basin：camera0 对 Kalibr joint 仍约 `95.7 mm / 0.858 deg`，但 residual 与当前 Kalibr-init tight joint 同量级。这说明 b09 的 benchmark C/K joint 差异不能只按“Ceres 错”解释。
- `--stage-solver-absolute-step-tolerances=-1,-1` 能证明默认 stage step stop 对 no-Kalibr seed 过早，但延长迭代本身不能把结果拉回好 basin，还会放大 accel outlier；`--solver-restore-best-state` 也不能修复，因为坏 tail 本身就是低 cost 状态。
- `b09/imu3` 的 C/K single outlier 不是单纯 time-shift prior 过硬造成；把 prior 从 `0.1 ms` 放宽到 `1 ms` 只把 time offset 差从 `2.17 ms` 降到 `0.79 ms`，平移仍有 `36.7 mm`。
- 加强 full-spline pose motion prior 可以压低 accel max，但不能把 b09 拉到 joint basin。b09 no-Kalibr 的更直接问题是：chain-prior 默认没有可靠非参考 IMU lever 初值，而 tight bound 又只允许 `±3 mm`，对真实十几厘米级 IMU 间平移不可能收敛。
- 新增 b09 重跑确认：保留 camera translation prior 的 no-Kalibr chain-prior 仍停在 `79.4 mm / 1.055 deg`；禁用 camera translation prior 后恶化到 `170.4 mm / 1.055 deg`。因此 camera prior 虽然有偏，但不是应该直接关掉的坏开关；真正缺口是可靠的 multi-IMU lever/camera basin 初始化。
- 新增显式实验开关 `--estimate-multi-imu-translation-prior`：联合估计 camera translation、非参考 IMU lever 和每路 accel bias 后，b09 accel mean 从 `0.88` 降到 `0.42 m/s^2`，非参考 effective chain 平移差从 `~0.20-0.25 m` 降到 `0.054-0.082 m`；但 camera0 仍为 `79.4 mm / 1.055 deg`。该方向有效，但还不能替代 Kalibr-init。
- 给 multi-IMU translation prior 加 lever/bias 正则后，b09 camera0 平移可从 `79.4 mm` 拉到 `55.3 mm`，更强正则可到 `35.7 mm`；但 rotation 仍约 `1.055 deg`，effective chain rotation 仍明显偏离 Kalibr joint。该结果说明“物理弱先验”能影响 translation basin，但不能单独解决 1cam+Nimu joint 精度。
- 放开显式 `--estimate-multi-imu-translation-prior` 在 `--init-from-result` 后运行后，Ceres-single seed 路径从 `95.7 mm / 0.858 deg` 改善到 `77.3 mm / 0.858 deg`，最终 accel mean 降到 `0.263 m/s^2`，非参考 effective chain 平移约 `38/50/36 mm`。这比原 Ceres-single seed 健康，但仍没有进入 Kalibr joint 同量级。
- 新增 camera translation prior sigma 实验：`--multi-imu-translation-prior-camera-sigma-m 0.02` 对 b09 没有增益，因为它围绕当前 `state.T_c_b` 加正则，而 no-Kalibr runner 在它之前已经跑了 reference-only camera translation prior，两者中心几乎重合。
- 新增 stage-specific IMU extrinsic bound：`0.05,0.05,0.005 rad` 的 wide/wide/tight 路径能把 b09 nonref effective rotation 从旧的 `2.9/4.55/1.78 deg` 降到 `0.32/0.94/0.26 deg`，accel mean 到 `0.259 m/s^2`，但 camera0 仍为 `81.6 mm / 0.858 deg`。它是 no-Kalibr accuracy preset 的必要机制，不是最终跨 basin 方案。
- Ceres-single seed + 强正则的 init-only 反而把 camera0 平移推到 `90.0 mm`，只让部分非参考 effective chain 略好；这条正则不适合直接叠到 single-seed 路径。
- 新增 residual-weighted single translation seed 诊断：b09 上能把 camera0 平移从 `95.7 mm` 降到 `21.3-23.4 mm`，但这是把四个 single `T_c_b` 平移按 residual 加权平均；这些 `T_c_b` 分别是 camera 到不同 IMU 的外参，不在同一个 body frame。b01 反例把原本健康的 joint 拉坏到 `79.5 mm / 0.230 deg`、accel mean `0.279 m/s^2`。该路线只能作为 basin 诊断，不能进入默认或 accuracy preset。
- 新增 stop policy 边界实验：b01 的 Ceres-single seed 默认短解被 `step_norm < 0.02` 过早停止，no-step 长解把 residual 拉回接近 Kalibr；但 b09 的 no-step 长解即使用更强 accel loss 也会让 accel health 和外参更差。因此 1cam+Nimu no-Kalibr 不能简单全局关闭 step stop，需要 short/long 候选加 residual-health 选择。
- 12 组 full adaptive joint 已跑完。runner 选了 `long` 11 组、`short` 1 组；b09 的 long 被 `short_accel_max_health` 正确拒绝，因为 long accel max 从 `5.92` 爆到 `26.84 m/s^2`。adaptive 能证明 `step_norm < 0.02` 在 1cam+Nimu 粗阶段经常早停，但不能把后半组外参拉回 Kalibr joint basin：C/K camera0 平移均/中/最大为 `33.1 / 24.6 / 95.7 mm`，effective chain case-max 为 `36.3 / 31.3 / 95.7 mm`。
- 后半组 `b07-b12` 仍是 no-Kalibr 1cam+Nimu 的主风险，但风险已经收缩：旧 adaptive single/long 路径里 `b07/b08/b10/b11/b12` 的 C/K camera0 为 `35-64 mm`，b09 为 `95.7 mm`；当前六候选 selector 已把 b07/b08/b10/b12 分别拉到 `4.8/22.2/7.4/6.7 mm`，b09 为 `37.4 mm / 0.148 deg`，b11 为 `19.4 mm / 0.562 deg`。后续主要风险不再是“某个数据集专用参数没调”，而是 no-Kalibr joint 与 Kalibr joint 在部分实测数据上会落入不同 basin。
- 新增 b11 stop policy 小样本：stage0 禁用 step stop、stage1 用 `absolute_cost_change_tolerance=5` 时，wall 约 `208s`，accel mean/max 为 `0.1375/1.31`，接近 full long 的 `0.1372/1.30`，但外参仍为 `63.7 mm / 0.197 deg`。这说明 cost-stop hybrid 可以省 stage1 尾部时间，但不修外参 basin。
- 新增 b09 固定 IMU extrinsics 小样本：Ceres-single seed 后加 `--fix-imu-extrinsics`，camera0 仍为 `97.7 mm / 0.455 deg`，accel mean/max 为 `0.603/22.05 m/s^2`。直接固定 single-seed IMU chain 不能替代 joint 初始化。
- 新增 b09 reference reorder 小样本：把 IMU 输入顺序改为 `data3,data1,data2,data4`，并用对应 Ceres single seed 初始化。转回原始 IMU 顺序后，原 imu0 effective chain 仍为 `93.1 mm / 1.18 deg`，原 imu1 为 `36.0 mm / 0.91 deg`。单纯换 reference IMU 不够，必须同时解决 time/lever/reference camera 的联合初始化。
- 新增 b09 四 reference 候选 + `--estimate-multi-imu-translation-prior`：ref_imu1 仍是最好，mapped max chain 为 `77.3 mm / 1.27 deg`；ref_imu4 接近但略差，为 `83.0 mm / 1.26 deg`；ref_imu2/ref_imu3 更差。再把 ref_imu1 的 stage0 改成长跑，只把 accel mean 从 `0.263` 降到 `0.260`，max chain 仍为 `77.3 mm`。这条组合路线没有把 b09 拉到 Kalibr-init 同量级。
- 优化后 spline 诊断显示：no-Kalibr chain-prior 的有效区间 spline acceleration p99 约 `1.75 m/s^2`，max 约 `4.69 m/s^2`，没有 Kalibr-init tight joint 日志中 `45.11s` 的 `18.6 m/s^2` 极端尖峰。no-Kalibr 的主要风险仍是外参 basin，不是单纯 spline 爆炸。
- 仿真 `1cam+4imu` 的 no-Kalibr Ceres 仍能贴近真值：默认 joint camera 约 `0.70 mm / 0.031 deg`，非参考 IMU lever 为 `8.4/10.3/11.3 mm`；Ceres single-seed joint camera 约 `0.77 mm / 0.084 deg`，lever 为 `7.0/9.8/11.8 mm`。这说明 Ceres 多 IMU 实现不是整体错误，b09 更像实测数据/初始化/可观性特例。
- 仿真上显式 multi-IMU translation prior 会把默认 camera 平移从 `0.70 mm` 推到 `6.53 mm`，强正则会到 `10.06 mm`；因此它目前只能作为消融/诊断开关，不能进入 `--corner-defaults` 默认。
- 已补一个防坏初值的代码门限：`--imu-chain-prior-max-lever-accel-rms`，并把 multi-IMU `--corner-defaults` 默认改为尝试 lever prior + `0.5 m/s^2` RMS gate。仿真 lever 候选 RMS 为 `0.056-0.117 m/s^2`，可接受；b09 候选 RMS 为 `0.703/0.863/0.717 m/s^2`，会被拒绝并保持 `r_b=0`。该补丁只防止坏 lever prior 被使用，不会单独解决 b09。
- 新增 camera-IMU time-shift prior 修复：旧实现对正值 gyro norm 做未去均值、未归一化 full-lag 点积，天然偏向 0 lag。改为去均值归一化相关后，当时的符号实现令 b09 的 chain-prior 初值从 `0 ms` 改成 `+4.007 ms`，与 Kalibr joint `+3.842 ms` 只差 `0.166 ms`。**2026-07-28 更正：**该 `+4.007 ms` 使用了与 `np.correlate`/Kalibr 不一致的 `shift=+lag\,dT`；修正为 Kalibr 的 `shift=-lag\,dT` 后，同一平坦峰报告 `-4.007 ms`。因此 b09 只能证明边界 gate 没触发，不能作为符号正确性的证据。同一符号错误也把 b01 的 full-search 结果记成了 `+122 ms`“假峰”；修正后是 `-122.074 ms`，与当前 Kalibr single `-117.531 ms` 同号且只差 `4.543 ms`。本轮 12 组复核表明，旧 `0.05 s` 窗口会误拒绝前 6 组约 `-0.09~-0.12 s` 的有效峰，因此当时默认窗先改为经该矩阵验证的 `0.2 s`，同时保留 `boundary_peak_rejected` 防止接受真正的裁剪峰。**2026-07-29 更新：**后续四 session 复跑证明 `0.2 s` 仍会误拒绝 9 路有效峰；当前默认已改为最小 `50%` 重叠限定的全范围搜索，详见 [20260729_相机IMU全范围时间偏移初始化复核.md](20260729_相机IMU全范围时间偏移初始化复核.md)。
- 在 time-shift 修复基础上，b09 的 no-Kalibr `chain-prior + multi-IMU translation prior + 0.01m/0.05m/s^2 正则 + wide/wide/tight stage` 从旧 `35.7 mm / 1.055 deg / -3.84 ms` 改到 `28.7 mm / 1.060 deg / -0.002 ms`，accel mean 从 `0.432` 降到 `0.326 m/s^2`。这是当前 b09 chain-prior 方向的最好结果，但 rotation 和 non-reference effective chain 仍明显落后 Kalibr-init tight，不满足完成条件。
- 新增 fixture-median translation seed 反例：从其它 11 个 Ceres single case 取 IMU 间 `r_b` 中位数是物理上比平均 `T_c_b` 更合理的尝试，但 b09 两阶段结果为 `106.5 mm / 1.77 deg / accel 0.833`，三阶段结果仍为 `105.2 mm / 2.64 deg / accel 0.817`。它会和目标数据自身的 rotation/time/pose basin 冲突，不作为默认或 accuracy preset。
- 新增 runner 级 `--ceres-multi-imu-candidate-preset no-kalibr-accuracy`：显式跑 `single_short`、`single_long`、`chain_multi_t_wide_tight`、`chain_multi_ograv_t_wide_tight`、`chain_accel_refine_t_wide_tight`、`chain_multi_single_time_t_wide_tight`、`chain_multi_ograv_single_time_t_wide_tight`、`chain_long_t_wide_tight`、`chain_long_single_time_t_wide_tight` 九个不使用 Kalibr 初始化/选择的候选。selector 只看 Ceres residual health、camera time-shift boundary gate、chain accel max 绝对门限和候选 residual score；Kalibr 只在候选选完后作为 benchmark compare。当前代码另有 `no-kalibr-accuracy-trimmed` 六候选入口，用于下一轮 12 组验证。
- 历史 preset smoke 在当时的 `0.05 s` 窗口下，b01 因 chain camera time-shift `boundary_peak_rejected=1` 选 `single_long`，C/K camera0 为 `8.84 mm / 0.230 deg / -1.60 ms`；该拒绝后来确认是窄窗截断有效峰，不是 b01 存在反号假峰。b09 因 chain time-shift 非边界且 residual score 明显更好，选 `chain_multi_t_wide_tight`，camera0 从 single 的 `95.7 mm` 降到 `28.7 mm`，time offset 到 `-0.002 ms`。但 b09 non-reference effective chain 仍有 `65-80 mm`，所以该 preset 是阶段性改善，不是完成态。
- 新增 b01 六候选复跑：selector 仍选 `single_long`，并因 `time_boundary` 拒绝四个 chain 候选。纯 chain 的 camera0 C/K 为 `131.1 mm` 且 accel max 可到 `17.3/59.4 m/s^2`；只复用 Ceres single time 的 chain 候选仍有 `86.9 mm` 平移差，accel max `6.87/6.95 m/s^2`。这说明 single time seed 只能避免明显 time 假峰，不能修 reference camera / multi-IMU translation basin。
- 仿真 replay 暴露了 selector 的边界：只看 residual score 会在 `one_cam_four_imus` 上误选 chain，camera GT 从 single_long 的 `0.205 mm / 0.0028 deg` 退化到 `10.03-27.86 mm`，虽然 residual score 更低。加入 `chain_accel_max_abs=3.0 m/s^2` 绝对健康门限后，早期四条 chain 候选因 accel max `3.80/4.14/4.59/5.15 m/s^2` 被拒绝并回退 `single_long`；当前六候选 replay 继续证明该 gate 能保护仿真 GT。
- 旧 12 组 benchmark 离线 replay 已被当前六候选 focused 结果部分替代：b08 当前 selector 选 `chain_long_single_time_t_wide_tight`，camera0 C/K 为 `22.2 mm / 0.045 deg / +0.33 ms`；b12 当前 selector 同样选 `chain_long_single_time_t_wide_tight`，camera0 C/K 为 `6.7 mm / 0.053 deg / +0.26 ms`；b09 当前等价六候选选择 `chain_accel_refine_t_wide_tight`，camera0 C/K 为 `37.4 mm / 0.148 deg / +0.04 ms`。因此旧离线统计 `13.4 / 9.5 / 32.4 mm` 不能再作为最新总览；下一步需要用当前六候选重跑完整 12 组后重算总表。
- 新增 b09 reference-order 当前复跑：用物理有效的 Ceres single reference seed，枚举 ref_imu1/2/3 并使用 no-step + cost-stop staged，mapped max chain 仍为 `95.7/97.0/97.9 mm`，max rotation 为 `1.77/1.77/1.63 deg`。因此“只换 reference IMU”不能解决 b09；ref_imu4 本轮被中断，旧 reference 枚举和当前 3/4 结果已经足以把该方向降级。
- 新增后半组 fixture-median extrinsic 反例：只用 b07/b08/b10/b11/b12 的 Ceres single 结果估 IMU 间 `r_b + r_i_b` 中位数，再跑 b09 ref_imu1 三阶段 wide/wide/tight，mapped max chain 仍为 `95.8 mm / 2.37 deg`，accel mean/max 为 `0.808 / 27.17 m/s^2`。这比只用 translation median 更坏，说明从 Ceres single 历史中位数直接构造夹具全外参 prior 不稳。
- 新增后半组 fixture-median camera-chain 反例：直接对 b07/b08/b10/b11/b12 的每路 Ceres single `T_c_i` 做跨 session translation median + rotation medoid，再由这些 `T_c_i` 组成 b09 joint seed。结果 mapped max chain 为 `124.1 mm / 3.11 deg`，accel mean `0.828 m/s^2`，比 b09 raw Ceres-single seed 更差。说明“同夹具 Ceres single 中位 camera-to-IMU 先验”也会继承 single basin 偏差，不能替代独立机械/CAD prior。
- 新增 multi-IMU orientation/gravity prior：在 IMU chain prior 之后，把各 IMU gyro 先转到 reference body frame 共同估计 `R_b_c`，再用四路 accel 平均 gravity。b09 的 gravity 初值从单 IMU prior 的 `[-0.128, -9.806, -0.048]` 改到 `[0.050, -9.806, 0.028]`，接近 Kalibr joint `[0.031, -9.806, -0.018]`。配合 chain multi-translation prior 后，b09 effective chain 平移从旧 `28.7/79.7/70.7/65.3 mm` 改到 `36.0/54.6/45.6/37.9 mm`，accel mean/max 从 `0.326/2.35` 降到 `0.302/1.50 m/s^2`。这是当前第一个同时降低 b09 nonref tail 和 accel tail 的 no-Kalibr 初始化改动；但 camera0 从 `28.7` 退到 `36.0 mm`，rotation 仍约 `1.04 deg`，还不是完成态。
- 新增 b09 multi-IMU orientation/gravity long 版本：禁用 stage step stop 后结果基本不变，camera0 `35.9 mm`，nonref `54.5/45.7/37.9 mm`，accel mean/max `0.302/1.50 m/s^2`。说明该候选的收益来自 multi-IMU gravity/rotation 初值，不来自更长迭代；不应仅为它增加默认耗时。
- 新增 b09 七候选 full preset 复跑：selector 选择 `chain_multi_ograv_t_wide_tight`，决策为 `chain_multi_ograv_t_wide_tight_score_within_single_short`。selected camera0 为 `36.0 mm / 1.044 deg / +0.007 ms`，effective chain 为 `36.0/54.6/45.6/37.9 mm`，accel mean/max 为 `0.302/1.50 m/s^2`。普通 chain 的 camera0 更近 `28.7 mm`，但 nonref tail 和 accel max 更差；long chain 和 single-time chain 没有收益，且 accel max 到 `2.96-4.67 m/s^2`。
- 新增 b09 Ceres-single seed + multi-ograv + multi-translation prior 混合候选：结果为 `80.5 mm / 1.535 deg / -2.96 ms`，accel mean/max `0.280/16.47 m/s^2`，wall `399 s`。它比 chain candidates 更慢且更差，说明 single-seed basin 与 multi-prior 强行叠加会制造 accel tail，不应加入 preset。
- 新增 multi-ograv 仿真 replay：`one_cam_four_imus` 上 selector 仍选择 `single_long`，因为所有 chain 候选被 `accel_max_abs` gate 拒绝。GT camera0：`single_long = 0.205 mm / 0.0028 deg`，`chain_multi_ograv_t_wide_tight = 11.33 mm / 0.0063 deg`。这证明 multi-ograv 候选必须受 residual-health gate 约束，不能进入 `--corner-defaults` 无条件默认。
- 新增旧四候选 trimmed preset 仿真 replay：四候选入口与 full replay 的选择一致，仍选择 `single_long`，拒绝 `chain_multi_t_wide_tight` 和 `chain_multi_ograv_t_wide_tight` 的原因都是 `accel_max_abs`。GT camera0 保持 `0.205 mm / 0.0028 deg / -0.097 ms`，两个 chain 的 camera0 分别为 `10.03 mm` 和 `11.33 mm`。这验证 trimmed 入口没有破坏仿真 camera 保护逻辑。
- 新增 IMU-IMU chain time offset normalized correlation：把 multi-IMU gyro norm delay 从原始点积改为去均值归一化 overlap correlation，并记录 IMU-chain boundary peak。b09 `chain_multi_ograv_t_wide_tight` 的 `imu1` offset 从 `0.2627s` 改到 `0.2647s`，camera0 仍约 `35.9 mm / 1.04 deg`，但 nonref effective chain tail 从 `54.6 mm` 降到 `46.1 mm`，accel max 从 `1.50` 到 `1.47 m/s^2`。仿真 trimmed preset 仍选择 `single_long`，GT camera0 保持 `0.205 mm / 0.0028 deg`，说明该改动是低风险小幅改善，不是完成态。
- 新增 runner audit 输出：`no-kalibr-accuracy*` 会为每个候选写出 `selection_status`，并在 summary 里保留候选的 camera time prior、IMU-chain prior peak/boundary、residual score 和 health 字段。选择逻辑不变；仿真 trimmed 旧结果复算仍选 `single_long`，两个 chain 候选状态为 `rejected:accel_max_abs,score`。
- 新增 b08 旧四候选 trimmed 复跑：selector 选 `single_long`，C/K camera0 为 `40.8 mm / 0.135 deg / -8.23 ms`，total preset wall `748.5 s`。两个 chain 候选的 IMU-chain prior 都健康（min peak `0.99975`、无边界峰），但 residual 退化到 accel mean/max `0.357-0.367 / 4.72-4.73 m/s^2`，状态为 `rejected:accel_mean,accel_max_abs,score`。该结果已经被后续六候选 current selector 取代，但它证明 b08 tail 不是 chain time/lever gate 漏放。
- 新增 b08 chain + single-time seed 聚焦验证：把 chain 候选的 camera time 初值从 normalized camera time prior 的 `-12.0 ms` 改成 selected `single_long` 的 `+10.76 ms`，camera0 C/K 从普通 chain 的 `62.6-68.0 mm / -30.9 ms` 改到 `37.1 mm / -8.20 ms`，但 accel mean/max 仍为 `0.261 / 4.34 m/s^2`，差于 selected `single_long` 的 `0.177 / 2.89 m/s^2`。因此 b08 的 false camera time prior 会放大 chain 偏差，但单独复用 single time 仍不能让 chain 通过 health gate。
- 新增 b08/b12 `chain_long_single_time_t_wide_tight` 聚焦验证：该候选不用 Kalibr 初始化，只复用 Ceres single 的 camera time seed，并用 no-step + cost-stop staged 解 chain。b08 从 trimmed selected `40.8 mm / 0.135 deg / -8.23 ms` 改到 `18.9 mm / 0.025 deg / +0.29 ms`，accel mean/max 为 `0.124 / 1.18 m/s^2`；b12 为 `18.2 mm / 0.056 deg / +0.26 ms`，accel mean/max 为 `0.126 / 1.12 m/s^2`。这说明 slow single-time chain 是 b08/b12 的有效 no-Kalibr 候选，值得放进 trimmed 入口，但必须继续受 health gate 约束。
- 新增 `chain_multi_ograv_single_time_t_wide_tight` 聚焦验证：该候选不使用 Kalibr 初始化，只把 multi-IMU orientation/gravity prior 与 Ceres single camera-time seed 组合。b09 camera0 从 `chain_multi_ograv` 的 `36.0 mm / 1.044 deg / +0.007 ms` 小幅变为 `33.1 mm / 1.031 deg / -3.22 ms`，non-reference max chain 约 `46.9 mm`，accel mean/max 为 `0.309 / 1.44 m/s^2`；b08 为 `37.9 mm / 0.416 deg / -8.24 ms`，但 accel mean/max 为 `0.254 / 4.38 m/s^2`，仍应被 `accel_max_abs` gate 拒绝。结论：这个组合是可加入 full preset 的候选，但不是默认完成态，也不应放进 trimmed 快速验证入口。
- 新增 b09 非参考 IMU time offset 小范围释放负样本：在 `chain_multi_ograv_single_time` 和 `chain_multi_t` 上启用 `--optimize-imu-time-offsets`，默认 `±5 ms` bound。前者 camera0 仍为 `33.1 mm / 1.031 deg / -3.23 ms`，non-reference max chain `49.8 mm`，wall 从约 `289s` 增到 `415s`；后者 camera0 仍为 `28.7 mm / 1.060 deg / +0.015 ms`，non-reference max chain `74.0 mm`，wall `434s`。因此当前 b09 tail 不是固定 IMU-IMU delay 的简单问题，offset refine 不应加入 no-Kalibr preset。
- 新增 b09 `chain_multi_t` init-only 对照：multi-IMU translation prior 之后、不做主优化时，camera0 已经是 `28.7 mm / 1.063 deg / +0.166 ms`，与优化后 camera0 基本同一 basin；但 non-reference effective rotation 是 `3.23 / 4.95 / 2.09 deg`，accel mean/max 为 `0.664 / 3.06 m/s^2`。主优化主要把 non-reference rotation 和 residual 拉健康，却没有把 camera0 平移/旋转跨到 Kalibr-init tight 量级。这把下一步重点从 time offset 排除，转向 IMU chain rotation/lever 初值质量与 Ceres-only basin 判别。
- 新增 b09 `chain_multi_t` init-only 禁用 gyro Ceres refine 对照：`--no-imu-chain-prior-ceres-refine` 后 `refine_iterations=0`，但 camera0、non-reference effective chain 和 accel residual 与默认 init-only 完全一致。说明 b09 的 IMU-chain rotation 初值不是被后续 gyro refine 拉坏，而是 gyro-only pair initialization 本身已经进入该 basin。
- 新增实验开关 `--imu-chain-prior-refine-with-accel`：在 IMU chain prior 内部联合 refine 非参考 IMU `R_i_b`、lever 和 accel-bias delta，默认关闭。b09 `gyro_weight=10 / accel_weight=1 / rotation_bound=0.1rad / lever_sigma=0.01m / accel_bias_sigma=0.05m/s^2` 后，camera0 为 `37.4 mm / 0.148 deg / +0.044 ms`，effective chain 为 `37.4/25.4/32.3/23.2 mm` 且旋转均低于 `0.13 deg`，accel mean/max 为 `0.212 / 1.90 m/s^2`。这是当前第一个明显修正 b09 chain rotation 的 no-Kalibr 候选，但 camera0 平移差仍高于 `chain_multi_t` 的 `28.7 mm`，gyro residual 也更高，不满足默认或完成条件。
- 新增 b09 accel-refine + multi-ograv 负样本：叠加 `--estimate-multi-imu-orientation-gravity-prior` 后 camera0 平移看似到 `15.8 mm`，但 rotation 退到 `1.69 deg`，accel mean/max 变为 `0.281 / 10.09 m/s^2`。该组合应被 residual-health gate 拒绝，不能加入 preset。
- 新增 b09 accel-refine `gyro_weight=50` init-only 负样本：更强 gyro 权重基本退回 gyro-only basin，camera0 为 `29.2 mm / 1.063 deg`，non-reference rotation 仍有 `1.77-4.41 deg`，accel mean/max 为 `0.644 / 3.00 m/s^2`。说明 accel term 必须足够强才会修 rotation，但强 accel term 又会带来平移/gyro权衡。
- 新增仿真和 b08/b12 推广验证：`one_cam_four_imus` 上同一 accel-refine 候选的 GT camera 为 `9.89 mm / 0.0068 deg`，比当前 selector 保护的 `single_long ~0.205 mm` 明显退化，且 accel max `3.81 m/s^2` 超过现有 `3.0` gate；b08 退化到 `62.7 mm / 0.458 deg / -30.9 ms`，accel max `4.74 m/s^2`；b12 为 `25.2 mm / 0.322 deg / -42.2 ms`，accel max `4.69 m/s^2`。因此 `--imu-chain-prior-refine-with-accel` 只能作为 health-gated candidate 进入 `no-kalibr-accuracy*`，不能下沉为 `--corner-defaults` 默认或无条件接受。
- 新增 b09 `post-accel-rot` hybrid 负样本：`--imu-chain-prior-refine-rotation-after-translation-prior` 先用普通 chain 保留 multi-IMU translation prior 的 `28.7 mm` camera basin，再只重写 accel-refine 后的 IMU chain rotation。主优化后结果仍退回普通 chain：camera0 为 `28.71 mm / 1.059 deg / -0.114 ms`，accel mean/max 为 `0.322 / 2.29 m/s^2`，non-reference effective chain 仍约 `69-72 mm`。它没有继承 accel-refine 的 `~0.15 deg` rotation 收益，且单次 b09 额外耗时约 `386 s`，因此从 `no-kalibr-accuracy*` preset 移除，只保留 C++ 手动诊断开关。
- 新增五候选 trimmed 仿真 replay：把 `chain_long_single_time_t_wide_tight` 加入 trimmed 后，selector 仍选择 `single_long`。GT camera0 保持 `0.205 mm / 0.0028 deg / -0.097 ms`；`chain_long_single_time_t_wide_tight` 自身 GT camera 为 `27.86 mm / 0.0084 deg / -0.138 ms`，accel max `5.15 m/s^2`，状态为 `rejected:accel_max_abs`。这说明 slow candidate 对 b08/b12 有收益，但在仿真上会牺牲 camera GT，不能直接做无条件默认。当前 trimmed 代码额外包含 `chain_accel_refine_t_wide_tight`，该候选也必须由同一 accel max gate 保护。
- 新增六候选 trimmed 仿真 replay：当前 trimmed 加入 `chain_accel_refine_t_wide_tight` 后仍选择 `single_long`，GT camera0 为 `0.205 mm / 0.0028 deg / -0.097 ms`。`chain_multi_t`、`chain_multi_ograv`、`chain_accel_refine` 的 GT camera 分别为 `10.03 / 11.33 / 9.89 mm`，accel max 为 `3.80 / 3.76 / 3.81 m/s^2`，均被 `rejected:accel_max_abs,score`；`chain_long_single_time` 的 GT camera 为 `18.60 mm`，accel max `4.63 m/s^2`，也被拒绝。该结果证明当前 6 候选 trimmed 在仿真 GT 上仍由 health gate 保护 camera 外参。
- 新增 b08/b12 当前六候选 selector 复跑：b08 选择 `chain_long_single_time_t_wide_tight`，camera0 C/K 从 `single_long` 的 `40.8 mm / 0.135 deg / -8.23 ms` 改到 `22.2 mm / 0.045 deg / +0.33 ms`，accel mean/max 为 `0.122 / 1.16 m/s^2`；b12 选择同一候选，camera0 C/K 从 `single_long` 的 `61.0 mm / 0.081 deg / -7.77 ms` 改到 `6.7 mm / 0.053 deg / +0.26 ms`，accel mean/max 为 `0.125 / 1.13 m/s^2`。这证明 slow single-time chain 已经可以通过 selector 自动接受，而不是只在 focused run 里有效。

当前应该把 b09 和 b11 作为收尾后的主要风险样本，而不是把 no-Kalibr 默认改成 unbounded、显式优化 offset，或只靠更强 pose prior。b07/b08/b10/b12 已确认可由当前六候选 selector 自动改善；仿真 replay 说明同一类 chain 候选必须继续受 `accel_max_abs` gate 保护。

同时，b01-b06 说明默认 step stop 对 Ceres-single seed 的前半组 benchmark 过早；b09 说明长跑必须有 residual-health gate。旧的 `--ceres-multi-imu-adaptive-long-solve` 仍保留为诊断入口；当前默认收尾在 runner 级 no-Kalibr trimmed 六候选 selector 上，不再把 adaptive short/long 当作最终策略。

## 背景

正式 SuiteB 当前的 tight joint 结果依赖 `--init-from-kalibr`，外参链 C/K joint effective 可以压到约 `<=5.2 mm`。实际使用不能先跑 Kalibr，因此需要探索 no-Kalibr 的 1cam+Nimu 路径。

本轮起初选择 b09，是因为它同时包含两个异常；后续 full adaptive 又把范围扩到 12 个 joint case：

- C/K single 最大项：`b09/imu3 = 41.6 mm / 0.831 deg / -2.17 ms`。
- Ceres single-joint 最大项：`b09/imu1 = 95.7 mm / 0.860 deg / -3.21 ms`。

Kalibr 自身 single-joint 最大也约 `96 mm`，所以 b09 的 single vs joint 不一致不能直接归因于 Ceres joint bug。

## 指标口径

| 指标 | 单位 | 含义 |
|---|---:|---|
| C/K Δt | mm | Ceres `T_c_b` 与 Kalibr `T_ci` 平移差 |
| C/K ΔR | deg | Ceres 与 Kalibr 旋转差 |
| Δτ | ms | Ceres time shift 减 Kalibr time shift |
| accel mean / max | m/s^2 | Ceres accel residual 均值 / 最大值 |
| residual delta accel | m/s^2 | Ceres accel mean 减 Kalibr accel mean |

注意：b09 没有金标。C/K 只说明两套工具是否一致，不直接证明谁更准确。

## Joint Ablation

| 配置 | 是否用 Kalibr init | 关键变化 | C/K Δt mm | C/K ΔR deg | Δτ ms | accel mean / max | 结论 |
|---|---|---|---:|---:|---:|---:|---|
| chain-prior + unbounded + no step stop + optimize IMU offset | 否 | gyro chain prior；非参考 offset 优化；外参无 bound | 79.4 | 1.056 | -3.87 | 0.790 / 7.22 | offset 优化不是主因 |
| Ceres-single seed + tight bound + pose prior + sign fix | 否 | 非参考 IMU 外参 `±3mm / ±0.005rad`；默认 stage step stop；`r_i_b` 按 Ceres 负号约定写入 | 95.7 | 0.858 | -3.26 | 0.844 / 5.92 | 符号已正确，但 camera0 仍保持 single basin |
| Ceres-single seed + tight bound + pose prior + sign fix + no step stop | 否 | 在上项基础上禁用 stage step stop | 95.7 | 0.858 | -3.25 | 0.828 / 26.84 | 多跑 stage 降 cost，但不修复 basin，accel outlier 变坏 |
| Ceres-single seed + sign fix + no step stop + restore best | 否 | 在上项基础上开启 `--solver-restore-best-state` | 95.7 | 0.858 | -3.25 | 0.828 / 26.84 | 与 no-step 相同；坏状态本身是低 cost 状态 |
| chain-prior + full-spline pose prior + stage variance `1/0.1` | 否 | `--pose-motion-all-segments`；stage pose variance 更强 | 79.4 | 1.055 | -3.85 | 0.906 / 4.62 | accel max 下降，但外参仍不对 |
| chain-prior + lever prior + accel RMS gate, init only | 否 | `--estimate-imu-chain-lever-prior --imu-chain-prior-max-lever-accel-rms 0.5`；stage iteration 0 | 79.4 | 1.055 | -3.84 | 1.033 / 5.23 | b09 lever 候选被正确拒绝，但初始化仍不够 |
| chain-prior + export spline controls | 否 | 当前 no-Kalibr chain-prior；保留 camera translation prior；导出 controls | 79.4 | 1.055 | -3.87 | 0.880 / 5.27 | 复现同一 basin，并留下 spline 诊断产物 |
| chain-prior + no camera translation prior | 否 | 显式 `--no-estimate-camera-translation-prior`；导出 controls | 170.4 | 1.055 | -3.87 | 0.883 / 5.47 | 更差；不能直接关闭 camera prior |
| chain-prior + multi-IMU translation prior | 否 | 显式 `--estimate-multi-imu-translation-prior`；联合估 `t_c_b/r_b/bias`；导出 controls | 79.4 | 1.055 | -3.84 | 0.420 / 4.27 | accel 与非参考 chain 明显改善，但 camera0 仍未跨 basin |
| chain-prior + multi-IMU translation prior + relaxed bound | 否 | 在上项基础上把非参考 IMU bound 放宽到 `0.2 m / 0.1 rad` | 79.4 | 1.054 | -3.79 | 0.329 / 6.25 | accel mean 更低，但 camera0 不动，effective chain 变差 |
| chain-prior + multi-IMU translation prior + reg `0.02m/0.10m/s^2` | 否 | 对非参考 lever 加 `0.02m` 正则，对 accel bias 加 `0.10m/s^2` 正则 | 55.3 | 1.055 | -3.84 | 0.423 / 4.21 | translation basin 改善，但 rotation 仍未解决 |
| chain-prior + multi-IMU translation prior + reg `0.01m/0.05m/s^2` | 否 | 更强 lever/bias 正则 | 35.7 | 1.055 | -3.84 | 0.432 / 4.08 | translation 继续接近 Kalibr joint，但 rotation/effective chain 仍明显异常 |
| chain-prior + normalized time shift + default staged | 否 | 历史运行接受 `+4.007ms` 非边界峰；该行使用了 2026-07-28 已纠正的旧 lag 符号，只能保留为历史结果 | 77.2 | 1.063 | +0.095 | 0.868 / 4.81 | normalized correlation 与边界 gate 有效，但本行不能证明 time-shift 符号正确 |
| chain-prior + normalized time shift + multi-IMU translation prior + reg `0.01m/0.05m/s^2` + wide/wide/tight | 否 | 在上一项基础上加 multi-IMU translation LS 正则和三阶段 IMU bound `0.05,0.05,0.005rad` | 28.7 | 1.060 | -0.002 | 0.326 / 2.35 | 当前 b09 chain-prior 最好结果；translation/time/residual 明显改善，但 rotation 和 nonref chain 仍未达 Kalibr-init tight |
| chain-prior + normalized camera/IMU-chain time shift + multi-ograv + multi-IMU translation prior + wide/wide/tight | 否 | IMU-IMU chain delay 也改为去均值归一化 correlation；记录并 gate IMU-chain boundary peak | 35.9 | 1.037 | +0.042 | 0.302 / 1.47 | nonref tail 从 `54.6 mm` 降到 `46.1 mm`，但 camera0 与 rotation 仍未达标 |
| Ceres-single seed + multi-IMU translation prior, init only | 否 | `--init-from-result` 后显式运行 multi-IMU translation prior；stage iterations 0 | 77.3 | 0.859 | -3.26 | 0.570 / 2.65 | 初值 cost 降到 `2.85e5`，effective chain 明显改善，但 camera0 仍不达标 |
| Ceres-single seed + multi-IMU translation prior | 否 | 上项再跑默认 staged joint | 77.3 | 0.858 | -3.23 | 0.263 / 1.25 | residual 明显健康，effective chain 到 `38/50/36 mm`，但 camera0 仍停在 single-joint 中间 basin |
| Ceres-single seed + multi-IMU translation prior + reg `0.01m/0.05m/s^2`, init only | 否 | 对 single-seed 路径加同样强正则 | 90.0 | 0.859 | -3.26 | 0.570 / 2.66 | camera0 更差，只改善部分非参考 chain；不跑完整 |
| chain-prior + multi-IMU translation prior + camera sigma `0.02m` | 否 | 对 multi-IMU translation LS 的 cam0 translation 加当前初值正则 | 79.4 | 1.055 | -3.84 | 0.420 / 4.27 | 中心等于 reference-only translation prior，结果无变化 |
| chain-prior + no reference camera translation prior + reg `0.02m/0.10m/s^2` | 否 | 禁用 reference-only camera translation prior，只用 multi-IMU translation prior + 正则 | 55.3 | 1.055 | -3.84 | 0.423 / 4.21 | translation 改善但 rotation 不动，accel 不优 |
| Ceres-single seed + multi-IMU translation prior + rotation bound `0.05rad` | 否 | 放宽非参考 IMU rotation bound；translation bound 仍 `0.003m` | 81.6 | 0.858 | -3.23 | 0.261 / 1.26 | nonref rotation/residual 改善，camera0 仍在 single basin |
| Ceres-single seed + multi-IMU translation prior + rotation bound `0.05rad` + reg `0.01m/0.05m/s^2` | 否 | 在宽 rotation bound 上加更强 LS 正则 | 90.1 | 0.858 | -3.23 | 0.261 / 1.29 | 正则没有跨 basin，camera0 更差 |
| Ceres-single seed + wide/wide/tight stage bound `0.05,0.05,0.005rad` | 否 | 新增逐阶段 IMU rotation bound；第三阶段 tight polish | 81.6 | 0.858 | -3.23 | 0.259 / 1.24 | staged bound 机制有效，residual 小幅改善，但不解决 camera0 translation |
| Ceres-single seed + residual-weighted camera translation, no multi prior | 否 | 用四个 single residual 反比平方加权平均 `T_c_b` 平移；不跑 multi-IMU translation prior | 21.3 | 0.858 | -3.26 | 0.836 / 5.74 | b09 camera0 平移改善，但 accel 爆高；这是跨 frame 平移平均，不物理 |
| Ceres-single seed + residual-weighted camera translation + camera sigma `0.001m` | 否 | 上项再跑 multi-IMU translation prior，并用 `0.001m` camera sigma 锚住 weighted seed | 23.4 | 0.858 | -3.22 | 0.263 / 1.34 | b09 residual 健康，但 b01 退化到 `79.5 mm`；作为默认被反例否决 |
| fixture-median IMU translation seed, two-stage | 否 | 用其它 11 个 Ceres single case 的 IMU 间 `r_b` 中位数替换目标 single 的 nonref translation | 106.5 | 1.766 | -4.26 | 0.833 / 5.45 | 物理上比平均 `T_c_b` 合理，但 b09 明显变差 |
| fixture-median IMU translation seed, wide/wide/tight | 否 | 上项改成三阶段 `0.05,0.05,0.005rad` | 105.2 | 2.636 | -4.26 | 0.817 / 5.70 | 放宽 rotation 也不解决，丢弃该方向 |
| post-half fixture-median full extrinsic seed, wide/wide/tight | 否 | 只用 b07/b08/b10/b11/b12 的 Ceres single 结果取 IMU 间 `r_b + r_i_b` 中位数 | 95.8 | 2.370 | -4.26 | 0.808 / 27.17 | 后半组同夹具假设也不稳，accel tail 爆炸 |
| post-half fixture-median camera-chain seed, wide/wide/tight | 否 | 只用 b07/b08/b10/b11/b12 的每路 Ceres single `T_c_i` 中位数/medoid 组成 joint seed | 124.1 | 3.107 | -4.26 | 0.828 / n/a | 比 raw single-seed 更差；Ceres single 跨 session 中位数不是可靠 joint prior |
| Ceres-single seed + `--fix-imu-extrinsics` | 否 | 固定 single-seed IMU chain，只优化 pose/bias/gravity/time/内参 | 97.7 | 0.455 | -1.68 | 0.603 / 22.05 | residual 比 raw single-seed 低，但 accel tail 很差，camera0 仍在 single basin |
| Ceres-single seed + reference reorder `data3,data1,data2,data4` | 否 | 以原 imu3 为 body 跑 joint，再离线转回原始 IMU 顺序 | 93.1 | 1.183 | -3.25 | 0.836 / 5.07 | 换 reference 不足以修原 imu0；time/lever/reference camera 仍需联合初始化 |

读数：

- no-Kalibr chain-prior 的 camera translation prior 会停在 `~79 mm` C/K 差附近；开放 IMU offset 后也没有跨到 Kalibr joint basin。
- 禁用 camera translation prior 后，camera0 translation 从接近 prior 的 `0.077/0.137/0.084 m` 退到近零，C/K 平移差扩大到 `170 mm`。这说明当前 prior 至少提供了有用的平移尺度，问题不是“prior 一关就好”，而是缺少能把 ref camera 与非参考 IMU lever 一起初始化到 joint basin 的机制。
- `--estimate-multi-imu-translation-prior` 将当前 reference-IMU camera prior 扩展为多 IMU 联合 LS。它能给出非参考 lever 初值并显著降低 Ceres accel residual，但解出的 `t_c_b` 仍与 reference-only prior 相同。因此 b09 的 camera0 basin 还需要额外信息或约束，单纯用所有 accel 样本加常值 bias 仍不足以改变 camera translation。
- 放宽非参考 IMU 外参 bound 后，Ceres 可以继续降 cost，accel mean 从 `0.420` 降到 `0.329 m/s^2`；但 camera0 仍停在 `79 mm`，非参考 effective chain 平移变成 `62-94 mm`。这排除了“只是 tight bound 太紧”的解释。
- 给 multi-IMU translation LS 加 lever/bias 正则能改变 camera0 translation：`0.02m/0.10m/s^2` 到 `55 mm`，`0.01m/0.05m/s^2` 到 `36 mm`。但 rotation 保持 `~1.055 deg`，非参考 effective chain rotation 仍有 `1.75-4.55 deg` 级偏差。它是有价值的初始化方向，但不能作为当前默认。
- Ceres-single seed 保留了 single 标定的状态。符号修正前的 Ceres-single ablation 已不作为当前结论依据；符号修正后，init-only 的 non-reference effective chain 已明显合理，例如 b09/imu3 初值约 `1.5 mm`；但 camera0 仍继承 `single_imu1` 与 joint 的约 `96 mm` 冲突。
- Ceres-single seed 再叠加 multi-IMU translation prior 后，初值 `T_c_b` 从 `0.086/0.112/0.079 m` 改成 `0.077/0.131/0.073 m`，C/K camera0 从 `95.7 mm` 降到 `77.3 mm`。完整 staged joint 后 camera0 基本不再移动，但 residual 从 init-only 的 `0.570 m/s^2` 降到 `0.263 m/s^2`，非参考 effective chain 平移为 `38/50/36 mm`，旋转为 `0.365/1.268/0.255 deg`。它是比 raw single-seed 更健康的 no-Kalibr 路径，但仍不是 Kalibr-init 同量级。
- 新增 camera sigma 只证明了一个 gauge 事实：当前 no-Kalibr runner 已经先跑 reference-only camera translation prior，所以后续 multi-IMU LS 中围绕当前 `T_c_b` 加 camera prior 不会提供新信息。要让 multi-IMU 数据影响 cam0 translation，必须禁用 reference-only prior 或引入与 fixture/lever 相关的独立约束。
- 新增 stage-specific bound 后，`0.05,0.05,0.005rad` 路径让非参考 IMU rotation 有足够自由度先修 chain，再 tight polish。b09 nonref effective rotation 降到 `0.32/0.94/0.26 deg`，accel mean 到 `0.259 m/s^2`；但 camera0 仍为 `81.6 mm`。这说明 wide bound 是 residual/chain 的必要改进，不是 camera translation basin 的充分条件。
- residual-weighted camera translation 是一个明确反例：b09 上 `single_imu1` 与 Kalibr single 几乎一致，但 Kalibr joint 把 `cam0-imu0` 拉向四路 single translation 的中心附近，所以加权平均会“看起来”接近 Kalibr joint；b01 四路 single 也都健康，但它们代表 camera 到四个不同 IMU 的物理外参，平均后落到夹具中心，导致 camera0 从正确的 imu0 外参退化到 `79.5 mm`。因此不能用 single residual 直接选择或平均 reference body translation。
- 对 Ceres-single seed 路径加 `0.01m/0.05m/s^2` 正则没有复现 chain-prior 路径的 camera translation 改善，camera0 反而变成 `90.0 mm`。这说明正则的作用依赖 seed basin，不能作为通用修复。
- normalized camera time-shift 修复解决了 b09 的 base time offset，但不解决 rotation basin。默认 staged 下 b09 time offset 从 `-3.84 ms` 改到 `+0.095 ms`，translation 仍约 `77 mm`；叠加 multi-IMU translation prior 后 translation 到 `28.7 mm`，但 rotation 仍约 `1.06 deg`，non-reference effective chain 仍有 `65-80 mm` 级平移差。后续不能只围绕 time shift 调参。
- **2026-07-28 更正：**b01 不是 normalized time-shift 的反例。`+122 ms` 来自后来确认的整体符号回归；正确的 applied shift 是 `-122.074 ms`，与 Kalibr/Ceres single/joint 的 `-117 ms` 附近一致。旧 `0.05 s` 窗口卡边界只是搜索范围不足。该阶段默认扩大为 `0.2 s`，`boundary_peak_rejected` 继续作为真正裁剪峰的防坏门限。**2026-07-29 更新：**当前默认为最小 `50%` 重叠限定的全范围粗到细搜索；`0.2 s` 只保留为用户显式可选的上限。
- 固定 IMU extrinsics 和换 reference IMU 都没有把 b09 拉到 Kalibr-init 同量级。固定外参会让 stage1 继续通过 pose/control 降 cost，但 accel tail 到 `22.05 m/s^2`；换到 imu3 body 后，转回原始 imu0 仍有 `93.1 mm`。这两条路线都不能单独作为 production 默认。
- 对 no-Kalibr seed，`step_norm < 0.02` 的 stage stop 确实过早：sign-fix 默认 stage0 在 iter 5 停止时 `cost_change=4312`。但禁用 step stop 后只是把 stage0 跑满 30 次、cost 从 `4.12e5` 降到 `2.86e5`，最终 camera0 仍是 `95.7 mm`，且 accel max 从 `5.92` 恶化到 `26.84 m/s^2`。开启 `--solver-restore-best-state` 后结果不变，说明需要 residual-health gate，而不是 cost best-state。
- 当前 Ceres Kalibr-init tight joint 的 b09 accel mean 也约 `0.865 m/s^2`，sign-fix Ceres-single seed 默认为 `0.844 m/s^2`。这里说的是 Ceres hot-start residual，不是 Kalibr Docker 自身 residual。因此 b09 的 no-Kalibr 风险不是 residual 数量级更差，而是 benchmark 无金标时 single basin 与 Kalibr joint basin 的外参差异不可直接判定。
- b09 的 chain-prior 日志显示：非参考 IMU rotation prior 大体合理，但默认 `lever_estimated=0`，`r_b_m=0`。Kalibr joint 中 `T_ib` 平移约为 `0.126 m`、`0.151 m`、`0.187 m` 量级；如果继续使用 `±3 mm` bound，no-Kalibr chain-prior 从零 lever 出发没有足够自由度到达正确链路。
- `--estimate-imu-chain-lever-prior` 在 b09 上估出的 lever 只有几厘米或更小，且旧实验会造成 `166 deg` 级错误；它在仿真上有帮助，但不能直接作为 b09 实测默认。
- `--imu-chain-prior-max-lever-accel-rms 0.5` 能把 b09 这些坏 lever 候选拒掉。拒绝后 rotation/time chain prior 仍可用，但非参考 IMU 平移又回到零初值；因此后续还需要更可靠的 lever 来源，而不是只做 gate。
- 后半组 fixture-median full extrinsic 继续失败，说明历史 Ceres single 本身携带 single basin 偏差；直接从这些 single 解统计 `r_b/r_i_b` 并不能替代 CAD/机械夹具先验。
- 后半组 fixture-median camera-chain 继续失败，说明即使直接统计每路 `T_c_i`，也不能把 b09 从 single basin 拉向 joint basin。若要用 fixture prior，需要来自独立测量或已验证稳定的生产先验，而不是从同一批 Ceres single outlier 中估。

wide/wide/tight 复现命令：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_04_19_19_20_46 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_wide_tight_bound_20260625 --ceres-multi-imu-init ceres-single --ceres-multi-imu-single-seed-root out/docker_benchmarks/multi_imu_arm64 --ceres-multi-imu-translation-prior-lever-sigma-m 0.02 --ceres-multi-imu-translation-prior-accel-bias-sigma 0.1 --ceres-multi-imu-stage-free pbg,pbegti,pbegti --ceres-multi-imu-stage-iterations 30 --ceres-multi-imu-stage-extrinsic-translation-bounds-m 0.003,0.003,0.003 --ceres-multi-imu-stage-extrinsic-rotation-bounds-rad 0.05,0.05,0.005
```

## Reference Candidate Ablation

本组离线枚举 reference IMU，只使用 Ceres single 结果生成 no-Kalibr
joint seed；Kalibr result 只在 Ceres 完成后用于 mapped effective chain
对比。这里保留结果结论，不把一次性枚举工具作为生产入口。

本组问题：如果 b09 的主风险是 reference IMU 选错，那么枚举四个 reference，并叠加当前最有效的 `--estimate-multi-imu-translation-prior`，是否能找到接近 Kalibr-init tight joint 的候选？

| Reference | Ceres IMU order | mapped max Δt | mapped mean Δt | mapped max ΔR | accel mean / max | orig imu1 / imu2 / imu3 / imu4 Δt |
|---|---|---:|---:|---:|---:|---:|
| ref_imu1 | `1,2,3,4` | 77.32 mm | 50.52 mm | 1.268 deg | 0.263 / 1.25 | 77.32 / 38.36 / 50.10 / 36.29 mm |
| ref_imu4 | `4,1,2,3` | 82.99 mm | 48.80 mm | 1.259 deg | 0.263 / 1.23 | 82.99 / 32.27 / 33.81 / 46.15 mm |
| ref_imu2 | `2,1,3,4` | 96.07 mm | 62.80 mm | 1.257 deg | 0.264 / 1.18 | 96.07 / 44.43 / 51.96 / 58.74 mm |
| ref_imu3 | `3,1,2,4` | 138.76 mm | 94.49 mm | 1.625 deg | 0.294 / 3.49 | 138.76 / 71.94 / 85.13 / 82.15 mm |

ref_imu1 仍然最好，说明 b09 不是简单“换个 reference IMU 就好”。ref_imu4 的 mean translation 略低，但 max translation 更高，且 max rotation 同量级；不能作为更稳候选。ref_imu3 与前一轮手工 reorder 结论一致：它只会让原 imu3 body 自身看起来较好，转回原始 imu0/imu1 后仍明显偏。

为了排除 ref_imu1 仍被 stage0 step stop 卡住，又补跑长 stage0 版本。结果：
mapped max Δt `77.32 mm`、mapped max ΔR `1.261 deg`、accel mean/max
`0.260 / 1.25`。长 stage0 只轻微改善 residual，不改变外参 basin。

## Stop Policy Boundary Check

这组只比较 Ceres-single seed 的 short/long 行为，不使用 Kalibr result 初始化。Kalibr 结果只作为 benchmark 对比。

| Case | 配置 | C/K Δt mm | C/K ΔR deg | Δτ ms | Ceres residual mean `reproj / gyro / accel` | accel max | 读数 |
|---|---|---:|---:|---:|---|---:|---|
| b01 | 默认 short | 8.83 | 0.230 | -1.66 | `0.323 / 0.0264 / 0.264` | 6.81 | step stop 在 2+1 次迭代后停止，residual 明显偏高 |
| b01 | no-step long | 8.83 | 0.230 | -1.55 | `0.203 / 0.0162 / 0.159` | 6.56 | residual 接近 Kalibr，但 reference camera0 外参仍停在 single basin |
| b09 | 默认 short | 95.72 | 0.858 | -3.26 | `0.216 / 0.0501 / 0.844` | 5.92 | 默认短解没有解决 camera0 basin，但 accel max 可控 |
| b09 | no-step long + accel loss width 3 | 95.72 | 0.858 | -3.28 | `0.190 / 0.0480 / 0.888` | 9.33 | reproj 更低，但 accel mean/max 和 gravity 更差；长解应被 health gate 拒绝 |
| b11 | stage0 no-step + stage1 step `0.02` | 63.68 | 0.197 | +0.34 | `0.197 / 0.0272 / 0.1378` | 1.32 | stage0 长跑后 residual 健康，但外参不变 |
| b11 | stage0 no-step + stage1 cost `5` | 63.68 | 0.197 | +0.35 | `0.197 / 0.0271 / 0.1375` | 1.31 | 比 step hybrid 多约 `9s`，接近 full long residual，但仍不修外参 |

读数：

- b01 证明 `step_norm < 0.02` 对 no-Kalibr Ceres-single seed 过早，不能直接沿用 Kalibr-init tight 的快速停止策略。
- b09 证明不能把 no-step 作为全局默认；低 reprojection / 低 robust cost 可能伴随更差 accel health。
- b11 证明 stage1 可以用 cost tolerance 省掉 long tail；但外参 basin 在 stage0 结束时已经确定，stage1 stop policy 不是精度修复手段。
- 当前策略方向是 runner 级 adaptive：先跑 short，再跑 no-step long；只有当 long 的 accel mean 不超过 short 的 `1.05x` 且 accel max 不超过 short 的 `1.5x` 时才接受 long。12 组 full adaptive 证明这个 gate 能拒绝 b09 的 accel tail，但不能解决后半组外参 basin。

## Full Adaptive 12 组结果

这组使用完整 12 个 benchmark multi-IMU session，只跑 joint `1cam+4imu`，Ceres 初始化来自 `out/docker_benchmarks/multi_imu_arm64` 中已有的 Ceres single 结果。Kalibr arm64 结果只通过 `--reuse-kalibr-from` 用于 compare，不参与 Ceres 初始化。

命令：

```bash
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_adaptive_joint_20260624 --ceres-multi-imu-init ceres-single --ceres-multi-imu-single-seed-root out/docker_benchmarks/multi_imu_arm64 --ceres-multi-imu-adaptive-long-solve
```

指标说明：

| 指标 | 单位 | 含义 |
|---|---:|---|
| C/K camera0 Δt / ΔR / Δτ | mm / deg / ms | selected Ceres result 与 Kalibr joint camera0 的差异 |
| max chain Δt / ΔR | mm / deg | 该 case 四路 `T_c_i` effective chain 中最大的 C/K 差异 |
| accel mean / max | m/s^2 | selected Ceres result 的 accel residual 健康度 |
| selected wall | s | 本轮已生成 summary 中被 adaptive 选中的 Ceres candidate wall time；不含另一个未选 candidate，因此不是 full adaptive 总墙钟。脚本已补 `ceres_adaptive_short/long/total_elapsed_s`，下一次重跑会直接记录总墙钟 |

| Case | selected | decision | C/K camera0 Δt | ΔR | Δτ | max chain Δt | max chain ΔR | accel mean / max | selected wall |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| b01 | long | long residual score | 8.83 | 0.230 | -1.55 | 8.83 | 0.230 | 0.159 / 6.56 | 308.7 |
| b02 | long | long residual score | 7.84 | 0.071 | -1.44 | 7.84 | 0.090 | 0.151 / 4.97 | 320.7 |
| b03 | long | long residual score | 10.43 | 0.172 | -1.23 | 18.66 | 0.249 | 0.161 / 6.46 | 301.7 |
| b04 | long | long residual score | 8.96 | 0.061 | -1.97 | 21.84 | 0.150 | 0.164 / 5.40 | 313.7 |
| b05 | long | long residual score | 9.43 | 0.106 | -1.16 | 9.43 | 0.326 | 0.176 / 8.01 | 315.8 |
| b06 | long | long residual score | 13.90 | 0.065 | -1.70 | 16.77 | 0.291 | 0.159 / 6.77 | 302.3 |
| b07 | long | long residual score | 41.71 | 0.051 | -4.70 | 42.73 | 0.109 | 0.121 / 1.78 | 395.2 |
| b08 | long | long residual score | 40.77 | 0.135 | -8.23 | 40.77 | 0.178 | 0.177 / 2.89 | 390.0 |
| b09 | short | short accel max health | 95.72 | 0.858 | -3.26 | 95.72 | 1.766 | 0.844 / 5.92 | 143.5 |
| b10 | long | long residual score | 35.25 | 0.050 | -0.66 | 44.88 | 0.128 | 0.109 / 1.01 | 392.8 |
| b11 | long | long residual score | 63.68 | 0.197 | +0.35 | 63.68 | 0.238 | 0.137 / 1.30 | 325.8 |
| b12 | long | long residual score | 60.97 | 0.081 | -7.75 | 64.06 | 0.319 | 0.170 / 2.87 | 398.5 |

汇总：

| 统计项 | camera0 Δt mm | max chain Δt mm | selected candidate |
|---|---:|---:|---|
| mean | 33.1 | 36.3 | long 11 / short 1 |
| median | 24.6 | 31.3 | b09 long 被 health gate 拒绝 |
| max | 95.7 | 95.7 | 最大仍为 b09 |

读数：

- adaptive 验证了 stop 问题：b07/b08/b10/b11/b12 的 stage0 在 `step_norm < 0.02` 之后仍能继续大幅降 cost，甚至会在 trust-region 半径放大后出现第二段大步下降。1cam+Nimu 粗阶段不能继续把 `step_norm=0.02` 当作默认收敛条件。
- adaptive 没有解决外参 basin 问题：b07/b08/b10/b11/b12 的 accel residual 已经健康，但 C/K camera0 仍是 `35-64 mm`。这说明“残差健康”和“与 Kalibr joint 外参一致”不是同一个条件。
- b09 的 long 被拒绝是正确的：long 虽然继续降 cost，但 accel max 恶化到 `26.84 m/s^2`，而 selected short 仍有 `95.7 mm / 0.858 deg` 的外参差。b09 是 chain lever prior 失败后的坏 basin，不是单纯早停。
- `b11` 是重要反例：三个非参考 IMU lever prior 都通过，long residual 也健康，但 camera0 仍有 `63.7 mm`。这说明只修非参考 IMU lever 还不够，reference camera/IMU 的 single seed basin 也需要被 joint 初始化或物理先验修正。
- 当前 adaptive 适合作为诊断/accuracy preset，不适合作为默认 running 配置。下一版应把 stage0 改成更稳的长粗优化，同时给 stage1 恢复 cost/health stop，并改进 no-Kalibr reference camera + multi-IMU chain 初始化。

## Single b09/imu3 Ablation

| 配置 | C/K Δt mm | C/K ΔR deg | Δτ ms | accel mean / max | residual delta accel | 结论 |
|---|---:|---:|---:|---:|---:|---|
| 默认 single | 41.6 | 0.831 | -2.17 | 0.343 / 19.60 | +0.030 | 当前 C/K single 最大异常 |
| time prior `1 ms` | 36.7 | 0.853 | -0.79 | 0.341 / 19.51 | +0.028 | time shift 改善，但外参仍异常 |
| time prior `1 ms` + 更强 pose prior | 39.3 | 0.799 | -0.26 | 0.340 / 19.49 | +0.027 | 强化 pose prior 没有压住 accel spike |

读数：

- b09/imu3 的 accel residual 在 Kalibr 侧也高：Kalibr accel mean `0.313 m/s^2`。Ceres 比 Kalibr 高约 `0.027-0.030 m/s^2`，不是数量级错误。
- 最大 outlier 里 `pose_accel_world_norm` 达到十几 `m/s^2`，说明角点 pose 轨迹二阶导或局部时间段需要单独检查。
- 调 time-shift prior 只能改善时间偏差，不能解决平移/旋转差。

## b09 Local Data / Spline Diagnostics

本组按原始角点 CSV、PnP corner pose CSV 和四路 IMU CSV 做局部数据健康检查，输出全局角点覆盖、PnP 位姿二阶导、指定时间窗内的 IMU norm 和采样间隔。这里保留诊断读数，不把一次性诊断工具作为生产入口。

读数：

| 时间窗 | 原始角点/PnP 信号 | 说明 |
|---|---|---|
| `40.38s ±0.5s` | 30 帧；角点均值 `276.5`；bbox ratio 均值 `0.914`；PnP accel max `1.68 m/s^2` | 原始角点覆盖和 PnP 二阶导正常 |
| `45.10s ±0.5s` | 30 帧；角点均值 `376.8`；bbox ratio 均值 `0.744`；PnP accel max `2.58 m/s^2` | 原始 PnP 不支持“45s 输入角点崩掉”的解释；Kalibr-init tight 日志里的 `pose_accel_world_norm=18.6` 更像优化后 spline 尖峰 |
| `52.24s ±0.5s` | 30 帧；角点中位 `243`，但全局 top PnP accel 处只有 `45` 个角点；PnP accel max `18.23 m/s^2` | 这是原始数据自身的高风险窗口，角点少且 target 覆盖局部化 |
| `49.28s ±0.5s` | 30 帧；角点均值 `348.3`；PnP accel max `5.90 m/s^2` | no-Kalibr chain-prior 的 residual outlier 落在较强运动段，但不是最坏输入窗口 |

优化后 spline 诊断命令：

```bash
python3 tools/export_ceres_spline_samples.py --input out/docker_benchmarks/multi_imu_arm64_chain_prior_export_spline_b09/benchmark_multi_imu/benchmark_multi_01_2025_04_19_19_20_46/joint_4imu/ceres/result.yaml --output-dir out/diagnostics/b09_chain_prior_export_spline_samples --sample-hz 500 --imu-count 4
```

有效内区间 `4.5s-61.0s` 的 Ceres pose spline acceleration 统计：

| 配置 | mean | median | p90 | p99 | max | top 时间 |
|---|---:|---:|---:|---:|---:|---|
| chain-prior + export controls | `0.648` | `0.582` | `1.184` | `1.752` | `4.689` | `25.496s` |

读数：

- no-Kalibr chain-prior 的优化后 spline 没有出现 Kalibr-init tight joint 日志里 `45.11s` 的 `18.6 m/s^2` 级尖峰。
- no-Kalibr chain-prior 的外参仍与 Kalibr joint 差 `79 mm`，且 residual 与 Kalibr-init Ceres joint 同数量级。这支持当前判断：b09 的 no-Kalibr 难点主要是外参 basin/初始化，而不是单纯 spline health。
- `52.24s` 是原始角点/PnP 的真实高风险窗口；后续 residual-health gate 应把这类窗口纳入诊断，但不能指望只靠过滤窗口解决 camera0 basin。

## b09 Single Seed Health

`--ceres-multi-imu-init=ceres-single` 当前用 `single_imu1` 作为 camera0/reference seed。这个选择从 Ceres-only single residual 看并不坏，反而是四路里最健康的一路。

| seed | C/K single Δt mm | ΔR deg | Δτ ms | Ceres reproj px | gyro rad/s | accel m/s^2 |
|---|---:|---:|---:|---:|---:|---:|
| single_imu1 | 0.56 | 0.000 | -0.70 | 0.170437 | 0.013303 | 0.096120 |
| single_imu2 | 7.27 | 0.091 | -1.33 | 0.183219 | 0.033756 | 0.204893 |
| single_imu3 | 41.56 | 0.831 | -2.17 | 0.196393 | 0.051092 | 0.343443 |
| single_imu4 | 25.45 | 0.069 | -0.93 | 0.179460 | 0.030872 | 0.200665 |

读数：

- `single_imu1` 与 Kalibr single 几乎一致，且 residual 最低；但它与 Kalibr joint basin 相差约 `95.7 mm`。
- 因此，不能只靠 Ceres single residual 或 C/K single 一致性自动判定 reference seed 是否适合 joint。
- b09 更像是“single 目标函数健康，但 joint 目标函数选择了另一个 basin”。没有金标时，benchmark 上强行追 Kalibr joint 不能单独证明 no-Kalibr 更准确。

## Simulation Sanity Check

为了避免只用 Kalibr 结果判断 b09，复跑了当前代码的 `simulation/generated/one_cam_four_imus`。该路径不使用 Kalibr result，命令来自生成目录的 `run_calibration.sh`。

| 配置 | camera Δt | camera ΔR | camera time | imu1 lever | imu2 lever | imu3 lever | 读数 |
|---|---:|---:|---:|---:|---:|---:|---|
| 旧默认 no-Kalibr, no lever prior | 0.699 mm | 0.0312 deg | -0.127 ms | 8.44 mm | 10.26 mm | 11.31 mm | 多 IMU 链路可回到真值附近，但 lever 略差 |
| 当前默认 auto lever + gate | 0.703 mm | 0.0307 deg | -0.159 ms | 5.48 mm | 7.82 mm | 10.76 mm | `--corner-defaults` + 多 IMU topology 默认路径 |
| 显式 `--estimate-imu-chain-lever-prior` | 0.703 mm | 0.0307 deg | -0.159 ms | 5.48 mm | 7.82 mm | 10.76 mm | 与当前默认一致 |
| Ceres single-seed sign fix | 0.768 mm | 0.0841 deg | -0.165 ms | 6.96 mm | 9.79 mm | 11.82 mm | 实际 no-Kalibr single -> joint 路径仍贴近真值 |
| 显式 multi-IMU translation prior | 6.53 mm | 0.0292 deg | -0.191 ms | 12.49 mm | 5.32 mm | 9.47 mm | camera translation 退化到毫米级上沿；不适合默认 |
| 显式 multi-IMU translation prior + reg `0.01m/0.05m/s^2` | 10.06 mm | 0.0154 deg | -0.168 ms | 7.02 mm | 6.24 mm | 9.65 mm | lever 略均衡，但 camera translation 明显退化 |

读数：

- Ceres no-Kalibr 1cam+4imu 在仿真 GT 上是毫米到厘米量级，不支持“joint 实现整体错误”的判断。
- Ceres single-seed sign-fix 路径在仿真 GT 上没有复现 b09 的 9 cm 错误；这支持“b09 的 benchmark C/K joint 差异需要结合金标或局部数据诊断，不应直接当作 no-Kalibr 精度崩坏”。
- b09 上 `--estimate-imu-chain-lever-prior` 失败，而仿真上有收益，说明该 initializer 对实测噪声、bias delta、运动激励或 fixture 几何更敏感；不能仅凭仿真把它直接默认打开。
- 显式 multi-IMU translation prior 在 b09 可降低 residual、改善部分 chain，但在仿真 GT 上把 camera translation 从 `0.70 mm` 推到 `6-10 mm`。它当前更像“帮助理解 b09 basin 的诊断器”，不是 production default。
- 当前已有 no-Kalibr 1cam+Nimu accuracy preset 的 runner 初版；仿真 replay 已证明 selector 不能只看 residual score，当前实现用 time boundary、accel ratio 和 `chain_accel_max_abs=3.0 m/s^2` 保护 camera 外参。当前六候选 focused 复跑已确认 b08/b12 能自动选 slow-chain 改善 tail，但仍需要完整 12 组当前代码重跑，确认 b07/b10/b11 是否也受益，并重算总览统计。
- 当前 runner 已补充 per-candidate `selection_status`，下一轮 12 组 trimmed 复跑时应直接按该字段统计 chain 被接受/拒绝的原因，避免只看最终 C/K 指标反推 selector 行为。
- b08 当前六候选 selector 已从 `single_long` 切到 `chain_long_single_time_t_wide_tight`，camera0 C/K 降到 `22.2 mm`，time-shift 差降到 `+0.33 ms`，accel max 降到 `1.16 m/s^2`。因此 b08 的下一步不是继续放宽 health gate，而是观察完整 12 组里 slow-chain 是否稳定覆盖同类 tail。
- b12 当前六候选 selector 同样切到 `chain_long_single_time_t_wide_tight`，camera0 C/K 降到 `6.7 mm`。这说明 slow single-time chain 已经从 focused 诊断提升为 selector 可自动接受的候选；但仿真上该候选会牺牲 GT camera，并被 `accel_max_abs` 拒绝，所以不能作为无条件默认。

## 代码修正

本轮保留五类已默认生效的代码修正，并新增一个显式实验开关和一个 runner 级 adaptive 评测入口。

第一个修正：`--ceres-multi-imu-init=ceres-single` 生成 joint 命令时，现在会自动带上：

```text
--pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1
```

原因是 `--init-from-result` 分支原来没有 pose motion prior，unbounded/no-step 实验会出现 `accel max ~26 m/s^2` 的不健康 spline/状态逃逸。这个修正不是精度结论，只是避免 no-Kalibr seed 路径缺少与 chain-prior / single 路径一致的基础正则。

验证：

```text
python3 -m py_compile tools/run_docker_benchmark.py
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_04_19_19_20_46 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root /tmp/ceres_print_single_seed_poseprior --ceres-multi-imu-init ceres-single --ceres-multi-imu-single-seed-root out/docker_benchmarks/multi_imu_arm64 --print-only
```

print-only 结果中 joint Ceres 命令包含 `--init-from-result ... --pose-motion-prior ...`，没有 `--kalibr-result` / `--init-from-kalibr`。

第二个修正：`--ceres-multi-imu-init=ceres-single` 写 `imu_chain.r_i_b` 时，改为调用 runner 里的 `ceres_rotation_matrix_to_vector()`。Ceres native 的 `rotationVectorToMatrix(r)` 对应 `Exp(-r)`，所以 YAML 里的 rotation-vector 必须是标准 SO3 log 的负号。旧 runner 使用标准正号，会把非参考 IMU chain rotation 初值写反。

验证：

```text
python3 -m py_compile tools/run_docker_benchmark.py
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_04_19_19_20_46 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_signfix_init_b09 --ceres-multi-imu-init ceres-single --ceres-multi-imu-single-seed-root out/docker_benchmarks/multi_imu_arm64 --ceres-multi-imu-stage-iterations 0
```

结果中 `cam_to_imu_delta_ceres_minus_kalibr` 已不再呈现整体符号错误。init-only 的 `imu3` effective chain 到 `1.5 mm`，但 camera0 仍为 `95.7 mm`，这与 b09 single-joint 冲突一致。

第三个修正：`tools/run_docker_benchmark.py` 不再把 single Ceres 行误标成 `ceres_init_kalibr_result`。single Ceres 命令实际没有传 `--kalibr-result` / `--init-from-kalibr`；旧 summary 里这个字段只是 runner 元数据误填，容易误导 no-Kalibr 判断。

第四个修正：新增 `--imu-chain-prior-max-lever-accel-rms`。当 `--estimate-imu-chain-lever-prior` 的候选 lever 拟合 RMS 超过门限时，候选不会写入 `r_b`，但仍保留 rotation/time chain prior 和诊断日志。runner 对应参数为 `--ceres-multi-imu-lever-prior-max-accel-rms`。

第五个修正：multi-IMU `--corner-defaults` 现在默认尝试 IMU chain lever prior，并在未显式传 `--imu-chain-prior-max-lever-accel-rms` 时使用 `0.5 m/s^2` gate。显式 `--no-estimate-imu-chain-lever-prior` 仍可关闭。日志中的 `corner defaults active` 会打印：

```text
imu_chain_lever_prior=1 imu_chain_lever_accel_rms_gate=0.5
```

验证：

```text
out/ablations/sim_one_cam_four_single_seed_signfix/joint_default_auto_lever
out/docker_benchmarks/multi_imu_arm64_chain_prior_auto_lever_gate_init_b09
```

仿真默认会接受 lever 候选；b09 的三个候选因为 RMS `0.703/0.863/0.717 m/s^2` 超过 `0.5` 被拒绝。

验证：

```text
cmake --build build --target calibrate_cam_imu
python3 -m py_compile tools/run_docker_benchmark.py
simulation/generated/one_cam_four_imus/run_calibration.sh --estimate-imu-chain-lever-prior --imu-chain-prior-max-lever-accel-rms 0.5
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_04_19_19_20_46 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_chain_prior_lever_gate_init_b09 --ceres-multi-imu-init chain-prior --ceres-multi-imu-estimate-lever-prior --ceres-multi-imu-lever-prior-max-accel-rms 0.5 --ceres-multi-imu-stage-iterations 0
```

新增实验开关：`--estimate-multi-imu-translation-prior`。它在 IMU chain rotation/time prior 之后运行，用同一个线性最小二乘联合估计 cam0 translation、非参考 IMU lever 和每路 accel bias。该开关当前不进入 `--corner-defaults` 默认，只用于验证 no-Kalibr 1cam+Nimu 的初始化方向。

该开关现在也允许在 `--init-from-result` 后显式运行，前提是已有 gravity seed。这样可以验证 “Ceres single -> joint” 的实际 no-Kalibr 路径，而不是只验证 cold chain-prior。Kalibr init 仍不会触发这个 prior。

该 initializer 还增加了两个默认关闭的正则参数：

```text
--multi-imu-translation-prior-lever-sigma-m
--multi-imu-translation-prior-accel-bias-sigma
```

这两个参数只作用在显式 multi-IMU translation prior 的线性 LS 上。正数启用 Tikhonov 正则，负数保持关闭；它们不是主优化 residual，也不是当前默认 running 配置。

验证：

```text
cmake --build build --target calibrate_cam_imu
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_04_19_19_20_46 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/multi_imu_arm64_chain_prior_multi_t_prior_b09 --ceres-multi-imu-init chain-prior --ceres-extra-arg=--estimate-multi-imu-translation-prior --ceres-extra-arg=--export-spline-controls
```

b09 上该开关把 chain-prior 初始 cost 从 `4.12e5` 降到 `3.19e5`，最终 accel mean 从 `0.88` 降到 `0.42 m/s^2`，但 camera0 C/K 仍为 `79.4 mm / 1.055 deg`。加入强正则后，camera0 translation 可降到 `35.7 mm`，但 rotation 仍为 `1.055 deg`。

在 Ceres-single seed 路径上，该开关把初始 cost 降到 `2.85e5`，完整 staged joint 后 residual 更健康，camera0 C/K 为 `77.3 mm / 0.858 deg`。它比 raw single-seed 有进步，但仍不能替代 Kalibr-init。

新增 runner 策略：`--ceres-multi-imu-adaptive-long-solve`。它只属于 `tools/run_docker_benchmark.py`，不改变 `build/calibrate_cam_imu --corner-defaults` 的生产默认。启用后，同一个 multi-IMU Ceres case 会先跑默认 short candidate，再跑禁用 stage step stop 的 long candidate：

```text
--solver-absolute-step-tolerance=-1
--stage-solver-absolute-step-tolerances=-1,-1
```

选择规则：

- long 返回码失败时选 short。
- long 的 accel mean 超过 short 的 `1.05x` 时选 short。
- long 的 accel max 超过 short 的 `1.5x` 时选 short。
- 缺少 accel health 指标时选 short。
- 通过 health gate 后，只有 long 的 `reproj_mean + gyro_mean + accel_mean` 明确低于 short 时才选 long；同分或差异极小时选 short。

summary 会记录 `ceres_candidate`、`ceres_adaptive_decision`、short/long result/log、short/long residual score、short/long/total elapsed 以及 accel health 字段。本轮 12 组 full adaptive 已用这些字段统计 stop policy 边界；本轮产物生成时尚未记录 total elapsed，后续重跑会补齐。该策略当前只是 no-Kalibr accuracy preset 的评测入口，不是已验证的最终默认。

验证：

```text
python3 -m py_compile tools/run_docker_benchmark.py
python3 tools/run_docker_benchmark.py --suite benchmark-multi-imu --benchmark-multi-subset joint --multi-session 2025_03_14_00_10_18 --kalibr-platform linux/arm64 --reuse-kalibr-from out/docker_benchmarks/multi_imu_arm64 --out-root out/docker_benchmarks/tmp_adaptive_smoke_b01 --ceres-multi-imu-init ceres-single --ceres-multi-imu-single-seed-root out/docker_benchmarks/multi_imu_arm64 --ceres-multi-imu-adaptive-long-solve --ceres-multi-imu-stage-iterations 0
```

smoke test 只验证 runner 能生成 short/long 两个候选和 adaptive summary 字段，不用于判断精度。

新增 runner 策略：`--ceres-multi-imu-candidate-preset no-kalibr-accuracy` / `no-kalibr-accuracy-trimmed`。它只属于 `tools/run_docker_benchmark.py`；当前 runner 默认值是 `no-kalibr-accuracy-trimmed`，裸 `build/calibrate_cam_imu --corner-defaults` 仍是单次求解。full 口径用于诊断，会跑九个候选：

```text
single_short
single_long
chain_multi_t_wide_tight
chain_multi_ograv_t_wide_tight
chain_accel_refine_t_wide_tight
chain_multi_single_time_t_wide_tight
chain_multi_ograv_single_time_t_wide_tight
chain_long_t_wide_tight
chain_long_single_time_t_wide_tight
```

trimmed 口径保留 `single_short/single_long/chain_multi_t_wide_tight/chain_multi_ograv_t_wide_tight/chain_accel_refine_t_wide_tight/chain_long_single_time_t_wide_tight`，作为本轮收尾的 `1cam+Nimu` runner 默认。它比 full preset 少三个诊断候选，但保留了 b09 rotation 最有效的 accel-refine 候选和 b08/b12/b11 这类最有效的 slow single-time chain 候选。两种口径都不使用 Kalibr result 做初始化或候选选择；Kalibr 只在 benchmark harness 最后用于 compare。

其中 `single_short/single_long` 用 Ceres single 结果生成 joint seed；`chain_*` 使用 no-Kalibr chain-prior、normalized camera time-shift、multi-IMU translation prior 正则和 `0.05,0.05,0.003m` / `0.05,0.05,0.005rad` 的 wide/wide/tight stage bound。`chain_multi_ograv_t_wide_tight` 在 IMU chain prior 后额外用四路 gyro/accel 估计 multi-IMU orientation/gravity prior；`chain_accel_refine_t_wide_tight` 在 IMU chain prior 内加入 accel residual refine，只能被 health gate 接受。`chain_long_*` 额外禁用 stage step stop，最后两阶段用 `absolute_cost_change_tolerance=5`；`chain_*_single_time_t_wide_tight` 只复用 Ceres single seed 的 camera time shift，不复用 Kalibr 或 Kalibr result。选择规则不使用 Kalibr：

- 先用 adaptive health gate 在 `single_short/single_long` 之间选一个 single 候选。
- 如果 chain 候选的 camera time-shift prior 触发 `boundary_peak_rejected=1`，直接回退 single 候选。
- 如果 chain 的 accel mean/max 相对 single 超过配置倍率，回退 single 候选。
- 如果 chain 的 accel max 超过 `3.0 m/s^2` 绝对门限，回退 single 候选。
- 如果 chain residual score 不超过 single 的 `1.03x`，在健康 chain 中选 residual score 最低者；否则选 single。这里允许 chain 略高于 single，是为了保留 b08/b11/b12 这类 benchmark 改善，但必须先通过 accel max 绝对门限，因为仿真显示 chain 可以用更低均值 residual 换取更差 camera 外参。

smoke 结果：

| Case | 选择 | 选择原因 | selected C/K camera0 | 关键读数 |
|---|---|---|---|---|
| b01 | `single_long` | `chain_time_boundary` | `8.84 mm / 0.230 deg / -1.60 ms` | chain time-shift prior 被边界 gate 拒绝，chain C/K 平移为 `131 mm` |
| b09, six-candidate | `chain_multi_t_wide_tight` | `chain_score_within_single_short` | `28.7 mm / 1.060 deg / -0.002 ms` | single_short 平移 `95.7 mm`；chain residual delta accel 从 `+0.630` 降到 `+0.113 m/s^2` |
| b09, seven-candidate + multi-ograv | `chain_multi_ograv_t_wide_tight` | `chain_multi_ograv_t_wide_tight_score_within_single_short` | `36.0 mm / 1.044 deg / +0.007 ms` | effective chain `36.0/54.6/45.6/37.9 mm`；accel mean/max `0.302/1.50 m/s^2`；total preset wall `2104 s` |
| b09, current 6-equivalent | `chain_accel_refine_t_wide_tight` | `chain_accel_refine_t_wide_tight_score_within_single_short` | `37.4 mm / 0.148 deg / +0.04 ms` | 七候选 run 中被移除的 `post_accel_rot` 未被选中，因此等价于当前六候选；effective chain rotation 明显好于普通 chain，但 camera0 平移仍未收口 |
| b09, Ceres-single + multi-ograv + multi-t | 不加入 preset | accel tail 明显恶化 | `80.5 mm / 1.535 deg / -2.96 ms` | wall `399 s`；accel mean/max `0.280/16.47 m/s^2` |
| simulation score-only | `chain_multi_t_wide_tight` 或 `chain_long_*` | chain residual score 更低 | GT `10.03-27.86 mm` | 错误接受 chain；camera 被 multi-IMU translation prior 推坏 |
| simulation + accel max abs + multi-ograv | `single_long` | chain `accel_max_abs` rejected | GT `0.205 mm / 0.0028 deg / -0.097 ms` | 当前 selector 行为；chain accel max 为 `3.80/3.76/4.14/4.59/5.15 m/s^2`，保护 camera 外参 |
| simulation old trimmed | `single_long` | `chain_multi_t_wide_tight:accel_max_abs,chain_multi_ograv_t_wide_tight:accel_max_abs` | GT `0.205 mm / 0.0028 deg / -0.097 ms` | 旧四候选入口行为与 full replay 一致；两个 chain 的 camera GT 为 `10.03/11.33 mm` |
| simulation trimmed + long single-time | `single_long` | `chain_long_single_time_t_wide_tight:accel_max_abs` | GT `0.205 mm / 0.0028 deg / -0.097 ms` | 五候选入口仍保护 camera GT；slow chain 自身 GT 为 `27.86 mm`，accel max `5.15 m/s^2` |
| simulation trimmed 6-candidate | `single_long` | `chain_accel_refine_t_wide_tight:accel_max_abs,score` 等 | GT `0.205 mm / 0.0028 deg / -0.097 ms` | 当前 trimmed 额外含 accel-refine；三个 short chain GT 为 `9.89-11.33 mm` 且 accel max `3.76-3.81 m/s^2`，均被拒绝 |
| b08 current 6-candidate | `chain_long_single_time_t_wide_tight` | `chain_long_single_time_t_wide_tight_score_within_single_long` | C/K `22.2 mm / 0.045 deg / +0.33 ms` | 自动 selector 从 `single_long` 的 `40.8 mm / -8.23 ms` 切到 slow-chain；accel mean/max `0.122/1.16 m/s^2` |
| b12 current 6-candidate | `chain_long_single_time_t_wide_tight` | `chain_long_single_time_t_wide_tight_score_within_single_long` | C/K `6.7 mm / 0.053 deg / +0.26 ms` | 自动 selector 从 `single_long` 的 `61.0 mm / -7.77 ms` 切到 slow-chain；accel mean/max `0.125/1.13 m/s^2` |

这个 preset 是 no-Kalibr 路径的收尾默认，不是 Kalibr-init tight 的完成态：b07/b08/b10/b12 已由 current 6-candidate selector 自动改善，b09 仍有 `37.4 mm` camera0 平移差，b11 仍有 `0.562 deg` 旋转 tail。加入 slow single-time chain 后，仿真同一候选会把 camera GT 推到 `18.60 mm` 并被 accel max gate 拒绝，因此它只能作为 health-gated candidate，而不是下沉为裸 `--corner-defaults` 的无条件单候选默认。

## 复现入口

主要输出目录：

| 目录 | 内容 |
|---|---|
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_unbounded_no_step_stop_opt_offsets_b09` | chain-prior + offset 优化 |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_signfix_init_b09` | Ceres-single seed 符号修正后的 init-only 验证 |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_signfix_b09` | Ceres-single seed 符号修正后的默认 staged joint |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_signfix_no_step_b09` | Ceres-single seed 符号修正后禁用 stage step stop |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_signfix_no_step_restore_b09` | Ceres-single seed 禁用 stage step stop + restore-best |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_allseg_stagevar_b09` | chain-prior + full-spline pose prior + 更强 stage variance |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_lever_gate_init_b09` | b09 lever prior RMS gate 初始化验证 |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_auto_lever_gate_init_b09` | b09 multi-IMU corner-defaults 默认 auto-lever + gate 初始化验证 |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_export_spline_b09` | no-Kalibr chain-prior + spline controls 导出 |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_no_cam_t_prior_b09` | no-Kalibr chain-prior + 禁用 camera translation prior |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_multi_t_prior_b09` | no-Kalibr chain-prior + 显式 multi-IMU translation prior |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_multi_t_relaxed_b09` | multi-IMU translation prior + relaxed non-reference IMU bound |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_multi_t_reg_strong_b09` | multi-IMU translation prior + `0.02m/0.10m/s^2` 正则 |
| `out/docker_benchmarks/multi_imu_arm64_chain_prior_multi_t_reg_vstrong_b09` | multi-IMU translation prior + `0.01m/0.05m/s^2` 正则 |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_multi_t_prior_init_b09` | Ceres-single seed + multi-IMU translation prior init-only |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_multi_t_prior_b09` | Ceres-single seed + multi-IMU translation prior 完整 staged joint |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_multi_t_reg_vstrong_init_b09` | Ceres-single seed + multi-IMU translation prior + 强正则 init-only |
| `out/docker_benchmarks/b09_no_kalibr_multi_t_camera_sigma_0p02_20260625` | chain-prior + multi-IMU translation prior + camera sigma `0.02m` |
| `out/docker_benchmarks/b09_no_kalibr_multi_t_no_ref_t_20260625` | chain-prior + 禁用 reference-only camera translation prior + multi-IMU translation prior 正则 |
| `out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_rotbound_0p05_20260625` | Ceres-single seed + multi-IMU translation prior + nonref rotation bound `0.05rad` |
| `out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_rotbound_0p05_reg_tight_20260625` | Ceres-single seed + `0.05rad` rotation bound + 更强 lever/bias 正则 |
| `out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_wide_tight_bound_20260625` | Ceres-single seed + stage-specific `0.05,0.05,0.005rad` rotation bound |
| `out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_weighted_accel_20260625` | b09 residual-weighted translation seed + multi-IMU translation prior；camera0 仍 `72.1 mm` |
| `out/docker_benchmarks/b09_no_kalibr_ceres_single_seed_weighted_accel_no_multiprior_20260625` | b09 residual-weighted translation seed，不跑 multi-IMU translation prior；translation 好但 accel 坏 |
| `out/docker_benchmarks/b09_no_kalibr_weighted_accel_multiprior_camsigma0p005_20260625` | b09 residual-weighted translation seed + camera sigma `0.005m` |
| `out/docker_benchmarks/b09_no_kalibr_weighted_accel_multiprior_camsigma0p001_20260625` | b09 residual-weighted translation seed + camera sigma `0.001m` |
| `out/docker_benchmarks/multi_imu_arm64_no_kalibr_weighted_accel_camsigma0p001_20260625` | residual-weighted seed 12 组尝试，b01 已退化后中断；用于证明该路线不可推广 |
| `out/ablations/b09_fixture_median_seed_20260625` | b09 fixture-median IMU translation seed 两阶段反例 |
| `out/ablations/b09_fixture_median_seed_3stage_20260625` | b09 fixture-median IMU translation seed 三阶段反例 |
| `out/ablations/b09_fixture_median_extrinsic_post_20260625` | b09 后半组 fixture-median full extrinsic 反例；`95.8 mm / 2.37 deg`，accel max `27.17 m/s^2` |
| `out/ablations/b09_fixture_median_camera_chain_post_20260625` | b09 后半组 fixture-median camera-chain 反例；`124.1 mm / 3.11 deg` |
| `out/ablations/b09_no_kalibr_chain_multi_t_strong_reg_20260625` | b09 multi-IMU translation prior 强正则反例；camera0 `84.0 mm`，nonref `71.9/128.4/106.4 mm` |
| `out/ablations/b09_no_kalibr_chain_multi_t_multi_ograv_20260625` | b09 新 multi-IMU orientation/gravity prior；nonref chain 降到 `54.6/45.6/37.9 mm`，accel max `1.50 m/s^2` |
| `out/ablations/b09_no_kalibr_chain_long_multi_ograv_20260625` | b09 multi-IMU orientation/gravity long 版本；结果与短版基本一致 |
| `out/ablations/b09_no_kalibr_accuracy_preset_multi_ograv_20260625` | b09 七候选 full preset；选择 `chain_multi_ograv_t_wide_tight`，total preset wall `2104 s` |
| `out/ablations/b09_no_kalibr_ceres_single_seed_multi_ograv_multi_t_20260625` | b09 Ceres-single + multi-ograv + multi-translation prior 负样本；`80.5 mm / 1.535 deg`，accel max `16.47 m/s^2` |
| `out/ablations/b09_normcorr_imu_chain_init_20260625` | b09 IMU-chain normalized correlation init-only；IMU-chain peaks 非边界，min correlation `0.9959` |
| `out/ablations/b09_normcorr_chain_multi_ograv_full_20260625` | b09 normalized IMU-chain + multi-ograv full run；camera0 `35.9 mm / 1.037 deg`，nonref tail `46.1 mm` |
| `out/ablations/b09_chain_ograv_single_time_20260625` | b09 multi-ograv + Ceres single camera-time seed；camera0 `33.1 mm`，nonref tail `46.9 mm`，但 time 差退到 `-3.22 ms` |
| `out/ablations/b09_chain_ograv_single_time_opt_imu_offsets_20260625` | b09 multi-ograv + single camera-time + `--optimize-imu-time-offsets`；结果基本不变，wall 增到 `415s` |
| `out/ablations/b09_chain_multi_t_opt_imu_offsets_20260625` | b09 chain multi-translation + `--optimize-imu-time-offsets`；camera0 保持 `28.7 mm`，nonref tail 退到 `74.0 mm` |
| `out/ablations/b09_chain_multi_t_init_only_20260625` | b09 chain multi-translation init-only；camera0 已在 `28.7 mm` basin，nonref rotation 初值仍有 `3-5 deg` 误差 |
| `out/ablations/b09_chain_multi_t_init_only_no_gyro_refine_20260625` | b09 chain multi-translation init-only + 禁用 gyro Ceres refine；结果与默认 init-only 一致，排除 refine 副作用 |
| `out/ablations/b09_chain_accel_refine_g10a1_20260625` | b09 accel-aided IMU chain refine；rotation/effective chain 明显改善，但 camera0 平移仍 `37.4 mm` |
| `out/ablations/b09_chain_accel_refine_g10a1_ograv_20260625` | b09 accel refine + multi-ograv 负样本；rotation `1.69 deg` 且 accel max `10.09 m/s^2` |
| `out/ablations/b09_chain_accel_refine_g50a1_init_20260625` | b09 accel refine `gyro_weight=50` init-only 负样本；基本退回 gyro-only rotation basin |
| `out/ablations/sim_one_cam_four_chain_accel_refine_g10a1_20260625` | 仿真 accel-refine 负样本；GT camera `9.89 mm` 且 accel max `3.81 m/s^2` |
| `out/ablations/b08_chain_accel_refine_g10a1_20260625` | b08 accel-refine 负样本；camera0 `62.7 mm`、time `-30.9 ms`、accel max `4.74 m/s^2` |
| `out/ablations/b12_chain_accel_refine_g10a1_20260625` | b12 accel-refine 负样本；camera0 `25.2 mm`、time `-42.2 ms`、accel max `4.69 m/s^2` |
| `out/ablations/b08_no_kalibr_trimmed_normcorr_audit_20260625` | b08 normalized IMU-chain + trimmed audit；选择 `single_long`，chain 被 `accel_mean/accel_max_abs/score` 拒绝 |
| `out/ablations/b08_chain_single_time_normcorr_20260625` | b08 chain + selected single time seed；camera0 `37.1 mm`，但 accel max `4.34 m/s^2`，仍应拒绝 |
| `out/ablations/b08_chain_ograv_single_time_20260625` | b08 multi-ograv + Ceres single camera-time seed；camera0 `37.9 mm`，但 accel max `4.38 m/s^2`，仍应拒绝 |
| `out/docker_benchmarks/b09_time_shift_normcorr_boundary_reject_init_20260625` | b09 normalized time-shift + boundary gate init-only；历史旧符号接受 `+4.007ms`，仅用于追溯 |
| `out/docker_benchmarks/b01_time_shift_normcorr_boundary_reject_init_20260625` | b01 normalized time-shift + 当时的 `0.05 s` boundary gate init-only；历史运行拒绝 `+50ms` 边界峰，后续确认同时包含符号回归与窄窗截断，只保留作追溯 |
| `out/docker_benchmarks/b09_no_kalibr_normcorr_default_20260625` | b09 normalized time-shift 后的 chain-prior default full run |
| `out/docker_benchmarks/b09_no_kalibr_normcorr_multi_t_reg_20260625` | b09 normalized time-shift + multi-IMU translation prior 正则 + wide/wide/tight 当前最好 chain-prior 结果 |
| `out/ablations/b01_no_kalibr_accuracy_preset_20260625` | no-Kalibr accuracy preset b01 smoke；选择 `single_long`，拒绝 chain 边界 time-shift |
| `out/ablations/b01_no_kalibr_accuracy_preset_single_time_short_20260625` | b01 六候选复跑；single-time chain 仍为 `86.9 mm` 级平移差，selector 回退 `single_long` |
| `out/ablations/b09_no_kalibr_accuracy_preset_20260625` | no-Kalibr accuracy preset b09 smoke；选择 `chain_multi_t_wide_tight` |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_20260625` | no-Kalibr accuracy preset 仿真 replay；旧 `chain_score_ratio=1.2` 误选 chain |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_ratio0p9_20260625` | no-Kalibr accuracy preset 仿真 replay；历史 `chain_score_ratio=0.9` 回退 single_long |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_chain_long_single_time_20260625` | no-Kalibr accuracy preset 仿真 replay；chain-long 与 single-time chain residual 更低但被 `accel_max_abs` 门限拒绝 |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_single_time_short_20260625` | no-Kalibr accuracy preset 仿真 replay；新增 short single-time chain，仍被 `accel_max_abs` 门限拒绝并选择 `single_long` |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_multi_ograv_20260625` | no-Kalibr accuracy preset 仿真 replay；新增 multi-ograv chain，仍被 `accel_max_abs` 门限拒绝并选择 `single_long` |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_trimmed_20260625` | no-Kalibr 旧四候选 trimmed preset 仿真 replay；仍选择 `single_long`，保护 camera GT |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_trimmed_normcorr_20260625` | normalized IMU-chain 后的 trimmed 仿真 replay；仍选择 `single_long`，保护 camera GT |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_trimmed_long_single_time_20260625` | 五候选 trimmed 仿真 replay；加入 slow single-time chain 后仍选择 `single_long`，slow chain 被 `accel_max_abs` 拒绝 |
| `out/simulation_ablation/one_cam_four_imus_accuracy_preset_trimmed_6cand_20260625` | 当前六候选 trimmed 仿真 replay；加入 accel-refine 后仍选择 `single_long`，所有 chain 候选被 health gate 拒绝 |
| `out/docker_benchmarks/b09_reference_ablation_current_20260625` | b09 reference-order 当前复跑；ref_imu1/2/3 均为 `95-98 mm` max chain，ref_imu4 本轮中断 |
| `out/docker_benchmarks/b08_no_kalibr_chain_long_20260625` | b08 chain-long focused run；camera0 C/K `32.4 mm / 0.022 deg` |
| `out/ablations/b08_chain_long_single_time_20260625` | b08 chain-long + Ceres single time focused run；camera0 C/K `18.9 mm / 0.025 deg`，accel max `1.18 m/s^2` |
| `out/docker_benchmarks/b11_no_kalibr_chain_long_20260625` | b11 chain-long focused run；camera0 C/K `9.6 mm / 0.027 deg` |
| `out/docker_benchmarks/b12_no_kalibr_chain_long_single_time_20260625` | b12 chain-long + Ceres single time focused run；camera0 C/K `18.2 mm / 0.056 deg` |
| `out/ablations/trimmed6_focus_b08_20260625` | b08 当前六候选 selector 复跑；自动选择 `chain_long_single_time_t_wide_tight`，camera0 C/K `22.2 mm / 0.045 deg`，accel max `1.16 m/s^2` |
| `out/ablations/trimmed6_focus_b12_20260627` | b12 当前六候选 selector 复跑；自动选择 `chain_long_single_time_t_wide_tight`，camera0 C/K `6.7 mm / 0.053 deg`，accel max `1.13 m/s^2` |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_b01_no_step_20260624` | b01 Ceres-single seed no-step long 边界检查 |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_b09_no_step_accel3_20260624` | b09 Ceres-single seed no-step long + accel loss width 3 边界检查 |
| `out/docker_benchmarks/tmp_adaptive_smoke_b01` | adaptive short/long runner smoke test |
| `out/docker_benchmarks/multi_imu_arm64_ceres_single_seed_adaptive_joint_20260624` | 12 组 no-Kalibr Ceres-single seed adaptive joint full run |
| `out/simulation_ablation/one_cam_four_imus_nokalibr_20260624` | 仿真 baseline / multi-IMU translation prior / 强正则 sanity check |
| `out/diagnostics/b09_local_data_diagnostics.md` | b09 原始角点/IMU 局部数据诊断 |
| `out/diagnostics/b09_local_data_diagnostics_more.md` | b09 `49.28s/13.13s` 补充窗口诊断 |
| `out/diagnostics/b09_chain_prior_export_spline_samples` | b09 no-Kalibr chain-prior 优化后 spline 采样 |
| `out/ablations/sim_one_cam_four_single_seed_signfix` | 仿真 four single -> sign-fix joint seed -> no-Kalibr joint |
| `out/ablations/b09_imu3_single_sigma1e-3` | b09/imu3 single, time prior 1 ms |
| `out/ablations/b09_imu3_single_sigma1e-3_poseprior_stronger` | b09/imu3 single, time prior 1 ms + stronger pose prior |
| `out/ablations/b09_reference_candidates_multi_t_20260624` | b09 四 reference + multi-IMU translation prior |
| `out/ablations/b09_reference_candidates_multi_t_stage0_long_20260624` | b09 ref_imu1 + multi-IMU translation prior + stage0 long |
| `simulation/generated/one_cam_four_imus/calibration_result.yaml` | 当前仿真 no-Kalibr 复跑结果 |

## 收尾后保留的风险

1. 当前 runner 默认已经是 trimmed 六候选 no-Kalibr selector。历史 Kalibr-init tight joint 必须显式传 `--ceres-multi-imu-candidate-preset none --ceres-multi-imu-init kalibr`。
2. b09 和 b11 仍是 tail：b09 主要是平移 basin，b11 主要是旋转 tail。继续盲扫 LS 正则、reference reorder、fixture median 或优化 IMU offset 已经没有稳定收益。
3. residual-health gate 是保护机制，不是精度证明。no-step 降 cost 会制造 `26.8 m/s^2` accel tail；time-shift normalized correlation 也会在 b01 产生边界假峰；gate 只能防止坏状态被接受，不能替代可靠初始化。
4. 若后续要继续推进，需要引入额外物理信息或更强金标：例如机械/CAD fixture prior，或在 simulation 中构造 b09-like 局部 pose acceleration spike / 角点异常，验证 Ceres 与 Kalibr single/joint 是否会出现同样的 single-joint 分歧。
