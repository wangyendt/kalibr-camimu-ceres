# 仿真多场景 Ceres 与 Kalibr 精度对比

## 结论先行

这组实验用可控仿真数据比较 Ceres 版标定器和 Kalibr Docker 在不同 camera-IMU 场景下的标定精度。仿真数据提供 ground truth，因此主要看外参、time shift 和多 IMU 相对位姿能否回到真值。

关键结论：

- Ceres 在三组仿真场景中的 camera 外参误差为 `0.0047-0.0312 deg`、`0.070-0.700 mm`；Kalibr target0 单相机路径为 `0.0740-0.0786 deg`、`0.752-3.34 mm`。
- Ceres 和 Kalibr 的多 IMU 相对旋转处于同一量级。Ceres 为 `0.0398-0.1419 deg`，Kalibr 为 `0.0531-0.0782 deg`。
- Ceres 多 IMU 平移误差仍在毫米到厘米量级，默认为 `7.64-11.26 mm`；Kalibr target0 为 `12.22-17.89 mm`。这部分还需要单独做轨迹激励、初始化和停止策略 ablation。
- Ceres 当前本机流程更快：`1cam+1imu` 约 `1.45 s`，`1cam+4imu` 约 `5.20 s`。Kalibr arm64 Docker 对应约 `6.18 s` 和 `16.57 s`。这个耗时反映当前工作流，不是严格同环境算法 benchmark。
- 之前看到的 Ceres 多 IMU 约 `2 deg` 旋转误差是评估脚本读取 Ceres `r_i_b` 旋转向量符号约定错误，不是真实标定差异。

## 背景

项目里有两条 camera-IMU 标定路径：本工程 Ceres 版标定器，以及 Kalibr Docker 基线。真实数据没有完整外参真值，难以判断误差来自数据、检测、模型还是优化器。仿真数据的价值在于把 ground truth 固定下来，让结果可以直接按误差量化。

这篇文档不是修复说明，而是一次精度对比记录。仿真端使用 `RotationSpline` 和一致的 IMU 运动学，只是为了保证输入数据适合比较；最终关注的是 Ceres 和 Kalibr 在不同标定场景下输出的外参、time shift、IMU 相对位姿和耗时。

## 对比范围

三组数据都由 `simulation/scripts/generate_examples.py` 生成。轨迹旋转后端为：

```yaml
trajectory:
  rotation_interpolation: rotation_spline
```

| 场景 | 数据目录 | 作用 | Ceres 覆盖 | Kalibr 覆盖 |
|---|---|---|---|---|
| 单相机单 IMU | `simulation/generated/one_cam_one_imu` | 基础 camera-IMU 外参和 time shift | 完整运行 | arm64 Docker，target0 |
| 单相机四 IMU | `simulation/generated/one_cam_four_imus` | camera 外参 + 多 IMU 相对外参 | 完整运行 | arm64 Docker，target0 |
| 双相机双 IMU | `simulation/generated/two_cams_two_imus` | 多 camera 链路 + 多 IMU | 完整运行 | 未做公平比较 |

Kalibr 当前走 patched `--corner_file` 入口。该入口能公平表达单 camera、单 target0；双相机和多 target 还需要真实图像/ROS bag，或继续扩展 Kalibr reader。因此双相机双 IMU 场景只列 Ceres 结果，不把 Kalibr 作为同条件对照。

## 指标口径

所有外参指标都和 `ground_truth.yaml` 比较。误差越小越好。

| 类别 | 指标 | 单位 | 含义 | 注意事项 |
|---|---|---:|---|---|
| Camera 外参 | `camera rot err` | deg | `T_cam_body` 估计值和真值的旋转夹角 | 衡量 camera-to-body/reference-IMU 旋转精度 |
| Camera 外参 | `camera trans err` | mm | `T_cam_body` 平移向量和真值的欧氏距离 | 衡量 camera-to-body/reference-IMU 平移精度 |
| Camera 时序 | `camera time shift err` | ms | 估计 camera-to-IMU time shift 减去真值 | 本批数据真值为 0 |
| 多 IMU 外参 | `IMU rot err` | deg | 非 reference IMU 相对 body/reference-IMU 的旋转误差 | reference IMU 固定为 0，不参与该指标 |
| 多 IMU 外参 | `IMU trans/lever err` | mm | 非 reference IMU 原点偏移误差 | Ceres 表示为 body-frame lever `r_b`；Kalibr 表示为 `T_i_b` 平移，二者都反映传感器原点偏移 |
| 多 IMU 时序 | `IMU time offset err` | ms | 非 reference IMU time offset 估计值减真值 | 本批数据真值为 0；Kalibr 本次输出为 0，未作为有效对比项 |
| 视觉残差 | `reprojection` | px | 视觉重投影残差 | Ceres 为 RMS，Kalibr report 为 mean，只能看量级 |
| IMU 残差 | `gyro` / `accel` | rad/s, m/s^2 | IMU 角速度/加速度残差 | Ceres 为 RMS，Kalibr report 为 mean，只能看量级 |
| 耗时 | `wall time` | s | 当前命令端到端运行时间 | Ceres 为本机 binary；Kalibr 为 Docker arm64 |

### Ceres 旋转向量口径

Ceres C++ SO(3) 工具使用 `rotationVectorToMatrix(r)=Exp(-r)`，SciPy 使用 `Exp(+r)`。评估 Ceres YAML 中的 `r_i_b` 时必须先按 Ceres 约定转成矩阵：

```python
def _ceres_rotvec_to_matrix(rotvec):
    return Rotation.from_rotvec(-np.asarray(rotvec, dtype=float)).as_matrix()
```

这个口径会直接影响多 IMU 旋转结果。按错误的 `Exp(+r)` 读取时，`one_cam_four_imus` 中 Ceres 的非 reference IMU 会被误报为约 `2 deg`；按 Ceres 约定读取后为 `0.0398-0.1419 deg`。

## 结果总览

这张表只放最关键的外参、time shift 和耗时。多 IMU 指标用非 reference IMU 的范围表示。

| 场景 | 工具 | 对比范围 | wall time | camera rot | camera trans | camera time | IMU rot | IMU trans/lever | IMU time |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1cam+1imu | Ceres | cam0 + reference IMU | 1.45 s | 0.0119 deg | 0.111 mm | -0.013 ms | n/a | n/a | n/a |
| 1cam+1imu | Kalibr | cam0 + reference IMU, target0 | 6.18 s | 0.0740 deg | 0.752 mm | -0.199 ms | n/a | n/a | n/a |
| 1cam+4imu | Ceres | cam0 + imu1/2/3 | 5.20 s | 0.0312 deg | 0.700 mm | -0.114 ms | 0.0398-0.1419 deg | 8.28-11.26 mm | 0.037-0.125 ms |
| 1cam+4imu | Kalibr | cam0 + imu1/2/3, target0 | 16.57 s | 0.0786 deg | 3.34 mm | -0.0627 ms | 0.0531-0.0782 deg | 12.22-17.89 mm | n/a |
| 2cam+2imu | Ceres | cam0/cam1 + imu1 | 3.37 s | 0.0047-0.0051 deg | 0.070 mm | -0.0099/-0.0382 ms | 0.0448 deg | 7.64 mm | 0.0585 ms |
| 2cam+2imu | Kalibr | 未公平比较 | n/a | n/a | n/a | n/a | n/a | n/a | n/a |

读表方式：

- Camera 外参：Ceres 在当前仿真 CSV 上更接近真值；Kalibr target0 也在合理范围内。
- 多 IMU 旋转：两者同量级，不存在 Ceres 真实差 `2 deg` 的问题。
- 多 IMU 平移：两者都还不是亚毫米级，后续应作为独立问题分析。
- 耗时：Ceres 当前流程更快，但 Ceres native 和 Kalibr Docker arm64 不是严格同环境 benchmark。

## 场景一：单相机单 IMU

### 场景含义

这是最小 camera-IMU 标定问题，只估计 cam0 相对 reference IMU/body 的外参和 camera time shift。没有非 reference IMU，因此多 IMU 指标不适用。

### 结果

| 工具 | wall time | camera rot | camera trans | camera time | reprojection | gyro | accel |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ceres | 1.45 s | 0.0119 deg | 0.111 mm | -0.013 ms | 0.229 px RMS | 0.00525 rad/s RMS | 0.0159 m/s^2 RMS |
| Kalibr arm64 | 6.18 s | 0.0740 deg | 0.752 mm | -0.199 ms | 0.199 px mean | 0.00139 rad/s mean | 0.0137 m/s^2 mean |

### 读数结论

Ceres 和 Kalibr 都能把基础 camera-IMU 外参恢复到亚 `0.1 deg`、毫米级以内。Ceres 的外参和 time shift 更接近仿真真值；Kalibr 的 residual mean 不能直接和 Ceres RMS 排序，只能说明两者残差量级正常。

## 场景二：单相机四 IMU

### 场景含义

这个场景在基础 camera-IMU 标定之外，同时估计 imu1、imu2、imu3 相对 reference IMU/body 的旋转、平移和时序偏移。它主要检验多 IMU 链路。

### Camera 结果

| 工具 | wall time | camera rot | camera trans | camera time | reprojection | gyro | accel |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ceres | 5.20 s | 0.0312 deg | 0.700 mm | -0.114 ms | 0.244 px RMS | 0.00373 rad/s RMS | 0.0578 m/s^2 RMS |
| Kalibr arm64 | 16.57 s | 0.0786 deg | 3.34 mm | -0.0627 ms | 0.203 px mean | 0.00176 rad/s mean | 0.0183 m/s^2 mean |

### 多 IMU 结果

| 工具 | IMU | rotation err | trans/lever err | time offset err |
|---|---:|---:|---:|---:|
| Ceres | imu1 | 0.0398 deg | 8.28 mm | 0.047 ms |
| Ceres | imu2 | 0.0934 deg | 10.21 mm | 0.037 ms |
| Ceres | imu3 | 0.1419 deg | 11.26 mm | 0.125 ms |
| Kalibr arm64 | imu1 | 0.0782 deg | 17.89 mm | n/a |
| Kalibr arm64 | imu2 | 0.0531 deg | 12.22 mm | n/a |
| Kalibr arm64 | imu3 | 0.0687 deg | 12.29 mm | n/a |

### 读数结论

Camera 外参仍然稳定，Ceres 的 camera translation 误差小于 Kalibr target0。本场景最重要的是多 IMU：Ceres 旋转误差为 `0.0398-0.1419 deg`，Kalibr 为 `0.0531-0.0782 deg`，两者处于同一精度量级。平移误差两者都在毫米到厘米量级，不能据此说某一侧已经完全解决多 IMU lever arm。

## 场景三：双相机双 IMU

### 场景含义

这个场景验证 Ceres 多 camera 输入和多 IMU 输入能否同时工作。当前 Ceres 命令使用 `--fix-camera-chain-extrinsics` 固定 camera chain baseline，因此 cam1 指标主要验证链路和 time shift，不代表独立优化双目 baseline 的极限能力。

Kalibr patched corner-file 路径尚不能公平表达双相机同条件输入，因此本节只列 Ceres。

### Camera 结果

| 工具 | camera | wall time | camera rot | camera trans | camera time | reprojection | gyro | accel |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Ceres | cam0 | 3.37 s | 0.0051 deg | 0.070 mm | -0.0099 ms | 0.232 px RMS | 0.00366 rad/s RMS | 0.0525 m/s^2 RMS |
| Ceres | cam1 | 3.37 s | 0.0047 deg | 0.070 mm | -0.0382 ms | 0.232 px RMS | 0.00366 rad/s RMS | 0.0525 m/s^2 RMS |

### 多 IMU 结果

| 工具 | IMU | rotation err | trans/lever err | time offset err |
|---|---:|---:|---:|---:|
| Ceres | imu1 | 0.0448 deg | 7.64 mm | 0.0585 ms |

### 读数结论

Ceres 在双相机双 IMU 输入下仍保持稳定：两个 camera 的外参误差都在 `0.005 deg` 左右，time shift 误差低于 `0.04 ms`，非 reference IMU 旋转误差为 `0.0448 deg`。这个结果证明 Ceres 的多 camera / 多 IMU 数据链路可用，但不能替代后续与 Kalibr 的同条件双相机对比。

## Ceres 参数对照

默认 Ceres 运行来自 `--corner-defaults`。该 preset 面向生产速度，包含绝对早停：

```text
absolute_cost_change_tolerance=0.05
absolute_parameter_tolerance=0.01
```

在 `one_cam_four_imus` 上，默认运行第 3 次迭代后因为 parameter tolerance 停止；关闭绝对早停后，cost 在 30 次迭代内仍继续下降。下面只比较多 IMU lever arm，因为它是最受影响的指标。

| 配置 | imu1 lever | imu2 lever | imu3 lever | 读数 |
|---|---:|---:|---:|---|
| 默认 | 8.28 mm | 10.21 mm | 11.26 mm | 速度优先，4 次迭代结束 |
| `--estimate-imu-chain-lever-prior` | 5.37 mm | 7.89 mm | 10.47 mm | lever 初值更好，但仍早停 |
| 关闭绝对早停 | 5.92 mm | 9.29 mm | 11.07 mm | residual 下降，imu1 改善明显 |
| lever prior + 关闭绝对早停 | 5.85 mm | 9.26 mm | 11.02 mm | 与关闭早停接近 |

推荐用于极限精度对照的附加参数：

```bash
--estimate-imu-chain-lever-prior \
--solver-absolute-cost-change-tolerance -1 \
--solver-absolute-parameter-tolerance -1 \
--solver-min-relative-decrease 1e-9
```

这个对照说明：默认参数适合作为当前生产工作流结果；如果要比较 Ceres 的极限精度，应单独跑 accuracy preset。`--estimate-imu-chain-lever-prior` 有帮助，但不是多 IMU 平移误差的完整解法。

## 结果边界

- Kalibr 只在单 camera、target0 条件下和 Ceres 对比。双相机、多 target 仍缺少公平 Kalibr 基线。
- Ceres residual 是 RMS，Kalibr report 是 mean，残差不能直接排序。
- Ceres native wall time 和 Kalibr Docker arm64 wall time 不代表严格同环境算法耗时。
- 仿真生成端使用 SO(3) `RotationSpline`；Ceres 优化端仍是 6D pose curve，不是完整 Lie-group cumulative B-spline。
- rolling shutter、曝光/模糊、位置相关角点协方差已经能在仿真端生成或部分建模，但本次表格只比较基础外参和 time shift，不展开这些因素的系统 ablation。

## 后续动作

1. 建立 Ceres `production` 和 `accuracy` 两套固定 preset，避免速度结果和极限精度结果混在一起。
2. 扩展 Kalibr 输入路径，支持同条件多 camera / 多 target 对比。
3. 对多 IMU 平移误差做 ablation：轨迹激励、IMU 噪声、lever prior、停止策略、IMU intrinsics 固定与否。
4. 增加多 seed 统计，用均值、标准差和最差值替代单次运行结论。

## 复现入口

生成仿真数据：

```bash
python3 -B simulation/scripts/generate_examples.py
```

Ceres 标定：

```bash
/usr/bin/time -p simulation/generated/one_cam_one_imu/run_calibration.sh
/usr/bin/time -p simulation/generated/one_cam_four_imus/run_calibration.sh
/usr/bin/time -p simulation/generated/two_cams_two_imus/run_calibration.sh
```

Kalibr arm64 target0：

```bash
python3 -B tools/run_kalibr_docker.py --dataset simulation/generated/one_cam_one_imu --image kalibr-camera-calibration:20.04-arm64 --platform linux/arm64 --corner-file kalibr_cam0_target0_corners.pkl --image-timestamp-file kalibr_cam0_target0_timestamps.txt --imu-data-file imu0.csv --target aprilgrid.yaml --cams camchain.yaml --imu imu0.yaml --trim-imu-edge-count 0 --max-iter 30 --out-root simulation/generated/one_cam_one_imu/kalibr_runs --run-name arm64_target0_rotation_spline_iter30
python3 -B tools/run_kalibr_docker.py --dataset simulation/generated/one_cam_four_imus --image kalibr-camera-calibration:20.04-arm64 --platform linux/arm64 --corner-file kalibr_cam0_target0_corners.pkl --image-timestamp-file kalibr_cam0_target0_timestamps.txt --imu-data-file imu0.csv imu1.csv imu2.csv imu3.csv --target aprilgrid.yaml --cams camchain.yaml --imu imu0.yaml imu1.yaml imu2.yaml imu3.yaml --trim-imu-edge-count 0 --max-iter 30 --timeoffset-padding 0.12 --out-root simulation/generated/one_cam_four_imus/kalibr_runs --run-name arm64_target0_rotation_spline_iter30_pad012
```
