# EuRoC native cpp_tools 角点导出验证

## 背景

当前 `pkl` 和 `bag` 输入仍然依赖 Kalibr Docker，因为它们分别需要读取 Kalibr pickle 对象和 ROS bag。EuRoC 目录格式本身已经包含 `mav0/camN/data.csv`、图片和 `imu0/data.csv`，理论上可以绕过 Kalibr Docker，直接生成本工程 C++ 标定器需要的中立 CSV。

本次改动新增 `tools/export_euroc_native_to_ceres.py`，使用 `cpp_tools/cv/apriltag_detection` 的 ETHZ AprilTags pybind 检测角点，读取 EuRoC IMU CSV，并用 OpenCV `solvePnP` 生成 `cam0_corner_poses.csv`。`cpp_tools/cv/camera_models` 作为同一子模块稀疏检出，保留给后续更完整的相机模型投影/反投影复用。

2026-06-18 追加对齐：Kalibr 并不是直接写出 ETHZ detector 的原始四角点，它在 `GridCalibrationTargetAprilgrid.cpp` 中有 AprilGrid 后处理。进一步核对官方 Kalibr 后，ETHZ detector 的 local gradient `step` 应为 `1`；Kalibr Docker 基线镜像按 `step=1` 构建，并以 `wang121ye/kalibr-camera-calibration:20.04` 发布。`tools/build_cpp_tools_pybinds.sh` 也改为默认使用 cpp_tools upstream `step=1`。

## 实验问题

1. EuRoC 输入是否可以在不调用 Kalibr Docker 的情况下生成 `imu.csv`、`camN_corners.csv` 和 `cam0_corner_poses.csv`。
2. cpp_tools 提取的 AprilGrid 角点与 Kalibr Docker 提取结果在同一 TUM EuRoC 数据集上的像素误差是多少。
3. native 生成的 CSV 是否能被当前多相机 C++ 标定器读入并建图。

## 假设

Kalibr 的 EuRoC 链路会先通过 `kalibr_bagcreater` 把图片目录转成 ROS bag，再由 `IccCameraChain.targetObservations` 写出角点。导出脚本现在优先使用 ROS time 的整数 `secs/nsecs` 或 `to_nsec()`，避免 `timestamp.toSec()` 的 double 舍入，因此 EuRoC 原始 timestamp 可以和 native CSV 精确对齐。

Kalibr 的 `corner_id` 编码是 AprilGrid 展开后的 grid point id，不是 `tag_id * 4 + corner_slot`。native 导出器默认 `--corner-id-mode kalibr`，因此对齐后可以按 `corner-id` 直接比较；如果使用 `--corner-id-mode tag`，则应按同一帧的 `target_x/y/z` 比较。

## 配置

| 项目 | 内容 | 说明 |
|---|---|---|
| 数据集 | `/Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16` | EuRoC/mav0 双相机数据 |
| camchain | `/Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml` | pinhole + equidistant |
| imu yaml | `/Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/imu_config.yaml` | Ceres 标定器参数输入 |
| target | `/Users/wayne/Documents/work/data/TUM/april_6x6_80x80cm.yaml` | 6x6 AprilGrid, tagSize 0.088, spacing 0.3 |
| step=1 复测 Kalibr 输出 | `/private/tmp/ceres_tum_imu2_euroc_kalibr_step1_exact_ts` | Kalibr Docker 基线镜像，整数 timestamp |
| step=1 host native 输出 | `/private/tmp/ceres_tum_imu2_euroc_native_step1_current` | macOS host OpenCV 路径 |
| step=1 Docker native 输出 | `/private/tmp/ceres_tum_imu2_euroc_native_in_kalibr_docker_step1_sorted` | 在同一 Kalibr Docker/OpenCV 4.2 环境中构建 cpp_tools pybind |
| 匹配口径 | `--timestamp-tolerance-ns 0 --match-key corner-id` | 精确 timestamp，按 Kalibr grid corner id 匹配 |

## 命令

```bash
tools/update_cpp_tools_sparse_submodule.sh
tools/build_cpp_tools_pybinds.sh
python3 tools/prepare_ceres_inputs.py --source-type euroc --euroc-backend kalibr-docker --euroc-dir /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16 --cams /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --imu /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/imu_config.yaml --target /Users/wayne/Documents/work/data/TUM/april_6x6_80x80cm.yaml --out-dir /private/tmp/ceres_tum_imu2_euroc_kalibr_step1_exact_ts
python3 tools/prepare_ceres_inputs.py --source-type euroc --euroc-backend native --euroc-dir /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16 --cams /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/camchain.yaml --imu /Users/wayne/Documents/work/data/TUM/dataset-calib-imu2_512_16/dso/imu_config.yaml --target /Users/wayne/Documents/work/data/TUM/april_6x6_80x80cm.yaml --out-dir /private/tmp/ceres_tum_imu2_euroc_native_step1_current
python3 tools/compare_corner_csv.py --reference /private/tmp/ceres_tum_imu2_euroc_kalibr_step1_exact_ts/cam0_corners.csv --candidate /private/tmp/ceres_tum_imu2_euroc_native_step1_current/cam0_corners.csv --timestamp-tolerance-ns 0 --label tum_imu2_cam0_host_native_current_vs_kalibr --match-key corner-id
python3 tools/compare_corner_csv.py --reference /private/tmp/ceres_tum_imu2_euroc_kalibr_step1_exact_ts/cam1_corners.csv --candidate /private/tmp/ceres_tum_imu2_euroc_native_step1_current/cam1_corners.csv --timestamp-tolerance-ns 0 --label tum_imu2_cam1_host_native_current_vs_kalibr --match-key corner-id
```

## 结果

复核官方 Kalibr 后，Kalibr Docker 基线镜像按 `step=1` 重建并发布。随后用同一 TUM EuRoC 数据做两组最新输出：

| 对比 | cam0 结果 | cam1 结果 | 结论 |
|---|---:|---:|---|
| host native vs Kalibr step=1 | reference/candidate `148233/148244`，common `148205`，reference_only/candidate_only `28/39`，mean/median `0.000853/0.000341 px`，RMS/P95/max `0.002988/0.002306/0.068158 px` | reference/candidate `148184/148183`，common `148161`，reference_only/candidate_only `23/22`，mean/median `0.000725/0.000288 px`，RMS/P95/max `0.002726/0.001908/0.061628 px` | macOS host 仍有 OpenCV/平台数值差异 |
| native-in-Kalibr-Docker vs Kalibr step=1 | rows/common `148233/148233`，reference_only/candidate_only `0/0`，mean/RMS/max `0/0/0 px`，`byte_equal=True` | rows/common `148184/148184`，reference_only/candidate_only `0/0`，mean/RMS/max `0/0/0 px`，`byte_equal=True` | 同一 OpenCV 4.2/Linux 环境下 corner CSV 已 byte-level 对齐 |

byte-level 对齐需要三个条件同时满足：

- Kalibr Docker 与 cpp_tools detector 都使用官方 `step=1`。
- Kalibr 导出脚本使用整数 timestamp，不经过 `toSec()` double 转换。
- native CSV 写出时按 Kalibr observation 的 `corner_id` 顺序排序，并把 target 坐标按 Kalibr 暴露的 float32 形态写出。

## 分析

Kalibr EuRoC 链路仍然适合作为基线，因为它复用 Kalibr 自己的 `IccCameraChain` 和 target observation 管线；但它不是 EuRoC 数据的必要依赖。native 链路直接读取 EuRoC 原始纳秒时间戳，避免了 EuRoC -> bag -> Kalibr observation 的中间转换。

最新结果表明，`step=1` 口径下 native exporter 的算法路径已经可以和 Kalibr 对齐。只要 native cpp_tools 也在 Kalibr Docker/OpenCV 4.2/Linux 环境中运行，`cam0_corners.csv` 和 `cam1_corners.csv` 与 Kalibr 导出结果逐字节一致。

macOS host native 仍有 `0.001 px` 量级尾部差异，原因是运行环境不同：host Python `cv2` 为 OpenCV 4.12.0，cpp_tools pybind 链接 Homebrew OpenCV 4.8.1，而 Kalibr Docker 使用 OpenCV 4.2.0。这个差异影响 host 上的 byte-level 复现，但不改变 EuRoC native 路径可独立生成 CSV 的结论。

## 结论

EuRoC/mav0 输入已经可以独立于 Kalibr Docker 生成本工程需要的 CSV。`.bag` 和 `.pkl` 仍然需要 Kalibr Docker；EuRoC 默认 backend 已切到 native，Kalibr Docker backend 保留为基线对比。

官方 `step=1` 口径下，算法路径已经可以对齐到 byte-level：在同一 Kalibr Docker/OpenCV 4.2 环境里构建并运行 native cpp_tools exporter，`cam0_corners.csv` 和 `cam1_corners.csv` 与 Kalibr 导出结果逐字节一致。macOS host native 输出不应承诺 byte-level；它和 Kalibr step=1 的公共角点 mean 误差约 `7e-4` 到 `9e-4 px`，P95 约 `0.002 px`，主要来自 OpenCV 版本和平台数值路径。

## 未覆盖问题

1. 当前 native PnP 覆盖 pinhole+radtan、pinhole+equidistant；其他 camera model 还需要接入 `cpp_tools/cv/camera_models` 或补充模型分支。
2. 当前只导出 EuRoC `imu0`；多 IMU EuRoC 目录如果需要，应扩展输出 `imuN.csv` 和 prepare 参数映射。
3. 如需在 host native 上也追求 byte-level，需要把 OpenCV 版本、编译器、架构和后处理路径固定到与 Kalibr Docker 一致；否则只能保证数值等价而不是文件字节一致。

## 下一步

用 native EuRoC CSV 跑完整 Ceres 标定，并和 Kalibr result 做精度与墙钟 benchmark；当前统一使用 `docker/Dockerfile` 构建 Ceres solver 镜像；该镜像基于 Kalibr DockerHub 镜像，可作为后续固定 Linux 运行环境的基础。
