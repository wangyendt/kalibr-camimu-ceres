# Kalibr Cam-IMU Ceres / Kalibr 推导到 Ceres 复现

从 Kalibr cam-IMU 的公式、Jacobian、初始化和因子图出发，写成一本可读的推导书，再用 C++/Ceres 复现一条可独立运行的标定链路。

一句话：这是 **Kalibr cam-IMU 的可解释 Ceres 版本**。它不把 Kalibr result 当默认求解输入；Kalibr 只作为基线、热启动诊断和部分格式转换环境。

## 为什么值得看

| 证据 | 当前结果 |
|---|---|
| 匿名基准数据集独立标定 | 全部收敛；外参平移差 `1.3-4.7 mm`，旋转多数 `<0.02 deg` |
| 墙钟 | Ceres 原生平均约 `103 s`，Kalibr Docker 基线平均约 `203 s` |
| 热启动诊断 | 从 Kalibr 解出发仍会漂 `0.19-2.63 mm`，说明两套优化问题不是逐位相同 |
| TUM 双目 | Ceres single-stage residual 达到 Kalibr 同量级，time shift 误差约 `0.155-0.174 ms` |
| 文档 | `docs/books/kalibr_cam_imu_from_equations_to_ceres` 从坐标系、残差、Jacobian 写到源码对应 |

速度数字来自本机 arm64 Ceres 原生二进制对比 amd64 Kalibr Docker；严格算法倍率需要在同一 native Linux 环境复测。

## 目录

```text
apps/        calibrate_cam_imu、check_dataset、compare_kalibr_result
include/     Ceres cam-IMU 头文件，命名空间保留 ceres_cam_imu
src/         相机模型、B-spline、IMU residual、初始化、优化器
tests/       轻量数值检查和 Jacobian finite-difference 复核
tools/       数据转换、Kalibr Docker 基线、批量 sweep、两阶段诊断
simulation/  cam-IMU 仿真系统、示例数据、Ceres-compatible 输出
docs/books/  Kalibr cam-IMU 推导成书
docs/        plan/todo/experiment/knowhow/常用命令
```

未带入旧目录中的 `build/`、`out/`、`__pycache__/` 和旧实验 Dockerfile。这个仓库默认通过 CMake 在本机生成二进制；可复现实验用的容器定义放在 `docker/` 下。

## 构建

依赖：C++17、CMake、Eigen3、Ceres。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 跑一次独立标定

```bash
build/calibrate_cam_imu --corner-defaults --cam /ABS/cam_imu/cam0-camchain-640x400.yaml --imu /ABS/cam_imu/imu.yaml --target /ABS/cam_imu/aprilgrid.yaml --imu-data /ABS/cam_imu/data1.csv --corners /ABS/cam_imu/cam0_640x400_corners.csv --corner-poses /ABS/cam_imu/cam0_640x400_corner_poses.csv --estimate-time-shift-prior --estimate-orientation-gravity-prior --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 --max-iterations 150 --solver-max-trust-region-radius 10000000 --output-result /private/tmp/ceres_independent.yaml
```

相机—IMU 互相关会保留两路数据各自的时间戳原点，默认在至少保留较短序列 `50%` 重叠的全部 lag 上做粗到细搜索，因此可以处理不同时间原点；重叠率可用 `--time-shift-min-overlap-fraction` 调整。`--time-shift-max-search-s` 可显式设置正的绝对 time-shift 上限，`0`（默认）表示仅由重叠率限定。若最优峰落在重叠域或显式上限的边界，程序会把它视为“未知”而不是零偏移：普通单相机入口会保留已有 result、Kalibr、camchain 或显式初值；没有有效兜底时，可以不加时间偏移锚点继续优化，但不能同时使用正的 `--time-shift-prior-sigma`、正的 staged prior 或 `--fix-time-shift`。此时应提供可信的 `--initial-time-shift-s`，或检查两路数据是否确实包含足够的共同运动。

多相机 + 多 IMU 的 `--corner-defaults` 走一条确定性的 no-Kalibr 初始化路径：每个相机都用自己的 pose 流与参考 IMU 做上述互相关，非参考 IMU 也在至少 `50%` 重叠的全 lag 上与参考 IMU 对齐；camchain 只提供相机链几何，里面已有的 `timeshift_cam_imu` 不作为时间初值。各路得到的 shift/offset 会先用于统一样条时间域，再写入联合问题，因而相机和各 IMU 可以使用彼此无关的时间戳原点。非整数采样周期的原点差会保留为秒值，不会为了离散日志字段而被重新量化。

互相关初值只有 IMU 采样周期量级的离散分辨率，默认示例不再额外添加 `0.1 ms` 的紧先验。2025-04-19 19:03:03 数据上，紧先验把最终 time shift 从无先验的 `17.206 ms`（Kalibr single 为 `16.983 ms`）拉到 `14.174 ms`。如实验确实需要固定或正则化 time shift，应显式给出 sigma，并留意程序在 sigma 小于相关采样周期时的警告。

对比 Kalibr result：

```bash
build/compare_kalibr_result --kalibr-result /ABS/cam_imu/cam0_640x400_corners-1-results-imucam.txt --ceres-result /private/tmp/ceres_independent.yaml
```

## 生成仿真数据

```bash
python3 simulation/scripts/generate_examples.py
simulation/generated/one_cam_one_imu/run_calibration.sh --dry-run
```

仿真系统说明见 [simulation/README.md](simulation/README.md)。当前示例会生成 1cam-1imu、1cam-4imu、2cam-2imu 三组 Ceres-compatible 数据。

## 和 Kalibr Docker 配合

需要 Kalibr 环境的转换和基线脚本默认优先使用本机已有的 `kalibr-camera-calibration:20.04`。如果本机没有这个镜像，脚本会自动从 DockerHub 拉取 `wang121ye/kalibr-camera-calibration:20.04` 并运行，因此外部用户不需要本地克隆 `kalibr-docker` 仓库。

```bash
docker pull wang121ye/kalibr-camera-calibration:20.04
```

用本仓库工具跑 Kalibr Docker 基线：

```bash
python3 tools/run_kalibr_docker.py --dataset /ABS/cam_imu --run-name kalibr_baseline --max-iter 30
```

如果需要固定 Linux 依赖环境运行 Ceres 求解器，可以构建本项目的 solver 镜像。该镜像基于 Kalibr DockerHub 镜像，并通过 `gettool ceres -v 2.2.0` 构建 Ceres：

```bash
docker build -f docker/Dockerfile -t kalibr-camimu-ceres-solver:20.04 .
```

## 文档

- 常用命令：[docs/常用命令.txt](docs/%E5%B8%B8%E7%94%A8%E5%91%BD%E4%BB%A4.txt)
- 参数速查：[docs/knowhow/20260618_CeresCamIMU参数速查表.md](docs/knowhow/20260618_CeresCamIMU%E5%8F%82%E6%95%B0%E9%80%9F%E6%9F%A5%E8%A1%A8.md)
- 推导书：[docs/books/kalibr_cam_imu_from_equations_to_ceres/README.md](docs/books/kalibr_cam_imu_from_equations_to_ceres/README.md)
- 速度精度实验：[docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md](docs/experiment/20260616_Ceres%E4%B8%8EKalibrDocker%E5%A4%9A%E6%95%B0%E6%8D%AE%E9%9B%86%E9%80%9F%E5%BA%A6%E7%B2%BE%E5%BA%A6%E5%AF%B9%E6%AF%94.md)
- 重构计划：[docs/plan/20260615_Ceres_Cam-IMU标定重构计划.md](docs/plan/20260615_Ceres_Cam-IMU%E6%A0%87%E5%AE%9A%E9%87%8D%E6%9E%84%E8%AE%A1%E5%88%92.md)

## English

Kalibr Cam-IMU Ceres is a readable derivation and standalone C++/Ceres reimplementation of Kalibr's camera-IMU calibration pipeline.

It contains:

- A book-style derivation from coordinate frames, residuals, Jacobians, initialization, and factor graphs to code.
- A native C++/Ceres solver that consumes neutral YAML/CSV inputs.
- Tools for converting Kalibr pkl, ROS bag, and EuRoC/TUM-style inputs into Ceres CSV files.
- A Kalibr Docker baseline runner for controlled comparisons.

On the anonymized benchmark set, standalone Ceres converges on every run and averages about `103 s` wall clock versus about `203 s` for the Kalibr Docker baseline on this machine. Treat that as an engineering benchmark, not a universal algorithmic speedup claim, because the Kalibr baseline is Docker/amd64 while Ceres is native arm64.

## License

This repository keeps the original Ceres/Kalibr-derived source files and documentation in project form. Check upstream Kalibr licensing before redistributing modified Kalibr components or generated Docker images.
