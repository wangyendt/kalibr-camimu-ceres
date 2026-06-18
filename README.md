# Kalibr Cam-IMU Ceres / Kalibr 推导到 Ceres 复现

从 Kalibr cam-IMU 的公式、Jacobian、初始化和因子图出发，写成一本可读的推导书，再用 C++/Ceres 复现一条可独立运行的标定链路。

一句话：这是 **Kalibr cam-IMU 的可解释 Ceres 版本**。它不把 Kalibr result 当生产输入；Kalibr 只作为基线、热启动诊断和部分格式转换环境。

## 为什么值得看

| 证据 | 当前结果 |
|---|---|
| Production 12 组独立标定 | 全部收敛；外参平移差 `1.3-4.7 mm`，旋转多数 `<0.02 deg` |
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
docs/books/  Kalibr cam-IMU 推导成书
docs/        plan/todo/experiment/knowhow/常用命令
```

未带入旧目录中的 `build/`、`out/`、`__pycache__/` 和实验 Dockerfile。这个仓库默认通过 CMake 在本机生成二进制。

## 构建

依赖：C++17、CMake、Eigen3、Ceres。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 跑一次生产独立标定

```bash
build/calibrate_cam_imu --corner-defaults --cam /ABS/cam_imu/cam0-camchain-640x400.yaml --imu /ABS/cam_imu/imu.yaml --target /ABS/cam_imu/aprilgrid.yaml --imu-data /ABS/cam_imu/data1.csv --corners /ABS/cam_imu/cam0_640x400_corners.csv --corner-poses /ABS/cam_imu/cam0_640x400_corner_poses.csv --estimate-time-shift-prior --estimate-orientation-gravity-prior --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 --max-iterations 150 --solver-max-trust-region-radius 10000000 --output-result /private/tmp/ceres_independent.yaml
```

对比 Kalibr result：

```bash
build/compare_kalibr_result --kalibr-result /ABS/cam_imu/cam0_640x400_corners-1-results-imucam.txt --ceres-result /private/tmp/ceres_independent.yaml
```

## 和 Kalibr Docker 仓库配合

如果本机还没有 `kalibr-camera-calibration:20.04` 镜像，可以在本仓库下克隆 Docker 包装仓库再构建：

```bash
git clone ../kalibr external/kalibr-docker
docker build -f external/kalibr-docker/docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 external/kalibr-docker
```

然后用本仓库工具跑 Kalibr Docker 基线：

```bash
python3 tools/run_kalibr_docker.py --dataset /ABS/cam_imu --run-name kalibr_baseline --max-iter 30
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

On the current 12 production datasets, standalone Ceres converges on every run and averages about `103 s` wall clock versus about `203 s` for the Kalibr Docker baseline on this machine. Treat that as an engineering benchmark, not a universal algorithmic speedup claim, because the Kalibr baseline is Docker/amd64 while Ceres is native arm64.

## License

This repository keeps the original Ceres/Kalibr-derived source files and documentation in project form. Check upstream Kalibr licensing before redistributing modified Kalibr components or generated Docker images.
