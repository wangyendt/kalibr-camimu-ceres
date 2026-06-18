# AprilTag 梯度 step 影响

## 场景

在对齐 `cpp_tools` 原生 EuRoC 角点导出和 Kalibr Docker 基线时，需要确认 ETHZ AprilTag detector 里的 local gradient `step` 是否一致。这个值不属于 Kalibr 的 AprilGrid 后处理参数，而是在 tag 检测早期就影响边缘分组、线段拟合和 quad 候选生成。

当前状态：

| 实现 | 源码位置 | step |
|---|---|---:|
| 官方 `ethz-asl/kalibr` master | `aslam_offline_calibration/ethz_apriltag2/src/TagDetector.cc` | `1` |
| Kalibr Docker 基线镜像 | `wang121ye/kalibr-camera-calibration:20.04` 内 Kalibr fork 源码 | `1` |
| 当前 `cpp_tools` | `third_party/cpp_tools/cv/apriltag_detection/src/ethz_apriltag2/src/TagDetector.cc` | `1` |

## 现象

`step` 不一致时，公共角点的亚像素坐标可能仍然非常接近，但参与标定的角点集合会不同。表现通常不是所有像素都偏一个固定量，而是某些边界帧、弱纹理帧、局部模糊帧里的 tag 被检出或被漏掉，最终导致 `camN_corners.csv` 行数、corner set 和 residual 数量不同。

在 TUM 双相机 EuRoC 对齐中，早期 Kalibr Docker fork 曾使用 `step=3`，因此直接使用 `cpp_tools` upstream detector 的 `step=1` 会比旧 fork 检出更多角点。复核官方 Kalibr 后，Kalibr Docker 基线已按 `step=1` 重建并发布；在同一 Kalibr Docker/OpenCV 4.2 环境里构建 cpp_tools exporter 后，native 与 Kalibr 的 `cam0_corners.csv`、`cam1_corners.csv` 已经 byte-level 一致。

## 原因

ETHZ AprilTag detector 在低通后的 `fimSeg` 图像上计算局部梯度。`step` 控制中心差分的采样半径：

```cpp
Ix = fimSeg.get(x + step, y) - fimSeg.get(x - step, y);
Iy = fimSeg.get(x, y + step) - fimSeg.get(x, y - step);
```

这个梯度会继续进入：

- `fimMag` 和 `fimTheta`，决定每个像素的梯度强度和方向；
- `Edge::calcEdges`，决定相邻像素是否形成可合并边；
- `Segment` 拟合，决定线段是否够长、方向是否稳定；
- quad 组合与 tag decode，决定一个 tag 是否进入后续 AprilGrid 观测。

因此 `step` 是检测器行为参数，不是后处理里的简单阈值。后面的 `minBorderDistance`、duplicate tag 检查、`cornerSubPix`、subpix 位移过滤、PnP 重投影外点过滤，只能在已经生成的 tag 候选上继续筛选，不能完全抵消 `step` 的影响。

## 方法

`step` 越小，梯度越局部：

| 取值方向 | 优点 | 代价 |
|---|---|---|
| 小，例如 `1` | 边缘响应更贴近局部像素；对小 tag、细边缘、清晰图像更敏感；符合官方 Kalibr upstream 和当前 `cpp_tools` upstream | 更容易受噪声、压缩、纹理、Bayer/锐化伪影影响；弱质量帧中可能生成更多候选或不同的边缘分组 |
| 大，例如 `3` | 梯度跨更宽像素间隔，等价于更强的局部平滑；对噪声和局部毛刺更不敏感；可用于复现实验早期旧 `kalibr-docker` fork | 会牺牲细节，对小 tag 或边缘很窄的图像更容易漏检；图像边缘会多损失 `step` 像素的可用区域；和官方 upstream 行为不一致 |

实际选择规则：

1. 如果目标是复现官方 `ethz-asl/kalibr` master、当前 Kalibr Docker 基线镜像或保持 `cpp_tools` upstream 行为，使用 `step=1`。
2. 如果需要复现实验早期旧 fork 的历史结果，可显式设置 `APRILTAG_GRADIENT_STEP=3` 或 `KALIBR_APRILTAG_COMPAT=1` 重新构建 cpp_tools pybind。
3. 如果要长期维护，不建议在本工程永久改脏 `cpp_tools` 子模块源码；更稳的是让 `cpp_tools` 暴露 `gradient_step` 参数，默认保持 upstream `1`，需要复现实验旧值时显式传 `3`。
4. 如果只是本工程内部验证，可以在构建脚本里复制一份临时源码再 patch `step`，这样不污染 submodule，也能固定 benchmark 口径。

## 验证

确认本地源码当前值：

```bash
rg -n "int step|for \\(int y = 1|for \\(int y = step|fimSeg\\.get\\(x\\+1|fimSeg\\.get\\(x\\+step" third_party/cpp_tools/cv/apriltag_detection/src/ethz_apriltag2/src/TagDetector.cc
```

确认 Kalibr Docker 基线镜像时，优先使用 DockerHub 镜像；本项目脚本在缺少本地默认镜像时会自动拉取 `wang121ye/kalibr-camera-calibration:20.04`。

确认影响时不要只看公共角点的像素误差，还要同时看：

- `reference_rows` / `candidate_rows`；
- `common_rows`；
- `reference_only` / `candidate_only`；
- 每帧缺失或新增的 `corner_id` 分布；
- 最终 Ceres problem 的 camera residual 数量。

公共角点 mean 误差接近 `0` 只说明共同保留下来的点坐标一致；如果 corner set 仍不同，标定问题规模和鲁棒核受力点仍然不同。

## 适用条件

这个判断适用于 ETHZ `ethz_apriltag2` 风格 detector。它不直接适用于 AprilTag 3、OpenCV ArUco、AprilGrid 后处理参数或 Ceres 优化器参数。

对齐 Kalibr 时要先明确“Kalibr”指哪个版本：

- 官方 upstream：`step=1`。
- 当前 Kalibr Docker 基线镜像：`step=1`。
- 实验早期旧 fork：曾为 `step=3`，只能作为历史复现实验口径。

不同口径不能混用同一个“Kalibr-compatible”标签；实验记录必须写明 step 值和运行环境。

## 注意事项

- `step` 改变的是 tag 检测阶段的边缘图，不是最终 subpixel refinement 的窗口大小。
- `step` 越大不一定越准；它更像鲁棒性和细节敏感性的取舍。
- 数据集中的 tag 尺寸、成像清晰度、运动模糊、曝光、压缩和边缘覆盖都会改变最佳取值。
- 做 benchmark 时必须把 `step` 写进实验文档或命令输出，否则同一数据集的角点行数差异很难追溯。
- 真正的 byte-level 对齐还依赖 OpenCV 版本和平台。当前验证表明：同一 Kalibr Docker/OpenCV 4.2 环境可 byte-identical；macOS host native 只能做到亚毫像素级数值贴近。
