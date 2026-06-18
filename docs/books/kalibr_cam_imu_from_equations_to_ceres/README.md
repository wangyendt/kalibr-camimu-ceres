# Kalibr Cam-IMU：从公式推导到 Ceres 复现

这组文档把 Kalibr cam-imu 标定按“从问题出发”的方式重写。它不是 API 手册，也不是只列公式的备忘录；它要回答的是：为什么这个标定问题可以被构造成一个非线性最小二乘问题，每个 residual 为什么长成现在这样，Jacobian 又是怎样一步步由链式法则得到的。

理论主线采用 Micro Lie Theory 记号：$\mathbf T\in SE(3)$、$\mathbf R\in SO(3)$、$\mathbf t\in\mathbb R^3$、$\mathrm{Exp}$、$\mathrm{Log}$、$\oplus$、$\ominus$ 和 $\mathrm{Ad}$。Jacobian 推导默认使用 Kalibr 的左侧/global 扰动约定，记作 $\boxplus_K$；源码对照时再映射到 Kalibr 变量名，例如 `T_w_b`、`T_cN_b` 和 expression node 的局部 Jacobian。

## 全书层次

这本书把 cam-imu 标定拆成四层。读的时候先建立问题和符号，再进入优化公共框架，然后逐个 residual 推 Jacobian，最后回到 Kalibr 源码和可复现验证。

| 层次 | 覆盖章节 | 核心问题 |
|---|---|---|
| 地图层 | 第 0-2 章 | 符号、坐标系、状态量、观测链路是什么 |
| 优化公共层 | 第 3 章 | 一个 residual 的 error、weight、Jacobian 如何进入 Gauss-Newton |
| residual / Jacobian 层 | 第 4-10 章 | camera、gyro、accel、time shift、extrinsics、bias、motion prior 和 IMU 扩展模型的具体公式怎么推 |
| 实现与验证层 | 第 11-13 章、附录 B | Kalibr expression graph、初始化链路和多传感器因子图怎么落到 Ceres，如何用最小 Python/C++ 程序复现和做 finite-difference check |

第 3 章不负责推完所有具体 Jacobian。它回答的是：**一旦某个 residual 的 Jacobian 推出来，优化器如何用它构造 $H\delta\theta=b$**。具体公式从第 4 章开始逐项展开。LM conditioner、CHOLMOD / QR 等线性求解器后端细节不是第 3 章的阅读前提。

## 阅读顺序

| 章节 | 文件 | 状态 | 作用 |
|---|---|---|---|
| 第 0 章 | [00_符号表与约定.md](00_符号表与约定.md) | draft | 固定 Micro Lie Theory 主记号、Kalibr 桥接表、残差方向和 Jacobian 方向 |
| 第 1 章 | [01_问题从哪里来.md](01_问题从哪里来.md) | draft | 从相机、IMU、标定板和连续轨迹解释为什么要这样构建优化问题 |
| 第 2 章 | [02_坐标系与刚体变换.md](02_坐标系与刚体变换.md) | draft | 从点变换推到 camera residual 的几何链路 |
| 第 3 章 | [03_最小二乘与残差方向.md](03_最小二乘与残差方向.md) | draft | 优化公共层：covariance、information matrix、robust kernel、线性化、$H$ 和 RHS |
| 第 4 章 | [04_相机观测模型.md](04_相机观测模型.md) | draft | 推导 reprojection residual、projection/distortion Jacobian、$\mathbf p_c$ 对 pose/extrinsic/time 的链式 Jacobian |
| 第 5 章 | [05_连续时间轨迹与B样条.md](05_连续时间轨迹与B样条.md) | draft | 推导 pose spline、姿态/位置控制点、速度、角速度、加速度和 time derivative 的 Jacobian |
| 第 6 章 | [06_陀螺仪残差.md](06_陀螺仪残差.md) | draft | 推导 gyro residual 对 pose spline、gyro bias、IMU 外参/内参的 Jacobian |
| 第 7 章 | [07_加速度计残差.md](07_加速度计残差.md) | draft | 推导 accelerometer residual 对旋转、线加速度、重力、accel bias、lever arm 和 IMU 参数的 Jacobian |
| 第 8 章 | [08_时间偏移与外参.md](08_时间偏移与外参.md) | draft | 汇总 time shift 和 sensor extrinsics 在 camera/IMU residual 中的共同 Jacobian 结构 |
| 第 9 章 | [09_bias与motion_prior.md](09_bias与motion_prior.md) | draft | 推导 bias spline、bias motion prior、pose motion prior 和 regularization residual/Jacobian |
| 第 10 章 | [10_扩展IMU模型.md](10_扩展IMU模型.md) | draft | 推导 scale、misalignment、size-effect、acceleration sensitivity 等扩展 IMU 参数的 Jacobian |
| 第 11 章 | [11_表达式图与源码对应.md](11_表达式图与源码对应.md) | draft | 把第 4-10 章公式映射到 Kalibr expression graph、error term、design variable 和源码调用链 |
| 第 12 章 | [12_Kalibr初始化迁移到Ceres.md](12_Kalibr初始化迁移到Ceres.md) | draft | 记录 time shift、pose spline、bias、orientation 和 gravity 初始化从 Kalibr 迁移到 Ceres 的状态与验证方法 |
| 第 13 章 | [13_多相机与多IMU因子图.md](13_多相机与多IMU因子图.md) | draft | 说明多相机 / 多 IMU 场景下 residual 索引、Jacobian block 稀疏性和变量更新规则怎样变化 |
| 附录 A | [appendix_A_SO3幂律与Rodrigues公式.md](appendix_A_SO3幂律与Rodrigues公式.md) | draft | 从 $[\boldsymbol\omega]_\times$ 的幂律推导 Rodrigues 公式，并连接第 0.10 节的 $SE(3)$ 指数映射 |
| 附录 B | [appendix_B_数值验证与最小复现.md](appendix_B_数值验证与最小复现.md) | draft | 用 Python/C++ 最小实现验证 SO(3)/SE(3)、projection、residual、analytic Jacobian 和 finite difference |
| 附录 C | [appendix_C_Jacobian速查表.md](appendix_C_Jacobian速查表.md) | draft | 调试与数值验证用的 Jacobian 速查：按 residual 分节列出几何 Jacobian 总表，每行标注推导出处 |

## 写作约定

每章按同一顺序展开：

1. 先讲直觉：这一章解决什么困惑。
2. 再讲模型：预测量怎么从状态量和观测时刻产生。
3. 再讲 residual：预测量和测量量按什么方向相减。
4. 再讲 Jacobian：先对中间量求导，再链到设计变量。
5. 最后给源码锚点：读代码时应该看哪里。

符号约定以第 0 章为准。后续章节如果必须引入新符号，先在本章局部定义，再回填到第 0 章。
