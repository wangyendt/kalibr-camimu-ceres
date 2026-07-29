# 第 13 章：多相机与多 IMU 因子图

前面第 4-10 章一直按“一个 camera、一个 IMU”的口径推 residual 和 Jacobian。这样写是为了把每条链式法则讲清楚：camera residual 怎么从角点走到像素，gyro residual 怎么从 body angular velocity 走到陀螺仪读数，accelerometer residual 怎么从 specific force 走到加速度计读数，扩展 IMU 参数又怎样包在外层。

但真实 rig 往往不是单传感器。双目、环视、多 IMU、参考 IMU 加辅助 IMU 都会问同一个问题：**多了传感器以后，residual、Jacobian 和变量更新到底哪里变了？**

本章的答案先说在前面：**局部 residual 公式不变，变化的是索引、变量归属和 block 稀疏结构。** 每个 camera 仍然生成第 4 章的 reprojection residual；每个 IMU 仍然生成第 6、7、10 章的 gyro/accel residual。真正要小心的是：这些 residual 连接到哪些传感器私有变量，哪些变量由所有传感器共享，以及优化器求出一条全局更新后应该更新哪些 block。

如果把单传感器公式看成一块砖，多相机 / 多 IMU 不是换一块砖，而是把很多块砖接到同一条 body trajectory 上。因子图层面的故事，就是本章要补上的部分。

## 13.1 一条公共轨迹，多组传感器接口

多传感器 cam-imu 标定的核心约束仍然是一条公共刚体轨迹：

$$
\mathbf T_{wb}(t).
$$

它描述 body frame 在 world frame 中的连续运动。所有 camera、所有 IMU 都被认为刚性安装在同一个 body 上，因此它们不应该各自拥有一条轨迹。它们的差异只体现在两类接口变量上：

1. **空间接口**：第 $n$ 个 camera 有自己的 $\mathbf T_{c_nb}$，第 $m$ 个 IMU 有自己的 $\mathbf R_{i_mb}$ 和 lever arm $\mathbf r_{b,m}$。
2. **时间接口**：第 $n$ 个 camera 有自己的 $\Delta t_n$，第 $m$ 个 IMU 如果参与时间标定，也有自己的 $\Delta t^i_m$。

再加上传感器内部参数：

| 变量族 | 是否共享 | 例子 |
|---|---|---|
| body trajectory | 全局共享 | pose spline control points $\mathbf c_j$ |
| gravity | 全局共享 | $\mathbf g_w$ |
| camera 参数 | camera 私有 | $\mathbf T_{c_nb}$、$\Delta t_n$、intrinsics、distortion |
| IMU 参数 | IMU 私有 | $\mathbf R_{i_mb}$、$\mathbf r_{b,m}$、bias splines、$\mathbf M_{a,m}$、$\mathbf M_{g,m}$、$\mathbf A_{g,m}$、$\mathbf R_{g_mi_m}$ |
| motion prior | 按变量族作用 | pose prior 作用在公共 pose spline，bias prior 作用在对应 IMU 的 bias spline |

这里的“私有”不是说它们不参与联合优化，而是说第 $n$ 个 camera 的 residual 不会直接对第 $q\ne n$ 个 camera 的 intrinsics 求导；第 $m$ 个 IMU 的 residual 不会直接对第 $r\ne m$ 个 IMU 的 bias 求导。它们仍然会通过共享的 pose spline 和 gravity 间接耦合。

## 13.2 Camera residual：从一类 residual 变成一组 residual

第 $n$ 个 camera 的第 $k$ 帧、第 $\ell$ 个角点 residual 写成：

$$
t_{n,k}
=
t^{\mathrm{cam}}_{n,k}
+
\Delta t_n^{\mathrm{prior}}
+
\Delta t_n,
$$

$$
\boxed{
\mathbf e^\pi_{n,k,\ell}
=
\mathbf y_{n,k,\ell}
-
\boldsymbol\pi_n\!\left(
\mathbf T_{c_nb}\mathbf T_{bw}(t_{n,k})\mathbf p^w_\ell;
\boldsymbol\eta_n,\boldsymbol\kappa_n
\right).
}
$$

和第 4 章相比，只是多了 camera index $n$：

| 单 camera 写法 | 多 camera 写法 | 含义 |
|---|---|---|
| $\mathbf T_{cb}$ | $\mathbf T_{c_nb}$ | 第 $n$ 台 camera 的 body-to-camera 外参 |
| $\Delta t$ | $\Delta t_n$ | 第 $n$ 台 camera 的 time shift correction |
| $\boldsymbol\pi$ | $\boldsymbol\pi_n$ | 第 $n$ 台 camera 的投影模型 |
| $\boldsymbol\eta,\boldsymbol\kappa$ | $\boldsymbol\eta_n,\boldsymbol\kappa_n$ | 第 $n$ 台 camera 的内参和畸变 |

如果使用第 8.5 节的 camera chain 参数化，则不是为每台 camera 独立放一个完整 $\mathbf T_{c_nb}$，而是：

$$
\mathbf T_{c_nb}
=
\mathbf T_n\mathbf T_{n-1}\cdots\mathbf T_1\mathbf T_0.
$$

这时 camera $n$ 的一个 reprojection residual 会连接 chain 中从 $\mathbf T_0$ 到 $\mathbf T_n$ 的所有 link；它不会连接 $\mathbf T_{n+1}$ 以及更下游、和它无关的 link。这个“只连接路径上的 link”就是多 camera Jacobian 稀疏性的第一条规则。

## 13.3 IMU residual：每个 IMU 复制一套 bias 和内参

第 $m$ 个 IMU 的查询时间记为：

$$
t_{m,k}
=
t^{\mathrm{imu}}_{m,k}
+
\Delta t^i_m.
$$

如果 IMU time offset 不作为优化变量，$\Delta t^i_m$ 就只是构造 residual 时的常量偏移；如果它作为 design variable，则第 8.3 节的 time-offset Jacobian 直接带上 index $m$。

普通 gyro residual 变成：

$$
\boxed{
\mathbf e^\omega_{m,k}
=
\mathbf R_{i_mb}\boldsymbol\omega_b(t_{m,k})
+
\mathbf b^g_m(t_{m,k})
-
\mathbf z^\omega_{m,k}.
}
$$

普通 accelerometer residual 变成：

$$
\boxed{
\mathbf e^a_{m,k}
=
\mathbf R_{i_mb}\mathbf u_{b,m}(t_{m,k})
+
\mathbf b^a_m(t_{m,k})
-
\mathbf z^a_{m,k}.
}
$$

其中：

$$
\mathbf u_{b,m}
=
\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)
+
\boldsymbol\alpha_b\times\mathbf r_{b,m}
+
\boldsymbol\omega_b\times
(\boldsymbol\omega_b\times\mathbf r_{b,m}).
$$

和单 IMU 公式相比，关键变化是 bias 和 IMU 内参都按 IMU index 分开：

| 变量 | 多 IMU 规则 |
|---|---|
| gyro bias spline | 每个 IMU 一条 $\mathbf b^g_m(t)$ |
| accel bias spline | 每个 IMU 一条 $\mathbf b^a_m(t)$ |
| IMU 外参旋转 | 非 reference IMU 有自己的 $\mathbf R_{i_mb}$ |
| IMU lever arm | 非 reference IMU 有自己的 $\mathbf r_{b,m}$ |
| 扩展 IMU 参数 | 每个 IMU 一套 $\mathbf M_{a,m}$、$\mathbf M_{g,m}$、$\mathbf A_{g,m}$、$\mathbf R_{g_mi_m}$ |

Reference IMU 需要单独看。若 body frame 被定义为 reference IMU frame，则：

$$
\mathbf R_{i_0b}=\mathbf I,
\qquad
\mathbf r_{b,0}=\mathbf 0.
$$

这两个量可以在实现里保留对象，但通常固定不优化。否则 body frame 和所有 IMU 外参会一起漂移，形成没有物理意义的 gauge 自由度。

扩展 IMU 模型也只是把第 10 章的外层函数按 $m$ 复制。例如 scale/misalignment accelerometer 可写成：

$$
\mathbf e^a_{m,k}
=
\mathbf M_{a,m}\mathbf R_{i_mb}\mathbf u_{b,m}
+
\mathbf b^a_m
-
\mathbf z^a_{m,k}.
$$

扩展 gyro 可写成：

$$
\mathbf e^\omega_{m,k}
=
\mathbf M_{g,m}\boldsymbol\omega_{g,m}
+
\mathbf A_{g,m}\mathbf a_{g,m}
+
\mathbf b^g_m
-
\mathbf z^\omega_{m,k},
$$

其中：

$$
\boldsymbol\omega_{g,m}
=
\mathbf R_{g_mi_m}\mathbf R_{i_mb}\boldsymbol\omega_b,
\qquad
\mathbf a_{g,m}
=
\mathbf R_{g_mi_m}\mathbf R_{i_mb}\mathbf u_{b,m}.
$$

所以多 IMU 并不要求重新推 $\mathbf M_a$、$\mathbf M_g$、$\mathbf A_g$ 或 $\mathbf R_{gi}$ 的 Jacobian；只要把“当前 IMU 的参数”接到“当前 IMU 的 residual”上，局部公式就是第 10 章那一套。

## 13.4 Jacobian 的变化：从公式变成 block 选择

多传感器以后，每个 residual 的局部 Jacobian 仍然来自第 4-10 章。真正容易写错的是 block 选择。

第 $n$ 个 camera residual 的非零 block 是：

| block | 是否非零 | 原因 |
|---|---|---|
| 当前时刻 active pose controls | 是 | $\mathbf T_{bw}(t_{n,k})$ 由公共 pose spline 给出 |
| camera $n$ 的 time shift $\Delta t_n$ | 是，若启用 | 改变 $t_{n,k}$ |
| camera $n$ 的 intrinsics / distortion | 是 | 进入 $\boldsymbol\pi_n$ |
| camera $n$ 的完整外参 $\mathbf T_{c_nb}$ | 是，若用完整外参参数化 | 进入 $\mathbf T_{c_nb}\mathbf T_{bw}$ |
| camera chain 中通往 $n$ 的 link | 是，若用 chain 参数化 | 这些 link 组成 $\mathbf T_{c_nb}$ |
| 其他 camera 的私有 intrinsics / time shift | 否 | 不在当前 projection 链路中 |
| IMU bias、IMU 内参、IMU lever arm | 否 | camera 前向模型不依赖这些变量 |

用附录 C 的记号，camera chain link 的 Jacobian 仍然是：

$$
\frac{\partial\mathbf e^\pi_{n,k,\ell}}
{\partial\boldsymbol\xi_{\mathbf T_m,K}}
=
\mathbf A_T\mathrm{boxTimes}(\mathbf P_m),
\qquad
m\le n.
$$

若 $m>n$，这个 block 为零。

第 $m$ 个 IMU residual 的非零 block 是：

| residual | 非零 block | 典型零 block |
|---|---|---|
| ordinary gyro | active pose controls、$\mathbf R_{i_mb}$、$\mathbf b^g_m$、optional $\Delta t^i_m$ | camera 参数、其他 IMU bias、gravity、$\mathbf r_{b,m}$ |
| ordinary accel | active pose controls、gravity、$\mathbf R_{i_mb}$、$\mathbf r_{b,m}$、$\mathbf b^a_m$、optional $\Delta t^i_m$ | camera 参数、gyro bias、其他 IMU bias |
| extended gyro | ordinary gyro blocks，加 $\mathbf M_{g,m}$、$\mathbf A_{g,m}$、$\mathbf R_{g_mi_m}$；若有 $\mathbf A_g\mathbf a_g$ 分支，则 gravity 和 $\mathbf r_{b,m}$ 也可能非零 | camera 参数、其他 IMU 的内参和 bias |
| extended accel | ordinary accel blocks，加 $\mathbf M_{a,m}$ 和 size-effect 相关 lever arm | camera 参数、其他 IMU 的内参和 bias |

这张表体现一个实用原则：**先问 residual 的 forward expression 里有没有这个变量；没有就直接是零 block。** 不要因为“同在一个优化问题里”就把所有传感器变量都连起来。传感器之间的耦合主要通过公共 pose spline 和 gravity 发生，而不是通过彼此的私有参数直接发生。

## 13.5 Hessian 装配：耦合来自共享变量

多传感器总 cost 可以写成：

$$
\begin{aligned}
E
=&
\sum_n\sum_k\sum_\ell
\left\|
\mathbf e^\pi_{n,k,\ell}
\right\|^2_{\boldsymbol\Omega^\pi_{n,k,\ell}}
\\
&+
\sum_m\sum_k
\left\|
\mathbf e^\omega_{m,k}
\right\|^2_{\boldsymbol\Omega^\omega_{m,k}}
+
\sum_m\sum_k
\left\|
\mathbf e^a_{m,k}
\right\|^2_{\boldsymbol\Omega^a_{m,k}}
\\
&+
E_{\mathrm{pose\ prior}}
+
\sum_m E_{\mathrm{bias\ prior},m}.
\end{aligned}
$$

每个 residual 线性化以后仍然按第 3 章和 11.7 节的规则装配：

$$
\mathbf H_{uv}
\mathrel{+}=
\bar{\mathbf J}_u^\top
\bar{\mathbf J}_v,
\qquad
\mathbf b_u
\mathrel{-}=
\bar{\mathbf J}_u^\top
\bar{\mathbf e}.
$$

多传感器带来的结构可以这样读：

1. 一个 camera residual 会把 camera $n$ 的变量和同一时刻的 pose controls 连在一起。
2. 一个 IMU residual 会把 IMU $m$ 的变量和同一时刻的 pose controls 连在一起。
3. 不同 camera、不同 IMU 之间通常没有直接 residual，但它们会因为连接到同一组 pose controls，在全局 Hessian 里间接耦合。
4. 如果使用 camera chain，靠近 body 的 link 会被多个 camera 共享，因此多个 camera residual 会直接对同一个 link block 累加信息。
5. Bias prior 是每个 IMU 私有的平滑约束，不会把不同 IMU 的 bias spline 连在一起，除非模型显式引入跨 IMU bias 先验。

这也是为什么多传感器问题更适合按 block sparse 的方式看，而不是把所有变量拼成一个没有结构的大向量。Jacobian 不是“多了传感器就变密”，而是“每个 residual 仍然很稀疏，但更多 residual 共享同一条 trajectory backbone”。

## 13.6 变量更新：全局解一次，按 block 各自回写

线性求解器解出的不是某个 camera 或某个 IMU 的局部更新，而是一整个全局增量：

$$
\delta\boldsymbol\theta
=
\left[
\delta\mathbf c^\top,
\delta\mathbf g^\top,
\delta\boldsymbol\theta_{\mathrm{cam},0}^\top,\ldots,
\delta\boldsymbol\theta_{\mathrm{cam},N-1}^\top,
\delta\boldsymbol\theta_{\mathrm{imu},0}^\top,\ldots,
\delta\boldsymbol\theta_{\mathrm{imu},M-1}^\top
\right]^\top.
$$

回写时按变量所在 manifold 更新：

| 变量 | 更新方式 |
|---|---|
| pose spline control $\mathbf c_j$ | 按第 5 章的 pose spline design variable 规则更新 |
| camera chain transform $\mathbf T_m$ | $\mathbf T_m^+=\mathbf T_m\boxplus_K\delta\boldsymbol\xi_m$ |
| complete camera transform $\mathbf T_{c_nb}$ | $\mathbf T_{c_nb}^+=\mathbf T_{c_nb}\boxplus_K\delta\boldsymbol\xi_n$ |
| camera time shift $\Delta t_n$ | $\Delta t_n^+=\Delta t_n+\delta\Delta t_n$ |
| camera intrinsics / distortion | 欧式加法，或按具体 camera model 的参数化规则更新 |
| IMU rotation $\mathbf R_{i_mb}$ | $\mathbf R_{i_mb}^+=\mathrm{Exp}(-\delta\boldsymbol\phi_{m,K})\mathbf R_{i_mb}$ |
| IMU lever arm $\mathbf r_{b,m}$ | $\mathbf r_{b,m}^+=\mathbf r_{b,m}+\delta\mathbf r_{b,m}$ |
| IMU bias control point $\mathbf d^g_{m,j},\mathbf d^a_{m,j}$ | 欧式加法 |
| IMU extended matrices $\mathbf M_{a,m},\mathbf M_{g,m},\mathbf A_{g,m}$ | 按 active mask 做欧式加法 |
| gyro sensing rotation $\mathbf R_{g_mi_m}$ | 按 rotation design variable 更新 |

共享变量只更新一次。比如同一个 pose control point 可能被多个 camera residual 和多个 IMU residual 同时约束，但求解器得到的是这个 control point 的一个总增量，而不是分别给每个传感器更新一份轨迹。

Reference IMU 的外参如果被固定，则它的 block 不进入活跃变量集合；即使公式里能写出 $\partial\mathbf e/\partial\mathbf R_{i_0b}$，装配时也不会为这个固定 block 加列。这个区别很重要：**公式上非零，不等于实现里一定是 active design variable。**

## 13.7 实现检查表

实现多相机 / 多 IMU 时，可以按下面顺序自查。

| 检查项 | 应该满足 |
|---|---|
| trajectory | 所有传感器 residual 查询同一条 pose spline |
| camera data loop | 按 camera index 建 residual，使用该 camera 的观测、内参、畸变、time shift |
| camera extrinsics | 若用 chain，只连接当前 camera 路径上的 link |
| IMU data loop | 按 IMU index 建 gyro/accel residual，使用该 IMU 的 bias spline、外参和内参 |
| reference IMU | body frame 定义清楚，reference 外参固定或有明确先验 |
| bias prior | 每个 IMU 的 gyro/accel bias prior 只作用在自己的 bias control points |
| variable ordering | block id/name 带 sensor index，避免 cam0 参数误连到 cam1 residual |
| reporting | 输出 per-camera 外参/time shift 和 per-IMU 外参/bias/扩展参数，不把它们平均成一个值 |

Kalibr 的 expression graph 天然适合这种组织方式：每个 residual 只把自己用到的 expression node 接进图里，JacobianContainer 只会收到这些 active block 的 Jacobian。Ceres 或其他后端若手写 residual，也应该保持同样的 block 选择规则；不要为了实现方便把无关传感器变量塞进同一个 residual parameter list。

## 13.8 单相机多 IMU：外参、杠杆臂与精确参数化

13.1-13.7 给的是因子图层面的结构规则。但本书的主线是“单相机 + 单 IMU 已对齐，现在要把它扩到单相机 + 多 IMU”。这一节开始，把多 IMU 真正新增的两个设计变量 —— **非参考 IMU 的外参旋转 $\mathbf R_{i_mb}$ 和杠杆臂 $\mathbf r_{b,m}$** —— 按 Kalibr 源码逐项推清楚。这两个量在单 IMU 里被固定为 $\mathbf I$ 和 $\mathbf 0$，所以第 6、7 章并没有真正推过它们的 Jacobian。本节及 13.9 节补的就是这块缺口。

> 重要更正：13.3 节说“多 IMU 的 Jacobian 只要照搬第 4-10 章”，这句话**只对 pose/bias/gravity/扩展内参成立**，对 $\mathbf R_{i_mb}$ 和 $\mathbf r_{b,m}$ 不成立——因为单 IMU 章节根本没有对它们求过导（它们当时是常量）。所以本节是对 13.3 节的必要补充，不是重复。

### 13.8.1 body frame 与 IMU 链的约定

约定 body frame 与 **reference IMU（imu0）**重合。Kalibr 源码里这体现为 `isReferenceImu=True` 时，旋转外参和杠杆臂都被建成 design variable 但 `setActive(False)`：

$$
\mathbf R_{i_0b}=\mathbf I,
\qquad
\mathbf r_{b,0}=\mathbf 0
\qquad(\text{固定不优化}).
$$

第 $m\ (m\ge1)$ 个非参考 IMU 有两个活跃设计变量：

| 设计变量 | 含义 | Kalibr 源码 | 参数化 |
|---|---|---|---|
| $\mathbf R_{i_mb}=\mathbf C(\boldsymbol\varphi_m)$ | body→imu$_m$ 旋转 | `self.q_i_b_Dv` | 旋转向量 $\boldsymbol\varphi_m\in\mathbb R^3$ |
| $\mathbf r_{b,m}\in\mathbb R^3$ | imu$_m$ 原点在 body 系中的位置（杠杆臂） | `self.r_b_Dv` | 欧式三维 |

二者一起组成 body→imu$_m$ 的刚体变换（对照 `getTransformationFromBodyToImu`）：

$$
\mathbf T_{i_mb}
=
\begin{bmatrix}
\mathbf R_{i_mb} & -\mathbf R_{i_mb}\,\mathbf r_{b,m}\\
\mathbf 0^\top & 1
\end{bmatrix},
\qquad
\mathbf p_{i_m}=\mathbf R_{i_mb}(\mathbf p_b-\mathbf r_{b,m}).
$$

即 $\mathbf r_{b,m}$ 是“在 body 系里 imu$_m$ 原点的坐标”，平移分量是 $-\mathbf R_{i_mb}\mathbf r_{b,m}$ 而不是 $\mathbf r_{b,m}$ 本身。这一点在写结果 YAML（`T_i_b`）时必须对齐，否则外参平移会差一个旋转。

### 13.8.2 旋转约定 $\mathbf C(\boldsymbol\varphi)=\exp(-[\boldsymbol\varphi]_\times)$

Ceres 复现采用与 Kalibr `sm::RotationVector` 一致的约定（见 `core/so3.h` 的 `rotationVectorToMatrix`）：

$$
\mathbf C(\boldsymbol\varphi)
=
\mathrm{Exp}_K(\boldsymbol\varphi)
=
\mathbf I-\sin\theta\,[\mathbf a]_\times+(1-\cos\theta)[\mathbf a]_\times^2
=
\exp(-[\boldsymbol\varphi]_\times),
\qquad
\theta=\|\boldsymbol\varphi\|,\ \mathbf a=\boldsymbol\varphi/\theta.
$$

注意这是**负号指数**：$\mathbf C(\boldsymbol\varphi)=\exp(-[\boldsymbol\varphi]_\times)=\exp([\boldsymbol\varphi]_\times)^\top$。整套左 Jacobian（`leftJacobianSO3`）和下面所有推导都建立在这个约定上：

$$
\mathbf J_l(\boldsymbol\varphi)
=
\mathbf I+\frac{1-\cos\theta}{\theta^2}[\boldsymbol\varphi]_\times+\frac{\theta-\sin\theta}{\theta^3}[\boldsymbol\varphi]_\times^2.
$$

**核心扰动引理（本章后续 Jacobian 的唯一来源）。** 设旋转用最小坐标 $\boldsymbol\varphi$ 表示、采用欧式更新 $\boldsymbol\varphi\to\boldsymbol\varphi+\boldsymbol\delta$，$\mathbf v$ 与 $\boldsymbol\varphi$ 无关，则在 $\mathbf C(\boldsymbol\varphi)=\exp(-[\boldsymbol\varphi]_\times)$ 约定下：

$$
\boxed{
\frac{\partial\big(\mathbf C(\boldsymbol\varphi)\,\mathbf v\big)}{\partial\boldsymbol\varphi}
=
\mathbf C(\boldsymbol\varphi)\,[\mathbf v]_\times\,\mathbf J_l(\boldsymbol\varphi).
}
$$

推导：用 BCH 一阶式 $\exp([\mathbf a+\mathrm d\mathbf a]_\times)\approx\exp([\mathbf a]_\times)\exp\big([\mathbf J_r(\mathbf a)\,\mathrm d\mathbf a]_\times\big)$，取 $\mathbf a=-\boldsymbol\varphi$、$\mathrm d\mathbf a=-\boldsymbol\delta$：

$$
\mathbf C(\boldsymbol\varphi+\boldsymbol\delta)
=
\exp\!\big([-\boldsymbol\varphi-\boldsymbol\delta]_\times\big)
\approx
\mathbf C(\boldsymbol\varphi)\exp\!\big(-[\mathbf J_r(-\boldsymbol\varphi)\boldsymbol\delta]_\times\big),
$$

$$
\mathbf C(\boldsymbol\varphi+\boldsymbol\delta)\mathbf v
\approx
\mathbf C(\boldsymbol\varphi)\big(\mathbf I-[\mathbf J_r(-\boldsymbol\varphi)\boldsymbol\delta]_\times\big)\mathbf v
=
\mathbf C(\boldsymbol\varphi)\mathbf v+\mathbf C(\boldsymbol\varphi)[\mathbf v]_\times\mathbf J_r(-\boldsymbol\varphi)\boldsymbol\delta,
$$

再用 $\mathbf J_r(-\boldsymbol\varphi)=\mathbf J_l(\boldsymbol\varphi)$ 即得引理。这一行就是 `gyroscope_residual.cpp` 里 `R_i_b * skew(omega_b) * leftJacobianSO3(r_i_b)` 和 `accelerometer_residual.cpp` 里 `R_i_b * skew(body_specific_force) * leftJacobianSO3(r_i_b)` 的来源——符号为正、带 $\mathbf J_l$，与代码逐项一致。

## 13.9 多 IMU 新增 Jacobian 的完整推导

### 13.9.1 前向预测量（与源码对齐的写法）

记 spline 在查询时刻给出的 body 量：旋转 $\mathbf R_{bw}$、世界系线加速度 $\mathbf a_w$、body 角速度 $\boldsymbol\omega_b$、body 角加速度 $\boldsymbol\alpha_b$。定义

$$
\mathbf h_b=\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)
\quad(\text{body 原点比力}),
\qquad
\mathbf u_{b,m}=\mathbf h_b+\underbrace{\boldsymbol\alpha_b\times\mathbf r_{b,m}+\boldsymbol\omega_b\times(\boldsymbol\omega_b\times\mathbf r_{b,m})}_{\text{杠杆臂输运项 }\boldsymbol\ell_m}.
$$

普通（calibrated）模型的预测量与残差（$\sigma_g,\sigma_a$ 为离散噪声标准差，各向同性白化）：

$$
\hat{\mathbf z}^\omega_m=\mathbf R_{i_mb}\boldsymbol\omega_b+\mathbf b^g_m,
\qquad
\hat{\mathbf z}^a_m=\mathbf R_{i_mb}\mathbf u_{b,m}+\mathbf b^a_m,
$$

$$
\mathbf e^\omega_m=\tfrac1{\sigma_g}(\hat{\mathbf z}^\omega_m-\mathbf z^\omega_m),
\qquad
\mathbf e^a_m=\tfrac1{\sigma_a}(\hat{\mathbf z}^a_m-\mathbf z^a_m).
$$

scale-misalignment 模型在外层包内参（$\mathbf M_a$ 下三角、$\mathbf M_g$ 下三角、$\mathbf A_g$ 满阵、$\mathbf R_{g_mi_m}$ sensing 旋转）：

$$
\hat{\mathbf z}^a_m=\mathbf M_a\big(\mathbf R_{i_mb}\mathbf u_{b,m}\big)+\mathbf b^a_m,
\qquad
\hat{\mathbf z}^\omega_m=\mathbf M_g\big(\mathbf R_{g_mb}\boldsymbol\omega_b\big)+\mathbf A_g\big(\mathbf R_{g_mb}\mathbf u_{b,m}\big)+\mathbf b^g_m,
$$

其中 $\mathbf R_{g_mb}=\mathbf R_{g_mi_m}\mathbf R_{i_mb}$。下面对 $\mathbf r_{b,m}$ 和 $\boldsymbol\varphi_m$ 的新 Jacobian 推导，calibrated 取 $\mathbf M_a=\mathbf I$ 即可。

### 13.9.2 杠杆臂 $\mathbf r_{b,m}$ 的 Jacobian

只有加速度计前向量依赖 $\mathbf r_{b,m}$（角速度与杠杆臂无关，这是刚体上各点角速度相同的物理事实）。先求输运项对杠杆臂的导数：

$$
\frac{\partial(\boldsymbol\alpha_b\times\mathbf r_{b,m})}{\partial\mathbf r_{b,m}}=[\boldsymbol\alpha_b]_\times,
\qquad
\frac{\partial\big(\boldsymbol\omega_b\times(\boldsymbol\omega_b\times\mathbf r_{b,m})\big)}{\partial\mathbf r_{b,m}}=[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times,
$$

所以

$$
\frac{\partial\mathbf u_{b,m}}{\partial\mathbf r_{b,m}}
=
[\boldsymbol\alpha_b]_\times+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times,
\qquad
\boxed{
\frac{\partial\mathbf e^a_m}{\partial\mathbf r_{b,m}}
=
\tfrac1{\sigma_a}\mathbf M_a\mathbf R_{i_mb}\Big([\boldsymbol\alpha_b]_\times+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times\Big).
}
$$

对照 `accelerometer_residual.cpp`：`d_body_d_r_b = skew(alpha_b) + skew(omega_b)*skew(omega_b)`，再左乘 `inv_sigma * M_accel * R_i_b`。陀螺侧 `gyroscope_residual.cpp`（scale-misalignment）的 `d_residual_d_r_b = inv_sigma * A_gyro_accel * R_gyro_b * d_a_d_r_b`——只有 $\mathbf A_g\mathbf a_g$ 分支才把杠杆臂带进陀螺残差，与 13.4 节的稀疏表一致。

### 13.9.3 外参旋转 $\boldsymbol\varphi_m$ 的 Jacobian

直接套 13.8.2 的核心引理。陀螺侧 $\mathbf v=\boldsymbol\omega_b$：

$$
\boxed{
\frac{\partial\mathbf e^\omega_m}{\partial\boldsymbol\varphi_m}
=
\tfrac1{\sigma_g}\mathbf R_{i_mb}[\boldsymbol\omega_b]_\times\mathbf J_l(\boldsymbol\varphi_m)
}
\quad(\text{calibrated}).
$$

加速度侧 $\mathbf v=\mathbf u_{b,m}$：

$$
\boxed{
\frac{\partial\mathbf e^a_m}{\partial\boldsymbol\varphi_m}
=
\tfrac1{\sigma_a}\mathbf M_a\mathbf R_{i_mb}[\mathbf u_{b,m}]_\times\mathbf J_l(\boldsymbol\varphi_m).
}
$$

scale-misalignment 陀螺把 $\boldsymbol\omega_g=\mathbf R_{g_mb}\boldsymbol\omega_b$ 和 $\mathbf a_g=\mathbf R_{g_mb}\mathbf u_{b,m}$ 都过 $\mathbf R_{i_mb}$，因此

$$
\frac{\partial\mathbf e^\omega_m}{\partial\boldsymbol\varphi_m}
=
\tfrac1{\sigma_g}\Big(
\mathbf M_g\mathbf R_{g_mi_m}\mathbf R_{i_mb}[\boldsymbol\omega_b]_\times\mathbf J_l(\boldsymbol\varphi_m)
+
\mathbf A_g\mathbf R_{g_mi_m}\mathbf R_{i_mb}[\mathbf u_{b,m}]_\times\mathbf J_l(\boldsymbol\varphi_m)
\Big),
$$

对照 `gyroscope_residual.cpp` 的 `d_residual_d_r_i_b`（两项分别带 $\mathbf M_g$ 与 $\mathbf A_g$）。sensing 旋转 $\mathbf R_{g_mi_m}=\mathbf C(\boldsymbol\varphi_{g})$ 的 Jacobian 同样套引理，但 $\mathbf v$ 换成已经过 $\mathbf R_{i_mb}$ 的量：$\mathbf R_{g_mi_m}[\mathbf R_{i_mb}\boldsymbol\omega_b]_\times\mathbf J_l(\boldsymbol\varphi_g)$，与代码 `d_residual_d_r_gyro_i` 一致。

### 13.9.4 block 布局与共享

实现上把 $(\mathbf r_{b,m},\boldsymbol\varphi_m)$ 合并成一个 6 维 parameter block `[r_b(3); r_i_b(3)]`：

| residual | 该 6 维 block 的非零列 | 原因 |
|---|---|---|
| 普通/扩展 gyro | 仅 $\boldsymbol\varphi_m$（列 3-5） | 角速度与杠杆臂无关；只有 $\mathbf A_g$ 分支才碰列 0-2 |
| 普通/扩展 accel | $\mathbf r_{b,m}$（列 0-2）与 $\boldsymbol\varphi_m$（列 3-5）都非零 | 比力依赖杠杆臂和外参旋转 |

reference IMU 的这个 block 存在但不 active，装配时不进 Hessian——这正是 13.6 节强调的“公式非零 ≠ 实现 active”。

## 13.10 IMU-IMU 初始化：冷启动的关键先验

13.9 的 Jacobian 保证“给定初值后能正确下降”，但不解决“初值从哪来”。非参考 IMU 的 $\boldsymbol\varphi_m$ 若从单位阵起步、$\mathbf r_{b,m}$ 从零起步，再叠加相机/time 一起放开，冷启动很容易掉进局部极小（实验记录里 4-IMU joint 冷启动 reprojection 一度到 $54.76$ px）。Kalibr 用 `IccImu.findOrientationPrior(referenceImu)` 给每个非参考 IMU 算两个先验，再进 bundle：

**第一阶段——时间对齐。** 先用 reference IMU 的陀螺拟合一条自由 body 角速度 spline $\boldsymbol\omega_{\mathrm{ref}}(t)$（同时估一个常值 ref gyro bias）。然后对两路陀螺模长做互相关：

$$
\boxed{
\Delta t^i_m=\arg\max_{\tau}\ \mathrm{xcorr}\big(\|\boldsymbol\omega_{\mathrm{ref}}(t+\tau)\|,\ \|\mathbf z^\omega_m(t)\|\big),
}
$$

即 **$\tau$ 加在参考 IMU 的 spline 上，不是加在被对齐 IMU 的测量上**。

> **符号陷阱：Kalibr 两条互相关路径的约定是相反的，不能互相照抄。**
>
> | 路径 | 源码 | 位移写法 |
> |---|---|---|
> | cam-IMU（`findTimeshiftCameraImuPrior`） | `IccSensors.py:277-283` | `shift = -discrete_shift*dT`（**有**负号） |
> | IMU-IMU（`findOrientationPrior`） | `IccSensors.py:920-925` | `shift =  discrete_shift*dT`（**无**负号） |
>
> 两处的 `np.correlate(...)` 与 `argmax - (len-1)` 完全同构，唯一差别就是这个负号。
> IMU-IMU 路径取的滞后 $L$ 满足 $\boldsymbol\omega_{\mathrm{ref}}[n+L]\approx\mathbf z^\omega_m[n]$，
> 所以 $\Delta t^i_m=L\cdot dT$ 直接就是"往参考侧前移"的量，与上式一致。
>
> **自洽性自查**：下面的查询式是 $t_{\text{query}}=t^{\mathrm{imu}}_{m,k}+\Delta t^i_m$，
> 代入得 $\boldsymbol\omega_{\mathrm{ref}}(t_{m,k}+\Delta t^i_m)\approx\mathbf z^\omega_m(t_{m,k})$——
> 和互相关式的方向一致。若把 cam-IMU 那条的负号套过来，会得到反号的 $\Delta t$，
> 即初值偏离真值 $2\Delta t$，多 IMU 初始化质量明显变差（是否收敛还取决于真实延迟大小、
> 搜索范围、运动激励和其它先验）。

离散峰值再用 `scipy.optimize.fmin` 在连续域细化（仅当 `estimateTimedelay` 且非 reference 时）。这个 $\Delta t^i_m$ **不是** bundle 设计变量，而是查询共享 spline 时的常量偏移：$t_{\text{query}}=t^{\mathrm{imu}}_{m,k}+\Delta t^i_m$（对照 `addAccelerometer/GyroscopeErrorTerms` 里 `tk = im.stamp.toSec() + self.timeOffset`）。

**第二阶段——相对旋转先验。** 用对齐后的时间，建陀螺残差 $\mathbf z^\omega_m\approx\mathbf C(\boldsymbol\varphi_m)\boldsymbol\omega_{\mathrm{ref}}(t)+\mathbf b$，优化得到 $\boldsymbol\varphi_m$ 的初值 `q_i_b_prior`。

**杠杆臂先验** 取 $\mathbf r_{b,m}=\mathbf 0$：纯陀螺信息看不到平移杠杆臂，它只能在 joint 阶段由加速度计的输运项约束，因此天然弱可观、毫米级漂移属正常。

> **Ceres 现状：这套初始化已经进入独立生产路径。** `estimateImuChainPairPrior(...)` 复现了“先时间、后旋转”的两阶段结构；`kalibr_style_multi_imu_initializer` 再把每个 camera 对 IMU0 的 shift、cam0 的旋转/重力/参考 gyro bias，以及每个非参考 IMU 的 offset/相对旋转按 Kalibr 的顺序组装起来。C++ 实现为了支持独立时间原点，把搜索扩展为归一化全范围粗到细互相关，并要求候选至少保留较短序列 `50%` 的共同运动。非参考 IMU 的全局边界峰不能解释为零，生产路径会明确拒绝继续。

## 13.11 鲁棒核与 IRLS：Kalibr M-estimator 的精确语义

历史冷启动诊断曾把鲁棒核线性化列为几何初值之外的另一项嫌疑；后续对照证明，对本章使用的 Cauchy/Huber，它并不是当前差异根因。要理解这个排除结论，先看 Kalibr 在 `corner_file` 路径（产线角点输入）怎样给不同 residual 指定 M-estimator：

| residual | M-estimator | 源码 | 形参 |
|---|---|---|---|
| 相机重投影 | `CauchyMEstimator` | `blakeZisserCam=10` | $\sigma^2=10$ |
| calibrated IMU gyro/accel | `CauchyMEstimator` | `huberGyro/huberAccel=10` | $\sigma^2=10$ |
| scale-misalignment IMU gyro/accel | `HuberMEstimator` | 同上 | $k=10$ |

**权重定义。** 记原始平方马氏误差 $s=\mathbf e^\top\mathbf R^{-1}\mathbf e$（`getRawSquaredError`）。`MEstimatorPolicies.cpp` 给出的 `getWeight(s)` 正是鲁棒损失 $\rho(s)$ 的一阶导 $\rho'(s)$：

$$
w_{\text{Cauchy}}(s)=\frac{1}{1+s/\sigma^2},
\qquad
w_{\text{Huber}}(s)=\begin{cases}1,&s<k^2\\[2pt]k/\sqrt{s},&s\ge k^2\end{cases},
\qquad
w_{\text{None}}(s)=1.
$$

**装配语义（IRLS）。** `ErrorTerm.hpp` 用的是“开方权重”方案：

$$
\sqrt{w}=\sqrt{\rho'(s)},
\quad
\bar{\mathbf e}=\sqrt{w}\,\mathbf R^{-1/2}\mathbf e,
\quad
\bar{\mathbf J}=\sqrt{w}\,\mathbf R^{-1/2}\mathbf J,
\quad
\mathbf H=\sum\bar{\mathbf J}^\top\bar{\mathbf J},
\quad
\mathbf b=-\sum\bar{\mathbf J}^\top\bar{\mathbf e}.
$$

关键点：**$\rho''$ 完全不参与**。Kalibr 只用 $\rho'$ 对残差和 Jacobian 同开方缩放，做标准 Gauss-Newton，即 iteratively reweighted least squares。

**与 Ceres 标准 loss 的差别：对 Cauchy 和 Huber 而言没有差别。** Ceres 标准 loss 走的也是纯 IRLS，不做 Triggs 修正。关键在 `ceres::internal::Corrector` 构造函数开头的短路：

```cpp
if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
  residual_scaling_ = sqrt_rho1_;
  alpha_sq_norm_ = 0.0;
  return;
}
```

Triggs 修正只在 $\rho''>0$ 时才启用。Cauchy 和 Huber 都是凹函数，$\rho''\le0$ 恒成立——Cauchy 严格小于零，Huber 在阈值内 $\rho_0(s)=s$ 是仿射的、$\rho''=0$，阈值外才严格小于零——两种情形都落进 `rho[2] <= 0.0` 这条短路，于是它每次都命中，Ceres 实际做的就是"$\sqrt{\rho'}$ 同时缩放残差和 Jacobian"——**和 Kalibr 的 IRLS 逐字相同**。所以把 $\rho''$ 显式置零并不改变任何数值：

$$
\rho'(s)=w(s),\qquad \rho''(s)=0 \quad(\text{与真实 }\rho''\le0\text{ 等效}),
$$

它表达的是意图，不是修正。真正需要认真对待的是 $\rho_0$：Ceres 用 $\rho_0$ 算 cost 和 trust-region 的 actual reduction，用 $\rho_1$ 算模型的 predicted reduction，两者必须自洽，即 $\rho_0'=\rho_1$。取 $\rho_0(s)=\int_0^s w(u)\,\mathrm du$ 就自动满足，Cauchy 是 $c^2\ln(1+s/c^2)$，Huber 是 $s$ / $2k\sqrt s-k^2$——这恰好又与 Ceres 内置的 `CauchyLoss`、`HuberLoss` 解析相同。

结论因此是反直觉的：**在 Ceres 当前的求解路径上，自定义的 Kalibr-style loss 与 `ceres::CauchyLoss(\sqrt{\text{width}})`、`ceres::HuberLoss(\text{width})` 数值等价。** `2025_03_14_00_10_18` 热启动 `pbg,pbegt` 口径下三者实测 reprojection 均值同为 `0.309145562193544 px`。

这个等价性依赖两件本身并不由我们控制的事，引用时要连着说清楚：一是 $\rho_0,\rho_1$ 解析相同——这一条 `tests/test_math.cpp` 已经逐点比对锁住；二是 `Corrector` 对 $\rho''\le0$ 短路——这是 Ceres 的实现细节，我们只在本机的 Ceres 2.1 上验证过，构建系统也没有把 Ceres 版本钉死。若将来换用一个**凸**的 $\rho$，或 Ceres 改掉这条短路，$\rho_2$ 的取值就会重新变得重要。保留自定义 loss 的价值也正在这里：显式表达 IRLS 语义，以及为接入 Ceres 没有内置的权重函数（Geman-McClure、Blake-Zisserman）留出位置。宽度换算见 `MEstimatorPolicies.cpp:46`，Kalibr 的 `_sigma2` 已是平方误差的分母，对应 `CauchyLoss` 的 $a=\sqrt{\sigma^2}$。

## 13.12 与 Ceres 实现的逐项对照与证据边界

**已验证一致（前向 + Jacobian）。** 在 $\mathbf C=\exp(-[\cdot]_\times)$ 约定 + 欧式更新下，下面三项与 `gyroscope_residual.cpp` / `accelerometer_residual.cpp` 逐项吻合，并已被中心差分单测覆盖：

| 量 | 本章公式 | 源码 |
|---|---|---|
| $\partial\mathbf e^\omega/\partial\boldsymbol\varphi_m$ | $\tfrac1{\sigma_g}\mathbf R_{i_mb}[\boldsymbol\omega_b]_\times\mathbf J_l$ | `R_i_b*skew(omega_b)*leftJacobianSO3` |
| $\partial\mathbf e^a/\partial\mathbf r_{b,m}$ | $\tfrac1{\sigma_a}\mathbf M_a\mathbf R_{i_mb}([\boldsymbol\alpha_b]_\times+[\boldsymbol\omega_b]_\times^2)$ | `M_accel*R_i_b*(skew(alpha)+skew(omega)^2)` |
| $\partial\mathbf e^a/\partial\boldsymbol\varphi_m$ | $\tfrac1{\sigma_a}\mathbf M_a\mathbf R_{i_mb}[\mathbf u_{b,m}]_\times\mathbf J_l$ | `M_accel*R_i_b*skew(body_specific_force)*leftJacobianSO3` |

**当前实现状态与仍需保留的证据边界。**

| 项目 | 当前状态 | 边界 / 后续关注点 |
|---|---|---|
| 非参考 IMU 冷启动初值 | 已实现 13.10 的 gyro-norm 互相关 time offset + 相对旋转先验，生产默认走一次 Kalibr-style 初始化和一次 joint solve | 真实数据分别覆盖 1cam+4imu 和 2cam+1imu；完整 Mcam+Nimu 组合目前依靠合成 2cam+2imu 端到端回归 |
| 鲁棒核线性化 | 无缺口：对 Cauchy/Huber，标准 Ceres loss 与 Kalibr IRLS 等价（13.11） | 保持 13.11 的 Kalibr-style M-estimator（$\rho_0=\int_0^s w,\ \rho'=w,\ \rho''=0$），按 model 分配 Cauchy/Huber；若将来引入 Ceres 无内置的权重函数，此处才有实质工作 |
| per-IMU time offset | 初始化器估计 applied offset，先用它创建对齐后的 spline 时间域，再接入各路 IMU residual 查询；默认按 Kalibr 语义固定 | 只有显式 `--optimize-imu-time-offsets` 才作为优化变量；两种模式都必须保持同一 applied-offset 符号约定 |
| 杠杆臂可观性 | 弱可观，joint 平移 tail 仍大于 single | 这不是初始化缺失；报告时应按 per-IMU 输出，不平均掩盖最差项 |

这张表是从“推导”过渡到“代码 + 实验”的接口：13.8-13.9 给出 Jacobian，13.10-13.11 给出冷启动和鲁棒核语义，当前 Ceres 已经把三者连成一条独立 joint 路径。未收口的重点是更广的真实拓扑证据和杆臂弱可观性，不是再补一套初始化代码。

## 13.13 本章小结

多相机 / 多 IMU 不改变第 4-10 章的局部物理模型。Camera residual 仍然是 measurement minus projection；gyro 和 accel residual 仍然是 prediction minus measurement；扩展 IMU 的 $\mathbf M_a$、$\mathbf M_g$、$\mathbf A_g$ 和 sensing-frame rotation 仍然按第 10 章的链式法则求 Jacobian。

单相机多 IMU 真正新增的，是 13.8-13.12 这几节：非参考 IMU 的外参旋转 $\mathbf R_{i_mb}$ 和杠杆臂 $\mathbf r_{b,m}$ 的精确 Jacobian（单 IMU 章节没推过）、冷启动初始化先验、以及 Kalibr IRLS 鲁棒核语义。

变化发生在因子图层：

1. residual 带上传感器 index；
2. camera 和 IMU 的私有变量按 index 分开；
3. pose spline 和 gravity 是跨传感器共享变量；
4. 每个 residual 只连接 forward expression 里真正出现的 block；
5. 优化器解一个全局增量，再按每个 block 的 manifold 回写。

因此排查多传感器 Jacobian 时，不要先怀疑第 4-10 章的单残差公式变了；先检查这个 residual 是否连到了正确的 sensor index、正确的 active spline window，以及正确的 reference body frame。
