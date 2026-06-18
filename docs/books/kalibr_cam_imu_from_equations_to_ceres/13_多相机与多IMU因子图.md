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

## 13.8 本章小结

多相机 / 多 IMU 不改变第 4-10 章的局部物理模型。Camera residual 仍然是 measurement minus projection；gyro 和 accel residual 仍然是 prediction minus measurement；扩展 IMU 的 $\mathbf M_a$、$\mathbf M_g$、$\mathbf A_g$ 和 sensing-frame rotation 仍然按第 10 章的链式法则求 Jacobian。

变化发生在因子图层：

1. residual 带上传感器 index；
2. camera 和 IMU 的私有变量按 index 分开；
3. pose spline 和 gravity 是跨传感器共享变量；
4. 每个 residual 只连接 forward expression 里真正出现的 block；
5. 优化器解一个全局增量，再按每个 block 的 manifold 回写。

因此排查多传感器 Jacobian 时，不要先怀疑第 4-10 章的单残差公式变了；先检查这个 residual 是否连到了正确的 sensor index、正确的 active spline window，以及正确的 reference body frame。
