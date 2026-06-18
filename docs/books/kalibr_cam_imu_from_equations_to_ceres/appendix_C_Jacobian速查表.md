# 附录 C：Jacobian 速查表

这份附录为一个具体场景服务：你正在调试优化器——也许在对照 `JacobianContainer` 里的某个 block，也许在写附录 B 那样的 finite-difference check——需要立刻知道“这个 residual 对这个变量块的 Jacobian 应该长什么样”，而不想翻回第 4-10 章重走一遍推导。

所以这里没有新的推导，只有结论和出处。每一节对应一类 residual：先用一两句话回忆这个 residual 在比较什么，然后给出 Jacobian 总表，表后标注每组结果是在哪一节推出来的。如果某行让你觉得突兀，跟着出处回正文，那里有完整的链式法则。多相机 / 多 IMU 时，单个 residual 的局部公式仍然查 C.4-C.10；传感器索引、非零 block 选择和变量更新规则查 C.12。

## C.1 表里的 Jacobian 是哪一层

全书的权重机制分三层（6.11.2 节把这三层在 gyro 上点过名），这张表只列最里面那层——**几何 Jacobian**：

$$
\mathbf J_i
=
\frac{\partial\mathbf e}{\partial\delta\boldsymbol\theta_i}.
$$

也就是纯前向模型对变量块的导数。它默认：

1. 使用 Kalibr expression tangent（第 0 章 0.8 节的 $\boxplus_K$ 约定）。
2. 不包含 information matrix whitening。
3. 不包含 robust weight。
4. 不包含 LM damping。
5. 不包含 design-variable scaling——cam-imu 主路径里它恒为 $1$（3.11 节）。

真正进入线性系统前还差最后一步加权，规则在第 3 章和 11.7 节：

$$
\bar{\mathbf e}
=
\sqrt{w}\,\mathbf L^\top\mathbf e,
\qquad
\bar{\mathbf J}_i
=
s_i\sqrt{w}\,\mathbf L^\top\mathbf J_i.
$$

如果你在做 finite-difference check，应该直接对几何 Jacobian 做数值比对；如果你在读优化器内部的矩阵，要先把 $\sqrt{w}\,\mathbf L^\top$ 除回去才能和这张表对上。

## C.2 怎么读这张表

表里的每个 Jacobian 都是同一种三段结构：

$$
\underbrace{\text{residual 对中间量的敏感度}}_{\text{左因子}}
\times
\underbrace{\text{中间节点的局部 Jacobian}}_{\text{中间因子}}
\times
\underbrace{\text{变量块接口}}_{\text{右因子}}.
$$

例如 gyro residual 对 pose 控制点的 $\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}$：左因子 $\mathbf R_{ib}$ 来自 frame-change 节点，右因子 $\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}$ 是第 5 章的 spline 接口。查表建议按四步走：

1. 按 residual 类型找到对应的节。
2. 在表里找变量块对应的行。
3. 检查稀疏性：所有 spline 控制点（pose 和 bias）只在当前时间的 active window 内非零，窗口外整块为零。
4. 检查符号来源。camera residual 的整体负号来自 measurement-minus-prediction 的 residual 方向（0.14 节、3.6 节）；所有 $[\cdot]_\times$ 前的正号来自 Kalibr 旋转扰动约定 $\mathbf R^+=\mathrm{Exp}(-\delta\boldsymbol\phi_K)\mathbf R$（0.8-0.9 节）。这两类符号来源不同，不要混在一起排查。

## C.3 公共符号

下面的记号全书通用，正式定义见标注的章节。

Camera residual（第 4 章）：

$$
\mathbf e^\pi
=
\mathbf y-\hat{\mathbf y},
\qquad
\hat{\mathbf y}
=
\boldsymbol\pi(\mathbf p_c;\boldsymbol\eta).
$$

residual 对 world-to-camera transform 的上游敏感度（4.7 节）：

$$
\mathbf A_T
\triangleq
\frac{\partial\mathbf e^\pi}
{\partial\delta\boldsymbol\xi_{T_{c_nw},K}}
=
-
\mathbf J_{\pi,\tilde{\mathbf p}_c}
\mathrm{boxMinus}(\tilde{\mathbf p}_c)
\in\mathbb R^{2\times6}.
$$

它已经吃进了 projection Jacobian、residual 方向负号和 transform action；后面 camera 表里的每一行都是 $\mathbf A_T$ 再向左乘搬运矩阵。

Camera transform 链（第 2 章、8.5 节）：

$$
\mathbf T_{c_nw}
=
\mathbf T_{c_nb}\mathbf T_{bw},
\qquad
\mathbf T_{bw}=\mathbf T_{wb}^{-1}.
$$

Pose spline（第 5 章）：

$$
\mathbf T_{wb}(t)=F(\mathbf s(t)),
\qquad
\mathbf J_{\mathbf T,\mathbf c_j}
=
\frac{\partial\delta\boldsymbol\xi_{T_{wb},K}}
{\partial\delta\mathbf c_j}.
$$

Accelerometer 中间量（7.4 节；lever-arm 两项的刚体运动学推导在 7.4.2 节）：

$$
\mathbf u_b
=
\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)
+
\boldsymbol\alpha_b\times\mathbf r_b
+
\boldsymbol\omega_b\times
(\boldsymbol\omega_b\times\mathbf r_b).
$$

Lever-arm 矩阵（7.7 节）：

$$
\mathbf A_r
\triangleq
[\boldsymbol\alpha_b]_\times
+
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times.
$$

Bias spline（6.4.3 节、9.2 节）：

$$
\mathbf b_g(t)=\sum_\ell\mu_\ell^{(0)}(t)\mathbf d^g_{j+\ell},
\qquad
\mathbf b_a(t)=\sum_\ell\nu_\ell^{(0)}(t)\mathbf d^a_{j+\ell}.
$$

若某控制点不在当前 active window 内，对应 Jacobian block 为零。

## C.4 Camera reprojection residual

比较的是标定板角点的预测像素和检测像素。前向链路：world 角点 $\to$ body $\to$ camera $\to$ 像素。

$$
\mathbf e^\pi_{n,k,\ell}
=
\mathbf y_{n,k,\ell}
-
\hat{\mathbf y}_{n,k,\ell}.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| complete camera 外参 $\mathbf T_{c_nb}$ | $\mathbf A_T$ | $2\times6$ |
| camera chain link $\mathbf T_m$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf P_m)$ | $2\times6$ |
| world-to-body pose $\mathbf T_{bw}$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf T_{c_nb})$ | $2\times6$ |
| body-to-world pose $\mathbf T_{wb}$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf T_{c_nb})[-\mathrm{boxTimes}(\mathbf T_{bw})]$ | $2\times6$ |
| active pose control point $\mathbf c_j$ | $\mathbf J_{\mathbf e^\pi,\mathbf T_{wb}}\mathbf J_{\mathbf T,\mathbf c_j}$ | $2\times6$ |
| camera time shift $\Delta t_n$ | $-\mathbf J_{\pi,\mathbf p_c}\dot{\mathbf p}_{c_n}$ | $2\times1$ |
| camera intrinsics $\boldsymbol\eta_n$ | $-\mathbf J_{\hat{\mathbf y},\boldsymbol\eta_n}$ | $2\times d_\eta$ |
| distortion coefficients $\boldsymbol\kappa_n$ | $-\mathbf J_{\hat{\mathbf y},\boldsymbol\kappa_n}$ | $2\times d_\kappa$ |

其中外侧链 $\mathbf P_m=\mathbf T_n\mathbf T_{n-1}\cdots\mathbf T_{m+1}$（8.6.1 节）。

推导出处：外参与 pose 链 4.7-4.8 节、8.6 节；time shift 8.3.1、8.4 节；intrinsics 与 distortion 4.5.1-4.5.2 节；pose spline 接口 $\mathbf J_{\mathbf T,\mathbf c_j}$ 5.8-5.9 节。

Camera residual 对 gyro/accel bias、IMU 内参、IMU lever arm、gravity 的直接 Jacobian 为零。

## C.5 Ordinary gyro residual

比较的是轨迹角速度旋转到 IMU frame 加 bias 后的预测读数和陀螺仪读数。

$$
\mathbf e^\omega_k
=
\mathbf R_{ib}\boldsymbol\omega_b(t_k)
+
\mathbf b_g(t_k)
-
\mathbf z^\omega_k,
\qquad
\mathbf y_\omega\triangleq\mathbf R_{ib}\boldsymbol\omega_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}$ | $3\times6$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $[\mathbf y_\omega]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf 0$ | $3\times3$ |
| gyro bias control $\mathbf d^g_{j+\ell}$ | $\mu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| optional IMU time offset $\Delta t_i$ | $\mathbf R_{ib}\dot{\boldsymbol\omega}_b(t_k)+\dot{\mathbf b}_g(t_k)$ | $3\times1$ |

推导出处：母式与变量块 6.6-6.9 节；$\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}$ 闭式 5.13.1 节；time offset 6.11.3、8.3.2 节；lever arm 为零的原因 6.2、8.8.1 节（刚体上所有点角速度相同）。

Ordinary gyro residual 对 camera 参数、accel bias、gravity、accelerometer scale/misalignment 的直接 Jacobian 为零。

## C.6 Ordinary accelerometer residual

比较的是 IMU 原点的预测 specific force（含 gravity 和 lever-arm 项）和加速度计读数。

$$
\mathbf e^a_k
=
\mathbf R_{ib}\mathbf u_b(t_k)
+
\mathbf b_a(t_k)
-
\mathbf z^a_k,
\qquad
\mathbf y_a\triangleq\mathbf R_{ib}\mathbf u_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| gravity $\mathbf g_w$ | $-\mathbf R_{ib}\mathbf R_{bw}$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $[\mathbf y_a]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| accel bias control $\mathbf d^a_{j+\ell}$ | $\nu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| optional IMU time offset $\Delta t_i$ | $\mathbf R_{ib}\dot{\mathbf u}_b(t_k)+\dot{\mathbf b}_a(t_k)$ | $3\times1$ |

其中 pose 分支内部（7.8.3 节）来自：

$$
\delta\mathbf u_b
=
\delta\mathbf h_b+\delta\boldsymbol\ell_b.
$$

前两项是 $\delta\mathbf h_b$ 的 specific-force 分支，后两项是 $\delta\boldsymbol\ell_b$ 的 lever-arm 分支：

$$
\mathbf J_{\mathbf u_b,\mathbf c_j}
=
\mathbf R_{bw}\mathbf J_{\mathbf a_w,\mathbf c_j}
+
[\mathbf h_b]_\times
\mathbf J_{\mathbf R_{bw},\mathbf c_j}^{\mathrm{rot}}
+
\mathbf A_\alpha
\mathbf J_{\boldsymbol\alpha_b^K,\mathbf c_j}
+
\mathbf A_\omega
\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}.
$$

推导出处：母式与变量块 7.6-7.12 节；lever-arm 局部矩阵 $\mathbf A_\alpha,\mathbf A_\omega,\mathbf A_r$ 7.7 节；gravity 负号来自 $\mathbf a_w-\mathbf g_w$（7.9 节）；time offset 7.13、8.3.3 节。

Ordinary accelerometer residual 对 camera 参数、gyro bias、gyro scale/misalignment 的直接 Jacobian 为零。

## C.7 Accelerometer scale/misalignment residual

普通 accel 模型外面包一层 scale/misalignment 矩阵。母式（10.3 节）是：

$$
\delta\mathbf e^a
=
\delta\mathbf M_{\mathrm{acc}}\,\mathbf x_i
+
\mathbf M_{\mathrm{acc}}\,\delta\mathbf x_i
+
\delta\mathbf b_a,
\qquad
\mathbf x_i=\mathbf R_{ib}\mathbf u_b.
$$

所以旧分支统一左乘 $\mathbf M_{\mathrm{acc}}$，bias 分支不变，另加一条 matrix 分支。

$$
\mathbf e^a_k
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf u_b
+
\mathbf b_a
-
\mathbf z^a.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| gravity $\mathbf g_w$ | $-\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf R_{bw}$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_{\mathrm{acc}}[\mathbf x_i]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| accel bias control $\mathbf d^a_{j+\ell}$ | $\nu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| accel matrix $\operatorname{vec}(\mathbf M_{\mathrm{acc}})$ | $[x_{i,1}\mathbf I_3\ x_{i,2}\mathbf I_3\ x_{i,3}\mathbf I_3]$ | $3\times9$ before mask |

实际 matrix 列数由 `MatrixBasicDv` 的 active mask 决定（lower-triangular mask 时少于 9 列，10.2 节）。

推导出处：10.2-10.3 节。

## C.8 Extended gyro residual

扩展 gyro 同时引入 gyro sensing frame 旋转 $\mathbf R_{gi}$、scale/misalignment $\mathbf M_g$ 和 acceleration sensitivity $\mathbf A_g$。母式（10.4 节，6.13 节预告）是：

$$
\delta\mathbf e^\omega
=
\delta\mathbf M_g\,\boldsymbol\omega_g
+
\mathbf M_g\,\delta\boldsymbol\omega_g
+
\delta\mathbf A_g\,\mathbf a_g
+
\mathbf A_g\,\delta\mathbf a_g
+
\delta\mathbf b_g.
$$

$$
\mathbf e^\omega_k
=
\mathbf M_g\boldsymbol\omega_g
+
\mathbf A_g\mathbf a_g
+
\mathbf b_g
-
\mathbf z^\omega,
\qquad
\boldsymbol\omega_g=\mathbf R_{gi}\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_g=\mathbf R_{gi}\mathbf R_{ib}\mathbf u_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf M_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}+\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_g\mathbf R_{gi}[\mathbf R_{ib}\boldsymbol\omega_b]_\times+\mathbf A_g\mathbf R_{gi}[\mathbf R_{ib}\mathbf u_b]_\times$ | $3\times3$ |
| gyro sensing rotation $\mathbf R_{gi}$ | $\mathbf M_g[\boldsymbol\omega_g]_\times+\mathbf A_g[\mathbf a_g]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| gyro bias control $\mathbf d^g_{j+\ell}$ | $\mu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| gyro matrix $\operatorname{vec}(\mathbf M_g)$ | $[\omega_{g,1}\mathbf I_3\ \omega_{g,2}\mathbf I_3\ \omega_{g,3}\mathbf I_3]$ | $3\times9$ before mask |
| accel sensitivity $\operatorname{vec}(\mathbf A_g)$ | $[a_{g,1}\mathbf I_3\ a_{g,2}\mathbf I_3\ a_{g,3}\mathbf I_3]$ | $3\times9$ |

两条退化检查：$\mathbf M_g=\mathbf I$、$\mathbf R_{gi}=\mathbf I$、$\mathbf A_g=\mathbf 0$ 时外参旋转行退回 $[\mathbf R_{ib}\boldsymbol\omega_b]_\times$（10.4.3 节）；lever arm 行只来自 acceleration sensitivity 分支——普通 gyro 对 $\mathbf r_b$ 仍是零（10.4.4 节）。

推导出处：10.4 节全部小节。

## C.9 Accelerometer size-effect residual

三根 accelerometer sensing axis 各有自己的 lever arm。

$$
\mathbf e^a
=
\mathbf M_{\mathrm{acc}}
\left[
\mathbf R_{ib}\mathbf h_b
+
\sum_{j\in\{x,y,z\}}
\mathbf I_j\mathbf R_{ib}\boldsymbol\ell_j
\right]
+
\mathbf b_a
-
\mathbf z^a,
$$

其中：

$$
\mathbf h_b=\mathbf R_{bw}(\mathbf a_w-\mathbf g_w),
\qquad
\mathbf r_j^b=\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_j^i,
\qquad
\boldsymbol\ell_j
=
\boldsymbol\alpha_b\times\mathbf r_j^b
+
\boldsymbol\omega_b\times(\boldsymbol\omega_b\times\mathbf r_j^b).
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| axis offset $\mathbf r_j^i$ | $\mathbf M_{\mathrm{acc}}\mathbf I_j\mathbf R_{ib}\mathbf A_r\mathbf R_{ib}^{-1}$ | $3\times3$ |
| base lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_{\mathrm{acc}}\left([\mathbf R_{ib}\mathbf h_b]_\times+\sum_j\mathbf I_j\left([\mathbf R_{ib}\boldsymbol\ell_j]_\times-\mathbf R_{ib}\mathbf A_r\mathbf R_{ib}^{-1}[\mathbf r_j^i]_\times\right)\right)$ | $3\times3$ |

其他 pose、gravity、bias、matrix 分支沿用 C.7，只是把 $\mathbf u_b$ 换成 bracket 内部表达。注意 $\mathbf R_{ib}$ 行的间接项：size-effect 下 $\mathbf R_{ib}$ 同时通过“旋到 IMU frame”和“$\mathbf R_{ib}^{-1}\mathbf r_j^i$ 改变 body-frame lever arm”两条链路进入。

推导出处：10.5 节；源码默认 `rx_i_Dv` inactive、`ry_i_Dv` / `rz_i_Dv` active（10.5.1 节）。

## C.10 Bias motion prior 与 pose motion prior

这两个不是普通的逐测量 residual，而是 spline 导数的二次型（第 9 章）。Kalibr 不显式构造 residual/Jacobian，而是直接装配：

$$
E_{b_g}
=
\mathbf d_g^\top\mathbf Q_g\mathbf d_g,
\qquad
E_{b_a}
=
\mathbf d_a^\top\mathbf Q_a\mathbf d_a,
\qquad
E_{\mathrm{pose}}
=
\mathbf c_s^\top\mathbf Q_s\mathbf c_s,
$$

$$
\boxed{
\mathbf H\mathrel{+}=\mathbf Q,
\qquad
\mathbf b_{\mathrm{rhs}}\mathrel{-}=\mathbf Q\mathbf c.
}
$$

如果一定要用 residual/Jacobian 语言（例如做 finite-difference check），取任意平方根分解：

$$
\mathbf Q=\mathbf S^\top\mathbf S,
\qquad
\mathbf e=\mathbf S\mathbf c,
\qquad
\mathbf J=\mathbf S.
$$

推导出处：$\mathbf Q$ 的导数积分构造 9.4 节；bias prior 9.5 节；pose prior 9.6 节；为什么源码 `evaluateJacobiansImplementation` 抛异常 9.8、11.8 节。

## C.11 零 block 总览

排查“这个 block 为什么是零/为什么不是零”时先查这张表。

| residual | 典型零 Jacobian |
|---|---|
| camera reprojection | gyro/accel bias、IMU scale/misalignment、IMU lever arm、gravity |
| ordinary gyro | camera intrinsics/extrinsics/time shift、accel bias、gravity、lever arm |
| ordinary accelerometer | camera intrinsics/extrinsics/time shift、gyro bias、gyro scale/misalignment |
| extended gyro | camera 参数、accel bias；但 lever arm 和 gravity 经 $\mathbf A_g\mathbf a_g$ 分支变为非零 |
| bias motion prior | pose、camera、IMU extrinsics、gravity、measurement noise 参数 |
| pose motion prior | camera intrinsics、bias、IMU intrinsics、gravity |

“零”表示该 residual 的 forward expression 不直接依赖这个变量。通过优化耦合，一个变量当然仍可能间接影响其他 residual 的最优解。

## C.12 多相机 / 多 IMU block 选择规则

第 13 章的核心结论是：多传感器不改变 C.4-C.10 的局部 Jacobian 公式，改变的是 residual index 和变量 block 归属。调试时先按 residual 的 sensor index 找私有变量，再按查询时刻找共享 spline window。

### C.12.1 Residual 连接哪些 block

| residual | 非零 block | 典型零 block |
|---|---|---|
| camera $n$ reprojection | active pose controls、camera $n$ 的 $\Delta t_n$、intrinsics、distortion、完整 $\mathbf T_{c_nb}$ 或通往 $n$ 的 camera-chain links | 其他 camera 的私有 intrinsics/time shift、所有 IMU bias、IMU 内参、IMU lever arm、gravity |
| ordinary gyro, IMU $m$ | active pose controls、$\mathbf R_{i_mb}$、gyro bias spline $\mathbf b^g_m$、optional IMU time offset $\Delta t^i_m$ | camera 参数、其他 IMU 的 bias/内参、gravity、$\mathbf r_{b,m}$ |
| ordinary accel, IMU $m$ | active pose controls、gravity、$\mathbf R_{i_mb}$、$\mathbf r_{b,m}$、accel bias spline $\mathbf b^a_m$、optional IMU time offset $\Delta t^i_m$ | camera 参数、gyro bias、其他 IMU 的 bias/内参 |
| extended gyro, IMU $m$ | ordinary gyro blocks，加 $\mathbf M_{g,m}$、$\mathbf A_{g,m}$、$\mathbf R_{g_mi_m}$；若有 acceleration sensitivity 分支，则 gravity 和 $\mathbf r_{b,m}$ 也可能非零 | camera 参数、其他 IMU 的 bias/内参 |
| extended accel, IMU $m$ | ordinary accel blocks，加 $\mathbf M_{a,m}$ 和 size-effect 相关 lever arm | camera 参数、其他 IMU 的 bias/内参 |
| bias motion prior, IMU $m$ | IMU $m$ 自己的 gyro 或 accel bias control points | pose、camera 参数、其他 IMU bias、IMU 外参 |
| pose motion prior | pose spline control points | camera 参数、IMU bias、IMU 内参、gravity |

Camera chain 的特殊规则是：

$$
\frac{\partial\mathbf e^\pi_{n,k,\ell}}
{\partial\boldsymbol\xi_{\mathbf T_m,K}}
=
\mathbf A_T\mathrm{boxTimes}(\mathbf P_m),
\qquad
m\le n,
$$

若 $m>n$，该 camera residual 对 $\mathbf T_m$ 的 block 为零。也就是说，camera $n$ 只连接从 body 到 camera $n$ 路径上的 link。

多 IMU 的特殊规则是：reference IMU 若定义为 body frame，则 $\mathbf R_{i_0b}$ 和 $\mathbf r_{b,0}$ 通常固定。固定 block 不进入活跃变量集合；公式上能写出 Jacobian，不代表实现里要给它装配列。

### C.12.2 多传感器变量更新速查

| 变量 | 更新规则 |
|---|---|
| 共享 pose control point | 所有传感器 residual 贡献同一个增量，按 pose spline design variable 规则回写一次 |
| gravity | 所有 accel residual 和扩展 gyro acceleration-sensitivity 分支共享同一个 $\mathbf g_w$ |
| camera chain transform $\mathbf T_m$ | $\mathbf T_m^+=\mathbf T_m\boxplus_K\delta\boldsymbol\xi_m$ |
| complete camera transform $\mathbf T_{c_nb}$ | $\mathbf T_{c_nb}^+=\mathbf T_{c_nb}\boxplus_K\delta\boldsymbol\xi_n$ |
| camera time shift $\Delta t_n$ | $\Delta t_n^+=\Delta t_n+\delta\Delta t_n$ |
| IMU rotation $\mathbf R_{i_mb}$ | $\mathbf R_{i_mb}^+=\mathrm{Exp}(-\delta\boldsymbol\phi_{m,K})\mathbf R_{i_mb}$ |
| IMU lever arm $\mathbf r_{b,m}$ | $\mathbf r_{b,m}^+=\mathbf r_{b,m}+\delta\mathbf r_{b,m}$ |
| IMU bias control point | 只更新对应 IMU 的 bias spline control point |
| IMU matrices $\mathbf M_{a,m}$、$\mathbf M_{g,m}$、$\mathbf A_{g,m}$ | 按 active mask 做欧式加法 |
| gyro sensing rotation $\mathbf R_{g_mi_m}$ | 按 rotation design variable 更新 |

最后提醒一句：这张表的每个结论都可以用附录 B 的 finite-difference 框架数值验证。如果某个 block 和你的数值对不上，先按 C.2 的第 4 步排查两类符号来源，再检查是不是把 whitened Jacobian 当成了几何 Jacobian（C.1），再检查是不是把 residual 连到了错误的 sensor index（第 13 章）。
