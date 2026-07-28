# 第 9 章：bias 与 motion prior

第 6 章和第 7 章已经把 gyro / accelerometer measurement residual 推完了：gyro residual 里有 gyro bias spline，accelerometer residual 里有 accel bias spline。读到那里，一个自然问题是：

既然 measurement residual 已经会优化 bias，为什么 Kalibr 还要加 **bias motion prior**？README 里的总目标函数还写了一个 **pose motion prior**，它又是什么？

这一章回答这两个问题。它不是再增加一种传感器测量，而是解释 Kalibr 如何给连续时间函数加平滑约束。核心链路是：

$$
\text{spline curve}
\longrightarrow
\text{time derivative}
\longrightarrow
\text{quadratic integral}
\longrightarrow
\mathbf c^\top\mathbf Q\mathbf c
\longrightarrow
\mathbf H,\mathbf b_{\mathrm{rhs}}.
$$

其中 $\mathbf c$ 是一整条 spline 的控制点堆叠向量，$\mathbf Q$ 是由 knot、basis、导数阶数和权重矩阵离线算出来的稀疏二次型矩阵。与普通 camera / IMU residual 不同，Kalibr 的 `BSplineMotionError` 不显式构造每个采样时刻的 residual 和 Jacobian，而是直接把 $\mathbf Q$ 加到 Hessian 里。

## 9.1 本章依赖顺序

| 步骤 | 章节 | 对象 | 本章要得到什么 |
|---|---|---|---|
| 1 | 9.2 | bias spline | 复习 bias 值怎样由控制点插值得到 |
| 2 | 9.3 | motion prior 的直觉 | 说明为什么要惩罚 spline 的时间导数 |
| 3 | 9.4 | quadratic integral | 从 $\int\mathbf x^{(r)\top}\mathbf W\mathbf x^{(r)}dt$ 推到 $\mathbf c^\top\mathbf Q\mathbf c$ |
| 4 | 9.5 | bias motion prior | 推导 gyro / accel bias prior 的 $\mathbf Q$、Hessian 和 RHS |
| 5 | 9.6 | pose motion prior | 说明它约束的是 pose spline 内部 $6$ 维曲线值 |
| 6 | 9.7-9.8 | 源码桥 | 对齐 `BSplineMotionError` 和 Kalibr 调用路径 |
| 7 | 9.8.1-9.8.2 | 三种等价装配 | 说明为什么 Ceres 要逐 segment 开方，以及 $\mathbf G_s^{(r)}$ 的闭式 |
| 8 | 9.9 | 速查表 | 汇总 bias Jacobian、prior 权重和三种装配形态 |

本章的重点不是重新推 projection、gyro 或 accelerometer 的 measurement Jacobian。那些已经在第 4、6、7 章完成。本章只处理额外的平滑 residual。

## 9.2 Bias spline 回顾

Bias 是 IMU 的慢变化系统误差。Kalibr 不给每个 IMU measurement 单独放一个 bias 变量，而是用连续时间 B-spline 表示：

$$
\mathbf b_g(t)\in\mathbb R^3,
\qquad
\mathbf b_a(t)\in\mathbb R^3.
$$

其中 $\mathbf b_g(t)$ 是 gyro bias，单位通常是 rad/s；$\mathbf b_a(t)$ 是 accelerometer bias，单位通常是 m/s$^2$。

为了和 pose spline 的控制点区分，本章记 gyro bias 控制点为：

$$
\mathbf d^g_j\in\mathbb R^3,
$$

accel bias 控制点为：

$$
\mathbf d^a_j\in\mathbb R^3.
$$

若时间 $t$ 落在某个 bias spline 的 active window，gyro bias 的窗口起始索引记为 $j_g(t)$，order 记为 $q_g$；accel bias 的窗口起始索引记为 $j_a(t)$，order 记为 $q_a$。两条 bias spline 在源码里通常用同一套 knot 密度和 order，但它们是两条不同的曲线，本书用不同字母避免把控制点窗口混在一起。

本书从这里开始采用约定：

| bias spline | basis weight | 控制点 |
|---|---|---|
| gyro bias $\mathbf b_g(t)$ | $\mu_\ell^{(0)}(t)$ | $\mathbf d^g_j$ |
| accel bias $\mathbf b_a(t)$ | $\nu_\ell^{(0)}(t)$ | $\mathbf d^a_j$ |

于是：

$$
\boxed{
\mathbf b_g(t)
=
\sum_{\ell=0}^{q_g-1}
\mu_\ell^{(0)}(t)\,
\mathbf d^g_{j_g(t)+\ell}.
}
$$

同理，accelerometer bias 是：

$$
\boxed{
\mathbf b_a(t)
=
\sum_{\ell=0}^{q_a-1}
\nu_\ell^{(0)}(t)\,
\mathbf d^a_{j_a(t)+\ell}.
}
$$

这里：

| 符号 | 含义 |
|---|---|
| $\mu_\ell^{(0)}(t),\mu_\ell^{(1)}(t)$ | gyro bias active basis 的零阶权重和一阶时间导数 |
| $\nu_\ell^{(0)}(t),\nu_\ell^{(1)}(t)$ | accel bias active basis 的零阶权重和一阶时间导数 |
| $q_g,q_a$ | 当前时间处参与 gyro / accel bias 插值的控制点个数 |
| $j_g(t),j_a(t)$ | 当前 gyro / accel active bias 控制点窗口的起始索引 |

这些权重只由 knot、order 和时间决定，不由 bias 控制点数值决定。

在 measurement residual 里，bias 的作用很直接。Gyro residual 是：

$$
\mathbf e^\omega_k
=
\mathbf R_{ib}\boldsymbol\omega_b(t_k)
+
\mathbf b_g(t_k)
-
\mathbf z^\omega_k.
$$

因此，对 active gyro bias 控制点：

$$
\boxed{
\frac{\partial\mathbf e^\omega_k}
{\partial\mathbf d^g_{j_g+\ell}}
=
\mu_\ell^{(0)}(t_k)\mathbf I_3.
}
$$

Accelerometer residual 是：

$$
\mathbf e^a_k
=
\mathbf R_{ib}\mathbf u_b(t_k)
+
\mathbf b_a(t_k)
-
\mathbf z^a_k.
$$

因此：

$$
\boxed{
\frac{\partial\mathbf e^a_k}
{\partial\mathbf d^a_{j_a+\ell}}
=
\nu_\ell^{(0)}(t_k)\mathbf I_3.
}
$$

这两个 Jacobian 只来自 measurement residual。接下来要讲的 bias motion prior 是另一类 residual：它不比较 IMU measurement，而是约束 bias spline 自己的形状。

## 9.3 为什么需要 motion prior

如果只靠 measurement residual，bias spline 可能变得太自由。直觉上，优化器会发现：

1. 某些轨迹误差可以被 bias 吸收。
2. 某些外参误差可以被 bias 吸收。
3. 某些时间偏移误差也可能被 bias 的局部变化部分吸收。

如果 bias 控制点足够密，而没有额外约束，bias spline 就可能用剧烈抖动去解释本来应该由轨迹、外参或时间对齐解释的误差。这会让标定问题变得病态。

真实 IMU bias 通常更像慢变化过程。对 gyro bias，一个常见先验是 random walk。用连续时间白噪声模型的 shorthand 写成：

$$
\dot{\mathbf b}_g(t)
\sim
\mathcal N(\mathbf 0,\boldsymbol\Sigma_{wg}),
$$

对 accel bias：

$$
\dot{\mathbf b}_a(t)
\sim
\mathcal N(\mathbf 0,\boldsymbol\Sigma_{wa}).
$$

如果某段时间里 bias 变化很快，$\dot{\mathbf b}(t)$ 很大，就应该被惩罚。把这个想法写成连续时间代价：

$$
\boxed{
E_b
=
\int_{t_0}^{t_1}
\dot{\mathbf b}(t)^\top
\mathbf W_b
\dot{\mathbf b}(t)
\,dt.
}
$$

其中：

$$
\mathbf W_b
=
\boldsymbol\Sigma_w^{-1}.
$$

这就是 bias motion prior。它不是说 bias 必须为零，而是说 bias 的变化率不应该无理由很大。

Pose motion prior 也是同一个思想，只是对象换成 pose spline 内部的 $6$ 维曲线值。它通常惩罚更高阶导数，比如 acceleration-like 的二阶导数，用来防止轨迹在缺少测量约束的时间段里出现不合理振荡。

## 9.4 从导数积分到二次型

本节先推一个通用 Euclidean spline。令：

$$
\mathbf x(t)\in\mathbb R^D
$$

是一条 $D$ 维 B-spline，控制点为：

$$
\mathbf c_j\in\mathbb R^D.
$$

在一个固定 knot segment $s$ 上，active 控制点是：

$$
\mathbf c_s,\mathbf c_{s+1},\ldots,\mathbf c_{s+q-1}.
$$

把它们堆成一个局部向量：

$$
\mathbf c^{(s)}
=
\begin{bmatrix}
\mathbf c_s\\
\mathbf c_{s+1}\\
\vdots\\
\mathbf c_{s+q-1}
\end{bmatrix}
\in\mathbb R^{Dq}.
$$

下面先推任意一条 Euclidean B-spline 的 motion prior。为了避免公式同时带 gyro/accel 两套字母，本节临时用 $\mu_\ell^{(r)}$ 表示这条 **泛化曲线** 的 basis weight 导数；代回 gyro bias 时它就是上一节的 $\mu_\ell^{(r)}$，代回 accel bias 时它对应上一节的 $\nu_\ell^{(r)}$。

第 5 章已经给出 basis weight 的 $r$ 阶时间导数：

$$
\mu_\ell^{(r)}(t),
\qquad \ell=0,\ldots,q-1.
$$

因此曲线的 $r$ 阶导数是：

$$
\mathbf x^{(r)}(t)
=
\sum_{\ell=0}^{q-1}
\mu_\ell^{(r)}(t)
\mathbf c_{s+\ell}.
$$

为了写成矩阵形式，定义：

$$
\mathbf A_s^{(r)}(t)
=
\begin{bmatrix}
\mu_0^{(r)}(t)\mathbf I_D&
\mu_1^{(r)}(t)\mathbf I_D&
\cdots&
\mu_{q-1}^{(r)}(t)\mathbf I_D
\end{bmatrix}
\in\mathbb R^{D\times Dq}.
$$

于是：

$$
\boxed{
\mathbf x^{(r)}(t)
=
\mathbf A_s^{(r)}(t)\mathbf c^{(s)}.
}
$$

现在考虑一段上的 motion prior：

$$
E_s
=
\int_{\tau_s}^{\tau_{s+1}}
\mathbf x^{(r)}(t)^\top
\mathbf W
\mathbf x^{(r)}(t)
\,dt,
$$

其中 $\mathbf W\in\mathbb R^{D\times D}$ 是对这个导数的 information matrix。代入上式：

$$
\begin{aligned}
E_s
&=
\int_{\tau_s}^{\tau_{s+1}}
\left(
\mathbf A_s^{(r)}(t)\mathbf c^{(s)}
\right)^\top
\mathbf W
\left(
\mathbf A_s^{(r)}(t)\mathbf c^{(s)}
\right)
dt
\\
&=
\mathbf c^{(s)\top}
\left[
\int_{\tau_s}^{\tau_{s+1}}
\mathbf A_s^{(r)}(t)^\top
\mathbf W
\mathbf A_s^{(r)}(t)
dt
\right]
\mathbf c^{(s)}.
\end{aligned}
$$

定义局部二次型矩阵：

$$
\boxed{
\mathbf Q_s^{(r)}
\triangleq
\int_{\tau_s}^{\tau_{s+1}}
\mathbf A_s^{(r)}(t)^\top
\mathbf W
\mathbf A_s^{(r)}(t)
dt
\in\mathbb R^{Dq\times Dq}.
}
$$

则：

$$
\boxed{
E_s
=
\mathbf c^{(s)\top}
\mathbf Q_s^{(r)}
\mathbf c^{(s)}.
}
$$

把 $\mathbf Q_s^{(r)}$ 按控制点 block 拆开，可以看得更清楚。第 $a$ 个和第 $b$ 个 active 控制点之间的 block 是：

$$
\boxed{
\mathbf Q^{(r)}_{s,ab}
=
\left[
\int_{\tau_s}^{\tau_{s+1}}
\mu_a^{(r)}(t)\mu_b^{(r)}(t)\,dt
\right]
\mathbf W
\in\mathbb R^{D\times D}.
}
$$

所以一个 segment 会同时连接当前 active window 内的 $q$ 个控制点。若 $q=6$，它会给这 $6$ 个控制点两两之间的 Hessian block 加贡献。

最后，把所有 segment 的局部矩阵装配到全局控制点向量：

$$
\mathbf c
=
\begin{bmatrix}
\mathbf c_0\\
\mathbf c_1\\
\vdots\\
\mathbf c_{N-1}
\end{bmatrix}
\in\mathbb R^{DN}.
$$

得到全局代价：

$$
\boxed{
E
=
\sum_s E_s
=
\mathbf c^\top\mathbf Q\mathbf c.
}
$$

这里 $\mathbf Q$ 是稀疏矩阵，因为每个 segment 只连接局部 $q$ 个控制点。Kalibr 的 `curveQuadraticIntegralSparse(W, derivativeOrder)` 做的就是这件事。

源码里的 `segmentQuadraticIntegral(...)` 用的是同一个积分，只是写成多项式系数矩阵的形式。对一个 segment，令 $\mathbf M_s$ 把局部控制点映射到该 segment 的多项式系数，令 $\mathbf V_s^{(r)}$ 存储 $r$ 阶导数后的单项式乘积积分：

$$
\mathbf V_s^{(r)}
\triangleq
\int_{\tau_s}^{\tau_{s+1}}
\mathbf u^{(r)}(t)\mathbf u^{(r)}(t)^\top dt.
$$

那么局部二次型可以写成：

$$
\boxed{
\mathbf Q_s^{(r)}
=
\mathbf M_s^\top
\left(
\mathbf W\otimes\mathbf V_s^{(r)}
\right)
\mathbf M_s.
}
$$

这就是源码中的：

```cpp
V = (Dm.transpose() * V * Dm).eval();
WV.block(...) = W(r,c) * V;
Q = M.transpose() * WV * M;
```

其中 `Dm` 负责把普通单项式积分矩阵变成导数阶数为 `derivativeOrder` 的版本，`WV` 是 $\mathbf W\otimes\mathbf V_s^{(r)}$ 的 block 展开，`M` 就是 $\mathbf M_s$。

## 9.5 Bias motion prior

Bias motion prior 是 9.4 的直接应用。对 gyro bias，取：

$$
D=3,
\qquad
r=1,
\qquad
\mathbf x(t)=\mathbf b_g(t).
$$

Kalibr 源码中：

```python
Wgyro = np.eye(3) / (self.gyroRandomWalk * self.gyroRandomWalk)
gyroBiasMotionErr = asp.BSplineEuclideanMotionError(self.gyroBiasDv, Wgyro, 1)
```

所以：

$$
\boxed{
\mathbf W_g
=
\frac{1}{\sigma_{wg}^2}\mathbf I_3,
\qquad
r=1.
}
$$

这里 $\sigma_{wg}$ 对应 `gyroRandomWalk`。对应代价是：

$$
\boxed{
E_{b_g}
=
\int_{t_0}^{t_1}
\dot{\mathbf b}_g(t)^\top
\mathbf W_g
\dot{\mathbf b}_g(t)
dt
=
\mathbf d_g^\top
\mathbf Q_g
\mathbf d_g.
}
$$

其中：

$$
\mathbf d_g
=
\begin{bmatrix}
\mathbf d^g_0\\
\mathbf d^g_1\\
\vdots\\
\mathbf d^g_{N_g-1}
\end{bmatrix}.
$$

对 accel bias 同理：

```python
Waccel = np.eye(3) / (self.accelRandomWalk * self.accelRandomWalk)
accelBiasMotionErr = asp.BSplineEuclideanMotionError(self.accelBiasDv, Waccel, 1)
```

因此：

$$
\boxed{
E_{b_a}
=
\int_{t_0}^{t_1}
\dot{\mathbf b}_a(t)^\top
\mathbf W_a
\dot{\mathbf b}_a(t)
dt
=
\mathbf d_a^\top
\mathbf Q_a
\mathbf d_a,
\qquad
\mathbf W_a
=
\frac{1}{\sigma_{wa}^2}\mathbf I_3.
}
$$

### 9.5.1 Bias prior 的 block Jacobian 视角

虽然 Kalibr 不显式构造 residual vector，但我们可以用一个等价的“虚拟 residual”理解它。

如果 $\mathbf Q_g$ 可以分解为：

$$
\mathbf Q_g
=
\mathbf S_g^\top\mathbf S_g,
$$

那么：

$$
E_{b_g}
=
\mathbf d_g^\top\mathbf Q_g\mathbf d_g
=
\|\mathbf S_g\mathbf d_g\|^2.
$$

这等价于定义：

$$
\mathbf e^{b_g}
=
\mathbf S_g\mathbf d_g,
\qquad
\mathbf J^{b_g}
=
\mathbf S_g.
$$

于是普通 Gauss-Newton 会给出：

$$
\mathbf H
\mathrel{+}=
\mathbf J^{b_g\top}\mathbf J^{b_g}
=
\mathbf Q_g,
$$

$$
\mathbf b_{\mathrm{rhs}}
\mathrel{-}=
\mathbf J^{b_g\top}\mathbf e^{b_g}
=
\mathbf Q_g\mathbf d_g.
$$

这正是源码中 `buildHessianImplementation` 的做法：

```cpp
_Q.multiply(&b_u, c);
*Hblock += *Qblock;
outRhs.segment(rowBase, rows) -= b_u.segment(i*rows, rows);
```

所以 bias motion prior 的 Jacobian 可以有两种等价读法：

| 读法 | 形式 | Kalibr 是否显式使用 |
|---|---|---|
| 虚拟 residual | $\mathbf e=\mathbf S\mathbf c,\ \mathbf J=\mathbf S$ | 否 |
| 二次型 Hessian | $E=\mathbf c^\top\mathbf Q\mathbf c,\ \mathbf H+=\mathbf Q,\ \mathbf b_{\mathrm{rhs}}-= \mathbf Q\mathbf c$ | 是 |

读源码时应该采用第二种；做数学直觉或 finite-difference check 时，第一种也成立。

## 9.6 Pose motion prior

Pose motion prior 使用同一个 `BSplineMotionError` 机制，但对象换成 pose spline 的内部曲线值：

$$
\mathbf s(t)
=
\begin{bmatrix}
\mathbf t_{wb}(t)\\
\boldsymbol\psi(t)
\end{bmatrix}
\in\mathbb R^6.
$$

第 5 章已经说明，Kalibr 的 pose spline 先在 $\mathbb R^6$ 中插值得到 $\mathbf s(t)$，再通过映射 $F$ 转成 $\mathbf T_{wb}(t)$。Pose motion prior 约束的是这条内部 $6$ 维曲线的时间导数，而不是直接在 $SE(3)$ 上写一个新的 $\mathrm{Log}$ residual。

源码入口是：

```python
wt = 1.0 / tv
wr = 1.0 / rv
W = np.diag([wt, wt, wt, wr, wr, wr])
asp.addMotionErrorTerms(problem, self.poseDv, W, errorOrder)
```

因此权重矩阵是：

$$
\boxed{
\mathbf W_m
=
\mathrm{diag}
\left(
\frac{1}{\sigma_t^2},
\frac{1}{\sigma_t^2},
\frac{1}{\sigma_t^2},
\frac{1}{\sigma_R^2},
\frac{1}{\sigma_R^2},
\frac{1}{\sigma_R^2}
\right).
}
$$

这里源码参数名是 `mrTranslationVariance` 和 `mrRotationVariance`，函数内部取倒数作为 information。为了和前面 covariance / information 的语言一致，本章把它们理解为：

$$
\sigma_t^2=\texttt{mrTranslationVariance},
\qquad
\sigma_R^2=\texttt{mrRotationVariance}.
$$

若导数阶数记为 $r_m$，pose motion prior 是：

$$
\boxed{
E_{\mathrm{pose}}
=
\int_{t_0}^{t_1}
\mathbf s^{(r_m)}(t)^\top
\mathbf W_m
\mathbf s^{(r_m)}(t)
dt
=
\mathbf c_s^\top\mathbf Q_s\mathbf c_s.
}
$$

这里 $\mathbf c_s$ 是所有 pose spline 控制点堆叠成的全局向量。若 $r_m=2$，这个 prior 就是 acceleration-like 的轨迹平滑项；若 $r_m=1$，它约束速度-like 的变化。C++ `BSplineMotionError` 的默认构造函数使用 `errorTermOrder=2`，但当前 cam-imu Python 主流程调用 `addPoseMotionTerms(...)` 时显式传入 `errorOrder`。这个变量在本仓库的该文件中没有局部定义，而且 `doPoseMotionError` 默认是 `False`，所以正常默认路径不会触发这一项。若启用 pose motion prior，需要先确认调用环境里 `errorOrder` 的定义。

这段 caveat 不改变公式。只要给定导数阶数 $r_m$，pose motion prior 与 bias motion prior 的推导完全相同：

$$
\mathbf H
\mathrel{+}=
\mathbf Q_s,
\qquad
\mathbf b_{\mathrm{rhs}}
\mathrel{-}=
\mathbf Q_s\mathbf c_s.
$$

## 9.7 为什么 motion prior 很稀疏

Motion prior 看起来是一个从 $t_0$ 积分到 $t_1$ 的全局项，但它的 Hessian 仍然是稀疏的。原因仍然是 B-spline 的局部支撑。

在某个 segment $s$ 上：

$$
\mathbf x^{(r)}(t)
=
\mathbf A_s^{(r)}(t)\mathbf c^{(s)}
$$

只依赖 $q$ 个 active 控制点。因此该 segment 的 $\mathbf Q_s^{(r)}$ 只会给这 $q$ 个控制点之间的 block 加贡献。不同 segment 的贡献累加后，全局 $\mathbf Q$ 呈带状稀疏结构。

这和 measurement residual 的稀疏性很像：

| residual 类型 | 为什么稀疏 |
|---|---|
| camera corner residual | 只连接当前时间附近的 pose 控制点、当前 camera 参数和少数标定变量 |
| gyro / accel residual | 只连接当前时间附近的 pose 控制点、bias 控制点和 IMU 参数 |
| motion prior | 每个 segment 只连接当前 segment 的 active spline 控制点 |

区别是：measurement residual 通常是“一个测量一条 residual”；motion prior 是“一个 segment 给一个二次型 block”。Kalibr 直接装配这些 block，而不是把积分离散成很多采样 residual。

## 9.8 源码桥

| 数学对象 | 源码位置 | 作用 |
|---|---|---|
| gyro bias spline $\mathbf b_g(t)$ | `self.gyroBias = bsplines.BSpline(...)` | 初始化 gyro bias 曲线 |
| accel bias spline $\mathbf b_a(t)$ | `self.accelBias = bsplines.BSpline(...)` | 初始化 accel bias 曲线 |
| bias 控制点 design variable | `EuclideanBSplineDesignVariable` | 把 bias spline 控制点加入优化 |
| bias motion prior | `imu.addBiasMotionTerms(problem)` | 给 gyro/accel bias spline 添加一阶导数平滑项 |
| gyro prior weight | `Wgyro = I / gyroRandomWalk^2` | $\mathbf W_g$ |
| accel prior weight | `Waccel = I / accelRandomWalk^2` | $\mathbf W_a$ |
| quadratic integral | `curveQuadraticIntegralSparse(W, errorTermOrder)` | 计算全局稀疏 $\mathbf Q$ |
| Hessian 直接装配 | `BSplineMotionError::buildHessianImplementation` | 把 $\mathbf Q$ 和 $-\mathbf Q\mathbf c$ 加入线性系统 |
| pose motion prior | `addPoseMotionTerms(...)` | 可选 pose spline regularization，默认关闭 |

`BSplineMotionError::evaluateJacobiansImplementation(...)` 在源码中直接抛异常，原因不是这个 prior 没有 Jacobian，而是它没有走普通 residual-Jacobian 评价路径。它的等价 Jacobian 已经通过 $\mathbf Q=\mathbf S^\top\mathbf S$ 被折叠进 Hessian。

### 9.8.1 三种等价装配：全局 sqrt / Kalibr 装 Hessian / Ceres 逐 segment residual

同一个代价 $E=\mathbf c^\top\mathbf Q\mathbf c$ 有三种装配方式。它们在数学上完全等价，但工程上适用的场合完全不同。要把这一章迁移到 Ceres，必须先分清这三者。

**装配 A：全局平方根。** 取 $\mathbf Q=\mathbf S^\top\mathbf S$，令 $\mathbf e=\mathbf S\mathbf c$、$\mathbf J=\mathbf S$。这就是 9.5.1 的虚拟 residual 视角，形式最干净，也是理解“二次型就是 residual 的平方和”最直接的桥。

它的问题不在数学而在工程。$\mathbf Q$ 是全局矩阵，维度是 $DN\times DN$：$N$ 是整条 spline 的控制点总数。一条 $200\ \mathrm s$、knot 间隔 $0.05\ \mathrm s$ 的 bias spline 就有 $N\approx4000$，$DN\approx12000$。

先澄清两个容易想当然、但都站不住的理由。

**其一，全局开方并不会毁掉 9.7 的稀疏结构。** 9.7 说明的是 $\mathbf Q$ 呈 **带状**，带宽由 spline 阶数决定——每个 segment 只连接 $q$ 个 active 控制点，所以半带宽约 $qD$。而带状矩阵的 Cholesky 因子仍然是带状的，带宽与原矩阵完全相同，带外一个 fill-in 都不会产生。稠密 fill-in 是一般稀疏矩阵在消元顺序不好时才会发生的事；带状矩阵的自然顺序本身就已经是最优顺序之一。

**其二，秩亏也不会逼着人放弃稀疏。** 全局 $\mathbf Q$ 确实只是半正定（本节稍后给出每个 segment 上 $\operatorname{rank}\mathbf G_s^{(r)}=q-r$），无 pivot 的 Cholesky 会在某些主元上遇到 $0$。但处理奇异只需要换成能容忍零主元的带状变体——带状 $\mathbf L\mathbf D\mathbf L^\top$，或在主元低于阈值时把该行整行置零——它们同样保持带宽。半正定与稀疏并不冲突。

结论是：$\mathbf S$ 本身就是带状的，它的每一行只碰到某个带内的 $O(q)$ 个控制点。按行分组之后，$\mathbf e=\mathbf S\mathbf c$ 完全可以拆成许多只挂少数控制点的局部 residual block 喂给 Ceres；分解代价也不是 $O((DN)^3)$，带状 Cholesky 是 $O\!\left(DN(qD)^2\right)$，对 $N$ 线性。**装配 A 是一条可行路线，不是一条数学上被堵死的路线。**

它的缺点是工程上的：

**第一，它需要一趟全局装配加一趟全局分解。** 必须先把整条 spline 的 $\mathbf Q$ 显式拼出来，整体分解一次拿到 $\mathbf S$，再把 $\mathbf S$ 的行切回局部 block。三步都是全局操作，而下面装配 C 从头到尾只在单个 segment 内部进行。

**第二，$\mathbf S$ 绑死在 knot 布局上。** knot 间隔、spline 阶数、时间区间任何一处改动，$\mathbf Q$ 的维度和带宽都跟着变，整个 $\mathbf S$ 必须从头重算。标定流程里 knot 布局是要随数据长度和 IMU 频率调整的，把一趟全局分解放进这个循环并不划算。

**第三，如果偷懒不做行分组，直接把 $\mathbf e=\mathbf S\mathbf c$ 当成一个 residual block 交给 Ceres，那才真正出事。** Ceres 的 `CostFunction` 接口是 per-residual-block 的：一个 block 必须在 `AddResidualBlock` 时显式列出它依赖的所有 parameter block，并在 `Evaluate` 里逐个填对应的 Jacobian 块。把 $4000$ 个控制点挂进同一个 block，意味着 $4000$ 个 parameter block 和一个名义上 $12000\times12000$ 的 Jacobian——即使 $\mathbf S$ 数值上是带状的，接口层面也没有地方声明这个带状性。带状性必须靠“按行拆 block”显式表达出来才能被 Ceres 利用。

换句话说，全局平方根是一条合法但绕远的路：绕完一圈得到的仍然是一组局部 residual block，而这组 block 本来就可以不经过任何全局分解直接构造——那就是装配 C。

**装配 B：直接装 Hessian。** 这是 Kalibr 的选择：不开方，把稀疏的 $\mathbf Q$ 原样加到 $\mathbf H$、把 $-\mathbf Q\mathbf c$ 加到 RHS。它避开了分解，也完整保留稀疏性。代价是它绕过了框架的 residual/Jacobian 接口，所以 `evaluateJacobiansImplementation` 只能抛异常。这条路要求优化器允许 error term 直接写线性系统——`aslam_backend` 允许，Ceres 不允许。

**装配 C：逐 segment residual。** Ceres 的 `CostFunction` 接口只接受“给我 residual 和 Jacobian”，所以必须回到 residual 形式。最省事的做法是干脆不在全局层面开方，而是在 **单个 segment** 上开方——同样得到一组局部 block，但整个过程是纯局部的，不需要装配全局 $\mathbf Q$，也不需要任何全局分解。

关键观察来自 9.4 的 block 形式：局部矩阵 $\mathbf Q_s^{(r)}$ 的第 $(a,b)$ 个 $D\times D$ block 是一个标量乘 $\mathbf W$。把那个标量单独拎出来，定义一个 $q\times q$ 的 **标量基函数积分矩阵**：

$$
\boxed{
\left[\mathbf G_s^{(r)}\right]_{ab}
\triangleq
\int_{\tau_s}^{\tau_{s+1}}
\mu_a^{(r)}(t)\mu_b^{(r)}(t)\,dt,
\qquad
a,b=0,\ldots,q-1.
}
$$

于是 9.4 的局部二次型就是一个 Kronecker 积：

$$
\boxed{
\mathbf Q_s^{(r)}
=
\mathbf G_s^{(r)}\otimes\mathbf W.
}
$$

$\mathbf G_s^{(r)}$ 只有 $q\times q$，$q=6$ 时就是一个 $6\times6$ 矩阵。对它做对称特征分解 $\mathbf G_s^{(r)}=\mathbf V\boldsymbol\Lambda\mathbf V^\top$，取

$$
\boxed{
\mathbf S_{\mathrm{seg}}
=
\boldsymbol\Lambda^{1/2}\mathbf V^\top
\in\mathbb R^{q\times q},
\qquad
\mathbf S_{\mathrm{seg}}^\top\mathbf S_{\mathrm{seg}}
=
\mathbf G_s^{(r)}.
}
$$

这是一次 $6\times6$ 的分解，离线算一次就够，完全不涉及全局规模。再配上 $\mathbf W=\mathbf W^{1/2\top}\mathbf W^{1/2}$（Kalibr 的 $\mathbf W$ 都是对角，所以 $\mathbf W^{1/2}$ 就是逐元素开方），这个 segment 的 residual 是：

$$
\boxed{
\mathbf e^{(s)}
=
\left(
\mathbf S_{\mathrm{seg}}\otimes\mathbf W^{1/2}
\right)
\mathbf c^{(s)}
\in\mathbb R^{qD}.
}
$$

Kronecker 的左右顺序由 $\mathbf c^{(s)}$ 的堆叠约定决定：9.4 把 $\mathbf c^{(s)}$ 按控制点分块、每块 $D$ 维，所以 **控制点索引在外、维度索引在内**，$\mathbf S_{\mathrm{seg}}$ 必须写在左边。写反了会得到一个维度相同但数值全错的矩阵，而且 residual 范数依然“看起来合理”，很难在调试时发现。

验证它确实等价，只要把范数展开：

$$
\left\|\mathbf e^{(s)}\right\|^2
=
\mathbf c^{(s)\top}
\left(
\mathbf S_{\mathrm{seg}}^\top\mathbf S_{\mathrm{seg}}
\otimes
\mathbf W
\right)
\mathbf c^{(s)}
=
\mathbf c^{(s)\top}
\left(
\mathbf G_s^{(r)}\otimes\mathbf W
\right)
\mathbf c^{(s)}
=
E_s.
$$

Jacobian 更省事：$\mathbf e^{(s)}$ 对 $\mathbf c^{(s)}$ 是线性的，所以

$$
\boxed{
\frac{\partial\mathbf e^{(s)}}
{\partial\mathbf c_{s+\ell}}
=
\left[\mathbf S_{\mathrm{seg}}\right]_{:,\ell}
\otimes
\mathbf W^{1/2}
\in\mathbb R^{qD\times D},
}
$$

是一个 **常数矩阵**，不依赖控制点当前值，也不需要在每次迭代重算。

本仓库 `include/ceres_cam_imu/residuals/spline_motion_prior.h` 的 `EuclideanSplineMotionPriorCost<Dimension>` 就是这个模板：它是一个 `SizedCostFunction<6*Dimension, Dimension, ..., Dimension>`，$6$ 个参数块正好是该 segment 的 $q=6$ 个 active 控制点。`src/residuals/bias_motion_prior.cpp` 用 `Dimension=3`、$r=1$、$\mathbf W^{1/2}=\mathbf I_3/\sigma_w$，得到 **$18$ 维 residual**；`src/residuals/pose_motion_prior.cpp` 用 `Dimension=6`，得到 $36$ 维 residual。`optimizer/calibration_problem.cpp` 对 spline 的每个 segment 各添加一个这样的 residual block，于是全局 $\mathbf Q$ 从来没有被显式构造过，稀疏性天然由“每个 block 只挂 $6$ 个参数块”保证。

| | 是否需要分解 | 分解规模 | 是否保稀疏 | 适用 |
|---|---|---|---|---|
| A 全局 sqrt | 是 | $DN\times DN$（带状） | 是（因子仍带状） | 讲数学；可行但需全局分解 |
| B 装 Hessian | 否 | — | 是 | Kalibr `aslam_backend` |
| C 逐 segment residual | 是 | $q\times q$ | 是 | Ceres |

**一个必须知道的秩亏细节。** $\mathbf G_s^{(r)}$ 不是满秩的。$r$ 阶导数会把次数小于 $r$ 的多项式打成零，而一个 segment 上的曲线恰好张成次数不超过 $q-1$ 的多项式空间，所以

$$
\boxed{
\operatorname{rank}\mathbf G_s^{(r)}
=
q-r.
}
$$

$q=6$、$r=1$ 时秩是 $5$，$r=2$ 时是 $4$。这正是“常值 bias 不被一阶 motion prior 惩罚”这句直觉的矩阵版本。后果有两个：第一，$\boldsymbol\Lambda$ 里会出现精确的零，以及浮点意义下的极小负数，开方前必须截断；第二，$18$ 维 bias residual 里只有 $15$ 个分量线性无关，$\mathbf S_{\mathrm{seg}}$ 有一整行是零。这不影响 Ceres——它只要求 $\mathbf J^\top\mathbf J$ 半正定——但做秩检查时看到亏秩不要当成 bug。

源码里的截断是相对阈值而不是绝对阈值：

```cpp
const double max_eigenvalue = std::max(0.0, eig.eigenvalues().maxCoeff());
const double threshold = std::max(1e-18, 1e-12 * max_eigenvalue);
```

之所以要用相对阈值，是因为 $\mathbf G_s^{(r)}$ 的条件数很大。以 $q=6$、$r=1$、$\Delta t=1$ 为例，六个特征值大致是 $0,\ 4.3\times10^{-7},\ 1.1\times10^{-5},\ 3.3\times10^{-3},\ 5.8\times10^{-2},\ 3.1\times10^{-1}$：$4.3\times10^{-7}$ 是真实的非零特征值，如果用一个绝对阈值（比如 $10^{-6}$）去截断，就会把一个真实约束方向误杀掉。

### 9.8.2 $\mathbf G_s^{(r)}$ 的闭式与 $\Delta t$ 缩放

$\mathbf G_s^{(r)}$ 有闭式，不需要数值积分。第 5 章已经给出 segment 的 **basis matrix** $\mathbf B$：把归一化局部时间记为

$$
\bar u=\frac{t-\tau_s}{\Delta t}\in[0,1],
\qquad
\Delta t=\tau_{s+1}-\tau_s,
$$

则第 $a$ 个 active 基函数在这一段上是一个多项式，$\mathbf B$ 的第 $i$ 行第 $a$ 列就是它的 $\bar u^i$ 次项系数：

$$
\mu_a^{(0)}(t)
=
\sum_{i=0}^{q-1}
B_{ia}\,\bar u^{\,i}.
$$

对时间求 $r$ 阶导数时，每求一次导都要多出一个 $1/\Delta t$，同时次数低于 $r$ 的项被打掉：

$$
\mu_a^{(r)}(t)
=
\Delta t^{-r}
\sum_{i\ge r}
B_{ia}
\frac{i!}{(i-r)!}
\bar u^{\,i-r}.
$$

代进积分，并用 $dt=\Delta t\,d\bar u$ 换元、$\int_0^1\bar u^{\,i+j-2r}d\bar u=\frac{1}{i+j-2r+1}$，就得到：

$$
\boxed{
\int_{\tau_s}^{\tau_{s+1}}
\mu_a^{(r)}\mu_b^{(r)}\,dt
=
\Delta t^{\,1-2r}
\sum_{i,j\ge r}
B_{ia}B_{jb}
\frac{i!}{(i-r)!}
\frac{j!}{(j-r)!}
\frac{1}{i+j-2r+1}.
}
$$

**最容易写错的就是那个指数 $1-2r$。** 记法是把它拆成 $\Delta t^{-r}\cdot\Delta t^{-r}\cdot\Delta t$：两个 $r$ 阶导数各贡献一个 $\Delta t^{-r}$，换元的 $dt$ 贡献一个 $\Delta t$。写成 $\Delta t^{-2r}$（漏掉换元）或 $\Delta t^{1-r}$（只算一次求导）都会让 prior 的强度随 knot 密度错误缩放，而且因为整体只是一个正的比例因子，优化仍然会收敛，只是 bias 的平滑程度和 `gyroRandomWalk` 对不上——这类错误极难通过“跑通了”发现。

对 $r=1$，指数是 $-1$：knot 越密，$\Delta t$ 越小，同一个 segment 的 prior 权重越大。这是对的——一阶 motion prior 近似的是 random walk，segment 变短意味着允许的 bias 变化量也应该变小。

`src/residuals/spline_motion_prior.cpp` 的 `segmentWeightDerivativeIntegral` 就是这个公式的逐字翻译：`dt_scale = pow(segment.dt_s, 1 - 2*derivative_order)`，两重循环从 `derivative_order` 起跳，`derivativeMultiplier(i, r)` 返回 $i!/(i-r)!$，分母是 `i_power + j_power + 1`。最后的 `0.5 * (integral + integral.transpose())` 只是对称化，抵消浮点求和顺序带来的不对称。

读者可以用十几行 numpy 自查这个闭式。取任意 $q\times q$ 的 $\mathbf B$（用真实 basis matrix 或随机矩阵都行），一边按闭式算，一边把 $\mu_a^{(r)}$ 采样后做数值积分，两者的相对误差应该落在数值积分精度内：

```python
import numpy as np
from math import factorial

def closed_form(B, r, dt):
    q = B.shape[0]
    Q = np.zeros((q, q))
    for a in range(q):
        for b in range(q):
            s = sum(B[i, a] * B[j, b]
                    * (factorial(i) // factorial(i - r))
                    * (factorial(j) // factorial(j - r))
                    / (i + j - 2 * r + 1)
                    for i in range(r, q) for j in range(r, q))
            Q[a, b] = dt ** (1 - 2 * r) * s
    return Q

# NumPy 2.0 把 np.trapz 改名为 np.trapezoid；np.trapz 在 2.x 里仍可用但已废弃，
# 而 NumPy 1.x（本书环境是 1.26.4）里只有 np.trapz。取到哪个用哪个，两边都能跑。
_trapz = getattr(np, "trapezoid", None) or np.trapz

def quadrature(B, r, dt, n=800001):
    q = B.shape[0]
    u = np.linspace(0, 1, n)
    mu = np.array([sum(B[i, a] * (factorial(i) // factorial(i - r)) * u ** (i - r)
                       for i in range(r, q)) * dt ** (-r) for a in range(q)])
    return np.array([[_trapz(mu[a] * mu[b], u) * dt for b in range(q)]
                     for a in range(q)])
```

对 $q=6$、$r\in\{1,2\}$、$\Delta t\in\{0.05,0.37,1.0\}$，两者的最大相对误差约 $10^{-12}$，即完全由梯形积分的离散误差主导。顺手把 `np.linalg.matrix_rank(closed_form(B, r, dt))` 打出来，会看到 $q-r$，正好复现上一节的秩亏结论。

## 9.9 速查表

### 9.9.1 Measurement residual 里的 bias Jacobian

| residual | 控制点 | Jacobian | 维度 |
|---|---|---|---|
| gyro | $\mathbf d^g_{j_g+\ell}$ | $\mu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| accel | $\mathbf d^a_{j_a+\ell}$ | $\nu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |

如果控制点不在当前时间的 active bias window 内，Jacobian block 为 $\mathbf 0_{3\times3}$。

### 9.9.2 Motion prior

| prior | spline | derivative order | weight | 二次型 |
|---|---|---:|---|---|
| gyro bias motion | $\mathbf b_g(t)$ | $1$ | $\mathbf I_3/\sigma_{wg}^2$ | $\mathbf d_g^\top\mathbf Q_g\mathbf d_g$ |
| accel bias motion | $\mathbf b_a(t)$ | $1$ | $\mathbf I_3/\sigma_{wa}^2$ | $\mathbf d_a^\top\mathbf Q_a\mathbf d_a$ |
| pose motion | $\mathbf s(t)\in\mathbb R^6$ | $r_m$ | $\mathrm{diag}(1/\sigma_t^2,1/\sigma_t^2,1/\sigma_t^2,1/\sigma_R^2,1/\sigma_R^2,1/\sigma_R^2)$ | $\mathbf c_s^\top\mathbf Q_s\mathbf c_s$ |

所有这些 prior 对线性系统的直接贡献都可以写成：

$$
\boxed{
\mathbf H
\mathrel{+}=
\mathbf Q,
\qquad
\mathbf b_{\mathrm{rhs}}
\mathrel{-}=
\mathbf Q\mathbf c.
}
$$

如果一定要用 residual/Jacobian 语言，可以取任意平方根分解：

$$
\mathbf Q=\mathbf S^\top\mathbf S,
\qquad
\mathbf e=\mathbf S\mathbf c,
\qquad
\mathbf J=\mathbf S.
$$

Kalibr 实现选择直接使用 $\mathbf Q$，因为这样避免显式构造一个可能很大的平方根 residual。要把这一项落到 Ceres 上，最省事的做法不是在全局层面开方（那条路可行但要多一趟全局分解，见 9.8.1），而是按 9.8.1 的装配 C 逐 segment 开方：

### 9.9.3 三种装配的落地形态

| 装配 | residual | Jacobian | 分解规模 |
|---|---|---|---|
| 全局 sqrt | $\mathbf e=\mathbf S\mathbf c\in\mathbb R^{DN}$ | $\mathbf S$（带状，半带宽约 $qD$） | $DN\times DN$（带状） |
| Kalibr 装 Hessian | 无 | 无（直接 $\mathbf H+\!\!=\mathbf Q$） | 不需要 |
| Ceres 逐 segment | $\mathbf e^{(s)}=(\mathbf S_{\mathrm{seg}}\otimes\mathbf W^{1/2})\mathbf c^{(s)}\in\mathbb R^{qD}$ | $[\mathbf S_{\mathrm{seg}}]_{:,\ell}\otimes\mathbf W^{1/2}$（常数） | $q\times q$ |

其中 $\mathbf S_{\mathrm{seg}}^\top\mathbf S_{\mathrm{seg}}=\mathbf G_s^{(r)}$，$\left[\mathbf G_s^{(r)}\right]_{ab}=\int_{\tau_s}^{\tau_{s+1}}\mu_a^{(r)}\mu_b^{(r)}dt$，闭式见 9.8.2。bias prior 取 $D=3$、$q=6$，residual 是 $18$ 维；pose prior 取 $D=6$，residual 是 $36$ 维。

## 9.10 常见混淆

第一，measurement residual 中的 bias Jacobian 和 bias motion prior 不是同一件事。前者来自 $\mathbf e^\omega$ 或 $\mathbf e^a$ 里加了 $\mathbf b(t_k)$；后者来自 $\dot{\mathbf b}(t)$ 的连续时间平滑约束。

第二，bias motion prior 不要求 bias 本身接近零。它惩罚的是 bias 的时间导数。一个非零但平滑的常值 bias 不会被一阶 motion prior 惩罚。

第三，`gyroRandomWalk` 和 `accelRandomWalk` 决定的是 bias motion prior 的权重，不是 gyro / accel measurement residual 的白噪声权重。measurement noise 用 `omegaInvR`、`alphaInvR` 和 `gyroNoiseScale` / `accelNoiseScale` 进入第 6、7 章的 residual。

第四，pose motion prior 约束的是 Kalibr pose spline 的内部 $6$ 维曲线 $\mathbf s(t)$。它有助于稳定轨迹，但不是相机、gyro 或 accel 的物理测量。

第五，`BSplineMotionError` 不显式提供普通 Jacobian 接口。调试这类项时，要看它对 Hessian block 和 RHS 的直接贡献，而不是期待它像 `EuclideanError` 那样返回 residual vector 和 dense Jacobian。

## 9.11 本章小结

Bias spline 让每个 IMU measurement 可以在自己的时间上取到连续 bias 值。Measurement residual 对 active bias 控制点的 Jacobian 是 basis weight 乘 $\mathbf I_3$：gyro bias 用 $\mu_\ell^{(0)}(t_k)\mathbf I_3$，accel bias 用 $\nu_\ell^{(0)}(t_k)\mathbf I_3$。Bias motion prior 则进一步约束 bias 的变化率，把 random-walk 直觉写成：

$$
\int \dot{\mathbf b}(t)^\top\mathbf W_b\dot{\mathbf b}(t)\,dt.
$$

对任意 Euclidean B-spline，导数积分都可以折成控制点二次型 $\mathbf c^\top\mathbf Q\mathbf c$。Kalibr 的 `BSplineMotionError` 直接把 $\mathbf Q$ 加到 Hessian，把 $-\mathbf Q\mathbf c$ 加到 RHS；等价地，它可以被看成一个虚拟 residual $\mathbf e=\mathbf S\mathbf c$，其中 $\mathbf Q=\mathbf S^\top\mathbf S$。

Ceres 走的是第三条路：不对全局 $\mathbf Q$ 开方，而是把 $\mathbf Q_s^{(r)}$ 拆成 $\mathbf G_s^{(r)}\otimes\mathbf W$，只对 $q\times q$ 的标量矩阵 $\mathbf G_s^{(r)}$ 做特征值分解，得到常数 residual $\mathbf e^{(s)}=(\mathbf S_{\mathrm{seg}}\otimes\mathbf W^{1/2})\mathbf c^{(s)}$ 和常数 Jacobian。$\mathbf G_s^{(r)}$ 本身有闭式，$\Delta t$ 的指数是 $1-2r$；它的秩是 $q-r$，所以 residual 天然亏秩，开方前需要按相对阈值截断特征值。

第 10 章会转向扩展 IMU 模型：scale、misalignment、size-effect 和 acceleration sensitivity。那些参数不是平滑 prior，而是真正进入 gyro / accelerometer 前向预测的传感器模型参数。
