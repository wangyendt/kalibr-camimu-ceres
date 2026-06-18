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
| 6 | 9.7-9.9 | 源码桥和速查表 | 对齐 `BSplineMotionError` 和 Kalibr 调用路径 |

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

Kalibr 实现选择直接使用 $\mathbf Q$，因为这样避免显式构造一个可能很大的平方根 residual。

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

第 10 章会转向扩展 IMU 模型：scale、misalignment、size-effect 和 acceleration sensitivity。那些参数不是平滑 prior，而是真正进入 gyro / accelerometer 前向预测的传感器模型参数。
