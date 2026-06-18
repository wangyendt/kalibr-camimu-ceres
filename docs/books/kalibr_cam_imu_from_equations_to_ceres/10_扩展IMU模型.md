# 第 10 章：扩展 IMU 模型

第 6 章和第 7 章先推了普通 IMU 模型：

$$
\hat{\boldsymbol\omega}_i
=
\mathbf R_{ib}\boldsymbol\omega_b+\mathbf b_g,
\qquad
\hat{\mathbf a}_i
=
\mathbf R_{ib}\mathbf u_b+\mathbf b_a.
$$

这个模型把 IMU 看成理想三轴传感器：每根轴彼此正交、尺度正确，gyro 和 accelerometer 的 sensing frame 与 IMU frame 重合。实际标定中，Kalibr 还支持更丰富的 IMU 模型。源码中相关类是：

| 源码类 | 模型 |
|---|---|
| `IccImu` | 普通 calibrated IMU |
| `IccScaledMisalignedImu` | scale/misalignment、gyro sensing frame、gyro acceleration sensitivity |
| `IccScaledMisalignedSizeEffectImu` | 在上一个模型基础上加入 accelerometer size-effect |

这一章把这些扩展项放回前向模型，并推它们的 Jacobian。它不是重新推 pose spline、bias spline 或 gravity；那些分支已经在第 6、7、8、9 章完成。本章只回答一个问题：当预测读数外面多了矩阵、额外旋转或 axis-specific lever arm 时，链式法则怎样继续传播？

## 10.1 本章符号和依赖

为避免第 6、7 章局部符号冲突，本章使用下面的统一记号：

| 符号 | 维度 | 源码名 | 含义 |
|---|---:|---|---|
| $\mathbf M_{\mathrm{acc}}$ | $3\times3$ | `M_accel_Dv` | accelerometer scale/misalignment matrix |
| $\mathbf M_g$ | $3\times3$ | `M_gyro_Dv` | gyro scale/misalignment matrix |
| $\mathbf A_g$ | $3\times3$ | `M_accel_gyro_Dv` / `Ma` | gyro acceleration sensitivity matrix |
| $\mathbf R_{gi}$ | $3\times3$ | `q_gyro_i_Dv` / `C_gyro_i` | IMU frame 到 gyro sensing frame 的旋转 |
| $\mathbf r_b$ | $3$ | `r_b_Dv` | body 原点到 IMU 原点，表达在 body frame |
| $\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i$ | $3$ | `rx_i_Dv`, `ry_i_Dv`, `rz_i_Dv` | 三根 accelerometer sensing axis 相对 IMU 原点的 offset，表达在 IMU frame |

本章沿用前面已经定义的运动量：

$$
\boldsymbol\omega_b(t),
\qquad
\boldsymbol\alpha_b(t)=\dot{\boldsymbol\omega}_b(t),
\qquad
\mathbf a_w(t),
\qquad
\mathbf g_w.
$$

并继续使用 accelerometer 中间量：

$$
\boxed{
\mathbf u_b
=
\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)
+
\boldsymbol\alpha_b\times\mathbf r_b
+
\boldsymbol\omega_b\times
\left(
\boldsymbol\omega_b\times\mathbf r_b
\right).
}
$$

Residual 方向仍然是 prediction minus measurement：

$$
\mathbf e^\omega=\hat{\boldsymbol\omega}-\mathbf z^\omega,
\qquad
\mathbf e^a=\hat{\mathbf a}-\mathbf z^a.
$$

所以本章的 Jacobian 不再额外加 residual 方向负号。

## 10.2 矩阵乘向量的基础 Jacobian

扩展 IMU 模型最常见的局部节点是：

$$
\mathbf y=\mathbf M\mathbf x.
$$

它有两类变化：

$$
\delta\mathbf y
=
\delta\mathbf M\,\mathbf x
+
\mathbf M\,\delta\mathbf x.
$$

对输入向量：

$$
\boxed{
\frac{\partial\mathbf y}{\partial\mathbf x}
=
\mathbf M
\in\mathbb R^{3\times3}.
}
$$

若先把 $\mathbf M$ 当成 full $3\times3$ matrix，并按列堆叠：

$$
\operatorname{vec}(\mathbf M)
=
\begin{bmatrix}
\mathbf m_1\\
\mathbf m_2\\
\mathbf m_3
\end{bmatrix},
\qquad
\mathbf M=
\begin{bmatrix}
\mathbf m_1&\mathbf m_2&\mathbf m_3
\end{bmatrix},
$$

则：

$$
\delta\mathbf M\,\mathbf x
=
x_1\delta\mathbf m_1
+
x_2\delta\mathbf m_2
+
x_3\delta\mathbf m_3.
$$

所以：

$$
\boxed{
\frac{\partial(\mathbf M\mathbf x)}
{\partial\operatorname{vec}(\mathbf M)}
=
\begin{bmatrix}
x_1\mathbf I_3&
x_2\mathbf I_3&
x_3\mathbf I_3
\end{bmatrix}
\in\mathbb R^{3\times9}.
}
$$

Kalibr 的 `MatrixBasicDv` 不一定启用 $9$ 个自由度。`M_accel_Dv` 和 `M_gyro_Dv` 使用 lower-triangular mask：

```python
np.array([[1, 0, 0],
          [1, 1, 0],
          [1, 1, 1]], dtype=int)
```

因此源码会从上面的 $3\times9$ full-matrix Jacobian 中选出 active entry 对应的列。`M_accel_gyro_Dv` 使用全 $1$ mask，所以 acceleration sensitivity matrix 有 $9$ 个 active entry。

## 10.3 Accelerometer scale/misalignment

先看扩展 accelerometer，但暂时不考虑 size-effect。普通模型的 raw accelerometer prediction 是：

$$
\mathbf x_i
=
\mathbf R_{ib}\mathbf u_b.
$$

加入 accelerometer scale/misalignment 后：

$$
\boxed{
\hat{\mathbf a}
=
\mathbf M_{\mathrm{acc}}\mathbf x_i
+
\mathbf b_a
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf u_b
+
\mathbf b_a.
}
$$

源码对应：

```python
M = self.M_accel_Dv.toExpression()
a = M * (C_i_b * (...))
aerr = EuclideanError(im.alpha, ..., a + b_i)
```

按第 6、7 章的习惯，先写这个扩展模型的 variation 母式，下面两个小节都只是从母式取分支。令 $\mathbf x_i=\mathbf R_{ib}\mathbf u_b$，则预测量是 $\hat{\mathbf a}=\mathbf M_{\mathrm{acc}}\mathbf x_i+\mathbf b_a$。$\mathbf M_{\mathrm{acc}}$ 是普通欧式矩阵参数，扰动就是逐元素相加；$\mathbf x_i$ 的扰动 $\delta\mathbf x_i$ 已经由 7.6 节的母式给出——pose、gravity、外参旋转、lever arm 的全部贡献都汇在它里面。特别是 $\mathbf u_b$ 内部仍然沿用第 7 章的分解：

$$
\delta\mathbf u_b
=
\delta\mathbf h_b+\delta\boldsymbol\ell_b,
$$

其中 $\delta\mathbf h_b$ 给出 specific-force 分支，$\delta\boldsymbol\ell_b$ 给出 lever-arm 分支。第 10 章不会重新展开这两支，只在它们外面再乘 IMU scale/misalignment、sensing rotation 或 acceleration sensitivity 矩阵。

对乘积 $\mathbf M_{\mathrm{acc}}\mathbf x_i$ 用乘积法则：

$$
\boxed{
\delta\mathbf e^a
=
\delta\mathbf M_{\mathrm{acc}}\,\mathbf x_i
+
\mathbf M_{\mathrm{acc}}\,\delta\mathbf x_i
+
\delta\mathbf b_a.
}
$$

三项各有去向：第一项是 10.3.2 的 matrix Jacobian；第二项解释了 10.3.1 的“所有旧分支左乘 $\mathbf M_{\mathrm{acc}}$”；第三项说明 bias 分支保持第 7 章原样——bias 加在 $\mathbf M_{\mathrm{acc}}$ 外面，所以不被左乘。

### 10.3.1 对旧变量的影响

母式第二项 $\mathbf M_{\mathrm{acc}}\,\delta\mathbf x_i$ 说明：凡是原来通过 $\mathbf x_i=\mathbf R_{ib}\mathbf u_b$ 进入 residual 的变量，Jacobian 左侧都多乘一个 $\mathbf M_{\mathrm{acc}}$。

例如 pose 控制点：

$$
\boxed{
\mathbf J_{\mathbf e^a,\mathbf c_j}
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}
\mathbf J_{\mathbf u_b,\mathbf c_j}.
}
$$

Gravity：

$$
\boxed{
\mathbf J_{\mathbf e^a,\mathbf g_w}
=
-
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf R_{bw}.
}
$$

IMU 外参旋转：

$$
\boxed{
\mathbf J_{\mathbf e^a,\boldsymbol\phi_{ib,K}}
=
\mathbf M_{\mathrm{acc}}
[\mathbf R_{ib}\mathbf u_b]_\times.
}
$$

Lever arm：

$$
\boxed{
\mathbf J_{\mathbf e^a,\mathbf r_b}
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}
\left(
[\boldsymbol\alpha_b]_\times
+
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times
\right).
}
$$

Accel bias 仍然是直接相加：

$$
\boxed{
\mathbf J_{\mathbf e^a,\mathbf d^a_{j+\ell}}
=
\nu_\ell^{(0)}(t_k)\mathbf I_3.
}
$$

### 10.3.2 对 $\mathbf M_{\mathrm{acc}}$ 本身的 Jacobian

令：

$$
\mathbf x_i=\mathbf R_{ib}\mathbf u_b.
$$

对 $\mathbf M_{\mathrm{acc}}$ 求导时，保持 $\mathbf x_i$ 和 bias 不变：

$$
\delta\mathbf e^a
=
\delta\mathbf M_{\mathrm{acc}}\mathbf x_i.
$$

因此 full matrix Jacobian 是：

$$
\boxed{
\frac{\partial\mathbf e^a}
{\partial\operatorname{vec}(\mathbf M_{\mathrm{acc}})}
=
\begin{bmatrix}
x_{i,1}\mathbf I_3&
x_{i,2}\mathbf I_3&
x_{i,3}\mathbf I_3
\end{bmatrix}.
}
$$

实际进入线性系统的列数由 `MatrixBasicDv` 的 active mask 决定。

## 10.4 扩展 gyro 模型

普通 gyro 模型只使用 body angular velocity：

$$
\hat{\boldsymbol\omega}
=
\mathbf R_{ib}\boldsymbol\omega_b+\mathbf b_g.
$$

扩展模型加入三件事：

1. Gyro sensing frame 可能和 IMU frame 有一个小旋转 $\mathbf R_{gi}$。
2. Gyro 三轴可能有 scale/misalignment，记为 $\mathbf M_g$。
3. Gyro 读数可能受线加速度影响，记为 acceleration sensitivity matrix $\mathbf A_g$。

先定义两个中间量：

$$
\boxed{
\boldsymbol\omega_g
=
\mathbf R_{gi}\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_g
=
\mathbf R_{gi}\mathbf R_{ib}\mathbf u_b.
}
$$

扩展 gyro prediction 是：

$$
\boxed{
\hat{\boldsymbol\omega}
=
\mathbf M_g\boldsymbol\omega_g
+
\mathbf A_g\mathbf a_g
+
\mathbf b_g.
}
$$

源码对应：

```python
C_i_b = self.q_i_b_Dv.toExpression()
C_gyro_i = self.q_gyro_i_Dv.toExpression()
C_gyro_b = C_gyro_i * C_i_b
M = self.M_gyro_Dv.toExpression()
Ma = self.M_accel_gyro_Dv.toExpression()
w = M * (C_gyro_b * w_b) + Ma * (C_gyro_b * a_b)
```

这里源码变量 `Ma` 对应本章的 $\mathbf A_g$，不是 accelerometer scale matrix $\mathbf M_{\mathrm{acc}}$。

和 10.3 一样，先写 variation 母式。预测量 $\hat{\boldsymbol\omega}=\mathbf M_g\boldsymbol\omega_g+\mathbf A_g\mathbf a_g+\mathbf b_g$ 有五个会变的输入：$\mathbf M_g$、$\mathbf A_g$ 是欧式矩阵参数；$\boldsymbol\omega_g$、$\mathbf a_g$ 是复合中间量，它们的扰动再由 $\mathbf R_{gi}$、$\mathbf R_{ib}$、pose 控制点和 lever arm 诱导；$\mathbf b_g$ 是 bias spline 值。对两个乘积分别用乘积法则，再把所有项相加：

$$
\boxed{
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
}
$$

这正是 6.13 节预告过的式子。后面四个小节按变量块把它拆开：10.4.1 取 $\delta\mathbf M_g$、$\delta\mathbf A_g$ 两个矩阵分支；10.4.2 把 $\delta\boldsymbol\omega_g$、$\delta\mathbf a_g$ 沿 $\mathbf R_{gi}$ 展开；10.4.3 沿 $\mathbf R_{ib}$ 展开；10.4.4 沿 pose 控制点、lever arm 和 bias 展开。每个小节求偏导时，其余变量块固定。

### 10.4.1 对 $\mathbf M_g$ 和 $\mathbf A_g$

对 gyro scale/misalignment：

$$
\delta\mathbf e^\omega
=
\delta\mathbf M_g\,\boldsymbol\omega_g.
$$

所以：

$$
\boxed{
\frac{\partial\mathbf e^\omega}
{\partial\operatorname{vec}(\mathbf M_g)}
=
\begin{bmatrix}
\omega_{g,1}\mathbf I_3&
\omega_{g,2}\mathbf I_3&
\omega_{g,3}\mathbf I_3
\end{bmatrix}.
}
$$

对 acceleration sensitivity：

$$
\delta\mathbf e^\omega
=
\delta\mathbf A_g\,\mathbf a_g.
$$

所以：

$$
\boxed{
\frac{\partial\mathbf e^\omega}
{\partial\operatorname{vec}(\mathbf A_g)}
=
\begin{bmatrix}
a_{g,1}\mathbf I_3&
a_{g,2}\mathbf I_3&
a_{g,3}\mathbf I_3
\end{bmatrix}.
}
$$

同样，$\mathbf M_g$ 的实际列数由 lower-triangular mask 决定，$\mathbf A_g$ 通常是 full $3\times3$ active matrix。

### 10.4.2 对 gyro sensing rotation $\mathbf R_{gi}$

对 $\mathbf R_{gi}$ 使用 Kalibr rotation tangent：

$$
\mathbf R_{gi}^+
=
\mathrm{Exp}(-\delta\boldsymbol\phi_{gi,K})\mathbf R_{gi}.
$$

令：

$$
\boldsymbol\omega_i=\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_i=\mathbf R_{ib}\mathbf u_b.
$$

则：

$$
\boldsymbol\omega_g=\mathbf R_{gi}\boldsymbol\omega_i,
\qquad
\mathbf a_g=\mathbf R_{gi}\mathbf a_i.
$$

由第 0 章的 rotation action Jacobian：

$$
\delta(\mathbf R_{gi}\boldsymbol\omega_i)
=
[\boldsymbol\omega_g]_\times
\delta\boldsymbol\phi_{gi,K},
$$

$$
\delta(\mathbf R_{gi}\mathbf a_i)
=
[\mathbf a_g]_\times
\delta\boldsymbol\phi_{gi,K}.
$$

代入扩展 gyro prediction：

$$
\delta\mathbf e^\omega
=
\mathbf M_g[\boldsymbol\omega_g]_\times\delta\boldsymbol\phi_{gi,K}
+
\mathbf A_g[\mathbf a_g]_\times\delta\boldsymbol\phi_{gi,K}.
$$

因此：

$$
\boxed{
\mathbf J_{\mathbf e^\omega,\boldsymbol\phi_{gi,K}}
=
\mathbf M_g[\boldsymbol\omega_g]_\times
+
\mathbf A_g[\mathbf a_g]_\times.
}
$$

### 10.4.3 对 IMU 外参旋转 $\mathbf R_{ib}$

现在扰动 $\mathbf R_{ib}$，保持 $\mathbf R_{gi}$、$\mathbf M_g$、$\mathbf A_g$、$\boldsymbol\omega_b$、$\mathbf u_b$ 不变。令：

$$
\boldsymbol\omega_i=\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_i=\mathbf R_{ib}\mathbf u_b.
$$

则：

$$
\delta\boldsymbol\omega_i
=
[\boldsymbol\omega_i]_\times
\delta\boldsymbol\phi_{ib,K},
\qquad
\delta\mathbf a_i
=
[\mathbf a_i]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

继续左乘 $\mathbf R_{gi}$：

$$
\delta\boldsymbol\omega_g
=
\mathbf R_{gi}[\boldsymbol\omega_i]_\times
\delta\boldsymbol\phi_{ib,K},
\qquad
\delta\mathbf a_g
=
\mathbf R_{gi}[\mathbf a_i]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

所以：

$$
\boxed{
\mathbf J_{\mathbf e^\omega,\boldsymbol\phi_{ib,K}}
=
\mathbf M_g\mathbf R_{gi}
[\mathbf R_{ib}\boldsymbol\omega_b]_\times
+
\mathbf A_g\mathbf R_{gi}
[\mathbf R_{ib}\mathbf u_b]_\times.
}
$$

这个式子是第 8 章普通 gyro 外参旋转 Jacobian 的扩展版。若 $\mathbf M_g=\mathbf I$、$\mathbf R_{gi}=\mathbf I$、$\mathbf A_g=\mathbf 0$，它退化为：

$$
[\mathbf R_{ib}\boldsymbol\omega_b]_\times.
$$

### 10.4.4 对 pose、lever arm 和 gyro bias

对 pose 控制点，扩展 gyro residual 同时通过角速度和 acceleration sensitivity 两条路径依赖轨迹：

$$
\boxed{
\mathbf J_{\mathbf e^\omega,\mathbf c_j}
=
\mathbf M_g\mathbf R_{gi}\mathbf R_{ib}
\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}
+
\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}
\mathbf J_{\mathbf u_b,\mathbf c_j}.
}
$$

对 lever arm，普通 gyro 没有依赖，但 acceleration sensitivity 分支通过 $\mathbf u_b$ 依赖 $\mathbf r_b$：

$$
\boxed{
\mathbf J_{\mathbf e^\omega,\mathbf r_b}
=
\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}
\left(
[\boldsymbol\alpha_b]_\times
+
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times
\right).
}
$$

对 gyro bias 控制点仍然是：

$$
\boxed{
\mathbf J_{\mathbf e^\omega,\mathbf d^g_{j+\ell}}
=
\mu_\ell^{(0)}(t_k)\mathbf I_3.
}
$$

## 10.5 Accelerometer size-effect

`IccScaledMisalignedSizeEffectImu` 进一步考虑一个细节：三根 accelerometer sensing axis 不一定在同一个物理点上。源码给每根轴一个 offset：

$$
\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i\in\mathbb R^3,
$$

它们表达在 IMU frame 中。转换到 body frame：

$$
\boxed{
\mathbf r_x^b
=
\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_x^i,
\quad
\mathbf r_y^b
=
\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_y^i,
\quad
\mathbf r_z^b
=
\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_z^i.
}
$$

源码对应：

```python
rx_b = r_b + C_i_b.inverse() * rx_i
ry_b = r_b + C_i_b.inverse() * ry_i
rz_b = r_b + C_i_b.inverse() * rz_i
```

定义 lever-arm acceleration 函数：

$$
\boldsymbol\ell(\mathbf r)
=
\boldsymbol\alpha_b\times\mathbf r
+
\boldsymbol\omega_b\times
\left(
\boldsymbol\omega_b\times\mathbf r
\right).
$$

并定义固定 selector：

$$
\mathbf I_x=\mathrm{diag}(1,0,0),
\quad
\mathbf I_y=\mathrm{diag}(0,1,0),
\quad
\mathbf I_z=\mathrm{diag}(0,0,1).
$$

Size-effect accelerometer prediction 是：

$$
\boxed{
\hat{\mathbf a}
=
\mathbf M_{\mathrm{acc}}
\left[
\mathbf R_{ib}\mathbf h_b
+
\mathbf I_x\mathbf R_{ib}\boldsymbol\ell(\mathbf r_x^b)
+
\mathbf I_y\mathbf R_{ib}\boldsymbol\ell(\mathbf r_y^b)
+
\mathbf I_z\mathbf R_{ib}\boldsymbol\ell(\mathbf r_z^b)
\right]
+
\mathbf b_a,
}
$$

其中：

$$
\mathbf h_b=\mathbf R_{bw}(\mathbf a_w-\mathbf g_w).
$$

### 10.5.1 对 axis-specific offset 的 Jacobian

先只看 $\mathbf r_x^i$。它只影响 $\mathbf r_x^b$：

$$
\delta\mathbf r_x^b
=
\mathbf R_{ib}^{-1}\delta\mathbf r_x^i.
$$

而：

$$
\delta\boldsymbol\ell(\mathbf r_x^b)
=
\mathbf A_r(\mathbf r_x^b)\delta\mathbf r_x^b,
$$

其中：

$$
\mathbf A_r(\mathbf r)
=
[\boldsymbol\alpha_b]_\times
+
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times.
$$

注意这个矩阵本身不依赖 $\mathbf r$，但写成 $\mathbf A_r(\mathbf r)$ 可以提醒读者它对应哪条 lever-arm 分支。

因此：

$$
\boxed{
\frac{\partial\mathbf e^a}{\partial\mathbf r_x^i}
=
\mathbf M_{\mathrm{acc}}
\mathbf I_x
\mathbf R_{ib}
\mathbf A_r
\mathbf R_{ib}^{-1}.
}
$$

同理：

$$
\boxed{
\frac{\partial\mathbf e^a}{\partial\mathbf r_y^i}
=
\mathbf M_{\mathrm{acc}}
\mathbf I_y
\mathbf R_{ib}
\mathbf A_r
\mathbf R_{ib}^{-1},
\qquad
\frac{\partial\mathbf e^a}{\partial\mathbf r_z^i}
=
\mathbf M_{\mathrm{acc}}
\mathbf I_z
\mathbf R_{ib}
\mathbf A_r
\mathbf R_{ib}^{-1}.
}
$$

源码中 `rx_i_Dv` 被设为 inactive，`ry_i_Dv` 和 `rz_i_Dv` active。因此默认实际优化的是 $y,z$ 两根轴相对 $x$ 轴的 size-effect offset。

### 10.5.2 对 base lever arm $\mathbf r_b$

Base lever arm $\mathbf r_b$ 同时进入三条 axis-specific branch：

$$
\delta\mathbf r_x^b
=
\delta\mathbf r_y^b
=
\delta\mathbf r_z^b
=
\delta\mathbf r_b.
$$

所以：

$$
\boxed{
\frac{\partial\mathbf e^a}{\partial\mathbf r_b}
=
\mathbf M_{\mathrm{acc}}
\left(
\mathbf I_x\mathbf R_{ib}\mathbf A_r
+
\mathbf I_y\mathbf R_{ib}\mathbf A_r
+
\mathbf I_z\mathbf R_{ib}\mathbf A_r
\right).
}
$$

由于本章的 $\mathbf A_r=[\boldsymbol\alpha_b]_\times+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times$ 不依赖具体 lever-arm 值，且三个 selector 相加为单位阵，上式可以直接化简成：

$$
\boxed{
\frac{\partial\mathbf e^a}{\partial\mathbf r_b}
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r.
}
$$

Size-effect 新增的不是 base lever arm Jacobian 的形式，而是三条 axis-specific offset 分支。

### 10.5.3 对 IMU 外参旋转的额外项

Size-effect 下，$\mathbf R_{ib}$ 出现了两次：

1. 直接把 body-frame 向量旋到 IMU frame。
2. 通过 $\mathbf R_{ib}^{-1}\mathbf r_j^i$ 改变 axis-specific lever arm 的 body-frame 表达。

这比普通模型多一条间接链路。对第 $j$ 条轴，记：

$$
\mathbf r_j^i\in\{\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i\},
\qquad
\mathbf I_j\in\{\mathbf I_x,\mathbf I_y,\mathbf I_z\},
$$

$$
\mathbf r_j^b
=
\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_j^i,
\qquad
\boldsymbol\ell_j
=
\boldsymbol\ell(\mathbf r_j^b).
$$

直接旋转项给出：

$$
\mathbf I_j[\mathbf R_{ib}\boldsymbol\ell_j]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

间接 lever-arm 变化来自：

$$
\delta(\mathbf R_{ib}^{-1}\mathbf r_j^i)
=
-
\mathbf R_{ib}^{-1}
[\mathbf r_j^i]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

所以：

$$
\delta\boldsymbol\ell_j
=
-
\mathbf A_r
\mathbf R_{ib}^{-1}
[\mathbf r_j^i]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

乘回输出：

$$
\mathbf I_j\mathbf R_{ib}\delta\boldsymbol\ell_j
=
-
\mathbf I_j\mathbf R_{ib}\mathbf A_r
\mathbf R_{ib}^{-1}
[\mathbf r_j^i]_\times
\delta\boldsymbol\phi_{ib,K}.
$$

再加上 base acceleration $\mathbf h_b$ 的直接旋转项，得到：

$$
\boxed{
\begin{aligned}
\mathbf J_{\mathbf e^a,\boldsymbol\phi_{ib,K}}
&=
\mathbf M_{\mathrm{acc}}
\Bigg(
[\mathbf R_{ib}\mathbf h_b]_\times
\\
&\quad+
\sum_{j\in\{x,y,z\}}
\mathbf I_j
\left[
[\mathbf R_{ib}\boldsymbol\ell_j]_\times
-
\mathbf R_{ib}\mathbf A_r
\mathbf R_{ib}^{-1}
[\mathbf r_j^i]_\times
\right]
\Bigg).
\end{aligned}
}
$$

如果 $\mathbf r_x^i=\mathbf r_y^i=\mathbf r_z^i=\mathbf 0$，间接项消失，三条 selector 加起来退化回普通 scale/misalignment accelerometer 的旋转 Jacobian：

$$
\mathbf M_{\mathrm{acc}}[\mathbf R_{ib}\mathbf u_b]_\times.
$$

## 10.6 源码桥

| 数学对象 | 源码变量 | 说明 |
|---|---|---|
| $\mathbf M_{\mathrm{acc}}$ | `M_accel_Dv` | lower-triangular active mask |
| $\mathbf M_g$ | `M_gyro_Dv` | lower-triangular active mask |
| $\mathbf A_g$ | `M_accel_gyro_Dv` / `Ma` | full $3\times3$ active matrix，gyro acceleration sensitivity |
| $\mathbf R_{gi}$ | `q_gyro_i_Dv`, `C_gyro_i` | IMU frame 到 gyro sensing frame |
| $\mathbf R_{gi}\mathbf R_{ib}$ | `C_gyro_b = C_gyro_i * C_i_b` | body 到 gyro sensing frame |
| $\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i$ | `rx_i_Dv`, `ry_i_Dv`, `rz_i_Dv` | size-effect axis offsets |
| $\mathbf I_x,\mathbf I_y,\mathbf I_z$ | `Ix_Dv`, `Iy_Dv`, `Iz_Dv` | fixed selectors |

第 10 章所有 matrix 参数的 Jacobian 先按 full $3\times3$ 写。源码实际列数由 `MatrixBasicDv` 的 mask 决定。读优化矩阵时，如果发现列数不是 $9$，先看该 matrix design variable 的 active mask。

### 10.6.1 Ceres 实现映射

`ceres_cam_imu` 把本章的扩展参数放在 `variables/imu_intrinsics.*` 中，而不是塞进普通 IMU 外参块。这样可以保持三层边界清楚：

1. `CameraExtrinsicBlock` 只表示 body/IMU 到 camera 的刚体外参。
2. `GravityBlock`、pose spline 和 bias spline 仍提供第 6、7 章的运动学中间量。
3. `ImuIntrinsicBlocks` 只表示本章新增的 scale、misalignment、gyro sensing rotation、acceleration sensitivity 和 size-effect axis offsets。

当前 CLI 用 `--imu-model` 选择三种前向模型：

| `--imu-model` | 前向模型 |
|---|---|
| `calibrated` | 第 6、7 章普通模型，对应 Kalibr `IccImu` |
| `scale-misalignment` | 本章 10.3 和 10.4，对应 `IccScaledMisalignedImu` |
| `scale-misalignment-size-effect` | 10.3、10.4、10.5 全部打开，对应 `IccScaledMisalignedSizeEffectImu` |

矩阵参数在 Ceres 中有两个约定。$\mathbf M_{\mathrm{acc}}$ 和 $\mathbf M_g$ 用 6 维下三角块存储，展开顺序是

$$
[m_{00},m_{10},m_{11},m_{20},m_{21},m_{22}]
\quad\longrightarrow\quad
\begin{bmatrix}
m_{00}&0&0\\
m_{10}&m_{11}&0\\
m_{20}&m_{21}&m_{22}
\end{bmatrix}.
$$

$\mathbf A_g$ 用完整 9 维 row-major 块存储。$\mathbf R_{gi}$ 用 3 维旋转向量存储，预测时通过 $\operatorname{Exp}$ 变成旋转矩阵。Size-effect 的 $\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i$ 是三个 3 维向量；默认固定 $\mathbf r_x^i$，只释放后两根轴，避免 size-effect gauge 过早污染普通外参平移。

前向预测集中在 `residuals/imu_model.*`：

$$
\hat{\boldsymbol\omega}
=
\mathbf M_g\mathbf R_{gi}\mathbf R_{ib}\boldsymbol\omega_b
+
\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf u_b
+
\mathbf b_g,
$$

$$
\hat{\mathbf a}
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}(\mathbf h_b+\boldsymbol\ell_b)
+
\mathbf b_a,
$$

以及 size-effect 的 axis-specific lever arm 版本。`optimizer/calibration_problem.*` 根据模型选择 residual 分支并添加对应参数块；`optimizer/residual_statistics.*` 使用同一套 prediction，保证结果文件里的统计和优化问题里的前向模型一致。

需要分清当前实现状态。普通 `calibrated` gyro/accelerometer residual 和扩展 IMU residual 都已经是手写解析 `SizedCostFunction`。扩展分支接入了变量块、前向模型、problem builder、stage snapshot、统计和 sweep summary；10.3、10.4、10.5 的 Jacobian 链分别落在 `gyroscope_residual.cpp` 的 `ScaleMisalignedGyroscopeCost`、`accelerometer_residual.cpp` 的 `ScaleMisalignedAccelerometerCost` 和 `SizeEffectAccelerometerCost` 中。新增测试先把 Ceres prediction 与 Kalibr 源码公式直接展开结果比较到 `1e-12`，再用中心差分复核每个参数块 Jacobian。

这组实现检查只证明模型级一致性和局部线性化正确性。全量扩展 IMU 标定还需要另一组实验：用带 `scale-misalignment` 或 `scale-misalignment-size-effect` 的 Kalibr 配置跑出 Docker 结果，再与 Ceres 同口径全量优化比较外参、time shift、IMU intrinsic 和 residual。

## 10.7 速查表

### 10.7.1 Accelerometer scale/misalignment

令：

$$
\mathbf x_i=\mathbf R_{ib}\mathbf u_b.
$$

| 变量块 | Jacobian |
|---|---|
| pose 控制点 $\mathbf c_j$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ |
| gravity $\mathbf g_w$ | $-\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf R_{bw}$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_{\mathrm{acc}}[\mathbf x_i]_\times$ |
| lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ |
| accel bias 控制点 | $\nu_\ell^{(0)}\mathbf I_3$ |
| $\operatorname{vec}(\mathbf M_{\mathrm{acc}})$ | $[x_{i,1}\mathbf I_3\ x_{i,2}\mathbf I_3\ x_{i,3}\mathbf I_3]$ |

### 10.7.2 Extended gyro

令：

$$
\boldsymbol\omega_g=\mathbf R_{gi}\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_g=\mathbf R_{gi}\mathbf R_{ib}\mathbf u_b.
$$

| 变量块 | Jacobian |
|---|---|
| pose 控制点 $\mathbf c_j$ | $\mathbf M_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}+\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_g\mathbf R_{gi}[\mathbf R_{ib}\boldsymbol\omega_b]_\times+\mathbf A_g\mathbf R_{gi}[\mathbf R_{ib}\mathbf u_b]_\times$ |
| gyro sensing rotation $\mathbf R_{gi}$ | $\mathbf M_g[\boldsymbol\omega_g]_\times+\mathbf A_g[\mathbf a_g]_\times$ |
| lever arm $\mathbf r_b$ | $\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf A_r$ |
| gyro bias 控制点 | $\mu_\ell^{(0)}\mathbf I_3$ |
| $\operatorname{vec}(\mathbf M_g)$ | $[\omega_{g,1}\mathbf I_3\ \omega_{g,2}\mathbf I_3\ \omega_{g,3}\mathbf I_3]$ |
| $\operatorname{vec}(\mathbf A_g)$ | $[a_{g,1}\mathbf I_3\ a_{g,2}\mathbf I_3\ a_{g,3}\mathbf I_3]$ |

### 10.7.3 Size-effect

令：

$$
\mathbf r_j^b=\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_j^i,
\qquad
\boldsymbol\ell_j=\boldsymbol\ell(\mathbf r_j^b),
\qquad
j\in\{x,y,z\}.
$$

| 变量块 | Jacobian |
|---|---|
| axis offset $\mathbf r_j^i$ | $\mathbf M_{\mathrm{acc}}\mathbf I_j\mathbf R_{ib}\mathbf A_r\mathbf R_{ib}^{-1}$ |
| base lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | 见 10.5.3 的展开式 |

## 10.8 常见混淆

第一，源码里的 `Ma` 不是 accelerometer scale matrix。它是 gyro acceleration sensitivity matrix，本章记为 $\mathbf A_g$。Accelerometer scale/misalignment 本章记为 $\mathbf M_{\mathrm{acc}}$。

第二，scale/misalignment matrix 的 full Jacobian 是 $3\times9$，但实际优化列数取决于 mask。Lower-triangular mask 会减少自由度。

第三，$\mathbf R_{gi}$ 和 $\mathbf R_{ib}$ 是不同旋转。$\mathbf R_{ib}$ 是 body 到 IMU，属于 sensor extrinsic；$\mathbf R_{gi}$ 是 IMU 到 gyro sensing frame，属于 gyro 内参模型。

第四，size-effect 的 $\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i$ 表达在 IMU frame，不是 body frame。进入刚体加速度公式前要先变成 $\mathbf R_{ib}^{-1}\mathbf r_j^i$，再加到 $\mathbf r_b$ 上。

第五，扩展 IMU 模型不改变 residual 方向、whitening 或 robust kernel。它只改变 predicted measurement 的 forward chain 和相应 Jacobian。信息矩阵、robust weight 和 LM damping 仍按第 3 章的规则进入线性系统。

## 10.9 本章小结

扩展 IMU 模型本质上是在普通 gyro / accel prediction 外面加矩阵、额外旋转和额外 lever-arm 分支。矩阵参数的核心 Jacobian 是：

$$
\frac{\partial(\mathbf M\mathbf x)}
{\partial\operatorname{vec}(\mathbf M)}
=
\begin{bmatrix}
x_1\mathbf I_3&
x_2\mathbf I_3&
x_3\mathbf I_3
\end{bmatrix}.
$$

Accelerometer scale/misalignment 会把第 7 章所有 raw accelerometer 分支左乘 $\mathbf M_{\mathrm{acc}}$，并新增对 $\mathbf M_{\mathrm{acc}}$ 的 matrix Jacobian。扩展 gyro 模型把 angular velocity 分支和 acceleration sensitivity 分支相加，因此对 pose、外参旋转、lever arm 和 matrix 参数的 Jacobian 都是两条链路的和。Size-effect 则把一个 lever arm 拆成三根 axis-specific lever arm，额外引入 $\mathbf r_x^i,\mathbf r_y^i,\mathbf r_z^i$ 以及 $\mathbf R_{ib}^{-1}$ 带来的间接旋转项。

下一章会从公式回到实现：Kalibr expression graph 如何把这些局部节点组合成完整 error term，优化器又如何收集 design variable 和 Jacobian block。
