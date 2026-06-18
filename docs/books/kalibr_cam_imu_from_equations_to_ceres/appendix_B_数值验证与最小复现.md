# 附录 B：数值验证与最小复现

前面章节给出了 analytic Jacobian。真正写代码复现时，最容易出错的不是公式看不懂，而是 finite difference 使用了不同扰动约定。本附录给出一套最小验证路线：先固定 perturbation，再用数值差分逐项检查。

## B.1 验证的基本形式

设 residual 是：

$$
\mathbf e(\boldsymbol\theta)\in\mathbb R^m.
$$

对某个变量块 $\boldsymbol\theta_i$ 的 analytic Jacobian 是：

$$
\mathbf J_i
=
\frac{\partial\mathbf e}{\partial\delta\boldsymbol\theta_i}
\in\mathbb R^{m\times d_i}.
$$

数值差分用第 $j$ 个 basis direction：

$$
\mathbf e_j=
\begin{bmatrix}
0&\cdots&1&\cdots&0
\end{bmatrix}^\top.
$$

中心差分为：

$$
\boxed{
\mathbf J_i(:,j)
\approx
\frac{
\mathbf e(\boldsymbol\theta_i\boxplus \epsilon\mathbf e_j)
-
\mathbf e(\boldsymbol\theta_i\boxplus -\epsilon\mathbf e_j)
}{2\epsilon}.
}
$$

关键是 $\boxplus$ 必须和 analytic Jacobian 的 tangent convention 一致。本书默认检查 Kalibr expression tangent：

$$
\mathbf R\boxplus_K\delta\boldsymbol\phi
=
\mathrm{Exp}(-\delta\boldsymbol\phi)\mathbf R.
$$

Transformation 的 Kalibr tangent 需要使用第 0 章的桥接矩阵，或者直接在复现里实现与 Kalibr `TransformationDv` 相同的更新。若只是验证第 4 章的 `boxMinus` / `boxTimes` 局部 Jacobian，可以先直接使用第 0 章给出的 Kalibr tangent 公式。

## B.2 推荐数值尺度

对 double precision，通常可以从：

$$
\epsilon=10^{-6}
$$

开始。若 residual 尺度很大或函数包含归一化，可以试：

$$
10^{-7}\le\epsilon\le10^{-5}.
$$

检查误差时不要只看绝对误差。推荐同时看：

$$
\|\mathbf J_{\mathrm{ana}}-\mathbf J_{\mathrm{num}}\|_\infty
$$

和相对误差：

$$
\frac{
\|\mathbf J_{\mathrm{ana}}-\mathbf J_{\mathrm{num}}\|_\infty
}{
\max(1,\|\mathbf J_{\mathrm{num}}\|_\infty)
}.
$$

如果绝对误差很小但相对误差很大，通常说明该列接近零；如果相对误差稳定很大，通常是符号、扰动方向或 residual 方向错了。

## B.3 SO(3) rotation action

最小函数：

$$
\mathbf y(\mathbf R)=\mathbf R\mathbf x.
$$

Kalibr 扰动：

$$
\mathbf R^+
=
\mathrm{Exp}(-\delta\boldsymbol\phi)\mathbf R.
$$

Analytic Jacobian：

$$
\boxed{
\frac{\partial(\mathbf R\mathbf x)}
{\partial\delta\boldsymbol\phi_K}
=
[\mathbf R\mathbf x]_\times.
}
$$

验证步骤：

1. 随机生成 $\mathbf R$ 和 $\mathbf x$。
2. 对三个方向分别计算 $\mathbf R^\pm=\mathrm{Exp}(\mp\epsilon\mathbf e_j)\mathbf R$。
3. 计算中心差分。
4. 对比 $[\mathbf R\mathbf x]_\times$。

如果得到负号，通常是 finite difference 用了 Micro left perturbation 而不是 Kalibr perturbation。

## B.4 SE(3) transform action

对 homogeneous point：

$$
\tilde{\mathbf y}
=
\mathbf T\tilde{\mathbf p},
\qquad
\tilde{\mathbf p}=
\begin{bmatrix}
\mathbf p\\w
\end{bmatrix}.
$$

Kalibr expression tangent 下，第 0 章得到：

$$
\boxed{
\delta\mathbf y
=
w\delta\boldsymbol\rho_K
+
[\mathbf y]_\times\delta\boldsymbol\phi_K.
}
$$

所以局部 Jacobian 是：

$$
\boxed{
\frac{\partial\mathbf y}
{\partial\delta\boldsymbol\xi_K}
=
\begin{bmatrix}
w\mathbf I_3&[\mathbf y]_\times
\end{bmatrix}.
}
$$

验证时只取前三维 $\mathbf y$。若 $w=1$，对应普通三维点；若 $w=0$，对应方向向量，translation block 应该为零。

## B.5 Transform product 和 inverse

Product：

$$
\mathbf T=\mathbf A\mathbf B.
$$

Kalibr expression graph 的局部规则是：

$$
\delta\boldsymbol\xi_{\mathbf T,K}
=
\delta\boldsymbol\xi_{\mathbf A,K}
+
\mathrm{boxTimes}(\mathbf A)
\delta\boldsymbol\xi_{\mathbf B,K}.
$$

因此要分别验证：

$$
\frac{\partial\boldsymbol\xi_{\mathbf T,K}}
{\partial\boldsymbol\xi_{\mathbf A,K}}
=
\mathbf I_6,
\qquad
\frac{\partial\boldsymbol\xi_{\mathbf T,K}}
{\partial\boldsymbol\xi_{\mathbf B,K}}
=
\mathrm{boxTimes}(\mathbf A).
$$

Inverse：

$$
\mathbf S=\mathbf T^{-1}
$$

的局部规则是：

$$
\boxed{
\frac{\partial\boldsymbol\xi_{\mathbf S,K}}
{\partial\boldsymbol\xi_{\mathbf T,K}}
=
-
\mathrm{boxTimes}(\mathbf S).
}
$$

验证这些 $6\times6$ Jacobian 时，需要一个 `boxminus_K` 把两个很接近的 transform 差值转成 Kalibr expression tangent。最小实现可以复用第 0 章的 $\mathbf S(\mathbf T)$ 桥接矩阵，把 Micro $\mathrm{Log}(\mathbf T_2\mathbf T_1^{-1})$ 的结果转换到 Kalibr tangent。

## B.6 Camera reprojection residual

最小 forward chain：

$$
\mathbf p_c
=
\mathbf R_{c b}\mathbf R_{bw}(\mathbf p_w-\mathbf t_{wb})
+
\mathbf t_{c b},
$$

$$
\hat{\mathbf y}
=
\boldsymbol\pi(\mathbf p_c;\boldsymbol\eta),
\qquad
\mathbf e^\pi
=
\mathbf y-\hat{\mathbf y}.
$$

建议分层验证：

1. 只验证 pinhole projection：
   $$
   \mathbf q=(x/z,y/z).
   $$
2. 加 distortion，分别验证 $\mathbf J_{\mathbf d,\mathbf q}$ 和 $\mathbf J_{\mathbf d,\boldsymbol\kappa}$。
3. 加 intrinsics，验证 $\mathbf J_{\pi,\mathbf p_c}$。
4. 加 $\mathbf T_{c w}$，验证第 4 章的 $\mathbf A_T$。
5. 最后加 camera extrinsic、body pose inverse 和 time shift。

每一层都要保持 residual 方向：

$$
\delta\mathbf e^\pi=-\delta\hat{\mathbf y}.
$$

如果 projection Jacobian 对，但 pose Jacobian 差一个负号，优先检查 residual 方向和 $T_{wb}$ / $T_{bw}$ inverse 链。

## B.7 Gyro residual

普通 gyro residual：

$$
\mathbf e^\omega
=
\mathbf R_{ib}\boldsymbol\omega_b(t)
+
\mathbf b_g(t)
-
\mathbf z^\omega.
$$

可以先把 $\boldsymbol\omega_b(t)$ 当作一个普通三维输入，验证：

$$
\frac{\partial\mathbf e^\omega}{\partial\boldsymbol\omega_b}
=
\mathbf R_{ib},
$$

$$
\frac{\partial\mathbf e^\omega}{\partial\delta\boldsymbol\phi_{ib,K}}
=
[\mathbf R_{ib}\boldsymbol\omega_b]_\times,
$$

$$
\frac{\partial\mathbf e^\omega}{\partial\mathbf b_g}
=
\mathbf I_3.
$$

再把 $\boldsymbol\omega_b(t)$ 换成 pose spline 计算结果，验证：

$$
\mathbf J_{\mathbf e^\omega,\mathbf c_j}
=
\mathbf R_{ib}
\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}.
$$

Bias spline 控制点验证：

$$
\frac{\partial\mathbf e^\omega}
{\partial\mathbf d^g_{j+\ell}}
=
\mu_\ell^{(0)}(t)\mathbf I_3.
$$

## B.8 Accelerometer residual

普通 accelerometer residual：

$$
\mathbf e^a
=
\mathbf R_{ib}\mathbf u_b(t)
+
\mathbf b_a(t)
-
\mathbf z^a.
$$

先把 $\mathbf u_b$ 当作普通三维输入，验证：

$$
\frac{\partial\mathbf e^a}{\partial\mathbf u_b}
=
\mathbf R_{ib},
\qquad
\frac{\partial\mathbf e^a}{\partial\delta\boldsymbol\phi_{ib,K}}
=
[\mathbf R_{ib}\mathbf u_b]_\times.
$$

再验证 $\mathbf u_b$ 内部：

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

这一步对应第 7 章的分解 $\mathbf u_b=\mathbf h_b+\boldsymbol\ell_b$。数值验证时也建议分成两组：先验证 $\mathbf h_b=\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)$ 的 rotation/action Jacobian，再验证 $\boldsymbol\ell_b$ 里的 cross product Jacobian。

最容易错的是 cross product 的左右导数。建议分别验证：

$$
\frac{\partial(\boldsymbol\alpha_b\times\mathbf r_b)}
{\partial\boldsymbol\alpha_b}
=
-[\mathbf r_b]_\times,
\qquad
\frac{\partial(\boldsymbol\alpha_b\times\mathbf r_b)}
{\partial\mathbf r_b}
=
[\boldsymbol\alpha_b]_\times.
$$

以及：

$$
\frac{\partial}{\partial\mathbf r_b}
\left[
\boldsymbol\omega_b\times
(\boldsymbol\omega_b\times\mathbf r_b)
\right]
=
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times.
$$

## B.9 Matrix 参数和扩展 IMU

对所有 matrix-vector 扩展项，先验证通用节点：

$$
\mathbf y=\mathbf M\mathbf x.
$$

Full matrix Jacobian：

$$
\frac{\partial\mathbf y}
{\partial\operatorname{vec}(\mathbf M)}
=
\begin{bmatrix}
x_1\mathbf I_3&
x_2\mathbf I_3&
x_3\mathbf I_3
\end{bmatrix}.
$$

再按 `MatrixBasicDv` 的 mask 取列。若 analytic full matrix 对，但和源码 active block 对不上，通常是 mask 顺序或 vectorization 顺序没有对齐。

扩展 gyro 建议分两条链验证：

$$
\mathbf M_g\boldsymbol\omega_g,
\qquad
\mathbf A_g\mathbf a_g.
$$

两条链都通过后，再验证相加后的完整 residual。这样可以避免把 gyro scale 和 acceleration sensitivity 的列混在一起。

## B.10 Motion prior

Motion prior 的目标是：

$$
E=\mathbf c^\top\mathbf Q\mathbf c.
$$

它不走普通 residual/Jacobian 路径。验证方式有两种。

第一种，直接验证梯度方向。对任意小增量 $\delta\mathbf c$：

$$
E(\mathbf c+\epsilon\delta\mathbf c)
-
E(\mathbf c-\epsilon\delta\mathbf c)
\approx
4\epsilon\,\delta\mathbf c^\top\mathbf Q\mathbf c.
$$

第二种，构造平方根 residual。若：

$$
\mathbf Q=\mathbf S^\top\mathbf S,
$$

则：

$$
\mathbf e=\mathbf S\mathbf c,
\qquad
\mathbf J=\mathbf S.
$$

用普通 finite difference 验证 $\mathbf e$ 对 $\mathbf c$ 的 Jacobian 即可。实际代码中可以用 Cholesky、SVD 或 eigen decomposition 得到一个数值平方根。

## B.11 推荐复现顺序

不要一开始就复现完整 Kalibr。推荐按下面顺序：

1. `skew`, `ExpSO3`, `LogSO3`。
2. Kalibr rotation action Jacobian。
3. SE(3) transform action、`boxMinus`、`boxTimes`。
4. Pinhole projection 和一个 distortion model。
5. Camera residual 对 $\mathbf p_c$、$\mathbf T_{cw}$、$\mathbf T_{cb}$、$\mathbf T_{wb}$。
6. Euclidean B-spline value 和 derivative。
7. Gyro residual。
8. Accelerometer residual。
9. Matrix-vector 扩展 IMU。
10. Motion prior 二次型。

每一步都保留一个 finite-difference check。当前一步通过后，再接下一层。这样定位错误会比直接调完整优化问题快很多。

## B.12 当前 Ceres 实现的检查点

`ceres_cam_imu/tests/test_math.cpp` 已经把上述顺序落成一组轻量检查。它不是大规模标定回归，而是用小的、可控的状态验证 residual 和 hand Jacobian 的一致性。

当前覆盖：

| residual / 模块 | 检查内容 |
|---|---|
| camera reprojection | 对 `T_c_b`、camera time shift、6 个 pose control blocks 做中心差分，检查解析 Jacobian |
| gyroscope | 对 IMU extrinsic、6 个 pose control blocks、6 个 gyro bias control blocks 做中心差分 |
| accelerometer | 对 IMU lever/rotation、gravity、6 个 pose control blocks、6 个 accel bias control blocks 做中心差分 |
| extended IMU | 检查 scale/misalignment 与 size-effect 前向公式等价 Kalibr 源码展开式，并对扩展 gyro/accel 的 IMU intrinsic、axis offset、pose、bias、gravity、外参参数块做中心差分 |
| time shift prior | 检查 residual 数值和 1 维 Jacobian，并用通用差分器复核 |
| bias motion prior | 检查常值 bias 的 residual 为零，检查二次型与数值积分一致，再检查 6 个 bias control blocks 的 Jacobian |
| pose motion prior | 检查常值 pose 的 residual 为零，检查二阶 motion prior 与数值积分一致，再检查 6 个 pose control blocks 的 Jacobian |

推荐命令：

```bash
ctest --test-dir build --output-on-failure
```

如果只改了 residual/Jacobian 层，这个命令应该先通过，再跑全量 `current-full` 标定回归。否则优化结果变差时，无法区分是求解策略问题还是局部 Jacobian 错误。
