# 附录 A：从叉乘矩阵幂律到 Rodrigues 公式

第 0 章推导 $SE(3)$ 指数映射时，用到了一个上三角分块矩阵的幂律：

$$
\left(\delta\boldsymbol\xi_L^\wedge\right)^n
=
\begin{bmatrix}
\boldsymbol\Phi^n
&
\boldsymbol\Phi^{n-1}\delta\boldsymbol\rho_L\\
\mathbf 0^\top & 0
\end{bmatrix}.
$$

这个式子精彩的地方在于：复杂的指数映射不是靠记忆公式得到的，而是靠矩阵幂的结构一步步从级数里长出来。

$SO(3)$ 的 Rodrigues 公式也可以这样推。它的核心同样不是背公式，而是看清楚叉乘矩阵 $[\boldsymbol\omega]_\times$ 的幂有什么规律。

## A.1 先固定对象

令：

$$
\boldsymbol\omega
=
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
\in\mathbb R^3,
\qquad
\boldsymbol\Omega
\triangleq
[\boldsymbol\omega]_\times.
$$

按第 0 章的定义：

$$
\boldsymbol\Omega\mathbf a
=
\boldsymbol\omega\times\mathbf a.
$$

旋转向量 $\boldsymbol\omega$ 的长度记作：

$$
\theta
\triangleq
\|\boldsymbol\omega\|.
$$

如果 $\theta\ne 0$，也可以写成：

$$
\boldsymbol\omega
=
\theta\mathbf u,
\qquad
\|\mathbf u\|=1.
$$

这里 $\mathbf u$ 是旋转轴，$\theta$ 是旋转角。

## A.2 先算二次幂

要理解 $\boldsymbol\Omega^2$，最直接的方法是让它作用在任意向量 $\mathbf a$ 上：

$$
\boldsymbol\Omega^2\mathbf a
=
\boldsymbol\Omega(\boldsymbol\Omega\mathbf a)
=
\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf a).
$$

用向量三重积公式：

$$
\mathbf x\times(\mathbf y\times\mathbf z)
=
\mathbf y(\mathbf x^\top\mathbf z)
-
\mathbf z(\mathbf x^\top\mathbf y).
$$

令 $\mathbf x=\boldsymbol\omega$、$\mathbf y=\boldsymbol\omega$、$\mathbf z=\mathbf a$，得到：

$$
\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf a)
=
\boldsymbol\omega(\boldsymbol\omega^\top\mathbf a)
-
\mathbf a(\boldsymbol\omega^\top\boldsymbol\omega).
$$

因为 $\boldsymbol\omega^\top\boldsymbol\omega=\theta^2$，所以：

$$
\boldsymbol\Omega^2\mathbf a
=
\left(
\boldsymbol\omega\boldsymbol\omega^\top
-
\theta^2\mathbf I
\right)\mathbf a.
$$

这个式子对任意 $\mathbf a$ 都成立，因此：

$$
\boldsymbol\Omega^2
=
\boldsymbol\omega\boldsymbol\omega^\top
-
\theta^2\mathbf I.
$$

这一步给了一个重要直觉：叉乘矩阵的平方不再是叉乘，而是把向量分解到旋转轴方向和垂直方向的一种线性算子。

## A.3 再算三次幂

继续乘一次：

$$
\boldsymbol\Omega^3
=
\boldsymbol\Omega\boldsymbol\Omega^2
=
\boldsymbol\Omega
\left(
\boldsymbol\omega\boldsymbol\omega^\top
-
\theta^2\mathbf I
\right).
$$

展开：

$$
\boldsymbol\Omega^3
=
\boldsymbol\Omega\boldsymbol\omega\boldsymbol\omega^\top
-
\theta^2\boldsymbol\Omega.
$$

但：

$$
\boldsymbol\Omega\boldsymbol\omega
=
\boldsymbol\omega\times\boldsymbol\omega
=
\mathbf 0.
$$

所以第一项为零，得到：

$$
\boldsymbol\Omega^3
=
-
\theta^2\boldsymbol\Omega.
$$

这就是 Rodrigues 推导的关键闭合关系。它说明 $\boldsymbol\Omega$ 的高次幂不会产生无限多种新矩阵；三次幂又回到了 $\boldsymbol\Omega$ 本身，只是多了系数 $-\theta^2$。

## A.4 所有高次幂都会闭合

由：

$$
\boldsymbol\Omega^3
=
-
\theta^2\boldsymbol\Omega
$$

可以得到两个递推族。

奇数次幂：

$$
\boldsymbol\Omega^{2k+1}
=
(-1)^k\theta^{2k}\boldsymbol\Omega,
\qquad
k=0,1,2,\cdots.
$$

偶数次幂：

$$
\boldsymbol\Omega^{2k+2}
=
(-1)^k\theta^{2k}\boldsymbol\Omega^2,
\qquad
k=0,1,2,\cdots.
$$

可以检查前几项：

$$
\boldsymbol\Omega^1=\boldsymbol\Omega,
\qquad
\boldsymbol\Omega^3=-\theta^2\boldsymbol\Omega,
\qquad
\boldsymbol\Omega^5=\theta^4\boldsymbol\Omega,
$$

以及：

$$
\boldsymbol\Omega^2=\boldsymbol\Omega^2,
\qquad
\boldsymbol\Omega^4=-\theta^2\boldsymbol\Omega^2,
\qquad
\boldsymbol\Omega^6=\theta^4\boldsymbol\Omega^2.
$$

这和第 0.10 节里的上三角分块幂律是同一种思路：先抓住矩阵乘法反复作用后的结构，再让指数级数自己收敛成闭式公式。

## A.5 把幂律放回指数映射

$SO(3)$ 的指数映射是普通矩阵指数：

$$
\mathrm{Exp}(\boldsymbol\omega)
=
\exp(\boldsymbol\Omega)
=
\sum_{n=0}^{\infty}
\frac{1}{n!}
\boldsymbol\Omega^n.
$$

把它拆成单位项、奇数项和偶数项：

$$
\exp(\boldsymbol\Omega)
=
\mathbf I
+
\sum_{k=0}^{\infty}
\frac{1}{(2k+1)!}
\boldsymbol\Omega^{2k+1}
+
\sum_{k=0}^{\infty}
\frac{1}{(2k+2)!}
\boldsymbol\Omega^{2k+2}.
$$

代入 A.4 的幂律：

$$
\sum_{k=0}^{\infty}
\frac{1}{(2k+1)!}
\boldsymbol\Omega^{2k+1}
=
\left(
\sum_{k=0}^{\infty}
\frac{(-1)^k\theta^{2k}}{(2k+1)!}
\right)
\boldsymbol\Omega.
$$

括号里的级数是：

$$
\frac{\sin\theta}{\theta}.
$$

所以奇数项合起来是：

$$
\frac{\sin\theta}{\theta}\boldsymbol\Omega.
$$

偶数项同理：

$$
\sum_{k=0}^{\infty}
\frac{1}{(2k+2)!}
\boldsymbol\Omega^{2k+2}
=
\left(
\sum_{k=0}^{\infty}
\frac{(-1)^k\theta^{2k}}{(2k+2)!}
\right)
\boldsymbol\Omega^2.
$$

括号里的级数是：

$$
\frac{1-\cos\theta}{\theta^2}.
$$

所以：

$$
\exp(\boldsymbol\Omega)
=
\mathbf I
+
\frac{\sin\theta}{\theta}\boldsymbol\Omega
+
\frac{1-\cos\theta}{\theta^2}\boldsymbol\Omega^2.
$$

这就是 Rodrigues 公式的旋转向量形式：

$$
\boxed{
\mathrm{Exp}(\boldsymbol\omega)
=
\mathbf I
+
\frac{\sin\theta}{\theta}[\boldsymbol\omega]_\times
+
\frac{1-\cos\theta}{\theta^2}[\boldsymbol\omega]_\times^2
}.
$$

如果使用单位轴 $\mathbf u$，因为 $\boldsymbol\omega=\theta\mathbf u$，有：

$$
[\boldsymbol\omega]_\times
=
\theta[\mathbf u]_\times.
$$

代回去得到更常见的轴角形式：

$$
\boxed{
\mathrm{Exp}(\theta\mathbf u)
=
\mathbf I
+
\sin\theta[\mathbf u]_\times
+
(1-\cos\theta)[\mathbf u]_\times^2
}.
$$

## A.6 小角度时为什么没有奇异

Rodrigues 公式里有 $\theta$ 和 $\theta^2$ 出现在分母：

$$
\frac{\sin\theta}{\theta},
\qquad
\frac{1-\cos\theta}{\theta^2}.
$$

这看起来在 $\theta=0$ 处奇异，但它们的极限存在：

$$
\lim_{\theta\rightarrow 0}
\frac{\sin\theta}{\theta}
=
1,
\qquad
\lim_{\theta\rightarrow 0}
\frac{1-\cos\theta}{\theta^2}
=
\frac12.
$$

因此小角度时：

$$
\mathrm{Exp}(\boldsymbol\omega)
\approx
\mathbf I
+
[\boldsymbol\omega]_\times
+
\frac12[\boldsymbol\omega]_\times^2.
$$

如果只保留一阶项，就是第 0 章反复使用的小扰动近似：

$$
\mathrm{Exp}(\boldsymbol\omega)
\approx
\mathbf I
+
[\boldsymbol\omega]_\times.
$$

## A.7 从同一幂律推出 $J_l$ 和 $J_r$

第 0.10 节里出现的 $\mathbf J_l$ 就是 Micro Lie Theory 对 $\mathrm{Exp}$ 定义的 left Jacobian。在 $SO(3)$ 上，它的级数形式是：

$$
\mathbf J_l(\boldsymbol\omega)
=
\sum_{k=0}^{\infty}
\frac{1}{(k+1)!}
\boldsymbol\Omega^k
$$

下面用本附录的幂律把它整理成 Micro Lie Theory 附录中常见的闭式公式。

先把 $k=0$ 的项单独拿出来：

$$
\mathbf J_l(\boldsymbol\omega)
=
\mathbf I
+
\sum_{k=1}^{\infty}
\frac{1}{(k+1)!}
\boldsymbol\Omega^k.
$$

余下部分继续拆成奇数次幂和偶数次幂：

$$
\mathbf J_l(\boldsymbol\omega)
=
\mathbf I
+
\sum_{m=0}^{\infty}
\frac{1}{(2m+2)!}
\boldsymbol\Omega^{2m+1}
+
\sum_{m=0}^{\infty}
\frac{1}{(2m+3)!}
\boldsymbol\Omega^{2m+2}.
$$

代入 A.4 的幂律：

$$
\boldsymbol\Omega^{2m+1}
=
(-1)^m\theta^{2m}\boldsymbol\Omega,
\qquad
\boldsymbol\Omega^{2m+2}
=
(-1)^m\theta^{2m}\boldsymbol\Omega^2.
$$

得到：

$$
\mathbf J_l(\boldsymbol\omega)
=
\mathbf I
+
\left(
\sum_{m=0}^{\infty}
\frac{(-1)^m\theta^{2m}}{(2m+2)!}
\right)
\boldsymbol\Omega
+
\left(
\sum_{m=0}^{\infty}
\frac{(-1)^m\theta^{2m}}{(2m+3)!}
\right)
\boldsymbol\Omega^2.
$$

第一个括号对应：

$$
\frac{1-\cos\theta}{\theta^2},
$$

因为：

$$
1-\cos\theta
=
\frac{\theta^2}{2!}
-
\frac{\theta^4}{4!}
+
\frac{\theta^6}{6!}
-\cdots.
$$

第二个括号对应：

$$
\frac{\theta-\sin\theta}{\theta^3},
$$

因为：

$$
\theta-\sin\theta
=
\frac{\theta^3}{3!}
-
\frac{\theta^5}{5!}
+
\frac{\theta^7}{7!}
-\cdots.
$$

因此：

$$
\boxed{
\mathbf J_l(\boldsymbol\omega)
=
\mathbf I
+
\frac{1-\cos\theta}{\theta^2}
[\boldsymbol\omega]_\times
+
\frac{\theta-\sin\theta}{\theta^3}
[\boldsymbol\omega]_\times^2
}.
$$

Micro Lie Theory 的 right Jacobian 是把 argument 的小变化映射到右侧 local tangent：

$$
\mathrm{Exp}(\boldsymbol\omega+\delta\boldsymbol\omega)
\approx
\mathrm{Exp}(\boldsymbol\omega)
\mathrm{Exp}\!\left(\mathbf J_r(\boldsymbol\omega)\delta\boldsymbol\omega\right).
$$

它和 left Jacobian 的关系是：

$$
\mathbf J_r(\boldsymbol\omega)
=
\mathbf J_l(-\boldsymbol\omega).
$$

由于 $[-\boldsymbol\omega]_\times=-[\boldsymbol\omega]_\times$，而 $[-\boldsymbol\omega]_\times^2=[\boldsymbol\omega]_\times^2$，所以：

$$
\boxed{
\mathbf J_r(\boldsymbol\omega)
=
\mathbf I
-
\frac{1-\cos\theta}{\theta^2}
[\boldsymbol\omega]_\times
+
\frac{\theta-\sin\theta}{\theta^3}
[\boldsymbol\omega]_\times^2
}.
$$

这就是为什么 $J_l$ 和 $J_r$ 很像，但不是同一个矩阵。它们的二阶项相同，叉乘一次项符号相反：

$$
\mathbf J_l(\boldsymbol\omega)
=
\mathbf I
+
\frac12[\boldsymbol\omega]_\times
+
\frac16[\boldsymbol\omega]_\times^2
+
O(\theta^3),
$$

$$
\mathbf J_r(\boldsymbol\omega)
=
\mathbf I
-
\frac12[\boldsymbol\omega]_\times
+
\frac16[\boldsymbol\omega]_\times^2
+
O(\theta^3).
$$

在 $\boldsymbol\omega=\mathbf 0$ 处，二者都退化成 $\mathbf I$。离开零点后，只要旋转不为零，左右扰动坐标就不同。

## A.8 和第 0.10 节的关系

第 0.10 节的推导对象是：

$$
\delta\boldsymbol\xi_L^\wedge
=
\begin{bmatrix}
\boldsymbol\Phi & \delta\boldsymbol\rho_L\\
\mathbf 0^\top & 0
\end{bmatrix}.
$$

它比 $\boldsymbol\Omega$ 多了右上角的平移块，所以矩阵幂多出：

$$
\boldsymbol\Phi^{n-1}\delta\boldsymbol\rho_L.
$$

这个右上角项在指数级数里累加成：

$$
\mathbf J_l(\delta\boldsymbol\phi_L)\delta\boldsymbol\rho_L.
$$

而左上角仍然是：

$$
\exp(\boldsymbol\Phi)
=
\mathrm{Exp}(\delta\boldsymbol\phi_L),
$$

也就是本附录推导的 Rodrigues 结构。

所以可以这样理解：

- $SO(3)$ 的 Rodrigues 公式来自 $[\boldsymbol\omega]_\times$ 的幂律闭合。
- $SE(3)$ 的指数映射来自上三角分块矩阵的幂律。
- $SE(3)$ 左上角沿用 $SO(3)$ 的 Rodrigues。
- $SE(3)$ 右上角因为平移块参与幂级数，额外产生 $\mathbf J_l(\boldsymbol\phi)\boldsymbol\rho$。
