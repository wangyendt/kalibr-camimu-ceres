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

$\mathbf J_r$ 在本书里会以好几种不同的名字反复出现，包括源码里那个看上去和上式毫无关系的 $\mathbf S_K$。A.9 把这些写法收在一起。

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

## A.9 $\mathbf J_r$ 的几个化身

读这本书时会有一种反复出现的既视感：某个 $3\times3$ 矩阵在第 0 章叫一个名字，在第 5 章叫另一个名字，在源码里又是第三个名字，但它们的数值总是一样。这不是巧合，它们本来就是同一个矩阵。本节把这条恒等链定为全书唯一出处：

$$
\boxed{
\mathbf S_K(\boldsymbol\psi)
=
\mathbf J_r(\boldsymbol\psi)
=
\mathbf J_l(-\boldsymbol\psi)
=
\mathrm{Exp}(-\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)
=
\mathbf C(\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)
}.
$$

下面逐段说明为什么这五个写法相等。全节沿用记号：

$$
\theta\triangleq\|\boldsymbol\psi\|,
\qquad
\mathbf u\triangleq\frac{\boldsymbol\psi}{\theta},
\qquad
\mathbf A\triangleq[\mathbf u]_\times,
$$

于是 $[\boldsymbol\psi]_\times=\theta\mathbf A$，并且 A.3 的闭合关系写成 $\mathbf A^3=-\mathbf A$。

### A.9.1 $\mathbf J_r(\boldsymbol\psi)=\mathbf J_l(-\boldsymbol\psi)$

这一段就是 A.7 的结论。因为 $[-\boldsymbol\psi]_\times=-[\boldsymbol\psi]_\times$ 而 $[-\boldsymbol\psi]_\times^2=[\boldsymbol\psi]_\times^2$，把 $\mathbf J_l$ 的闭式里的 $\boldsymbol\psi$ 换成 $-\boldsymbol\psi$，只有叉乘一次项翻号，二次项不动，得到的正是 $\mathbf J_r$ 的闭式。

### A.9.2 $\mathbf J_l(-\boldsymbol\psi)=\mathrm{Exp}(-\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)$

同一个 argument 变化 $\delta\boldsymbol\psi$，既可以映射到左侧，也可以映射到右侧：

$$
\mathrm{Exp}(\boldsymbol\psi+\delta\boldsymbol\psi)
\approx
\mathrm{Exp}\!\left(\mathbf J_l(\boldsymbol\psi)\delta\boldsymbol\psi\right)
\mathrm{Exp}(\boldsymbol\psi)
\approx
\mathrm{Exp}(\boldsymbol\psi)
\mathrm{Exp}\!\left(\mathbf J_r(\boldsymbol\psi)\delta\boldsymbol\psi\right).
$$

把右边那个式子的 $\mathrm{Exp}(\boldsymbol\psi)$ 移到左边，用 $SO(3)$ 的共轭恒等式 $\mathbf R\,\mathrm{Exp}(\mathbf v)\mathbf R^\top=\mathrm{Exp}(\mathbf R\mathbf v)$：

$$
\mathrm{Exp}\!\left(\mathbf J_l(\boldsymbol\psi)\delta\boldsymbol\psi\right)
=
\mathrm{Exp}(\boldsymbol\psi)
\mathrm{Exp}\!\left(\mathbf J_r(\boldsymbol\psi)\delta\boldsymbol\psi\right)
\mathrm{Exp}(\boldsymbol\psi)^\top
=
\mathrm{Exp}\!\left(
\mathrm{Exp}(\boldsymbol\psi)\mathbf J_r(\boldsymbol\psi)\delta\boldsymbol\psi
\right).
$$

对任意 $\delta\boldsymbol\psi$ 成立，所以：

$$
\mathbf J_l(\boldsymbol\psi)
=
\mathrm{Exp}(\boldsymbol\psi)\mathbf J_r(\boldsymbol\psi),
\qquad
\mathbf J_r(\boldsymbol\psi)
=
\mathrm{Exp}(-\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi).
$$

这正是第 0.10 节引用过的 $\mathbf J_l=\mathrm{Ad}_{\mathrm{Exp}(\boldsymbol\tau)}\mathbf J_r$ 在 $SO(3)$ 上的形态。再配合 A.9.1，就得到 $\mathbf J_l(-\boldsymbol\psi)=\mathrm{Exp}(-\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)$。

如果不想借用 adjoint，也可以直接把两个闭式乘开验证。用 $\mathbf A^3=-\mathbf A$、$\mathbf A^4=-\mathbf A^2$，并记 $s=\sin\theta$、$c=\cos\theta$：

$$
\mathrm{Exp}(-\boldsymbol\psi)
=
\mathbf I-s\mathbf A+(1-c)\mathbf A^2,
\qquad
\mathbf J_l(\boldsymbol\psi)
=
\mathbf I
+\frac{1-c}{\theta}\mathbf A
+\frac{\theta-s}{\theta}\mathbf A^2.
$$

乘开后 $\mathbf A$ 的系数是：

$$
\frac{1-c}{\theta}-s+\frac{s(\theta-s)}{\theta}-\frac{(1-c)^2}{\theta}
=
\frac{1}{\theta}
\left[
(1-c)-s^2-(1-c)^2
\right]
=
-\frac{1-c}{\theta},
$$

其中用了 $s^2=(1-c)(1+c)$。$\mathbf A^2$ 的系数同样合并成 $(\theta-s)/\theta$。两个系数正是 $\mathbf J_r$ 的，恒等式成立。

### A.9.3 $\mathbf C(\boldsymbol\psi)=\mathrm{Exp}(-\boldsymbol\psi)$

$\mathbf C(\cdot)$ 是 Kalibr 记号，指 `RotationVector::parametersToRotationMatrix` 由旋转向量参数算出的旋转矩阵，本仓库 Ceres 版对应 `rotationVectorToMatrix`。两处实现算的都是：

$$
\mathbf C(\boldsymbol\psi)
=
\mathbf I
-\sin\theta\,[\mathbf u]_\times
+(1-\cos\theta)[\mathbf u]_\times^2.
$$

和 A.5 的轴角形式 Rodrigues 公式对照，这就是把转角换成 $-\theta$ 的结果：

$$
\mathbf C(\boldsymbol\psi)
=
\mathrm{Exp}(-\theta\mathbf u)
=
\mathrm{Exp}(-\boldsymbol\psi).
$$

也就是说，Kalibr 的旋转向量参数是 Micro 标准指数坐标的相反数——这和 0.8 节 $\boxplus_K$ 里那个负号同源。因此 $\mathbf C(\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)$ 并不是新东西，它只是 $\mathrm{Exp}(-\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)$ 换了个源码里的名字。

### A.9.4 $\mathbf S_K(\boldsymbol\psi)=\mathbf J_r(\boldsymbol\psi)$

`RotationVector::parametersToSMatrix` 的实现是：

$$
\mathbf S_K(\boldsymbol\psi)
=
\mathbf I
+c_1\mathbf A
+c_2\mathbf A^2,
\qquad
c_1=-\frac{2\sin^2(\theta/2)}{\theta},
\qquad
c_2=\frac{\theta-\sin\theta}{\theta}.
$$

注意源码里的 $\mathbf A$ 用的是 **单位轴** $\mathbf u$ 而不是 $\boldsymbol\psi$ 本身，即 $\mathbf A=[\mathbf u]_\times$；这也是它看上去和 $\mathbf J_r$ 闭式不像的唯一原因。把 $\mathbf J_r$ 的闭式换成同一套记号：

$$
\mathbf J_r(\boldsymbol\psi)
=
\mathbf I
-\frac{1-\cos\theta}{\theta^2}[\boldsymbol\psi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\boldsymbol\psi]_\times^2
=
\mathbf I
-\frac{1-\cos\theta}{\theta}\mathbf A
+\frac{\theta-\sin\theta}{\theta}\mathbf A^2.
$$

$\mathbf A^2$ 的系数已经逐字相同。$\mathbf A$ 的系数用半角公式 $1-\cos\theta=2\sin^2(\theta/2)$：

$$
-\frac{1-\cos\theta}{\theta}
=
-\frac{2\sin^2(\theta/2)}{\theta}
=
c_1.
$$

所以 $\mathbf S_K=\mathbf J_r$。源码在 $\theta<10^{-14}$ 时直接返回 $\mathbf I$，对应的正是 $\mathbf J_r(\mathbf 0)=\mathbf I$。

### A.9.5 为什么会在三个地方遇见它

同一个矩阵之所以到处出现，是因为它回答的是同一个问题：**旋转向量参数的一点变化，等价于多大的旋转扰动。** 只是这个问题在三个场合被问了三次。

| 出现位置 | 名字 | 它在回答什么 |
|---|---|---|
| 5.8.1 pose spline | $\mathbf S_K(\boldsymbol\psi)$ | 曲线值里的 $\delta\boldsymbol\psi$ 对应多大的 Kalibr rotation expression tangent $\delta\boldsymbol\phi_K$ |
| A.7 Micro Lie Theory | $\mathbf J_r(\boldsymbol\psi)$ | argument 的 $\delta\boldsymbol\psi$ 映射到右侧 local tangent 是多少 |
| 0.10.4 Ceres manifold | $\mathbf C(\mathbf r)\mathbf J_l(\mathbf r)$ | `Pose6Manifold::MinusJacobian` 的旋转块 |

三者数值完全相同，所以任何一处的实现都可以拿另外两处当参考实现来对拍。

### A.9.6 数值自查

这条恒等链很适合用二三十行 numpy 验证，也建议在移植代码时真的跑一次。做法是随机取若干 $\boldsymbol\psi$，覆盖从 $10^{-6}$ 到接近 $\pi$ 的角度范围，分别按上面五个公式算出矩阵，再看逐元素最大偏差：

$$
\max_{jk}
\left|
\left[
\mathbf S_K(\boldsymbol\psi)-\mathbf C(\boldsymbol\psi)\mathbf J_l(\boldsymbol\psi)
\right]_{jk}
\right|.
$$

五个表达式在数学上严格相等，但**双精度下的实测偏差并不是在所有角度上都等于机器精度**，这一点必须先说清楚，否则很容易去追一个不存在的 bug。把 $\mathbf J_l$、$\mathbf J_r$ 按闭式朴素实现（不加小角度分支），实测结果是：

| $\theta$ | $\max_{jk}\lvert[\mathbf S_K-\mathbf C\mathbf J_l]_{jk}\rvert$ |
|---:|---:|
| $10^{-8}$ | $4\times10^{-9}$ |
| $10^{-6}$ | $4\times10^{-11}$ |
| $10^{-4}$ | $2\times10^{-13}$ |
| $10^{-3}$ | $6\times10^{-15}$ |
| $10^{-2}$ | $1\times10^{-15}$ |
| $\ge 0.5$ | $\le 3\times10^{-16}$ |

$\theta\gtrsim10^{-2}$ 时确实是机器精度，但**角度越小偏差越大**，到 $\theta=10^{-8}$ 已经涨到 $10^{-9}$。这不是恒等式不成立，而是朴素闭式里的相消误差，而且罪魁祸首是 $\mathbf J_l$ 一侧不是 $\mathbf S_K$ 一侧。

> **先固定基。** 下面沿用 A.9.2 的写法，$\mathbf A\triangleq[\mathbf u]_\times$ 是**单位轴**的反对称矩阵，于是
> $$
> \mathbf J_l(\boldsymbol\psi)
> =\mathbf I
> +\underbrace{\frac{1-\cos\theta}{\theta}}_{\textstyle a_1(\theta)}\mathbf A
> +\underbrace{\frac{\theta-\sin\theta}{\theta}}_{\textstyle a_2(\theta)}\mathbf A^2 .
> $$
> 如果换成 $[\boldsymbol\psi]_\times=\theta\mathbf A$ 当基，两个系数要各除一个 $\theta$、$\theta^2$，写成 $\frac{1-\cos\theta}{\theta^2}$ 与 $\frac{\theta-\sin\theta}{\theta^3}$。**两套系数绝不能混用**，混了就会得到既不是 $\theta/2$ 也不是 $\theta^2/6$ 的第三种极限。本节全部用上面这一套。

把 $a_1,a_2$ 各自的双精度绝对误差单独测出来，就能看清是谁在漏精度：

| $\theta$ | $a_1=\frac{1-\cos\theta}{\theta}$ 误差 | $a_2=\frac{\theta-\sin\theta}{\theta}$ 误差 |
|---:|---:|---:|
| $10^{-8}$ | $5\times10^{-9}$ | $1.7\times10^{-17}$ |
| $10^{-6}$ | $4.5\times10^{-11}$ | $2.2\times10^{-17}$ |
| $10^{-4}$ | $2.6\times10^{-13}$ | $8.3\times10^{-17}$ |
| $10^{-2}$ | $1.4\times10^{-13}$ | $2.1\times10^{-16}$ |

**丢精度的是 $\mathbf A$ 的系数 $a_1$，不是 $\mathbf A^2$ 的系数 $a_2$。** 原因是 $1-\cos\theta$ 这个减法：$\cos\theta$ 本身是 $O(1)$ 的数，无论它多接近 $1$，绝对舍入误差都是一个 ulp 量级即 $\varepsilon/2\approx1.1\times10^{-16}$；再除以小小的 $\theta$，误差被放大成

$$
\left|\Delta a_1\right|\ \approx\ \frac{\varepsilon}{2\theta},
$$

$\theta=10^{-6}$ 代进去是 $1.1\times10^{-10}$，与实测的 $4.5\times10^{-11}$ 同量级。到 $\theta=10^{-8}$ 时 $\cos\theta=1-5\times10^{-17}$ 已小于半个 ulp，直接舍入成 $1.0$，$1-\cos\theta$ 算出 $0$，$a_1$ 丢掉**全部**有效位——误差就等于它自己的真值 $\theta/2=5\times10^{-9}$（误差不会再涨过真值，这也是上表第一行的上限）。

反观 $a_2=1-\frac{\sin\theta}{\theta}$：$\sin\theta/\theta$ 同样是 $O(1)$，减法后绝对误差也是 $\varepsilon/2$ 量级，但**它后面没有再除以 $\theta$**，所以绝对误差就停在 $10^{-16}$ 不动。$a_2$ 的真值 $\theta^2/6$ 在小角度下确实被相对地毁掉了（$\theta=10^{-8}$ 时真值才 $1.7\times10^{-17}$，早就淹没在舍入噪声里），但它对最终矩阵的**绝对**贡献本来就只有 $10^{-17}$，构不成表里 $10^{-9}$ 的偏差。

反观 $\mathbf S_K$。要先纠正一个容易顺口说出来的错判：A.9.4 抄下来的 $c_1=-\frac{2\sin^2(\theta/2)}{\theta}$ 与 $c_2=\frac{\theta-\sin\theta}{\theta}$ **同样带 $\theta$ 分母**，源码 `RotationVector.cpp:93-99` 里那个 `recip_angle` 就是它。稳的原因不在分母有没有，而在分子怎么写。$c_1$ 走的是**半角形式** $2\sin^2(\theta/2)$ 而不是 $1-\cos\theta$：$\sin(\theta/2)$ 是一个从零出发的量，双精度求它只有相对误差 $O(\varepsilon)$，平方后仍是相对误差，于是 $2\sin^2(\theta/2)\approx\theta^2/2$ 的绝对误差是 $O(\varepsilon\theta^2)$；再除以 $\theta$ 得

$$
|\Delta c_1|=O(\varepsilon\theta),
$$

不但没被放大，反而随 $\theta$ 一起变小。$1-\cos\theta$ 那条路上的 $\varepsilon/2$ 则是**减法相消**留下的绝对误差，它与 $\theta$ 无关，除以 $\theta$ 才炸成 $\varepsilon/(2\theta)$。同理 $c_2$ 的分子 $\theta-\sin\theta$ 虽然也相消，但两个被减数本身就是 $O(\theta)$，相消后的绝对误差是 $O(\varepsilon\theta)$，除以 $\theta$ 停在 $O(\varepsilon)$——正是上表 $a_2$ 那一列的表现。所以源码这两个系数在小角度下更稳，功劳属于半角改写，而不是"没有分母"，这也是 Kalibr 直接用它的原因之一。

所以自查时的判据应当是：

- 偏差随 $\theta$ 减小而**按 $1/\theta$ 增大**，且 $\theta\ge10^{-2}$ 时在 $10^{-15}$ 量级——正常，与上表一致。注意这条 $1/\theta$ 律只在 $\cos\theta$ 还留有有效位时成立：$\theta\lesssim\sqrt\varepsilon\approx1.5\times10^{-8}$ 之后 $1-\cos\theta$ 直接算成 $0$，误差被自己的真值 $\theta/2$ 封顶，不再继续按 $1/\theta$ 涨，反而随 $\theta$ 一起回落，上表第一行就是这个饱和段的起点；
- 在 $\theta\sim1$ 这种"良性"角度上就看到 $10^{-8}$ 甚至更大——这才是真 bug，多半是某个实现的小角度分支阈值设反了，或者两套基的系数混用了（少除或多除了一个 $\theta$）；
- 偏差是 $O(1)$——符号约定用反了，最常见的是把 $\mathbf C(\boldsymbol\psi)$ 当成 $\mathrm{Exp}(+\boldsymbol\psi)$。

如果确实需要在极小角度下也保持机器精度，把 $\theta<10^{-4}$ 的分支换成泰勒展开（仍是 $\mathbf A$ 基）

$$
a_1(\theta)=\frac{\theta}{2}-\frac{\theta^3}{24}+O(\theta^5),
\qquad
a_2(\theta)=\frac{\theta^2}{6}-\frac{\theta^4}{120}+O(\theta^6),
$$

即可，上表最上面两行会立刻回落到 $10^{-16}$。真正必须换的是 $a_1$；$a_2$ 换不换都不影响绝对精度，顺手一起换是为了让代码在两条分支上保持同样的展开阶数。

同一段代码还可以顺手验证 0.10.4 的 manifold 契约：

$$
\mathbf J_l(-\mathbf r)^{-1}
\cdot
\mathbf C(\mathbf r)\mathbf J_l(\mathbf r)
=
\mathbf I,
$$

即 `PlusJacobian` 的旋转块和 `MinusJacobian` 的旋转块互为逆矩阵。
