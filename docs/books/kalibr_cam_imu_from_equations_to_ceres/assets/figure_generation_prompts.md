# Figure Generation Prompts

本文件记录第 4 章相机观测模型中几张示意图的生成提示词。推荐导出为 PNG，并使用下面给出的文件名直接替换同目录资源：

```text
docs/knowhow/cam_imu_calibration_book/assets/
```

如果生成工具能导出更清晰的 SVG，也可以临时保存为 SVG，但本书正文当前统一引用 PNG。

## 通用风格要求

所有图统一使用下面的风格：

```text
Clean academic vector diagram for a robotics/state-estimation textbook. White background, no shadows, no gradients, no decorative background. Use thin dark gray outlines, muted accent colors, clear arrows, aligned boxes, consistent spacing, and readable sans-serif labels. Keep the figure flat and precise, like a LaTeX textbook or diagrams.net / Figma technical diagram. Avoid marketing style, 3D perspective, pictorial camera illustrations, decorative icons, or unnecessary visual noise. Use exact text labels as requested.
```

## `camera_projection_jacobian_flow.png`

用途：解释 projection 层的 forward path 和 Jacobian path。放在第 4.4 末尾、第 4.5 开头附近。

Prompt:

```text
Create a clean academic vector flow diagram titled "Projection layer: forward values and Jacobian flow".

Canvas: wide 16:5 aspect ratio, white background.

Main horizontal pipeline, left to right:
1. Box: "p_c = [X,Y,Z]^T" with small subtitle "camera point, 3x1".
2. Arrow labeled "J_{q,p}  (2x3)" to box: "q = [X/Z, Y/Z]^T" with subtitle "normalized coordinate, 2x1".
3. Arrow labeled "J_{d,q}  (2x2)" to box: "q_d = d(q; kappa)" with subtitle "distorted coordinate, 2x1".
4. Arrow labeled "K_f  (2x2)" to box: "y_hat = K_f q_d + c" with subtitle "predicted pixel, 2x1".
5. Arrow labeled "-I_2" to box: "e = y - y_hat" with subtitle "reprojection residual, 2x1".

Add two parameter side inputs:
- From below into the distortion box: "kappa" with arrow labeled "J_{d,kappa}".
- From below into the pixel box: "f_u, f_v, c_u, c_v" with arrow labeled "J_intr".

Add a small formula strip at the bottom:
"J_pi,p = K_f J_d,q J_q,p,   J_e,p = -J_pi,p"

Style: clean vector boxes, straight arrows, muted blue for operation boxes, muted orange for parameter boxes, dark gray text. Do not use decorative icons. Make all text large and legible.
```

## `radial_scaling_jacobian_flow.png`

用途：解释 equidistant 和 FOV 这类径向缩放模型的 Jacobian。放在通用径向缩放推导开始处。

Prompt:

```text
Create a clean academic vector diagram titled "Radial scaling models: q_d = s(r, kappa) q".

Canvas: wide 16:6 aspect ratio, white background.

Show a computation graph with nodes:
- Left node: "q = [x,y]^T" subtitle "2x1".
- Upper middle node: "r = ||q||" subtitle "scalar".
- Upper right node: "s = s(r, kappa)" subtitle "scalar scale".
- Right node: "q_d = s q" subtitle "2x1".
- Lower parameter node: "kappa" subtitle "model parameters".

Edges:
- q -> r, labeled "delta r = (q^T / r) delta q".
- r -> s, labeled "delta s = s'_r delta r".
- kappa -> s, dashed arrow labeled "partial s / partial kappa".
- s -> q_d, labeled "q delta s".
- q -> q_d, curved lower arrow labeled "direct term: s delta q".

At the bottom, show a formula banner:
"delta q_d = s delta q + q s'_r (q^T/r) delta q"
"J_{d,q} = s I_2 + (s'_r/r) q q^T"

Style: flat vector diagram, green accent for radial scaling nodes, orange for parameter node, dark gray arrows, exact formula text, no decorative background.
```

## `transform_jacobian_flow.png`

用途：解释 camera residual 中 Kalibr expression graph 的 transform 链式传播。放在第 4.8 开头。

Prompt:

```text
Create a clean academic vector expression-graph diagram titled "Transform and projection chain for one camera residual".

Canvas: wide 5:2 aspect ratio, white background.

Draw a directed graph from left to right:
1. Top-left node: "T_wb(t)" subtitle "pose spline".
2. Arrow to node: "inverse" subtitle "T_bw = T_wb^{-1}". Label this edge "-boxTimes(T_bw)".
3. Lower-left node: "T_cnb" subtitle "camera extrinsic".
4. Both "T_bw" and "T_cnb" feed into a center node: "product" subtitle "T_cnw = T_cnb T_bw".
   - Edge from T_cnb to product labeled "left child: identity".
   - Edge from T_bw to product labeled "right child: boxTimes(T_cnb)".
5. Lower center node: "p_w" subtitle "target corner".
6. Product node and p_w feed into node: "point action" subtitle "p_c = T_cnw p_w".
   - Edge from product to point action labeled "boxMinus(p_c)".
7. Arrow to node: "projection" subtitle "y_hat = pi(p_c)", edge labeled "J_pi,p".
8. Arrow down or right to node: "residual" subtitle "e = y - y_hat", edge labeled "-I_2".

Add a dashed side path from time shift:
Node: "Delta t_n" -> "query time t" -> "T_wb(t)" labeled "time derivative path".

Add a bottom formula strip:
"A_T = - J_{pi,p_c} boxMinus(p_c)"
"J_{e,T_wb} = A_T boxTimes(T_cnb) [-boxTimes(T_bw)]"

Style: technical textbook graph, no 3D, no camera icon, no decorative arrows. Use orange for design variables, blue for expression nodes, green for target point, dark gray text. Keep all labels readable.
```

## `camera_distortion_effects.png`

用途：解释不同畸变模型的几何效果。它不是节点流图，但和 4.5.1 的理解强相关。

Prompt:

```text
Create a clean academic vector illustration titled "Distortion maps normalized coordinates q to q_d".

Canvas: wide 16:5 aspect ratio, white background.

Create four side-by-side panels with equal square normalized image-plane grids.
Each panel should show a faint gray undistorted square grid and a colored distorted grid.

Panel 1 title: "NoDistortion"
Grid: colored grid exactly overlaps the gray grid. Caption: "q_d = q".

Panel 2 title: "RadialTangential"
Grid: show radial bending plus slight asymmetric tangential skew. Caption: "radial scale + tangential skew".

Panel 3 title: "Equidistant"
Grid: show direction-preserving radial remapping with curved grid lines around center. Caption: "direction fixed, radius r maps to theta_d".

Panel 4 title: "FOV"
Grid: show direction-preserving radial remapping from r to atan(2 r tan(w/2))/w. Caption: "radius r maps to atan(2 r tan(w/2))/w".

Use subtle arrows near one corner in each distorted panel to indicate motion from gray grid to colored grid. No photographic camera, no lens illustration, no decorative background. Use muted colors and clear labels.
```
