# Swarm Controller 公式（控制器实现）

## 1. 真实坐标到虚拟坐标映射
$$
p_i,\ v_i\in\mathbb R^3,\qquad p_l,\ v_l\in\mathbb R^3,\qquad i=1,\ldots,n
$$

$$
p_l=\begin{bmatrix}x_{p_l}\\y_{p_l}\\z_{p_l}\end{bmatrix},\qquad
x_u=x_{p_l},\quad y_u=y_{p_l},\quad z_u=z_{p_l}+h_u
$$

$$
q_u=\begin{bmatrix}x_u\\y_u\\z_u\end{bmatrix}
=
p_l+\begin{bmatrix}0\\0\\h_u\end{bmatrix}
$$

$$
\mathcal V=\{1,\ldots,n,u,l\},\qquad |\mathcal V|=n+2
$$

$$
q_l=[x_l,y_l,z_l]^T,\qquad
q=\begin{bmatrix}q_1^T & \cdots & q_n^T & q_u^T & q_l^T\end{bmatrix}^T\in\mathbb R^{3(n+2)}
$$

$$
q_l=p_l
$$

$$
\alpha_i=\frac{|z_u-z_l|}{|z_l-z_i|},\qquad
q_i=p_l+\alpha_i(p_i-p_l),\qquad i=1,\ldots,n
$$

$$
q_i-q_l=\alpha_i(p_i-p_l)
$$

其中：
- $p_i,v_i\in\mathbb R^3$ 为第 $i$ 架 UAV 的真实位置/速度，$p_l,v_l\in\mathbb R^3$ 为载荷真实位置/速度。
- $h_u>0$ 为虚拟托盘中心相对载荷的竖直高度偏置。
- $q_i,\dot q_i,\ddot q_i\in\mathbb R^3$ 为虚拟节点状态；$q_u$ 为虚拟托盘中心，$q_l$ 为虚拟载荷节点。
- $n$ 为 UAV 数量；$n+2$ 来自 $n$ 个 UAV 节点 + 托盘中心节点 $u$ + 载荷节点 $l$。

## 2. 虚拟网络力
$$
W=[w_{ij}]\in\mathbb R^{(n+2)\times(n+2)}
$$

$$
W=
\begin{bmatrix}
0 & w_{12} & \cdots & w_{1u} & w_{1l}\\
w_{21} & 0 & \cdots & w_{2u} & w_{2l}\\
\vdots & \vdots & \ddots & \vdots & \vdots\\
w_{u1} & w_{u2} & \cdots & 0 & w_{ul}\\
w_{l1} & w_{l2} & \cdots & w_{lu} & 0
\end{bmatrix}
$$

$$
w_{ii}=0,\qquad
\mathcal N_i=\{j\in\mathcal V\mid w_{ij}=1,\ j\neq i\}
$$

若采用固定全连接去对角拓扑，则可直接取
$$
W=\mathbf 1\mathbf 1^T-I_{n+2}
$$

例如当 $n=3$（节点顺序为 $[1,2,3,u,l]$，总维度 $n+2=5$）：
$$
W=
\begin{bmatrix}
0&1&1&1&1\\
1&0&1&1&1\\
1&1&0&1&1\\
1&1&1&0&1\\
1&1&1&1&0
\end{bmatrix}
$$

虚拟力由三部分组成：结构保持弹性力、相对速度阻尼力、以及本地摩擦耗散力。

$$
F_{s,i}(q_i)=\sum_{j\in\mathcal N_i}k_{ij}w_{ij}\left(1-\frac{\bar l_{ij0}}{\bar l_{ij}}\right)(q_i-q_j),\qquad
\bar l_{ij}=\|q_i-q_j\|
$$

$$
F_{d,i}(q_i,\dot q_i)=c_1\sum_{j\in\mathcal N_i}w_{ij}(\dot q_i-\dot q_j)
$$

$$
F_{f,i}(\dot q_i)=c_2\dot q_i
$$

其中：
- $F_{s,i}$：结构保持项（虚拟弹簧力），用于将节点间距离拉回到期望值 $\bar l_{ij0}$。
- $F_{d,i}$：相对速度耗散项（虚拟阻尼力），用于抑制邻居间相对运动。
- $F_{f,i}$：本地速度耗散项（虚拟摩擦力），用于提供额外阻尼并提升收敛稳定性。
- $w_{ij}$ 为连接权重（本版常用二值），若拓扑无向常取 $w_{ij}=w_{ji}$。
- $k_{ij}>0$ 为虚拟弹簧刚度；$c_1>0,c_2>0$ 分别为阻尼系数和摩擦系数。
- $\bar l_{ij0}$ 为期望节点间距。

## 3. 位置外环与速度跟踪控制律
$$
e_p=p_l-p_l^d
$$

$$
v_d=-K_p^p e_p
$$

$$
e_i=\dot q_i-v_d
$$

$$
u_i=-\left(K_p e_i+K_I\int e_i\,dt+K_d\frac{d e_i}{dt}\right)
$$

$$
\ddot q_{id}=-g-\frac{F_{s,i}+F_{d,i}+F_{f,i}}{m_i}
$$

$$
\ddot q_{iu}=\frac{u_i}{m_i}
$$

$$
\ddot q_i=\ddot q_{id}+\ddot q_{iu}
$$

其中：
- $p_l^d$ 为载荷目标位置，$e_p$ 为载荷位置误差。
- $K_p^p$ 为位置外环比例增益，用于将位置误差直接映射成期望速度 $v_d$。
- $e_i$ 为第 $i$ 个虚拟节点的速度跟踪误差。
- $K_p,K_I,K_d$ 为虚拟节点速度 PID 内环增益（可取标量或对角矩阵）。
- $u_i$ 为虚拟节点输入，$m_i$ 为第 $i$ 架 UAV 质量。

## 4. 虚拟加速度映射到真实 UAV
$$
\beta_i=\frac{|z_l-z_i|}{|z_u-z_l|}=\frac{|z_l-z_i|}{h_u}
$$

$$
a_{id,i}=\beta_i\,\ddot q_i
$$

其中：
- $\beta_i$ 为第 $i$ 架 UAV 的虚实映射系数，对应实现中的 `state.beta[i]`。
- $a_{id,i}$ 为真实系统中第 $i$ 架 UAV 的期望加速度，对应实现中的 `mapped_acceleration`。
- 由于实现中使用竖直距离绝对值，映射比例只与高度差大小有关。

## 5. CFO 轴向扰动观测器
$$
t_i=p_l-p_i,\qquad t_i^0=\frac{t_i}{\|t_i\|}
$$

$$
v_i^{\parallel}=v_i^T t_i^0,\qquad
u_i^{\parallel}=\frac{(T_{i,\text{prev}}+G_i)^T t_i^0}{m_i}
$$

$$
e_k=v_k^{\parallel}-\hat v_k^{\parallel}
$$

$$
\hat v_{k+1}^{\parallel}=\hat v_k^{\parallel}+T_s\left(u_k^{\parallel}+\hat d_k^{\parallel}+l_1 e_k\right)
$$

$$
\hat d_{k+1}^{\parallel}=\hat d_k^{\parallel}+T_s\left(l_2\,\mathrm{sat}\!\left(\frac{e_k}{\phi}\right)\right),\qquad
\mathrm{sat}(x)=\mathrm{clip}(x,-1,1)
$$

$$
\hat\tau_i=\mathrm{clip}\left(-m_i\hat d_{k+1}^{\parallel},0,f_{\max,i}\right),\qquad
\hat f_i=\hat\tau_i(-t_i^0)
$$

其中：
- $T_{i,\text{prev}}$ 为上一周期推力；$G_i=[0,0,-m_i g]^T$（ENU 常用）。
- $\hat v^{\parallel},\hat d^{\parallel}$ 为观测器状态；$l_1,l_2,\phi$ 为观测器参数。
- $\hat\tau_i\in[0,f_{\max,i}]$ 保证“只拉不推”。

## 6. 推力指令与约束
$$
T_{id,i}=m_i a_{id,i}-G_i+\hat f_i
$$

$$
T_{cmd,i}=\mathrm{norm\_clip}(T_{id,i},T_{\max,i}),\qquad \|T_{cmd,i}\|\le T_{\max,i}
$$

其中：$T_{id,i}$ 为未限幅推力，$T_{cmd,i}$ 为可执行推力命令。

## 7. 异常退化
若 $dt$ 越界或 $\|t_i\|<l_{\min}$，则本步采用：

$$
\hat f_i=0,\qquad \hat\tau_i=0,\qquad
T_{cmd,i}=\mathrm{norm\_clip}(m_i a_{id,i}-G_i,T_{\max,i})
$$

并冻结观测器状态 $\hat v^{\parallel},\hat d^{\parallel}$。
