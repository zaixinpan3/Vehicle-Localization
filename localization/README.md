# Continuous-Time Cascaded Vehicle Observer: Mathematical Design

This document defines the lateral observer, its velocity/angle interface, the global observer, their gain-design conditions, and a conditional local input-to-state stability theorem for the complete twelve-state estimation error. All results concern continuous time. The operating domain, measurement conditions, and actual LiDAR-feedback conditions are explicit hypotheses.

## Notation and admissible signals

Let
$$
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix},\quad
R(\phi)=\begin{bmatrix}\cos\phi&-\sin\phi\\\sin\phi&\cos\phi\end{bmatrix},\quad
t_\phi=R(\phi)e_1,\quad n_\phi=Jt_\phi .
$$
Here $e_i$ denotes a coordinate unit vector of the dimension indicated by its use. Write $\operatorname{He}(M)=M+M^\top$ and $\operatorname{Sym}(M)=\operatorname{He}(M)/2$. Vector norms are Euclidean; matrix norms are induced two-norms. The notation $\|w\|_{\infty,[0,t]}$ denotes the essential supremum over $[0,t]$.

The global state, at one fixed vehicle output point, is
$$
x=[X,V_X,A_X,Y,V_Y,A_Y,\psi,r]^\top,\qquad
p=[X,Y]^\top,\quad v=[V_X,V_Y]^\top,\quad a=[A_X,A_Y]^\top .
$$
Heading is $\psi$, yaw rate is $r$, course angle is $\phi=\psi+\beta$, course speed is $s=\|v\|$, and sideslip is $\beta$. Thus
$$
v=s\,t_\phi,\qquad
a=b\,t_\phi+s q\,n_\phi,\qquad b=\dot s,\quad q=\dot\phi=r+\dot\beta .
\tag{1}
$$
The main motion domain is
$$
s_0\le s\le V,\qquad |q|\le Q,\qquad b^2+s^2q^2\le A^2,\qquad s_0>0.
\tag{2}
$$
The lateral scheduling domain is separately $u_-\le u_x\le u_+$, with $0<u_-<u_+$, and a declared bound on the derivative of the scheduling speed. Course speed and longitudinal speed are different variables.

The true trajectory remains in a compact admissible physical operating set, or a set with equivalent uniform derivative bounds. This physical operating-set hypothesis is supplied by the vehicle model and operating conditions. The observer does not establish it. The intrinsic drift, auxiliary output maps and smooth global schedules have uniformly bounded second derivatives on a neighborhood of that set. Angles are compared on common local lifts, and all required motion outputs remain available. The LiDAR and GNSS hypotheses are specified below.

Disturbances may be correlated and time varying. They include tire-force mismatch, sensor and output-point errors, intrinsic-model residuals, LiDAR bias, and the physical forcing associated with angle-filter lag. Independence or zero-mean noise is not required by the deterministic stability theorem.

## Lateral LPV observer

### Plant and measurement model

Let $z=[v_y^I,r]^\top$ be the lateral state at the observer point, and let $\delta$ be steering angle. With vehicle parameters $m,I_z,l_f,l_r,C_f,C_r>0$,
$$
\begin{aligned}
\dot z&=A_L(\rho)z+B_L\delta+f_L(z)+E_Lw_L,\\
y_L&=C_L(\rho)z+D_L\delta+F_Lw_L+\nu_L,\qquad
y_L=[a_y^I,r]^\top,\\
\rho&=[u_x,u_x^{-1}]^\top ,
\end{aligned}
\tag{3}
$$
where
$$
\begin{gathered}
A_L(\rho)=A_0+\rho_1A_v+\rho_2A_q,\qquad
A_0=0,\quad A_v=\begin{bmatrix}0&-1\\0&0\end{bmatrix},\\
A_q=\begin{bmatrix}
-(C_f+C_r)/m&(l_rC_r-l_fC_f)/m\\
(l_rC_r-l_fC_f)/I_z&-(l_f^2C_f+l_r^2C_r)/I_z
\end{bmatrix},\\
B_L=\begin{bmatrix}C_f/m\\l_fC_f/I_z\end{bmatrix},\qquad
D_L=\begin{bmatrix}C_f/m\\0\end{bmatrix},\\
c^\top=[-(C_f+C_r)/m,\ (l_rC_r-l_fC_f)/m],\\
C_L(\rho)=\begin{bmatrix}\rho_2c^\top\\e_2^\top\end{bmatrix},\quad
E_L=\begin{bmatrix}1&1\\ml_f/I_z&-ml_r/I_z\end{bmatrix},\quad
F_L=\begin{bmatrix}1&1\\0&0\end{bmatrix}.
\end{gathered}
\tag{4}
$$
The components of $w_L$ are axle-force errors divided by mass. The copied nonlinearity satisfies
$\|f_L(z)-f_L(\hat z)\|\le\gamma_f\|z-\hat z\|$ on the stated domain; $f_L=0$ is permitted.

The observer is
$$
\dot{\hat z}=A_L(\rho)\hat z+B_L\delta+f_L(\hat z)
+L(\rho)\bigl[y_L-C_L(\rho)\hat z-D_L\delta\bigr].
\tag{5}
$$
Initially suppose plant and observer use the same scheduling signal. For $e_L=z-\hat z$, positive diagonal disturbance scales $W_w,W_\nu$, and
$$
\xi_L=\begin{bmatrix}W_w^{-1}w_L\\W_\nu^{-1}\nu_L\end{bmatrix},
\quad B_d(\rho)=[(E_L-LF_L)W_w,\ -LW_\nu],
$$
the exact error equation is
$$
\dot e_L=(A_L-LC_L)e_L+\Delta f_L+B_d\xi_L,\qquad
\|\Delta f_L\|\le\gamma_f\|e_L\|.
\tag{6}
$$

### Scheduling triangle and derivative hull

Put $a=u_-$ and $b=u_+$. The physical curve lies in the triangle with vertices
$$
\rho^{[1]}=\begin{bmatrix}a\\a^{-1}\end{bmatrix},\quad
\rho^{[2]}=\begin{bmatrix}b\\b^{-1}\end{bmatrix},\quad
\rho^{[3]}=\begin{bmatrix}2ab/(a+b)\\2/(a+b)\end{bmatrix}.
$$
Indeed, the convex graph $1/u_x$ lies below its endpoint chord and above both endpoint tangents; the tangents meet at $\rho^{[3]}$. Define
$$
T_\rho=\begin{bmatrix}\rho^{[1]}&\rho^{[2]}&\rho^{[3]}\\1&1&1\end{bmatrix},
\qquad
\alpha=T_\rho^{-1}\begin{bmatrix}\rho\\1\end{bmatrix},\qquad
\alpha_i\ge0,\quad\sum_i\alpha_i=1.
\tag{7}
$$
If the actual scheduling signal is absolutely continuous with
$\dot u_x\in[\sigma_-,\sigma_+]$, then
$$
\dot\alpha=T_\rho^{-1}[\sigma,-\sigma z,0]^\top,\qquad
z=u_x^{-2}\in[b^{-2},a^{-2}].
$$
This map is multi-affine in $(\sigma,z)$. Its four corner values $d^{[\ell]}$ contain every admissible $\dot\alpha$ in their convex hull. In particular $\sum_i d_i^{[\ell]}=0$. Taking the product of the position triangle and derivative hull is a conservative enclosure of the physical scheduling trajectories.

### Structured gains and exact affine extension

Use
$$
L(\rho)=[g_0+\rho_1g_1,\ h_0+\rho_1h_1+\rho_2h_2],
\quad g_0,g_1,h_0,h_1,h_2\in\mathbb R^2 .
\tag{8}
$$
This restriction removes the reciprocal-speed term from the first gain column. The remaining product $\rho_1\rho_2$ is eliminated using its physical identity $\rho_1\rho_2=1$:
$$
\widetilde A_e(\rho)=A_L(\rho)-L(\rho)C_L(\rho)
+(\rho_1\rho_2-1)g_1c^\top .
\tag{9}
$$
Expanding gives the affine extension
$$
\widetilde A_e=A_0-g_1c^\top-h_0e_2^\top
+\rho_1(A_v-h_1e_2^\top)+\rho_2(A_q-g_0c^\top-h_2e_2^\top).
$$
It equals the actual error matrix on every point of the physical curve.

For a common slack $X$ and $Y_i=XL_i$, impose
$$
Y_3(:,1)=\frac{b}{a+b}Y_1(:,1)+\frac{a}{a+b}Y_2(:,1),
\qquad H=\frac{Y_2(:,1)-Y_1(:,1)}{b-a}=Xg_1 .
\tag{10}
$$
Let $\delta_i$ be the three entries of $[0,0,-(b-a)^2/(a+b)^2]$ and define
$$
\mathcal R_i=XA_L(\rho^{[i]})-Y_iC_L(\rho^{[i]})+\delta_iHc^\top,\qquad
\mathcal S_i=[(XE_L-Y_iF_L)W_w,\ -Y_iW_\nu].
$$
Their barycentric interpolations are $X\widetilde A_e$ and $XB_d$. The third-vertex correction in $\mathcal R_3$ is essential.

### Twelve lateral LMIs and ISS proof

Fix $\kappa_L,\tau,\varepsilon_L>0$. Choose $P_i=P_i^\top\succeq I_2$, a common $X$, $Y_i$ satisfying (10), and $\gamma_L^2\ge0$. Set
$$
P_L(\rho)=\sum_i\alpha_iP_i,\qquad
\dot P_\ell=\sum_j d_j^{[\ell]}P_j,
$$
and require, for $i=1,2,3$ and $\ell=1,\ldots,4$,
$$
\begin{bmatrix}
\Phi_{i\ell}&P_i-X+\varepsilon_L\mathcal R_i^\top&\mathcal S_i&\sqrt\tau P_i\\
*&-\varepsilon_L(X+X^\top)&\varepsilon_L\mathcal S_i&0\\
*&*&-\gamma_L^2I_4&0\\
*&*&*&-I_2
\end{bmatrix}\prec0,
\quad
\Phi_{i\ell}=\operatorname{He}(\mathcal R_i)+\dot P_\ell
+2\kappa_LP_i+\frac{\gamma_f^2}{\tau}I_2 .
\tag{11}
$$
These are LMIs when $\tau,\varepsilon_L$ are fixed. Minimizing $\gamma_L^2$ defines the first design criterion. A finite search over positive $(\tau,\varepsilon_L)$ remains a finite collection of convex designs.

**Proposition 1.** Conditions (7)--(11) imply
$$
\dot V_L\le-2\kappa_LV_L+\gamma_L^2\|\xi_L\|^2,\qquad
V_L=e_L^\top P_L(\rho)e_L .
\tag{12}
$$

**Proof.** The complete matrix in (11) is affine separately in $\alpha$ and $\dot\alpha$, so its value is a convex combination of the twelve strict vertex matrices. Its second diagonal block implies $X+X^\top\succ0$, hence $X$ is nonsingular and $L_i=X^{-1}Y_i$ exists. Schur-complementing the last block and applying the full-column-rank congruence
$$
N=\begin{bmatrix}I_2&0\\A_L-LC_L&B_d\\0&I_4\end{bmatrix}
$$
cancels the slack terms and yields
$$
\begin{bmatrix}
\operatorname{He}(P_L(A_L-LC_L))+\dot P_L+2\kappa_LP_L
+\tau P_L^2+(\gamma_f^2/\tau)I_2&P_LB_d\\
*&-\gamma_L^2I_4
\end{bmatrix}\prec0 .
$$
Finally,
$2e_L^\top P_L\Delta f_L\le\tau e_L^\top P_L^2e_L+
(\gamma_f^2/\tau)\|e_L\|^2$ proves (12). The affine extension equals the physical error matrix by (9), so the argument covers the entire physical speed interval. $\square$

Let $\bar p_L=\max_i\lambda_{\max}(P_i)$. Then
$$
\|e_L(t)\|\le\sqrt{\bar p_L}\,e^{-\kappa_Lt}\|e_L(0)\|
+\frac{\gamma_L}{\sqrt{2\kappa_L}}\|\xi_L\|_{\infty,[0,t]}.
\tag{13}
$$
This follows by integrating (12) and using $I\preceq P_L\preceq\bar p_LI$, for as long as the scheduling and physical-domain hypotheses hold.

A second convex design may fix a relaxed bound
$\gamma_L\le(1+\epsilon_\gamma)\gamma_{L,\min}$ and minimize $\sum_i t_i$ subject to the same LMIs and
$$
X+X^\top\succeq2I_2,\qquad
\begin{bmatrix}t_iI_2&Y_i^\top\\Y_i&t_iI_2\end{bmatrix}\succeq0 .
\tag{14}
$$
Since $\|X^{-1}\|\le1$ under this normalization, $\|L_i\|\le\|Y_i\|\le t_i$.
This is a gain-norm upper-bound objective subject to the ISS requirement.

### Scheduling measurement error

Suppose the observer uses $u_m$ while the physical speed is $u_x$. Both signals must stay in the positive interval; the derivative hull must bound $\dot u_m$, the scheduling signal actually used. An amplitude bound on $u_m-u_x$ alone supplies no derivative bound.

The mismatch can nevertheless be included without assuming a new unproved lateral error bound. Define
$$
\Delta_A=[A_L(\rho_x)-A_L(\rho_m)]z,\qquad
\Delta_C=[C_L(\rho_x)-C_L(\rho_m)]z.
$$
Because $\det E_L=-m(l_f+l_r)/I_z\ne0$, the exact plant can be rewritten at $\rho_m$ using
$$
w_{\rm eff}=w_L+E_L^{-1}\Delta_A,\qquad
\nu_{\rm eff}=\nu_L+\Delta_C-F_LE_L^{-1}\Delta_A .
\tag{15}
$$
On the compact positive-speed domain these extra terms are bounded by a constant times $|u_m-u_x|$. Steering error is included by adding $B_L(\delta-\delta_m)$ to $\Delta_A$ and $D_L(\delta-\delta_m)$ to $\Delta_C$. Thus (12) applies with the corresponding effective disturbance vector, provided the measured scheduling trajectory remains admissible.

## Velocity transport and the sideslip interface

If the lateral-observer point lies $d$ metres forward of the global output point, rigid-body kinematics give
$$
v_y=v_y^I-d r,\qquad
\hat v_y=\hat v_y^I-d r_m,\qquad
|\hat v_y-v_y|\le\|e_L\|+|d|\,|\nu_r|.
\tag{16}
$$
Bounded lever-arm calibration error adds a term proportional to the bounded yaw rate. Let $u_m$ be the measured longitudinal velocity at this output point. Define the angle command
$$
\beta_c=\operatorname{atan2}(\hat v_y,u_m),\qquad
\beta=\operatorname{atan2}(v_y,u_x).
$$
On a common positive-longitudinal-speed domain $u_x,u_m\ge u_*>0$, the segment between these velocity vectors remains in the right half-plane. Since the gradient norm of $\operatorname{atan2}$ is $1/\|[u_x,v_y]\|$,
$$
|\beta_c-\beta|\le u_*^{-1}
\sqrt{|u_m-u_x|^2+|\hat v_y-v_y|^2}.
\tag{17}
$$

The angle filter is
$$
\dot{\hat\beta}=\hat\omega,\qquad
\dot{\hat\omega}=\omega_n^2\operatorname{wrap}(\beta_c-\hat\beta)
-2\zeta\omega_n\hat\omega,\qquad \omega_n,\zeta>0.
\tag{18}
$$
It does not modify $\hat v_y$. On the branch $|\beta_c-\hat\beta|<\pi$, put
$z_\beta=[\hat\beta-\beta,\hat\omega-\dot\beta]^\top$. Then
$$
\dot z_\beta=A_\beta z_\beta+B_\beta
\bigl[\omega_n^2(\beta_c-\beta)-d_\beta\bigr],
\quad
A_\beta=\begin{bmatrix}0&1\\-\omega_n^2&-2\zeta\omega_n\end{bmatrix},
\quad B_\beta=\begin{bmatrix}0\\1\end{bmatrix},
\quad d_\beta=\ddot\beta+2\zeta\omega_n\dot\beta .
\tag{19}
$$
Thus zero LPV error alone does not eliminate angle-filter lag. For any $Q_\beta\succ0$, let $P_\beta\succ0$ solve
$A_\beta^\top P_\beta+P_\beta A_\beta=-Q_\beta$. Young's inequality, (16), and (17) give constants $a_\beta,b_{\beta L},c_\beta>0$ such that
$$
\dot V_\beta\le-a_\beta\|z_\beta\|^2+b_{\beta L}\|e_L\|^2+c_\beta\|w\|^2,
\qquad V_\beta=z_\beta^\top P_\beta z_\beta ,
\tag{20}
$$
where $w$ includes sensor/output-point errors and $d_\beta$.

This forcing is a physical maneuver term. If $j=\dot a$, then
$$
\dot\beta=\frac{v\times a}{s^2}-r,\qquad
\ddot\beta=\frac{v\times j}{s^2}
-2\frac{(v\times a)(v^\top a)}{s^4}-\dot r .
\tag{21}
$$
Positive speed and bounded velocity, acceleration, jerk and yaw acceleration bound these terms. Their bounds are assumptions on the physical trajectory, not assumed bounds on the observer's own errors.

## Global kinematics, outputs and normalization

### Intrinsic model

The global plant is
$$
\begin{aligned}
\dot p&=v,& \dot v&=a,&
\dot a&=q^2v+2qJa+d_j,\\
\dot\psi&=r,& \dot r&=d_r,& q&=r+\dot\beta .
\end{aligned}
\tag{22}
$$
For a differentiable trajectory satisfying (1),
$d_j=\dot b\,t_\phi+s\dot q\,n_\phi$. Thus the predictor does not require constant speed or course rate: departures from that ideal intrinsic model are explicit process disturbances. Denote its drift without $(d_j,d_r)$ by $f_G(x,\dot\beta)$.

The five scalar outputs are
$$
h(x,\beta)=
\begin{bmatrix}
v^\top v\\v^\top a\\v\times a\\n_{\psi+\beta}^\top v\\r
\end{bmatrix},\qquad
y_h=
\begin{bmatrix}
u_m^2+\hat v_y^2\\
u_m a_{x,m}+\hat v_y a_{y,m}\\
u_m a_{y,m}-\hat v_y a_{x,m}\\
0\\r_m
\end{bmatrix}.
\tag{23}
$$
The ideal body-frame acceleration is $R(\psi)^\top a$. Sensor-location, gravity-compensation and acceleration errors are included in the measurement disturbance. Shared primitive velocity and acceleration errors produce correlated errors in the first three outputs. The fourth entry is a kinematic constraint, not an independent physical sensor.

For $H(x,\beta)=\partial h/\partial x$, let
$$
\begin{gathered}
\mathcal T=\operatorname{diag}
(\theta,\theta^2,\theta^3,\theta,\theta^2,\theta^3,\theta/\ell,\theta^2/\ell),\\
\epsilon=\mathcal T^{-1}(x-\hat x),\qquad \theta>0,\quad\ell>0,\\
\mathcal D=\operatorname{diag}
\left(\frac1{2V\theta^2},\frac1{V\theta^3},\frac1{V\theta^3},
\frac1{\theta^2},\frac{\ell}{\theta^2}\right),\qquad
\bar H=\mathcal D H\mathcal T .
\end{gathered}
\tag{24}
$$
These are fixed invertible normalizations. They contain no inverse of the instantaneous vehicle speed.

Let $C$ select coordinates $(1,4,7)$ and $C_p$ select $(1,4)$ from an eight-vector. For the physical pose $\pi=[X,Y,\psi]^\top$, define
$$
D_\pi=\operatorname{diag}(1,1,1/\ell),\qquad
\eta=D_\pi^{-1}(\hat\pi-\pi)=-\theta C\epsilon,\qquad
\mathcal C_\pi=D_\pi^{-1}C\mathcal T=\theta C .
\tag{25}
$$
Heading differences in (25) use the same local angular branch.

### GNSS observation

For a fixed body-frame lever $d_G\in\mathbb R^2$,
$$
y_G=p+R(\psi)d_G+\nu_G .
\tag{26}
$$
A positive-semidefinite information matrix $I_G$ supplies a bounded weight
$W_G=I_G(I_G+\lambda_GI_2)^{-1}$, $\lambda_G>0$.
The nominal synthesis below assumes $W_G=wI_2$, with fixed $w\in[0,1]$.
The case $w=0$ is a separately declared no-GNSS design. Mere packet availability does not imply isotropy or the chosen nominal information strength.

### Semantic overlap field

For each admitted class, let the fixed and source distributions have component means $m_i,\mu_j$, positive covariances $P_i,Q_j$, and nonnegative weights $u_i,v_j$. Class balancing and evidence weighting determine these weights. At query pose $q_\pi=[t^\top,\chi]^\top$ put
$$
d_{ij}=R(\chi)\mu_j+t-m_i,\qquad
\Sigma_{ij,\sigma}=P_i+R(\chi)Q_jR(\chi)^\top+\sigma^2I_2 .
$$
For a fixed preparation anchor $\bar\pi$, the frame-overlap cost is
$$
\begin{aligned}
E_\sigma(q_\pi;\bar\pi)
 &=\sum_{\text{same-class }i,j}u_i v_j
    \mathcal N(d_{ij};0,\Sigma_{ij,\sigma}),\\
\mathcal L(q_\pi;\bar\pi)&=-\sum_{\sigma\in\mathcal S}
               \log E_\sigma(q_\pi;\bar\pi),\qquad
g_L(\bar\pi)=
\left.\frac{\partial\mathcal L(q_\pi;\bar\pi)}{\partial q_\pi}\right|_{q_\pi=\bar\pi}.
\end{aligned}
\tag{27}
$$
Class normalization factors are included in $u_i v_j$.
Map cropping, view conditioning and source preparation are frozen when taking this partial derivative. Their dependence on the preparation anchor remains part of the resulting feedback field $g_L$. In general, $g_L(\bar\pi)$ need not be the gradient of one globally fixed scalar potential.

Covariance rotation contributes to the yaw derivative in (27). With pair responsibilities
$\pi_{ij,\sigma}=u_i v_j\mathcal N(d_{ij};0,\Sigma_{ij,\sigma})/E_\sigma$ and
$J_{ij}=\partial d_{ij}/\partial q_\pi$, the geometric information surrogate is
$$
M_L=\sum_{\sigma,i,j}\pi_{ij,\sigma}
J_{ij}^\top\Sigma_{ij,\sigma}^{-1}J_{ij}\succeq0 .
\tag{28}
$$
It is not generally the Hessian of $\mathcal L$. Mixture-responsibility and covariance-derivative terms, as well as anchor-dependent preparation, matter to the actual field derivative.

The alternative landmark cost replaces each frame energy by
$$
-\sum_j v_j\log\left(\varepsilon_o+
 \sum_{i:\,c_i=c_j}u_i k_{ij}\mathcal N(d_{ij};0,\Sigma_{ij,\sigma})\right),
\quad \varepsilon_o>0 .
\tag{29}
$$
An optional axis factor has the form
$k_{ij}=\exp[-\kappa_{ij}\sin^2(\vartheta_i-\vartheta_j-\chi)/2]$,
with finite nonnegative $\kappa_{ij}$ determined by support-axis uncertainty.
The stability argument applies to either cost through the actual-field condition below.

For $\bar M=D_\pi^\top M_LD_\pi=U\operatorname{diag}(\mu_i)U^\top$,
$\mu_i\ge0$, regularization $\lambda_L>0$, and reliability factors
$\rho_i\in[0,1]$, define
$$
\begin{aligned}
F_L^\pi&=U\operatorname{diag}\left(\frac{\rho_i}{\mu_i+\lambda_L}\right)U^\top,\\
S_L^\pi&=F_L^\pi\bar M
 =U\operatorname{diag}\left(\frac{\rho_i\mu_i}{\mu_i+\lambda_L}\right)U^\top,\\
\Pi_L&=\text{orthogonal projector onto the admitted information range},\\
\rho_L(\hat\pi)&=F_L^\pi\Pi_LD_\pi^\top g_L(\hat\pi).
\end{aligned}
\tag{30}
$$
Reliability is constant on each repeated eigenspace, making the matrices independent of its arbitrary eigenvector basis. No positive eigenvalue is invented for an unsupported direction.

**Actual LiDAR condition.** On a correct local-alignment patch, uniformly in time and in admissible source preparation,
$$
\rho_L(\hat\pi)=K_L(\eta)\eta+b_L,\qquad
\operatorname{Sym}K_L(\eta)\succeq S_*,
\tag{31}
$$
where
$$
S_*=\operatorname{diag}(p_*,p_*,h_*)-(b_*+\delta_*)I_3\succ0 .
\tag{32}
$$
Here $p_*,h_*>0$ are pose-strength bounds, $b_*\ge0$ bounds translation-heading coupling, $\delta_*\ge0$ bounds actual-operator mismatch, and $b_L$ is a bounded observation bias. The repeated translation bound makes $S_*$ invariant under any rotation of the horizontal axes.

A sufficient local interpretation is a uniform bound on the symmetric filtered strength $S_L^\pi$
with translation block at least $p_*I_2$, yaw entry at least $h_*$ and off-diagonal block norm at most $b_*$, together with an actual-operator discrepancy of norm at most $\delta_*$. The off-diagonal matrix has norm at most $b_*$, which proves (32). The discrepancy must concern the complete normalized field, including its preparation and regularization dependence.

Assume the actual field is measurable in time and uniformly locally Lipschitz in pose, ensuring a locally well-posed observer. If this field is continuously differentiable in pose, one can set
$$
b_L=\rho_L(\pi),\qquad
K_L(\eta)=\int_0^1 D_\zeta\widetilde\rho_L(t\eta)\,dt,\qquad
\widetilde\rho_L(\zeta)=\rho_L(\pi+D_\pi\zeta).
$$
The derivative here is the total derivative of the actual normalized field: it includes anchor-dependent preparation, $F_L^\pi$, and $\Pi_L$. At a smooth anchor $a$, for a normalized direction $h$, the product and chain rules give
$$
\begin{aligned}
D_\zeta\widetilde\rho_L(\zeta)[h]
={}&\bigl(D_aF_L^\pi[D_\pi h]\,\Pi_L
       +F_L^\pi D_a\Pi_L[D_\pi h]\bigr)D_\pi^\top g_L\\
&+F_L^\pi\Pi_LD_\pi^\top
 \bigl(\mathcal L_{qq}+\mathcal L_{qa}\bigr)D_\pi h,
\qquad a=\pi+D_\pi\zeta,\quad(q,a)=(a,a).
\end{aligned}
$$
The mixed derivative $\mathcal L_{qa}$ describes the anchor-dependent preparation omitted by the frozen-anchor partial gradient. A uniform bound on the difference between this full normalized derivative and $S_L^\pi$ supplies the discrepancy bound used in (32). A sufficient differentiable setting keeps the admitted support fixed with a uniform spectral gap and makes all preparation and reliability maps smooth. Support changes or hard crop boundaries require the finite contract and well-posedness to be justified directly; the partial cost Hessian alone is insufficient. Condition (31) is a geometric assumption on this integral operator. Full rank of $M_L$, a positive overlap score, or a positive yaw diagonal alone does not establish it. Correct map alignment and adequate geometry are separate assumptions.

## Continuous global observer

Define the bounded schedule and auxiliary weight matrix
$$
\chi(s_m^2)=\frac{V^2}{s_0^2+s_m^2},\qquad
s_m^2=u_m^2+\hat v_y^2,\qquad
K_h=\operatorname{diag}(\chi c_1,\chi c_2,\chi c_2,c_3,0),
\tag{33}
$$
with $c_1,c_2,c_3>0$. Let $\kappa>0$, $g_r>0$, and $\gamma_r=\theta g_r$.
With $P=P^\top\succ0$, the complete continuous observer is
$$
\begin{aligned}
\dot{\hat x}={}&f_G(\hat x,\hat\omega)
+\mathcal T\kappa P^{-1}C_p^\top W_G
       [y_G-\hat p-R(\hat\psi)d_G]\\
&+\theta\mathcal T P^{-1}
 \bar H(\hat x,\hat\beta)^\top K_h\mathcal D
       [y_h-h(\hat x,\hat\beta)]\\
&+\gamma_r e_8(r_m-\hat r)
-\frac{\kappa}{\theta}\mathcal T P^{-1}\mathcal C_\pi^\top\rho_L(\hat\pi).
\end{aligned}
\tag{34}
$$
The gyro row has zero weight in $K_h$ and is used once, in its separate rate-state correction. The predictor uses the internal rate state and the upstream angle-rate state. Raw acceleration and gyro samples occur in measurement corrections and the lateral observer.

The common pose gain $\kappa$ is shared by LiDAR and GNSS, while their observation weights and noise scales are distinct. Every correction acts on the same global state. The lateral observer receives no feedback from (34).

The gain recovery is entirely specified by (24), (30), (33), and (34):
the auxiliary physical injection matrix is
$\theta\mathcal T P^{-1}\bar H^\top K_h\mathcal D$,
the GNSS gain before $W_G$ is $\mathcal T\kappa P^{-1}C_p^\top$,
and the normalized LiDAR metric gain is $\kappa/\theta$.

## Single convex design

### Fixed metric structure and gyro design

Use the rotation-invariant metric
$$
P=\operatorname{diag}(P_c,P_c,P_h),\quad
P_c=U^\top\operatorname{diag}(p_c,\mathcal R)U,\quad
U=\begin{bmatrix}1&-1&1\\0&1&0\\0&0&1\end{bmatrix},
\quad P\succeq I_8 ,
\tag{35}
$$
where $p_c>0$, $\mathcal R=\mathcal R^\top\succ0$ and $P_h\succ0$.
A diagonal $P_h$ is an admissible additional structural restriction.
This fixed shear defines a particular convex metric class; it is not a parameterization of every positive-definite metric.

Partitioning $P_c$ into position and motion coordinates gives the Schur complement
$$
P_c(2{:}3,2{:}3)-P_c(2{:}3,1)P_c(1,1)^{-1}P_c(1,2{:}3)
=\mathcal R,\qquad (P_c^{-1})_{2{:}3,2{:}3}=\mathcal R^{-1}.
\tag{36}
$$
This identity follows directly from $U$. It makes the motion metric affine in the same optimization variables as $P$.

Let $\sigma_{\dot r}>0$ and $\sigma_r>0$ be yaw-acceleration and gyro-error design scales. With $\eta_r=\sigma_{\dot r}/\sigma_r$, the independent scalar design is
$$
\min_{\gamma_r>0,\ q_r}q_r,\qquad
\begin{bmatrix}q_r&\eta_r&\gamma_r\\
\eta_r&2\gamma_r&0\\
\gamma_r&0&2\gamma_r\end{bmatrix}\succeq0 .
\tag{37}
$$
The Schur complement gives
$q_r\ge\eta_r^2/(2\gamma_r)+\gamma_r/2$.
Its unconstrained optimum is $\gamma_r=\eta_r$, $q_r=\eta_r$.
If a positive lower gain bound $\underline\gamma$ is imposed, the optimum is
$\gamma_r=\max(\eta_r,\underline\gamma)$ with the displayed value of $q_r$.
Set $g_r=\gamma_r/\theta$ in the global design. This fixes a stabilizing scalar rate channel; it does not jointly optimize the gyro gain over every possible global metric.

### Course-aligned nominal matrices

Let $\mathcal E_v=[e_2,e_5]$ and $\mathcal E_a=[e_3,e_6]$ in $\mathbb R^8$.
At the true course-aligned state $v=[s,0]^\top$, $a=[b,sq]^\top$, put
$d=s_0^2+s^2$ and
$$
f=(f_1,\ldots,f_8)=
\left(q,q^2,b,\frac{s^2}{d},\frac{sb}{d},\frac{s^2q}{d},s,s^2\right).
\tag{38}
$$
Define
$$
\begin{aligned}
A_{\rm ch}&=e_1e_2^\top+e_2e_3^\top+e_4e_5^\top+e_5e_6^\top+e_7e_8^\top,\\
A_0(f)&=A_{\rm ch}
+\frac{f_2}{\theta^2}\mathcal E_a\mathcal E_v^\top
+\frac{2f_1}{\theta}\mathcal E_aJ\mathcal E_a^\top
+\frac{2f_3}{\theta^2\ell}e_6e_8^\top-g_re_8e_8^\top,\\
G_s(f)&=f_4e_2e_2^\top,\\
T_a&=e_2e_3^\top-e_5e_6^\top,\qquad
N_a=e_2e_6^\top+e_5e_3^\top,\\
G_a(f)&=f_4(e_3e_3^\top+e_6e_6^\top)
+\frac{f_5}{\theta}\operatorname{He}(T_a)
+\frac{f_6}{\theta}\operatorname{He}(N_a),\\
G_d(f)&=e_5e_5^\top-\frac{f_7}{\theta\ell}
       (e_5e_7^\top+e_7e_5^\top)
+\frac{f_8}{(\theta\ell)^2}e_7e_7^\top .
\end{aligned}
\tag{39}
$$
The coefficient $A_0(6,8)$ follows from
$2qv+2Ja=[0,2b]^\top$ in this frame.

The exact scheduled squared-speed Gram equals $G_s$.
For the normalized dot/cross rows
$$
C_a=\frac1V\begin{bmatrix}
b/\theta&sq/\theta&s&0\\
sq/\theta&-b/\theta&0&s
\end{bmatrix}
$$
in the order $(\epsilon_{V_X},\epsilon_{V_Y},\epsilon_{A_X},\epsilon_{A_Y})$,
their scheduled Gram equals
$$
\chi C_a^\top C_a
=G_a+\frac{b^2+s^2q^2}{\theta^2d}
(e_2e_2^\top+e_5e_5^\top)\succeq G_a .
\tag{40}
$$
The omitted diagonal is positive semidefinite; every cross term is retained.
The exact direction Gram is $G_d=(e_5-s e_7/(\theta\ell))(e_5-s e_7/(\theta\ell))^\top$ at physical values of $f$.

### Analytic coefficient enclosure

Let $Q_e=\min(Q,A/s_0)$. Every physical point in (2) satisfies the following independent intervals:
$$
\begin{array}{c|c}
f_1&[-Q_e,Q_e]\\
f_2&[0,Q_e^2]\\
f_3&[-A,A]\\
f_4&[1/2,V^2/(s_0^2+V^2)]\\
f_5&[-A/(2s_0),A/(2s_0)]\\
f_6&[-\min(Q_e,A/(2s_0)),\min(Q_e,A/(2s_0))]\\
f_7&[s_0,V]\\
f_8&[s_0^2,V^2].
\end{array}
\tag{41}
$$
For example $s/(s_0^2+s^2)\le1/(2s_0)$, $|sq|\le A$, and $s^2/d\le1$ imply the mixed bounds. These intervals form an outer box. Its vertices need not correspond to physically realizable motion or positive Gram matrices.

The matrix
$$
\begin{aligned}
\mathcal F(f)={}&
\operatorname{He}\bigl(PA_0(f)-c_1G_s(f)-c_2G_a(f)-c_3G_d(f)\bigr)\\
&-2\kappa C^\top S_*C-\kappa w C_p^\top C_p
+\kappa w\rho_G^2e_7e_7^\top+\lambda I_8,\qquad
\rho_G=\|d_G\|/\ell,\quad\lambda>0
\end{aligned}
\tag{42}
$$
is affine in $f$ for fixed decisions. Impose
$$
\mathcal F(f^{[j]})\prec0
\quad\text{at every Cartesian vertex of (41)}.
\tag{43}
$$
There are at most $2^8=256$ such conditions; degenerate intervals reduce this number.

**Proposition 2.** Conditions (31)--(32) and (43), with nominal isotropic GNSS weight $wI_2$, give a uniform strict nominal global-error dissipation margin on the physical domain.

**Proof.** For a nondegenerate interval write
$f_i=(1-t_i)\underline f_i+t_i\overline f_i$, $t_i\in[0,1]$.
Products of $t_i$ and $1-t_i$ give nonnegative vertex weights summing to one, so $\mathcal F(f)$ is exactly their convex combination. Hence it is negative throughout the outer box and on its physical subset. Equation (40) only adds nominal dissipation to this bound.

In course coordinates, the negative LiDAR contribution is bounded by
$-2\theta\kappa\epsilon^\top C^\top S_*C\epsilon$.
The translation part of $S_*$ is isotropic, so this bound is preserved by the course rotation. Invariance of $P$ alone would not preserve an arbitrary anisotropic pose bound.

For the finite GNSS lever error,
$\|[R(\psi)-R(\hat\psi)]d_G\|
\le\|d_G\||\psi-\hat\psi|$.
Young's inequality bounds its cross term with position by
$$
-\theta\kappa w\|C_p\epsilon\|^2+
\theta\kappa w\rho_G^2|\epsilon_7|^2 .
\tag{44}
$$
The corresponding nominal Jacobian therefore satisfies
$\dot V_G\le-\theta\lambda\|\epsilon\|^2$ before nonlinear remainders and disturbances, where $V_G=\epsilon^\top P\epsilon$.

More explicitly, simultaneous planar rotations act through
$\mathcal Q=\operatorname{diag}(R(\phi)\otimes I_3,I_2)$.
Both $\mathcal T$ and $P$ commute with this action.
The additional rotating-frame term is skew with respect to $P$ and contributes zero to $\dot V_G$. This justifies a single course-aligned calculation for every heading. $\square$

### Motion-response and feedback-authority requirements

For a prescribed nominal rate $\alpha>0$, the position-independent straight-course motion subsystem uses
$$
A_m=\begin{bmatrix}0&\theta\\0&0\end{bmatrix},\qquad
\dot z_m=\left[A_m-\theta\mathcal R^{-1}
 \left(c_1\begin{bmatrix}r_s&0\\0&0\end{bmatrix}
 +c_2\begin{bmatrix}0&u_s/\theta\\u_s/\theta&r_s\end{bmatrix}\right)\right]z_m ,
$$
where $z_m=[\epsilon_{V_\parallel},\epsilon_{A_\parallel}]^\top$, $r_s=s^2/d$, and $u_s=sb/d$. With exact heading, exact packets, $q=0$, and pose correction absent, (36) gives this motion block. The omitted acceleration-dependent velocity diagonal in (40) adds dissipation.

Require at the four interval corners of $(r_s,u_s)$:
$$
\operatorname{He}(\mathcal R A_m)+2\alpha\mathcal R
-2\theta\left(
c_1\begin{bmatrix}r_s&0\\0&0\end{bmatrix}
+c_2\begin{bmatrix}0&u_s/\theta\\u_s/\theta&r_s\end{bmatrix}
\right)\prec0 .
\tag{45}
$$
Convex interpolation proves $\dot{(z_m^\top\mathcal R z_m)}
\le-2\alpha z_m^\top\mathcal R z_m$ for this nominal subsystem.
This is the mathematical role of the motion-response target.

Write $p_L=S_*(1,1)$, $h_L=S_*(3,3)$ and $\bar s=(s_0+V)/2$.
Three additional affine inequalities prescribe direct correction authority:
$$
\kappa p_L\ge\frac{\alpha}{\theta}P_{11},\qquad
\kappa h_L\ge\frac{\alpha}{\theta}P_{77},\qquad
c_3\left(\frac{\bar s}{\theta\ell}\right)^2
 \ge\frac{\alpha}{\theta}P_{77}.
\tag{46}
$$
For a positive-definite matrix, $(P^{-1})_{ii}\ge1/P_{ii}$.
Thus the respective nominal direct position, direct heading, and midpoint-speed direction-heading correction coefficients are at least $\alpha$.
These component-authority conditions and (45) are prescribed response requirements. They are not necessary conditions for ISS, and they do not prove full-pose settling time.

Normalized self-feedback caps are LMIs of the form
$$
P-\frac{k}{g_{\max}}e_ie_i^\top\succeq0
\quad\Longleftrightarrow\quad
k\,e_i^\top P^{-1}e_i\le g_{\max},
\qquad k\ge0,\quad g_{\max}>0 .
\tag{47}
$$
The equivalence follows by congruence with $P^{-1/2}$ and the single nonzero eigenvalue of a rank-one matrix. The design uses $(k,i)=(\kappa,1)$ for each pose-source position cap, $(c_3,5)$ for direction feedback, and $(\kappa,7),(c_3,7)$ for the two heading caps. They specify allowed authority; they are not runtime clipping rules.

### Primitive-noise objective and convexity

Let $\sigma_p,\sigma_\psi,\sigma_G,\sigma_v,\sigma_a,\sigma_r,\sigma_j,\sigma_{\dot r}$ be fixed positive design scales. For midpoint velocity $\bar v$ and acceleration $\bar a$, define the exact primitive increment map
$$
N(\bar v,\bar a)=\begin{bmatrix}
2\bar v_x&2\bar v_y&0&0&0\\
\bar a_x&\bar a_y&\bar v_x&\bar v_y&0\\
\bar a_y&-\bar a_x&-\bar v_y&\bar v_x&0\\
0&0&0&0&0\\
0&0&0&0&1
\end{bmatrix}.
\tag{48}
$$
Its first block row contains two velocity entries followed by three zeros; all rows have five entries in the order $(\delta v_x,\delta v_y,\delta a_x,\delta a_y,\delta r)$. The matrix gives the exact increments of the quadratic outputs through midpoint evaluation. The fourth row is zero.

At each fixed noise-design point $o$ use
$$
\begin{aligned}
D_N^{[o]}&=\mathcal D
 \left[N(v_o,a_o)\operatorname{diag}(\sigma_v,\sigma_v,\sigma_a,\sigma_a,\sigma_r),
             \ \sigma_v e_4\right],\\
\mathcal G_h^{[o]}&=-\theta\bar H_o^\top K_h^{[o]}D_N^{[o]}
 -P e_8\frac{g_r\ell\sigma_r}{\theta}e_5^\top,\\
\mathcal G_\pi&=[-\kappa C^\top S_0 D_p,\ -\kappa w C_p^\top D_G],\\
S_0&=\operatorname{diag}(p_*,p_*,h_*),\quad
D_p=\operatorname{diag}(\sigma_p,\sigma_p,\ell\sigma_\psi),\quad
D_G=\sigma_GI_2,\\
\mathcal G_d&=P[e_3,e_6,e_8]
 \operatorname{diag}(\sigma_j/\theta^3,\sigma_j/\theta^3,
                         \ell\sigma_{\dot r}/\theta^2).
\end{aligned}
\tag{49}
$$
In the gyro term of $\mathcal G_h$, $e_5^\top$ is the fifth coordinate selector in the six-dimensional primitive-input space. This term shares the existing gyro column. In the extra sixth column, $\sigma_v e_4$ is a declared direction-error design proxy. It does not assert an independent physical direction sensor.

The noise-design points are
$s\in\{s_0,(s_0+V)/2,V\}$ and
$a\in\{0,Ae_1,-Ae_1,Ae_2,-Ae_2\}$ in course coordinates, with duplicate speed points omitted. These fixed objective-evaluation points are distinct from the coefficient vertices used for stability.

For every column $\mathcal G_j$ impose an epigraph
$$
\begin{bmatrix}\eta_j&\mathcal G_j^\top\\\mathcal G_j&P\end{bmatrix}\succeq0
\quad\Longleftrightarrow\quad
\eta_j\ge\mathcal G_j^\top P^{-1}\mathcal G_j.
\tag{50}
$$
Minimize the sum of these column bounds for pose and process channels plus their average over the auxiliary noise-design points. Keeping each primitive column intact preserves its cross-effects across outputs. This objective is a normalized Lyapunov-input energy proxy under the declared scales and representative information $S_0$; it is not a measured covariance, a bound on every omitted disturbance, or a minimum attainable position RMSE.

For fixed $\theta,\ell,\lambda,\alpha,w,g_r$ and operating-domain bounds, the variables are $p_c,\mathcal R,P_h,\kappa,c_1,c_2,c_3$ and the epigraph scalars. Equations (35), (43), (45)--(47), and (50) are affine matrix/scalar inequalities in these variables. The objective is linear. Thus they define one SDP. Optional fixed metric-balance restrictions preserve convexity. All inverse metrics occur only in recovered gains or in the Schur-complement interpretation, not as nonlinear decision expressions in the SDP.

## Finite nonlinear errors and the local global-observer bound

A nominal Jacobian certificate does not by itself prove the same inequality for arbitrary estimation errors. The distinction can be seen directly in the quadratic outputs. Put $\delta v=v-\hat v$, $\delta a=a-\hat a$ and use arithmetic midpoints $\bar v=(v+\hat v)/2$, $\bar a=(a+\hat a)/2$. Then
$$
\begin{aligned}
\|v\|^2-\|\hat v\|^2&=2\bar v^\top\delta v,\\
v^\top a-\hat v^\top\hat a&=\bar a^\top\delta v+\bar v^\top\delta a,\\
v\times a-\hat v\times\hat a&=(-J\bar a)^\top\delta v+(J\bar v)^\top\delta a.
\end{aligned}
\tag{51}
$$
The innovation therefore uses a midpoint Jacobian, whereas its injection in (34) uses a Jacobian at the estimate. Their product is generally not a positive Gram matrix. A claim of global dissipation obtained by identifying these two matrices would be invalid.

Let the ideal upstream packet use the true velocity, sideslip, and sideslip rate. On a uniform smooth neighborhood of the admissible physical set, the difference between the exact ideal error dynamics and its nominal linearization is bounded by $c_N\|\epsilon\|^2$, with finite $c_N\ge0$. This follows from Taylor's theorem with bounded second derivatives. It includes the estimate-dependent auxiliary Jacobian and speed schedule, the intrinsic predictor, and angular innovations. The actual LiDAR term is instead handled directly by its finite-increment condition (31), with $b_L$ counted as a disturbance. No positive-Gram assertion is needed for the finite auxiliary error.

The packet and predictor perturbations caused by the lateral and angle-interface errors are bounded by a constant times
$\|e_L\|+\|z_\beta\|+\|w\|$ on this neighborhood. This follows from (16)--(19), bounded velocities/accelerations, and smoothness of the packet and predictor. The disturbance vector $w$ includes the scaled intrinsic residuals $d_j,d_r$, effective lateral disturbances in (15), GNSS noise, LiDAR bias $b_L$, sensor/calibration errors, and $d_\beta$. It is a finite-dimensional stacking of these signals; their correlations do not matter.

Let $p_{\max}=\lambda_{\max}(P)$. Proposition 2 and these bounds give constants $L_N,c_N<\infty$ such that
$$
\dot V_G\le-\theta\lambda\|\epsilon\|^2
 +2p_{\max}c_N\|\epsilon\|^3
 +2p_{\max}L_N\|\epsilon\|
   (\|e_L\|+\|z_\beta\|+\|w\|).
\tag{52}
$$
Choose a sufficiently small radius, for example
$r_G\le\theta\lambda/(8p_{\max}c_N)$ when $c_N>0$, also contained in the smooth alignment neighborhood. Apply Young's inequality to each cross term, reserving a positive fraction of the quadratic margin. This gives constants $a_G>0$, $b_{GL},b_{G\beta},c_G\ge0$ such that
$$
\dot V_G\le-a_G\|\epsilon\|^2
 +b_{GL}\|e_L\|^2+b_{G\beta}\|z_\beta\|^2+c_G\|w\|^2
\quad\text{for }\|\epsilon\|\le r_G.
\tag{53}
$$
This proves a local nonlinear ISS estimate for the main observer with upstream errors as inputs. The radius is an analytical existence bound; no numerical attraction radius is asserted by the nominal SDP. Uniformity of the smoothness, measurement-sector, and operating-domain hypotheses is essential.

## Complete in-domain cascade theorem

**Theorem.** Suppose the lateral LMIs (11), the nominal global LMIs (43), and the actual-field condition (31) hold with strict margins. Suppose the true trajectory and measured scheduling signal remain in their declared positive-speed domains, including the measured scheduling-derivative bound. Suppose the stated uniform smoothness, angular-branch, nominal GNSS-weight, and motion-output-availability hypotheses hold on a neighborhood of the true trajectory. Then the complete error
$$
E=[e_L^\top,z_\beta^\top,\epsilon^\top]^\top\in\mathbb R^{12}
$$
is locally ISS with respect to the combined disturbance $w$. Initial errors may be nonzero, provided they and the disturbance are sufficiently small for the common neighborhood. Arbitrary bounded initial errors are not covered.

**Proof.** After collecting the effective lateral input into $w$, (12), (20), and (53) have the triangular form
$$
\begin{aligned}
\dot V_L&\le-a_L\|e_L\|^2+c_L\|w\|^2,\qquad a_L=2\kappa_L>0,\\
\dot V_\beta&\le-a_\beta\|z_\beta\|^2+b_{\beta L}\|e_L\|^2+c_\beta\|w\|^2,\\
\dot V_G&\le-a_G\|\epsilon\|^2+b_{GL}\|e_L\|^2
+b_{G\beta}\|z_\beta\|^2+c_G\|w\|^2.
\end{aligned}
\tag{54}
$$
The first inequality uses $P_L\succeq I$. Choose positive weights satisfying
$$
\mu_\beta>\frac{b_{G\beta}}{a_\beta},\qquad
\mu_L>\frac{b_{GL}+\mu_\beta b_{\beta L}}{a_L},\qquad
V=V_G+\mu_\beta V_\beta+\mu_LV_L.
\tag{55}
$$
There are finite uniform constants $0<q_-\le q_+$ such that
$q_-\|E\|^2\le V\le q_+\|E\|^2$. The strict inequalities in (55) imply
$$
\dot V\le-a_*\|E\|^2+C_*\|w\|^2
\le-\nu V+C_*\|w\|^2,\qquad \nu=a_*/q_+>0,
\tag{56}
$$
where $a_*$ is the minimum of the three remaining positive quadratic coefficients and
$C_*=c_G+\mu_\beta c_\beta+\mu_Lc_L$.

It remains to justify that the local inequalities continue to apply. Choose $r>0$ so that $\|E\|<r$ stays within the smooth neighborhood, the correct LiDAR alignment patch, and every required angle branch; require $r\le r_G$. The positive measured-speed and measured-derivative conditions remain separate signal hypotheses. Also choose an input bound $\delta_0>0$ sufficiently small that algebraic measurement perturbations keep the angle command and all local maps inside this neighborhood. Choose a sublevel $v_0<q_-r^2$. If
$$
V(0)<v_0,\qquad \|w\|_\infty<\delta_0,\qquad
C_*\|w\|_{\infty}^2\le\nu v_0/2,
\tag{57}
$$
then on the boundary $V=v_0$, (56) gives $\dot V\le-\nu v_0/2<0$. A first-exit argument proves forward invariance of this error sublevel. When $C_*=0$, no disturbance-size restriction is needed from (57). This proves error-neighborhood invariance; it does not prove invariance of the vehicle's physical operating set.

Integration of (56) now gives
$$
\|E(t)\|\le
\sqrt{\frac{q_+}{q_-}}e^{-\nu t/2}\|E(0)\|
+\sqrt{\frac{C_*}{\nu q_-}}\|w\|_{\infty,[0,t]}.
\tag{58}
$$
The coordinate scaling $\mathcal T$ is fixed and invertible, so (58) also bounds the physical global error with changed constants. This proves the stated local ISS result. $\square$

Zero initial error is sufficient but unnecessary. Sufficiently small nonzero initial errors are allowed by (57). A disturbance-free exponential-convergence statement requires all modeled forcing to vanish, including map bias, intrinsic residuals, and the angle-filter forcing $d_\beta$. An exact lateral estimate with a time-varying sideslip does not in general meet that last condition.

## Scope of the stability statement

The gain inequalities establish lateral ISS and a strict nominal simultaneous-fusion margin. The complete nonlinear theorem additionally needs the actual LiDAR-field sector, smoothness, admissible scheduling, and local-neighborhood hypotheses. A feasible SDP alone does not verify those additional hypotheses for a particular map or recording.

The nominal global certificate uses one fixed GNSS weight and a uniformly informative LiDAR field together. It is not a certificate for arbitrary switching between GNSS-only and LiDAR-only sensing. A configuration with $w=0$ can instead be synthesized and checked as a distinct LiDAR-supported design. GNSS availability by itself is not the present mathematical definition of sufficient information. To guarantee the logical alternative “adequate GNSS or nondegenerate LiDAR,” one would need inequalities covering both sensor regimes with the required common metric or a justified switching argument; those conditions are not implied by (43).

The positive-speed theorem does not cover zero speed, loss of lateral validity, leaving and re-entering the LPV domain, or a complete drive–stop–restart cycle. The global prediction and LiDAR corrections contain no reciprocal instantaneous-speed term, so they can still operate outside that theorem. At standstill an informative map-referenced LiDAR measurement can constrain position and heading. A stationary single-point GNSS position measurement and yaw-rate measurement cannot determine a constant heading offset. No heading-stability claim is made when that absolute heading information is absent.

These are continuous-time local tracking results. They do not establish global map capture, convergence from an arbitrary pose offset, robustness to incorrect associations, or a sampled-data guarantee for a particular numerical integration interval.

## Relation to the observer literature

The high-gain normalization and multi-output injection follow the general architecture of Bessafa et al., *Generalized multi-output high-gain observer with application to ego vehicle trajectory and orientation estimation*, Automatica 188 (2026), 112915, DOI: [10.1016/j.automatica.2026.112915](https://doi.org/10.1016/j.automatica.2026.112915). Their Lemma 1 motivates explicit finite-increment bounds; their Remark 3 treats a compact invariant operating region as an assumption. Neither reference supplies the map-specific sector condition or the cascade proof above. The lateral scheduling reduction, information-weighted overlap field, gain-design objectives, and composite local theorem must be assessed through the equations and hypotheses stated here.
