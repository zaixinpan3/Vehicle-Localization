# Continuous-time ISS of the improved seven-state MO-HGO

Revision date: 2026-09-13.

This is the current theoretical specification. It treats two separate,
continuously measured systems: GNSS position, and uniformly informative LiDAR
pose with a known constant delay. The seven-state vehicle model and observer
are continuous-time differential equations. The LiDAR observer is a retarded
functional differential equation because its correction uses a past estimate.
Numerical integration does not change this modeling choice.

The matrices used below are constant within each continuous measurement mode.
The modes have separate gains and certificates; no claim about switching
between them is made.

The GNSS interface in this repository measures position only. Consequently,
its seven-state result requires motion and a local heading chart. An optional
independent GNSS heading output removes that motion requirement, as stated
separately below. LiDAR measures all three planar pose coordinates and its
fixed delay is retained explicitly. Neither measurement continuity nor
full-rank information alone is a stability certificate.

The current `runImprovedVehicleObserver.m` integrates the GNSS equations (8)
and the delayed LiDAR equation (23). Its numerical steps approximate these
continuous equations. The implementation uses separate, reverified constant
certificates, an explicit LiDAR initial history, and immutable state outputs.
See [the runtime contract](localization/README.md) and
[implementation validation](research/continuous_observer_runtime_20260913/validation.md).
Numerical success does not verify all physical, initial-error or disturbance
hypotheses.

## 1. State, continuous plant, and disturbance interface

Let

$$
z=[X,V_X,A_X,Y,V_Y,A_Y,\psi]^\top,\quad
v_g=[V_X,V_Y]^\top,\quad a_g=[A_X,A_Y]^\top,
$$

and let $e_i$ denote the $i$th column of $I_7$. Define

$$
A_3=\begin{bmatrix}0&1&0\\0&0&1\\0&0&0\end{bmatrix},\quad
A=\operatorname{diag}(A_3,A_3,0),\quad
B=[e_3,e_6],\quad b(t)=e_7 r_m(t),
$$

$$
F(q)=\begin{bmatrix}
0&q^2&0&0&0&-2q&0\\
0&0&2q&0&q^2&0&0
\end{bmatrix},\qquad M(q)=A+BF(q).
$$

The measured course-rate interface is $q=r_m+\dot{\hat\beta}$, and the plant is

$$
\boxed{\dot z=M(q(t))z+b(t)+w(t).} \tag{1}
$$

Here $w=Bd_j-e_7\nu_r$, with $r_m=\dot\psi+\nu_r$. Other bounded model
residuals can also be included in $w$. In particular, $d_j$ is not set to zero
merely to obtain a stability result.

For nonzero speed, write $v_g=v[\cos\theta_c,\sin\theta_c]^\top$,
$\theta_c=\psi+\beta$, $q_c=\dot\theta_c$, and
$J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$. Differentiation gives

$$
\dot a_g=q_c^2v_g+2q_cJa_g+
\ddot v\begin{bmatrix}\cos\theta_c\\\sin\theta_c\end{bmatrix}
+\dot q_cJv_g. \tag{2}
$$

Thus the precise model residual is

$$
d_j=\ddot v\begin{bmatrix}\cos\theta_c\\\sin\theta_c\end{bmatrix}
+\dot q_cJv_g+(q_c^2-q^2)v_g+2(q_c-q)Ja_g. \tag{3}
$$

Equation (1) itself has no division by speed. At zero speed one defines $d_j$
from the actual Cartesian jerk minus the nominal jerk, without introducing
an undefined course angle. Bounded $q$ and bounded $w$ are explicit hypotheses,
not consequences of a position measurement.

The upstream lateral observer uses wheel speed, steering and IMU signals,
without feedback from the global observer. For a cascade result, its complete
interface error, including the error of $\dot{\hat\beta}$, must satisfy an ISS
bound. An ISS bound on $\hat\beta-\beta$ alone does not prove a bound on its
derivative. Derivatives of bounded sensor noise are not assumed bounded.

## 2. Auxiliary outputs and high-gain scaling

After gravity and lever-arm compensation, body inertial acceleration is
$a_b=[a_x,a_y]^\top$ and body velocity is $[v_x,v_y]^\top$. Rotation preserves
squared norm, inner product, and planar cross product. The auxiliary outputs are

$$
h(z,t)=\begin{bmatrix}
V_X^2+V_Y^2\\ V_XA_X+V_YA_Y\\ V_XA_Y-V_YA_X\\
V_Y\cos(\psi+\hat\beta)-V_X\sin(\psi+\hat\beta)
\end{bmatrix},\qquad
 y_h=\begin{bmatrix}
v_x^2+\hat v_y^2\\ v_xa_x+\hat v_ya_y\\ v_xa_y-\hat v_ya_x\\0
\end{bmatrix}=h(z,t)+n_h. \tag{4}
$$

In particular $n_{h,4}=-v\sin(\beta-\hat\beta)$, so
$|n_{h,4}|\le v|\beta-\hat\beta|$. The other three errors include wheel-speed,
acceleration and lateral-velocity errors. Products of bounded signals admit
class-$\mathcal K$ disturbance bounds on a stated operating envelope; no
statistical independence or white-noise assumption is used.

Assume the true velocity and acceleration components obey
$|V_X|,|V_Y|\le\bar v$ and $|A_X|,|A_Y|\le\bar a$.
Use the extension $h_{\rm ext}$ obtained by clipping only these four arguments
of $h$ to their respective intervals. Position, heading, the estimated state,
and the linear predictor are not clipped. The extension agrees with the true
output inside the envelope and has bounded generalized Jacobians. Along the
line segment between $z$ and $\hat z$,

$$
h_{\rm ext}(z,t)-h_{\rm ext}(\hat z,t)=H(t)(z-\hat z). \tag{5}
$$

The thirteen possible nonzero entries of $H$ obey the following outer box;
all unlisted entries are zero. Clip derivatives in $[0,1]$ preserve these bounds.

| Output row | State columns | Absolute coefficient bounds |
|---|---|---|
| 1 | 2, 5 | $2\bar v,2\bar v$ |
| 2 | 2, 5, 3, 6 | $\bar a,\bar a,\bar v,\bar v$ |
| 3 | 2, 5, 6, 3 | $\bar a,\bar a,\bar v,\bar v$ |
| 4 | 2, 5, 7 | $1,1,2\bar v$ |

For a fixed design parameter $\theta\ge1$, define

$$
T=\operatorname{diag}(\theta,\theta^2,\theta^3,
\theta,\theta^2,\theta^3,\theta),\quad
L=TK,\quad L_h=TN/\theta^3. \tag{6}
$$

The scaling parameter is fixed during operation. In particular,
$T^{-1}AT=\theta A$ and $CT=\theta C$ for a pose-selection matrix $C$.
The invariant error enters as $-\theta^{-3}NHT\varepsilon$, where
$\varepsilon=T^{-1}(z-\hat z)$. All measurement-noise terms have the negative
sign implied by defining the error as true state minus estimate.

## 3. Mode G: continuous GNSS position

### 3.1 Why the seventh state needs a hypothesis

The actual GNSS output is

$$
y_G=C_Gz+n_G,\qquad C_G=\begin{bmatrix}e_1^\top\\e_4^\top\end{bmatrix}. \tag{7}
$$

At rest, take $v_g=a_g=0$, $r_m=0$, and zero disturbances. Every constant
heading produces the same GNSS and auxiliary outputs. Two different headings
are therefore indistinguishable. A seven-state zero-input error cannot be
required to converge to zero from all such initial conditions. This is a
counterexample to unconditional seven-state ISS from position-only GNSS.
A small nonzero yaw-rate bias also causes uncorrected heading drift at rest.

The following result supplies the needed motion and heading-domain assumptions
and an explicit gain structure. It does not silently borrow LiDAR heading.

### 3.2 Continuous triangular MO-HGO

Put $x=[X,V_X,A_X,Y,V_Y,A_Y]^\top$, let $M_6$ be the upper-left block of $M$,
and let $h_3$ denote the first three auxiliary outputs. Define
$T_6=\operatorname{diag}(\theta,\theta^2,\theta^3,\theta,\theta^2,\theta^3)$
and $C_6x=[X,Y]^\top$. Choose

$$
\begin{aligned}
\dot{\hat x}&=M_6(q)\hat x+T_6K_G(y_G-C_6\hat x)
+T_6N_G\theta^{-3}[y_{h,1:3}-h_{3,\rm ext}(\hat x)],\\
\dot{\hat\psi}&=r_m+k_\psi[\hat V_Y\cos(\hat\psi+\hat\beta)
-\hat V_X\sin(\hat\psi+\hat\beta)],\quad k_\psi>0.
\end{aligned} \tag{8}
$$

The yaw row uses the raw estimated velocity; it remains linear in that
velocity. Its error will be bounded by the preceding six-state estimate.
This is a member of the seven-state MO-HGO family: the first six rows of
$N$ use the first three auxiliary outputs and its last row has
$N_{7,4}=-k_\psi\theta^2$, with all other entries in the last row zero.
The GNSS gain has a zero last row. The fourth auxiliary output is not fed
back into the first six rows. This triangular design avoids asking a
sign-indefinite global heading-Jacobian box to establish heading observability.
It is an explicit new gain constraint, not a claim about arbitrary stored $N$.

Let $\epsilon=T_6^{-1}(x-\hat x)$. Its exact dynamics are

$$
\dot\epsilon=A_G(q,H_3)\epsilon+u_G,\quad
A_G=T_6^{-1}M_6T_6-\theta K_GC_6-\theta^{-3}N_GH_3T_6,
$$

$$
u_G=T_6^{-1}w_{1:6}-K_Gn_G-\theta^{-3}N_Gn_{h,1:3}. \tag{9}
$$

### 3.3 Constant-matrix six-state certificate and feasibility

For all admissible $q$ and incremental matrices $H_3$, require a constant
$P_6=P_6^\top\succ0$ and $a>0$ such that

$$
\operatorname{He}(P_6A_G)\preceq-aI_6. \tag{10}
$$

This is a finite LMI test after enclosing $(q,q^2)$ by four vertices and
$H_3$ by its coefficient box. At fixed $\theta$, synthesis can use
$Y_G=P_6K_G$ and $Z_G=P_6N_G$. The same matrices must satisfy every vertex.

The condition is not vacuous. Select $K_G$ so that
$A_6-K_GC_6$ is Hurwitz, and solve
$\operatorname{He}[P_6(A_6-K_GC_6)]=-I_6$.
For example each third-order chain can use $[3,3,1]^\top$.
For any fixed finite nonzero $N_G$, write
$A_G=\theta(A_6-K_GC_6)+E_G$. Uniformly for $\theta\ge1$,

$$
\|E_G\|\le\bar q\sqrt{\bar q^2+4}+\|N_G\|L_3,
\quad L_3=\sqrt{12\bar v^2+4\bar a^2}, \tag{11}
$$

where $L_3$ is the Frobenius upper bound of the ten-entry $H_3$ box.
The exact known-input contribution before relaxing $\theta$ is
$|q|\sqrt{q^2/\theta^2+4}$. Therefore (10) holds with

$$
a=\theta-2\|P_6\|[\bar q\sqrt{\bar q^2+4}+\|N_G\|L_3]>0. \tag{12}
$$

This proves existence for the six-state block at sufficiently large fixed
$\theta$. It is conservative; solving (10) can permit smaller gains.

### 3.4 Seven-state ISS theorem for moving GNSS

**Theorem G.** Suppose (1), (4), and (7)--(10) hold, all indicated inputs are
bounded, and the true speed satisfies
$0<v_*\le v(t)\le v^*$. Let $e_\psi=\psi-\hat\psi$ be a consistently lifted
heading error. Choose $0<\delta<\pi/2$ and restrict the initial error and
inputs as specified below so that $|e_\psi|\le\delta$ is invariant.
Then the observer (8) is locally exponentially ISS in all seven errors.

**Proof.** With $e_v=v_g-\hat v_g$ and
$\tilde\beta=\beta-\hat\beta$, the yaw innovation satisfies

$$
\hat V_Y\cos(\hat\psi+\hat\beta)-\hat V_X\sin(\hat\psi+\hat\beta)
=v\sin(e_\psi+\tilde\beta)-
[-\sin(\hat\psi+\hat\beta),\cos(\hat\psi+\hat\beta)]e_v.
$$

Consequently

$$
\dot e_\psi=-k_\psi v\sin e_\psi+d_\psi,\quad
|d_\psi|\le k_\psi\theta^2\|\epsilon\|+u_\psi,\quad
u_\psi=k_\psi v^*|\tilde\beta|+|\nu_r|. \tag{13}
$$

Define $a_\psi=k_\psi v_*\sin\delta/\delta$ and $b_\psi=k_\psi\theta^2$.
For $|e_\psi|\le\delta$, $e_\psi\sin e_\psi\ge
(\sin\delta/\delta)e_\psi^2$. Choose a constant

$$
0<c\le\frac{a a_\psi}{8b_\psi^2},\qquad
V_G=\epsilon^\top P_6\epsilon+c e_\psi^2. \tag{14}
$$

Writing $p=\lambda_{\max}(P_6)$, differentiating (14), and applying Young's
inequality to the three cross terms gives

$$
\begin{aligned}
2p\|\epsilon\|\|u_G\|&\le\tfrac a4\|\epsilon\|^2+
\tfrac{4p^2}{a}\|u_G\|^2,\\
2c b_\psi|e_\psi|\|\epsilon\|&\le\tfrac a4\|\epsilon\|^2+
\tfrac{4c^2b_\psi^2}{a}e_\psi^2,\\
2c|e_\psi|u_\psi&\le\tfrac{c a_\psi}{2}e_\psi^2+
\tfrac{2c}{a_\psi}u_\psi^2.
\end{aligned}
$$

Together with (10) and (14), these imply

$$
\dot V_G\le-\tfrac a2\|\epsilon\|^2-c a_\psi e_\psi^2+
\tfrac{4p^2}{a}\|u_G\|^2+\tfrac{2c}{a_\psi}u_\psi^2
\le-\lambda_G V_G+D_G(t), \tag{15}
$$

where $\lambda_G=\min\{a/(2p),a_\psi\}>0$ and $D_G$ is the displayed
nonnegative disturbance sum. Define the constant physical-error metric

$$
\bar P_G=\operatorname{diag}(T_6^{-\top}P_6T_6^{-1},c),\quad
\bar p_- =\lambda_{\min}(\bar P_G),\quad
\bar p_+ =\lambda_{\max}(\bar P_G).
$$

Scalar comparison proves the explicit seven-state ISS estimate

$$
\boxed{\|e(t)\|\le
\sqrt{\bar p_+/\bar p_-}\,e^{-\lambda_G t/2}\|e(0)\|
+\sqrt{\frac{\|D_G\|_{[0,t],\infty}}{\lambda_G\bar p_-}}.} \tag{16}
$$

There is no circular invariance assumption. From (9)--(10), independently of
yaw, with $\kappa_6=p/\lambda_{\min}(P_6)$,

$$
\|\epsilon(t)\|\le R_\epsilon:=
\sqrt{\kappa_6}\|\epsilon(0)\|+
\frac{2p\sqrt{\kappa_6}}{a}\|u_G\|_\infty. \tag{17}
$$

If $|e_\psi(0)|<\delta$ and

$$
b_\psi R_\epsilon+\|u_\psi\|_\infty
<k_\psi v_*\sin\delta, \tag{18}
$$

then at either boundary $e_\psi=\pm\delta$ the yaw derivative points strictly
inward. A first-exit argument establishes invariance, validating the use of
(15) for all $t\ge0$. Equations (16) and (18) are a local ISS result with a
specified disturbance neighborhood. They do not cover standstill or arbitrary
heading errors. In the noise-free case the error converges exponentially.
This proves Theorem G.

### 3.5 If GNSS also independently measures heading

If an actual sensor provides $y_G=Cz+n_G$, with
$C=[e_1,e_4,e_7]^\top$, one may instead use all four extended auxiliary outputs
and the usual continuous observer
$\dot{\hat z}=M\hat z+b+TK_G(y_G-C\hat z)+L_h(y_h-h_{\rm ext}(\hat z))$.
For its scaled matrix
$A_G=T^{-1}MT-\theta K_GC-\theta^{-3}N_GHT$, a constant certificate
$\operatorname{He}(P_GA_G)\preceq-2\rho_GP_G$ gives

$$
\|e(t)\|\le\theta^2\sqrt{\kappa(P_G)}e^{-\rho_Gt}\|e(0)\|
+\frac{\theta^3\sqrt{\kappa(P_G)}}{\rho_G}\|u_G\|_{[0,t],\infty}. \tag{19}
$$

This follows from the Dini derivative of $\sqrt{\epsilon^\top P_G\epsilon}$.
It permits standstill, subject to the bounded-input and lifted-heading
assumptions. Course over ground at zero speed is not such an independent
heading measurement. This is an alternative measurement contract, not the
position-only interface used for Theorem G.

## 4. Mode L: continuous, informative LiDAR with a fixed delay

### 4.1 Measurement and uniform information condition

Let $d_L>0$ be known and constant. Fix the physical pose scales
$S_p=\operatorname{diag}(s_X,s_Y,s_\psi)\succ0$, and express the measured
pose and its noise in these normalized coordinates:

$$
y_L(t)=Cz(t-d_L)+n_L(t),\qquad
C=S_p^{-1}\begin{bmatrix}e_1^\top\\e_4^\top\\e_7^\top\end{bmatrix}. \tag{20}
$$

This output is available continuously. Its timestamp offset is always $d_L$.
The pose at $t-d_L$ and its information matrix belong to the same physical
measurement. No delivery clock is introduced.

Information must be compared in the same dimensionless pose coordinates. Let
$I_L$ be the physical pose information, and
$\bar I_L=S_p^\top I_LS_p$. Require a uniform lower bound

$$
\bar I_L(t)\succeq\iota_*I_3,\qquad\iota_*>0. \tag{21}
$$

A convenient bounded weight is

$$
W=\bar I_L(\bar I_L+\lambda_I I_3)^{-1},\quad\lambda_I>0,
\qquad w_* I_3\preceq W(t)\preceq I_3,\quad
w_*=\frac{\iota_*}{\iota_*+\lambda_I}>0. \tag{22}
$$

This is a spectral matrix function, retaining translation-heading cross terms.
It does not insert an artificial eigenvalue floor. Pointwise nonsingularity
with eigenvalues tending to zero would not meet (21). The information matrix
must have a defensible pose-error interpretation; a registration Hessian by
itself does not establish a deterministic noise bound. For example an
independently justified bound
$n_L^\top\bar I_L n_L\le\eta^2$ implies
$\|n_L\|\le\eta/\sqrt{\iota_*}$, or a physical-noise bound
$\|S_p n_L\|\le\|S_p\|\eta/\sqrt{\iota_*}$.
Otherwise bounded $n_L$ remains a separate assumption.

In the following algebra $W$ acts on the normalized pose residual in (20);
$CT=\theta C$ still holds. The same convention must be used in a numerical
certificate. The equal-unit formulas use $S_p=I_3$.

### 4.2 Continuous delayed observer and exact error equation

Choose

$$
\boxed{\dot{\hat z}(t)=M(q(t))\hat z(t)+b(t)
+TK_LW(t)[y_L(t)-C\hat z(t-d_L)]
+TN_L\theta^{-3}[y_h(t)-h_{\rm ext}(\hat z(t),t)].} \tag{23}
$$

The pose residual compares two states at $t-d_L$. The current auxiliary
outputs still compare current states. Seven differential state coordinates
are retained, together with a continuous history on $[-d_L,0]$.
Retarded equations require a history; they are continuous-time systems, though
their phase space is infinite-dimensional. This is not state replay or
input-flow transport.

Subtracting (23) from (1), and using (5), gives exactly

$$
\boxed{\dot\varepsilon(t)=A_0(t)\varepsilon(t)
+A_d(t)\varepsilon(t-d_L)+u_L(t),} \tag{24}
$$

$$
\begin{aligned}
A_0&=T^{-1}M(q)T-\theta^{-3}N_LHT,\\
A_d&=-\theta K_LWC,\\
u_L&=T^{-1}w-K_LWn_L-\theta^{-3}N_Ln_h.
\end{aligned}  \tag{25}
$$

In particular, the delayed error is not an independent disturbance. Substituting
$C\hat z(t)$ for $C\hat z(t-d_L)$ would add the motion term
$C[z(t-d_L)-z(t)]$ and generally destroy zero-error invariance even in noiseless
constant-velocity motion. That is a different observer.

### 4.3 Constant-matrix Lyapunov--Krasovskii condition

Fix $\theta$, $d_L$, the gains, and a target rate $\rho>0$. Set
$r_d=e^{-2\rho d_L}$. Seek constant matrices
$P=P^\top\succ0$, $Q=Q^\top\succ0$, $R=R^\top\succ0$ and $g>0$.
For every admissible pair $(A_0,A_d)$ require

$$
\boxed{
\begin{bmatrix}
\Pi & d_L\mathcal F^\top R\\
d_L R\mathcal F & -R
\end{bmatrix}\prec0,\qquad
\mathcal F=[A_0\;A_d\;I_7],} \tag{26}
$$

where the $21$-by-$21$ symmetric block is

$$
\Pi=\begin{bmatrix}
\operatorname{He}(PA_0)+2\rho P+Q-r_dR & PA_d+r_dR & P\\
* &-r_d(Q+R)&0\\
*&*&-gI_7
\end{bmatrix}. \tag{27}
$$

All matrices here are constant; the functional's history integrals carry the
delay. This is an LMI in $P,Q,R,g$ for fixed gains, rate and delay. Joint
optimization over gains and these matrices is generally bilinear and must
not be described as this same convex feasibility problem.

If $A_0,A_d$ lie in a verified convex hull, checking (26) at every hull vertex
suffices, because its Schur form is affine in the two coefficient matrices.
The four-vertex $(q,q^2)$ enclosure and the thirteen-entry output box are
valid choices. A full-matrix $W$ sector must also be enclosed, including
all off-diagonal entries. Checking only $W=w_*I$ and $W=I$ does not cover
arbitrary anisotropic matrices. An entrywise outer box centered at
$(1+w_*)I/2$ with halfwidth $(1-w_*)/2$ on its six independent symmetric
entries is valid but conservative. No bound on $\dot W$ or $\dot H$ is needed.
A failed outer-box LMI means this certificate is unavailable; it is not a
proof that the actual system is unstable.

### 4.4 LiDAR ISS theorem and proof

**Theorem L.** Suppose the bounded-model/output hypotheses, (20)--(22), and
(26) hold uniformly along a forward-complete trajectory in a consistently
lifted pose chart. For every continuous initial error history and bounded
$u_L$, observer (23) is exponentially ISS in all seven errors, with respect
to the supremum norm of that history.

**Proof.** For brevity write $d=d_L$ and $x=\varepsilon$. For an absolutely
continuous solution segment, use

$$
\begin{aligned}
V_L(x_t)={}&x(t)^\top Px(t)
+\int_{t-d}^t e^{2\rho(s-t)}x(s)^\top Qx(s)\,ds\\
&+d\int_{-d}^0\int_{t+s}^t e^{2\rho(u-t)}
\dot x(u)^\top R\dot x(u)\,du\,ds.
\end{aligned} \tag{28}
$$

It satisfies $V_L\ge p_-\|x(t)\|^2$, where $p_-=\lambda_{\min}(P)$.
Let $x_d=x(t-d)$. Differentiating the integrals almost everywhere yields

$$
\begin{aligned}
\dot V_L+2\rho V_L={}&2x^\top P\dot x+2\rho x^\top Px
+x^\top Qx-r_dx_d^\top Qx_d+d^2\dot x^\top R\dot x\\
&-d\int_{t-d}^t e^{2\rho(u-t)}\dot x(u)^\top R\dot x(u)\,du.
\end{aligned}  \tag{29}
$$

Since $e^{2\rho(u-t)}\ge r_d$, Jensen's inequality and
$x-x_d=\int_{t-d}^t\dot x(u)\,du$ imply

$$
-d\int_{t-d}^t e^{2\rho(u-t)}\dot x^\top R\dot x\,du
\le-r_d(x-x_d)^\top R(x-x_d). \tag{30}
$$

With $\zeta=[x^\top,x_d^\top,u_L^\top]^\top$, equation (24) gives
$\dot x=\mathcal F\zeta$. Equations (29)--(30) become

$$
\dot V_L+2\rho V_L-g\|u_L\|^2
\le\zeta^\top[\Pi+d^2\mathcal F^\top R\mathcal F]\zeta<0
\quad(\zeta\ne0), \tag{31}
$$

where the strict inequality is exactly the Schur complement of (26).
Integration from any time $t_0$ with an admissible solution segment gives

$$
\|x(t)\|\le
\sqrt{V_L(x_{t_0})/p_-}\,e^{-\rho(t-t_0)}
+\sqrt{g/(2\rho p_-)}\,\|u_L\|_{[t_0,t],\infty}. \tag{32}
$$

To establish ISS in the ordinary continuous-history norm, one must not
bound $V_L(x_0)$ using an unprovided derivative of the initial history.
The following first-interval argument removes that gap. Let

$$
m=\sup_t\|A_0(t)\|+\sup_t\|A_d(t)\|,\quad
M_0=\|x_0\|_{[-d,0],\infty},\quad U_t=\|u_L\|_{[0,t],\infty}.
$$

On $[0,d]$, the integral equation and Gronwall's inequality give
$\sup_{[-d,d]}\|x\|\le e^{md}(M_0+dU_d)$ and
$\|\dot x\|_{[0,d],\infty}\le m e^{md}(M_0+dU_d)+U_d$.
With $p_+=\lambda_{\max}(P)$, $q_+=\lambda_{\max}(Q)$,
$r_+=\lambda_{\max}(R)$, define

$$
\begin{aligned}
b_0&=\sqrt{p_++dq_+},\qquad b_1=\sqrt{d^3r_+/2},\\
C_0&=e^{md}(b_0+mb_1),\qquad C_1=dC_0+b_1,\\
A_L&=e^{\rho d}\max\{e^{md},C_0/\sqrt{p_-}\},\\
B_L&=\max\{de^{md},C_1/\sqrt{p_-}+\sqrt{g/(2\rho p_-)}\}.
\end{aligned}  \tag{33}
$$

The weights in (28) are at most one; the triangular double-integral area is
$d^2/2$. Thus $\sqrt{V_L(x_d)}\le C_0M_0+C_1U_d$, where $x_d$ here denotes
the history segment at time $d$. Applying (32) from $t_0=d$, and using the
first-interval estimate for $0\le t<d$, proves

$$
\|x(t)\|\le A_Le^{-\rho t}M_0+B_LU_t,\qquad t\ge0. \tag{34}
$$

Finally $\|T\|=\theta^3$ and $\|T^{-1}\|=\theta^{-1}$. Returning to the
physical seven-state error yields

$$
\boxed{\|e(t)\|\le\theta^2A_Le^{-\rho t}
\|e_0\|_{[-d_L,0],\infty}
+\theta^3 B_L\|u_L\|_{[0,t],\infty}.} \tag{35}
$$

The disturbance on the right is external and bounded by

$$
\|u_L\|\le\theta^{-1}\|w\|+
\|K_L\|\|n_L\|+\theta^{-3}\|N_L\|\|n_h\|. \tag{36}
$$

Using the structure $w=Bd_j-e_7\nu_r$ improves its first term to
$\theta^{-3}\|d_j\|+\theta^{-1}|\nu_r|$. Zero external disturbances yield
exponential convergence from any admitted history. This proves Theorem L.

The heading qualification is explicit: pose yaw must be continuously lifted
on a consistent branch, or the result applies only while a valid local chart
is maintained. It does not prove global stability on the circle for arbitrary
principal-value yaw representatives or cycle slips. For GNSS Theorem G the
stronger local restriction (18) is also required.

### 4.5 Why delay and gain must be checked together

Even the scalar continuously measured heading channel

$$
\dot e_\psi(t)=-\ell e_\psi(t-d_L),\qquad\ell>0, \tag{37}
$$

is exponentially stable only for $0<\ell d_L<\pi/2$.
Indeed the characteristic equation is $s+\ell e^{-sd_L}=0$; its first
imaginary-axis crossing is $s=i\ell$, $\ell d_L=\pi/2$.
At the boundary the nondecaying solution $e_\psi(t)=\sin(\ell t)$ already
contradicts ISS with zero input. The crossing is toward the right half-plane
as delay increases. This elementary special case disproves any claim that
full information permits arbitrary fixed delay or arbitrarily large gain.

Condition (26) is a sufficient delay-dependent test and need not recover
this exact scalar limit. A value such as the earlier runtime's 150 ms is
certified only after the new inequalities are checked with the new gains,
actual information sector, and operating bounds. The historical gain files
supply no such conclusion. Small-delay feasibility follows by continuity
from a strict delay-free certificate: choose $R$ large enough to absorb the
cross term in $x-x_d$, then choose $Q$ sufficiently small and $g$ sufficiently
large. The remaining positive term $d^2\mathcal F^\top R\mathcal F$ is a
small perturbation for sufficiently small $d>0$. This establishes a nonempty
small-delay range whenever a strict delay-free common certificate exists;
it does not specify its numerical upper endpoint.

## 5. Cascade interpretation and design procedure

If the upstream interface error $\eta$ is ISS in exogenous disturbances $d_u$,
and $w,n_h$ obey bounds of the form
$\|[w,n_h]\|\le\alpha_1(\|\eta\|)+\alpha_2(\|d_u\|)$ on the stated envelope,
then (16) or (35), combined with the upstream estimate, gives cascade ISS.
For example an exponentially decaying upstream transient enters the downstream
variation-of-constants integral as a convolution of decaying exponentials;
this also decays, including the equal-rate case $te^{-\rho t}$.
The remaining term is a class-$\mathcal K$ function of the exogenous input
supremum. No small-gain feedback condition is needed because the connection
is one-way. GNSS retains its local invariant-domain restriction.

A complete numerical design follows this order:

1. Declare the actual GNSS output (position alone, or independently measured
   position and heading) and fix the physical state/pose units.
2. Declare bounded $q$, velocity, acceleration and all upstream interface
   residuals. For position-only GNSS, declare $v_*$, $\delta$ and the admitted
   initial-error/input set.
3. For GNSS position, impose the triangular gains (8), verify (10), select
   $c$ by (14), and verify (18). A general unsigned heading-Jacobian box
   cannot replace this observability argument.
4. For LiDAR, establish the uniform three-direction information bound and
   pose-noise contract. Select $d_L$, $\theta,K_L,N_L$ jointly as a design
   choice, then verify (26) for the entire uncertainty enclosure using
   constant $P,Q,R$ and a strictly positive margin.
5. Report the resulting rate and ISS constants. A solver success flag,
   frozen-system eigenvalues, sampled weight directions, or numerical
   trajectories alone are not exhaustive robust verification.

No continuous-mode switching theorem, certificate for missing measurements,
or certificate for the old event-based runtime follows from these results.
The two theorems prove the continuous measurement contracts stated above.

## 6. Sources and verification scope

The improved model and mode-specific proofs above are project derivations.
The multi-output scaling and incremental-output construction build on
Bessafa et al., Section 4, equations (41)--(58), especially Theorem 5.
The supplied local PDF was inspected; its six-state observer is not asserted
to contain the new seven-state GNSS or delayed-LiDAR theorem. The constant-matrix
history-functional method follows the Lyapunov--Krasovskii framework described
by Fridman; equations (28)--(35) are derived explicitly here.

- H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani,
  *Generalized multi-output high-gain observer with application to ego vehicle
  trajectory and orientation estimation*, Automatica 188 (2026), 112915.
  [DOI](https://doi.org/10.1016/j.automatica.2026.112915);
  [author institution record](https://eprints.soton.ac.uk/509851/).
- E. Fridman, *Tutorial on Lyapunov-based methods for time-delay systems*,
  European Journal of Control 20(6) (2014), 271--283.
  [DOI](https://doi.org/10.1016/j.ejcon.2014.10.001).

No supplied paper PDF is redistributed. The numerical checks accompanying
this revision are documented in
[the validation record](research/continuous_observer_iss_20260913/validation.md).
They distinguish a verified conservative parameter neighborhood from certification
of the full operating and information sectors used by the older runtime.
