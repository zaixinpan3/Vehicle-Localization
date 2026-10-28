# Coupled invariant-output gain synthesis

The current synthesis retains the complete local information of the dot/cross
pair and its dependence on the same motion variables as the vehicle dynamics.
The five physical outputs, matched runtime correction, scalar gyro design,
noise objective and sensor-information contracts are unchanged. There is no
prescribed physical response floor or nominal metric-relative decay target.
The implementation is `coupledMotionSupport` and `sensorRedundancySupport`.

The angular metric can be a full symmetric positive definite 2-by-2 block. Both
heading error and yaw-rate error are unchanged by a global planar rotation,
so rotational symmetry does not require their Lyapunov cross term to vanish.
The diagonal restriction excludes such certificates and is not a symmetry
requirement. The default nevertheless retains the previous diagonal block:
the full-block experiment reduced the noise objective but failed three
existing recovery tests. It is not promoted by weakening those tests.
The existing heading feedback caps retain their exact meaning through

\[
k\,e_7^\top P^{-1}e_7\leq g
\quad\Longleftrightarrow\quad
P-(k/g)e_7e_7^\top\succeq0,
\]

for `P>0`, `k>=0`, `g>0`. Replacing this condition by `k<=g P77` would be
incorrect for a general non-diagonal angular block. No cap is loosened to
enable the enlarged metric class.

## Course coordinates and the exact pair

Let the true course speed be `s>0`, course rate be `q`, and tangential
acceleration be `b`. On the existing kinematic manifold,

\[
v=(s,0)^\top,\qquad a=(b,sq)^\top,\qquad
b^2+s^2q^2\leq A_{\max}^2.
\]

The constant metric `P=blkdiag(Pc,Pc,Ph)` commutes with simultaneous planar
rotation of position, velocity and acceleration errors. The skew term from
rotation of the course frame therefore contributes zero to its Lyapunov
derivative. This allows one course-coordinate calculation for all headings.
It does not claim that this is the most general rotation-invariant metric.

In normalized error coordinates ordered locally as
`[dv_parallel,dv_normal,da_parallel,da_normal]`, the two normalized rows are

\[
C_{\rm pair}(s,q,b)=
\begin{bmatrix}
b/(\theta V)&sq/(\theta V)&s/V&0\\
sq/(\theta V)&-b/(\theta V)&0&s/V
\end{bmatrix},\qquad V=V_{\max}.
\]

Define `u=s/V`, `alpha=b/(theta V)`, `beta=sq/(theta V)`, and
`B=[[alpha,beta],[beta,-alpha]]`. Then the exact Gram is

\[
G_{\rm pair}=C_{\rm pair}^{\top}C_{\rm pair}
=\begin{bmatrix}
(\alpha^2+\beta^2)I_2&uB\\
uB&u^2I_2
\end{bmatrix}\succeq0.
\]

The nominal Lyapunov term is `-2 theta k_a G_pair`. No paired Young
inequality replaces it by a negative acceleration term plus a positive
velocity penalty. The cross blocks are retained. The same `q` and `b` enter
the normalized dynamics through `q^2/theta^2`, `2q J/theta`, and
`d(f_a)/d(r)=[0;2b]`. Squared-speed and direction-output Grams use the same
`s` and `s^2`, rather than independent uncertainty intervals.

## Finite conditions over a continuous physical domain

For fixed metric and gains, every L/G/M Lyapunov matrix is a polynomial in
`(s,q,b)` of degree at most two in each variable. Its coefficients are
affine in the SDP decisions. The necessary scalar features are

\[
(q,q^2,b,s,s^2,b^2+s^2q^2,sb,s^2q).
\]

The domain is covered by boxes as follows:

1. Partition `[s_min,V]` geometrically. In speed slab `[s_l,s_u]`, every
   physical point satisfies `|q|<=min(Q,A_max/s_l)`.
2. Split that rate interval at zero and any additional configured boundaries.
   For a rate cell, let `q_near` be its minimum absolute rate, or zero if it
   contains zero. Every point in that cell satisfies
   `|b|<=sqrt(A_max^2-s_l^2*q_near^2)`.
3. Partition the resulting tangential-acceleration interval at zero.

These are outer boxes: every physical point is covered, although some box
points can exceed the coupled acceleration limit. The declared speed,
acceleration and rate limits have not been reduced to obtain feasibility.

### Retaining the physical acceleration set inside each box

Requiring the original Lyapunov matrix to be negative on every outer-box
point is unnecessarily strong. For example, `s=10`, `q=-0.2`, `b=2` implies
an acceleration norm of `sqrt(8)`, exceeding the declared `A_max=2`. Speed
subdivision cannot remove this corner at the minimum speed. The certificate
now uses the exact defining polynomial

\[
g(s,q,b)=1-\frac{b^2+s^2q^2}{A_{\max}^2}.
\]

For each sensor family `f` and multiplier region `j`, solve for a symmetric
matrix `T_fj >= 0` together with the original metric and gains. Require the
Bernstein coefficients of

\[
\widetilde F_f(s,q,b)=F_f(s,q,b)+g(s,q,b)T_{fj}
\]

to be strictly negative on each covering box in that region. On a physical
point, `g>=0`, so

\[
F_f=\widetilde F_f-gT_{fj}\preceq\widetilde F_f\prec0.
\]

At an inadmissible point, `g<0`; the augmented inequality need not force
`F_f<0`. Thus the nonphysical corner no longer constitutes a mandatory
stability requirement. This is a sufficient matrix-multiplier condition,
not a claim that a finite relaxation is lossless. It uses the same physical
domain, all five outputs, full coupled dynamics and existing sensor contracts.
Neither acceleration component is independently reduced or discarded.

The scalar `g` has degree `(2,2,2)`, with exact coefficient
`1-(b^2+s^2*q^2)_ijk/A_max^2`. The implementation uses the existing sixth
feature, so no polynomial fitting, sampled approximation, extra polynomial
degree or extra coefficient matrices are needed. Multiplication by a
constant matrix variable remains affine in every SDP decision. If `A_max=0`,
the covering boxes already have `q=b=0`; multipliers are zero and no division
by zero is evaluated.

The multiplier region can be the complete domain, one of four sign regions
in `(q,b)`, or an individual covering box. Duplicating a shared matrix into
its child regions proves that these are nested certificate classes. Setting
all multipliers to zero recovers every previous sufficient condition. A
fixed-box subdivision can inherit its parent's multiplier, preserving the
usual Bernstein refinement argument. One constant metric and one gain set
are still shared across all regions. Multipliers appear only in the offline
proof; no runtime switching, extra output, derivative of a multiplier, or
new minimum feedback requirement is introduced.

The physical-domain candidate uses one shared multiplier per sensor family. Comparisons with
four sign regions and forty individual boxes in the small-offset case, and
with forty boxes in the standard case, gave the same objective to numerical
precision. A shared multiplier therefore suffices for the tested physical-domain
designs; larger certificate classes remain available for different inputs.

The production default remains the zero-multiplier member of this same
framework. With the unchanged noise objective, the less conservative candidate
regressed the existing unmeasured-velocity recovery test (0.05062 m/s error
at 12 s versus its unchanged 0.05 m/s limit) and worsened the development
scene-0103 replay. The physical-domain method is fully implemented but is not
promoted into default gains by weakening a test or adding a response floor.
To synthesize and execute it explicitly:

```matlab
cfg = mncavFullObserverConfig();
cfg.invariantObserver.redundancy.accelerationMultiplierPartition = "global";
[design, cfg] = synthesizeMncavInvariantDesign(cfg);
```

`"sign"` and `"box"` select the larger certificate classes. `"zero"` fixes
the same multiplier variables to zero and recovers the previous conditions;
there is no second observer, retired output definition, or historical gain file.

This construction is a direct special case of polynomial domain multipliers;
the broader matrix-polynomial approach is discussed by Hol and Scherer,
*Sum of Squares Relaxations for Robust Polynomial Semi-Definite Programs*
(IFAC 2005, [primary paper](https://skoge.folk.ntnu.no/prost/proceedings/ifac2005/Fullpapers/04156.pdf)).
The implication used here is proved explicitly above; this implementation
does not claim the asymptotic completeness of a full matrix-SOS hierarchy.

Map each box to `[0,1]^3`. Write its matrix polynomial exactly as

\[
\widetilde F(t_1,t_2,t_3)=\sum_{i,j,k=0}^{2}
B_i^2(t_1)B_j^2(t_2)B_k^2(t_3)\widetilde F_{ijk}.
\]

Bernstein basis functions are nonnegative and sum to one. Consequently,
`Ftilde_ijk < 0` for every coefficient implies `Ftilde(t)<0` everywhere in
the box, and hence `F(t)<0` on its physical subset. These are coefficient
matrices, not matrix values sampled at 27 states.
For example, the three controls for `s^2` on `[l,r]` are
`[l^2,l*r,r^2]`, not `[l^2,((l+r)/2)^2,r^2]`.
Tensor products construct all mixed terms exactly. The program constructs
these coefficients directly without polynomial fitting.

This is the tensor Bernstein enclosure principle described, for example,
by Titi and Garloff, *Matrix methods for the tensorial Bernstein form*,
Applied Mathematics and Computation 346 (2019), 254-271
([author-hosted paper](https://www-home.htwg-konstanz.de/~garloff/AMC-2.pdf)).
The matrix negativity implication above follows directly by convexity.

## Why geometric speed cells matter

A coefficient of a PSD polynomial Gram need not itself be PSD. Thus even
an exact polynomial representation can yield conservative finite conditions.
For a degree-two term on `[l,r]`, the midpoint control discrepancy is
proportional to `(r-l)^2/4`. Relative to the low-speed information scale,
the relevant resolution is approximately `(r-l)/l`.

Ten equal-width cells on `[1,30]` start with `[1,3.9]`, whereas ten geometric
cells start with approximately `[1,1.405]`. They use the same number of
conditions but avoid concentrating the worst relative enclosure error near
the lowest speed. This is numerical resolution of the proof, not stronger
sensor information, a smaller physical domain, or an imposed gain lower bound.

Subdividing a fixed box produces coefficient matrices that are convex
combinations of its parent coefficients. Such subdivision cannot worsen a
valid coefficient certificate and can tighten a failed sufficient bound.
Automatically rebuilding an entire outer-box cover need not be a nested
subdivision. No universal non-conservatism or improvement over every older
certificate is claimed for one finite partition.

## Solver and verification

The default 10 speed, 2 rate and 2 acceleration partitions yield 1,080
motion coefficient tuples. With nonzero GNSS lever arm and the existing
sensor enclosures, these give 4,320 L, 25,920 G and 103,680 Mixed controls.
Only active cuts are instantiated symbolically. Every candidate is evaluated
against all 133,920 numeric matrices before it can be accepted. Final
verification does not depend on YALMIP. Existing strict numerical margins
and homogeneous normalization are retained.

The common solver first uses SeDuMi adaptive step differentiation and retries
its alternative strategy only after a solver/base-residual failure. Neither
attempt changes the objective or inequalities. Each attempt is recorded.
For local multipliers, initial cuts touch every box and subsequent violated
cuts prioritize distinct multiplier regions. Unnecessary multipliers can be
set to zero only after all unaugmented controls of their region have passed;
this certificate simplification does not regularize the gain objective.
The exact simplified multipliers are returned and independently reverified.

Numerical PSD error is accounted for explicitly. If a symmetric stored
multiplier has minimum eigenvalue `-epsilon`, then `0<=g<=1` on the physical
set gives `F <= Ftilde + epsilon I`. The verifier charges this nonnegative
penalty to the family residual, rejects substantial indefiniteness or
asymmetry, and uses the resulting physical bound in the regional ISS proof.
The multipliers receive the same homogeneous factor as the metric and gains;
otherwise the coefficient inequality would not scale correctly. Current
design metadata invalidates gains synthesized without this certificate.

The full translation-heading coupling of the Mixed LiDAR condition remains.
PSD plus a heading diagonal is never used to infer a diagonal Loewner bound.
Sensor contributions are factored from motion controls only for construction
efficiency; all Cartesian combinations are still verified.

## Finite-error and performance scope

This improvement concerns the nominal Jacobian family used by the existing
regional ISS argument. Runtime feedback uses the estimated Jacobian, while
the exact nonlinear residual uses a finite-increment matrix. Their difference
still enters the existing cubic/quartic remainder bounds. The resulting
region and ISS rate are computed after synthesis; a large useful basin is
not implied by a successful nominal LMI.

All five outputs must be available and the actual sensor/operator conditions
must hold. A weak-LiDAR/no-GNSS replay is not automatically covered by the
strong L-family contract. Zero speed has finite runtime behavior but remains
outside the guaranteed interval. No sampled-system, map-correctness or
upstream-cascade certificate is added by this change.
