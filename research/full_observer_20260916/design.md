# Full GNSS/LiDAR observer with independent channel availability

## Material Passport

Date: September 16, 2026. Scope: executable design and conditional stability
analysis, with deterministic tests and a recorded MnCAV replay. The user
clarified that separate GNSS/LiDAR ISS proofs are analysis cases, while the
actual localization module must accept both signals simultaneously and
tolerate either missing. This corrects the previous conflation of independent
proofs with a mutually exclusive runtime interface.

The new entry is `runFullLocalizationObserver`, configured by
`fullObserverConfig`. It extends the most recent motion-aided seven-state
structure; it does not revert the useful motion correction or pretend that
the legacy MO-HGO gains prove this different structure. The legacy independent
GNSS and fixed-delay LiDAR proofs and runners are preserved.

## Equations and source roles

Write the state as `(p,v,a,psi)` in the repository's interleaved order
`[X,Vx,Ax,Y,Vy,Ay,psi]`, and let `J=[0,-1;1,0]`.
The actual implemented equations are

\[
\begin{aligned}
\dot{\hat p}&=\hat v+s_G k_G W_G(\tilde p_G-\hat p)
                       +s_L k_p W_{L,p}(\tilde p_L-\hat p),\\
\dot{\hat v}&=\hat a+k_v(R(\hat\psi)b_v-\hat v),\\
\dot{\hat a}&=q^2\hat v+2qJ\hat a+k_a(R(\hat\psi)b_a-\hat a),\\
\dot{\hat\psi}&=r_m+h_L k_\psi w_{L,\psi}
                 \operatorname{wrap}(\tilde\psi_L-\hat\psi)
                 +h_G k_{G,\psi}\sin(\tilde\psi_G-\hat\psi).
\end{aligned}
\]

Here `s_G,s_L` are independent position-channel availability flags. Both
position residuals are active when both measurements are valid and fresh;
neither substitutes for or resets the other. `h_L` requires a qualified
LiDAR yaw weight. `h_G` requires fresh GNSS, a valid past displacement window
and sufficient current speed, and is used only when `h_L=0`.

The frozen physical motion gains remain `[kp,kv,ka,kpsi]=[4,4,12,4]`.
Added GNSS gains are `kG=1` and `kGpsi=0.5`, selected before the recorded
comparison, without a gain sweep on evaluation errors. `q=r_m+betaDot` retains
the existing turning model. A slow LiDAR motion discrepancy estimate adds
to the supplied lateral velocity; its filtered-rate contribution is included
in the slip-rate input. Fast participation/sampling changes and upstream
state/point mismatch remain part of the model-error budget.

The recorded GNSS channel uses **ODOM XY**, matching the previous GNSS adapter.
Only its two position coordinates and recorded diagonal position covariance
are read. ODOM quaternion, velocity and INSPVA pose are not GNSS measurement
inputs. ODOM is a GNSS/INS navigation product, not pure GNSS. Its mixed
position provenance and unknown physical output-point transform remain
limitations. The reported covariance forms a tuning weight, not an assertion
of independent errors relative to LiDAR or INSPVA:

\[
W_G=I_G(I_G+16I_2)^{-1},\qquad
W_L=I_L(I_L+0.001I_3)^{-1}.
\]

The LiDAR XY principal block and yaw diagonal are used, as in the existing
motion-aided design. This interface admits full-information packets only;
directional-only registrations remain unavailable full poses. No arbitrary
coordinate representative in a registration nullspace is treated as a pose.

## Sampled interface and missing data

Each stream has its own increasing acquisition timestamps, a logical validity
flag, and `delay=0`. A missing stream is allowed. Invalid packet payloads may
be NaN and are not evaluated. An invalid packet immediately withdraws the
previous anchor. A stopped stream expires after its configured maximum age
(0.2 s for each source); old measurements are not reused indefinitely.
Packet and expiry times are numerical integration boundaries, while requested
output times remain on the high-rate grid. High-rate motion/lateral samples
are left-held. States are integrated by RK4 with maximum step 0.005 s.

Between packets, the latest anchor is propagated by body velocity and gyro.
For a held `(vx,vy,r)` interval, the complex displacement is

\[
\Delta p=e^{i\psi_0}(v_x+i v_y)\,\Delta t\,
e^{i r\Delta t/2}\operatorname{sinc}(r\Delta t/2),
\]

where `sinc(x)=sin(x)/x`. LiDAR starts this propagation from its own heading.
The GNSS position anchor starts from the current estimated heading; it does
not read a GNSS/INS heading. These transported anchors are predictions from
old measurements, not newly acquired packets. Their original timestamps
and ages remain unchanged. There is no state reset, delayed-state replay,
future-pose interpolation, or truth-based correction. Nonzero processing
delay is rejected explicitly; this preserves the user's zero-delay setting.

`onlineZ` and `z` are identical and never revised. This causality claim is
about the runtime relative to its supplied streams. Existing sensor-clock
preparation, calibration, lateral input integration and map matching are
separate upstream processes with their own assumptions. The full pipeline
is not claimed to be independent of the reference or demonstrated online.

## GNSS-only heading from past positions

Position-only GNSS cannot supply stationary heading. For endpoints `a,b`
at least 2 s apart, retain past gyro angle `gamma` and integrate measured
body velocity (including the available slow lateral correction):

\[
D_b=e^{-i\gamma(b)}\int_a^b e^{i\gamma(s)}
(v_x(s)+i\hat v_y(s))\,ds,\qquad
\tilde\psi_G(b)=\arg[p_G(b)-p_G(a)]-\arg(D_b).
\]

This accounts for turning and changing body velocity over the window instead
of treating its chord as instantaneous vehicle heading. Under exact motion
and position it reconstructs endpoint heading. Actual position, gyro,
sideslip and quadrature errors enter its output error. It is an added dynamic
position/motion reconstruction, **not an independently measured GNSS yaw**.

Admission requires speed at least 1 m/s at recorded GNSS endpoints and at
injection, both displacement magnitudes at least 2 m, no GNSS packet gap over
0.25 s, and window duration 2--2.25 s. An explicit invalid GNSS packet clears
the history. The heading is propagated with gyro until the next packet; it
is disabled if the new packet cannot support a window. These engineering
admission checks do not themselves prove a bound on heading reconstruction
error or establish motion between every source endpoint.

The slow lateral correction likewise uses only past accepted LiDAR endpoints,
raw lateral estimates and integrated gyro. Its 2 s window, 0.25 s gap limit,
5 m/s endpoint-speed threshold, 0.8 m/s candidate bound and 4 s filter are
fixed. Invalid LiDAR packets clear its window. Its current bias state is
retained through outages, but no new outage measurements are invented. This
is an effective mapped-point correction, not identification of true CG slip.

## Independent cases and a common conditional bound

For the continuous equations driven by the declared reconstructed signals,
let `ep=pHat-p`, and similarly `ev,ea`. With bounded sensor/model disturbances,
bounded body motion and heading discrepancy, the translational error is

\[
\dot e_p=e_v-A_p e_p+d_p,\quad
\dot e_v=e_a-k_v e_v+d_v(e_\psi),\quad
\dot e_a=q^2 e_v+(2qJ-k_aI)e_a+d_a(e_\psi),
\]

where `A_p=sG*kG*WG+sL*kp*WLp`. Rotation error contributes at most a constant
times `|ePsi|` to `d_v,d_a` on bounded body signals. For
`Vt=|ep|^2+|ev|^2+|ea|^2`, the skew term cancels, and

\[
\dot V_t\le-\xi^T M(\alpha,q^2)\xi
             +2\|e_t\|\|d_t\|,\quad
M=\begin{bmatrix}
2\alpha&-1&0\\-1&2k_v&-(1+q^2)\\0&-(1+q^2)&2k_a
\end{bmatrix},
\]

where `xi=(|ep|,|ev|,|ea|)` and `alpha=lambda_min(Ap)`.
`designFullObserverGains` checks `q^2=0,0.16` and three **independent cases**:

- GNSS only, `WG >= 0.25 I`: `alpha >= 0.25`.
- LiDAR only, `WLp >= 0.25 I`: `alpha >= 1`.
- Both qualified: `alpha >= 1.25`.

All cases use the same `P=I6`; positivity at the endpoints covers the affine
`q^2` interval and arbitrary orientation of anisotropic weights. Larger
positive injection adds dissipation. Qualification is a hypothesis, not a
reason to clamp measured vehicle motion or inflate poor information weights.
The actual-weight dissipation margin at the declared worst-case course-rate
bound is also reported along the replay.

For LiDAR heading, a consistent local angular lift gives
`ePsiDot=-kpsi*w*ePsi+dPsi`. For admitted GNSS reconstruction,
`ePsiDot=-kGpsi*sin(ePsi)+dPsi`. The sine's Lipschitz property places bounded
heading-reconstruction error in `dPsi`. In `|ePsi|<=delta<pi/2`,

\[
\dot{|e_\psi|}\le-a_\psi|e_\psi|+\bar d_\psi,\quad
a_\psi=\min(k_\psi w_{min},k_{G,\psi}\sin\delta/\delta)>0.
\]

An invariant local region additionally needs the initial error inside the
region and `bar dPsi < aPsi*delta`; measurement lifts must avoid wrap branch
ambiguity there. This condition is **not verified from the recorded data**.
The common scalar heading bound and common translation function form an ISS
cascade when both bounds' hypotheses hold. Equivalently a sufficiently large
constant weight on `ePsi^2` absorbs the heading-to-motion cross terms into a
common composite Lyapunov function. This establishes conditional local ISS
for GNSS only after its displacement window is informative, LiDAR only, and
their combination, and permits channel changes that preserve those bounds.
It does not rely on merely adding two unrelated single-mode certificates.

Finite anchor age permits a finite reconstruction-error bound: for bounded
true/measured speeds, position transport error is bounded by endpoint error
plus `(Vtrue+Vmeasured)*maximumAge`. A useful small-error heading window bound
requires nonzero true displacement and bounded endpoint/integrated-motion
errors; admission alone does not establish those physical bounds. Upstream
CG/frame, calibration and lateral-observer errors belong in these budgets.

At standstill/startup without LiDAR yaw, the required heading contraction is
absent. With both position sources absent, `alpha=0` and this translation
matrix is not positive definite. Consequently **arbitrary sensor outages do
not inherit ISS**. A finite common outage admits only finite-horizon drift
bounds, followed by conditional recovery when informative data return. A
global intermittent-observation theorem would additionally require a uniform
net-contraction/coverage condition over time windows; none is inferred from
an isolated successful replay. The runtime always reports
`allTheoremHypothesesVerified=false` and no unconditional ISS claim.

For context on the distinction between mode-wise stability and switching
conditions, see Hespanha and Morse,
[Stability of Switched Systems with Average Dwell-Time](https://web.ece.ucsb.edu/~hespanha/published/avedwell.pdf).
The common certificate above is a project-specific derivation, not a theorem
claimed to appear in that paper. Receiver semantics follow NovAtel's
[BESTPOS position-type definitions](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm)
and [BESTGNSSPOS definition](https://docs.novatel.com/OEM7/Content/SPAN_Logs/BESTGNSSPOS.htm).
