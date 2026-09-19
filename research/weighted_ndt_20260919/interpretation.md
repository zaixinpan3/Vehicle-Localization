# Interpreting the NDT candidate regression

Date: 2026-09-19. Code and experiment reviewed: commit
`cdec5f49326437c0bfd6811f981d46b76a438c9b` and this directory's comparison.
This is a literature/code interpretation; no new registration run or parameter
change was performed for this note.

## What the comparison establishes

The evaluated candidate is a custom, semantic, stability-weighted Gaussian
product-integral matcher. The experiment establishes a regression of that
implementation relative to the current geometric matcher. It does not establish
that NDT generally performs worse or that existing map stability weights should
be removed. The two methods differ in objective, geometric constraints, source
weighting, associations and class-consistency diagnostics; this is not an
isolated experiment on one design choice.

## Verified objective difference

Gupta, Andreasson, Magnusson, Julier and Lilienthal, *Revisiting
Distribution-Based Registration Methods*, Section II-A, equation (4), express
D2D-NDT using a sum of Mahalanobis exponential kernels with shared constants.
Section III and Figure 1 discuss the consequences of including the Gaussian
normalization factor. Their P2D example shows additional local minima and a
minimum displaced from the known rotation. This is evidence of a possible
mechanism, not a causal result on our Mississippi sequence.
[Author manuscript](https://discovery.ucl.ac.uk/id/eprint/10217483/1/Revisiting%20Distribution-Based%20Registration%20Methods.pdf).

Our `semanticNdtSupport.evaluate` uses, in XY,

\[
  \pi_j K_{ij}=\frac{\pi_j}{2\pi\sqrt{\det C_{ij}}}
    \exp\left(-\frac12\delta_{ij}^{T}C_{ij}^{-1}\delta_{ij}\right).
\]

The map weight enters once, but the covariance-dependent amplitude also affects
the score. At equal stored weight and zero displacement, halving both principal
standard deviations of the combined covariance increases the XY kernel peak
fourfold. This is valid Gaussian-overlap mathematics; its desirability for
geometric localization must be measured. A narrow spatial distribution is not
by itself a calibrated estimate of pose accuracy. Removing the determinant
factor would be a change in matching score, not a change to the stored map's
stability weights. It has not been validated as a full-sequence remedy.

The current PCL P2D implementation likewise uses shared `gauss_d1_` and
`gauss_d2_` constants derived from resolution and outlier ratio. Its official
tutorial explains the dependence on spatial scale and initial alignment.
[PCL source](https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/impl/ndt.hpp),
[PCL tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html).
P2D and D2D remain distinct variants; neither reference is a promise of better
results with our scan-to-GMM representation.

## Confirmed diagnostics versus hypotheses

- Confirmed from code: the geometric control uses normal-only residuals for
  elongated curb components. The candidate fits finite density distributions,
  which can also respond to along-curb density and endpoints. Single-frame
  pillar statistics and multiframe map components need not have identical
  spatial density. This identifies a plausible bias mechanism, not its measured
  contribution to the observed error.
- Confirmed from the prior experiment: frame 214 has a 0.009044 m raw candidate
  discrepancy but is rejected because its curb-class correction exceeds the
  0.50 gate. Candidate class rejections rise from the control's 96 to 354.
  The rejection rule therefore discards at least some accurate candidates.
- Confirmed from the identical 597 accepted recursive frames: control RMSE is
  0.200412 m and candidate RMSE is 0.281346 m. Rejection alone cannot explain
  the regression. Resetting each initial guess near the reference also leaves
  a regression; this does not eliminate local optimization as a possible cause.
- Confirmed from code: the candidate has one kernel scale and bounded local
  SQP, with an optional reduced-subspace solve. It does not implement a
  multiscale registration schedule. Scale and initialization sensitivity have
  not been comprehensively assessed.

The next discriminating evaluation should hold scan, map and existing map
weights fixed, inspect objective values around the reference pose, and compare
raw optimized candidates separately from admission decisions. A displaced
objective optimum would indicate model/score bias; failure to reach an optimum
near the reference would indicate an optimization issue; a good candidate
subsequently rejected would indicate admission behavior. These are proposed
checks, not completed new experiments. INSPVA remains a recorded reference,
not independent ground truth. The default and opt-in configurations are unchanged.
