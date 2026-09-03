% runLateralObserverDesign: Synthesize the LPV lateral-velocity observer and
% exercise it on the synthetic scenario. The synthesis solves the gridded
% H2 LMI over the configured speed and longitudinal acceleration ranges,
% re-checks the recovered gains against the original certificate, and the
% scenario then reports the estimation error under measurement noise.
% Requires YALMIP and an SDP solver (SeDuMi by default) on the MATLAB path.
%
% Input:
%   lateralObserverOverrideCfg: optional workspace configuration override
%
% Output:
%   design and result in the workspace
setupVehicleLocalization();
assert(exist("sdpvar", "file") == 2, ...
    "Add YALMIP and an SDP solver to the MATLAB path before running the lateral observer design.");
if exist("lateralObserverOverrideCfg", "var") && ~isempty(lateralObserverOverrideCfg)
    cfg = lateralObserverOverrideCfg;
else
    cfg = lateralObserverConfig();
end

design = designLateralObserverGains(cfg);
fprintf("tau = %.4g | eta = %.4g | H2 bound mu = %.5g\n", design.tau, design.eta, design.h2Bound);
fprintf("worst certificate eigenvalue = %.3e (negative definite required)\n", design.maxCertificateMargin);
fprintf("slowest error mode = %.3f 1/s\n", design.maxErrorEigenvalueRealPart);

result = simulateLateralObserverScenario(design, cfg);
fprintf("peak |vy| = %.3f m/s | peak side slip = %.2f deg\n", ...
    result.metrics.peakLateralVelocity, rad2deg(result.metrics.peakSideSlipAngle));
fprintf("settled RMSE: vy = %.4f m/s | r = %.5f rad/s | side slip = %.4f deg\n", ...
    result.metrics.settledLateralVelocityRmse, result.metrics.settledYawRateRmse, ...
    rad2deg(result.metrics.settledSideSlipRmse));
