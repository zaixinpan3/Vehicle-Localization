% runLateralObserverDesign Synthesize and report the lateral LPV gains.
% Solves 24 scheduled-metric ISS LMIs for the virtual output Vx*ay.
% Verifies numerical LMI negativity; no automatic simulation is performed.
% Requires YALMIP and an SDP solver on the MATLAB path.
% Optional input: lateralObserverOverrideCfg. Output: design.
setupVehicleLocalization();
assert(exist("sdpvar", "file") == 2, ...
    "Add YALMIP and an SDP solver to the MATLAB path before running the lateral observer design.");
if exist("lateralObserverOverrideCfg", "var") && ~isempty(lateralObserverOverrideCfg)
    cfg = lateralObserverOverrideCfg;
else
    cfg = lateralObserverConfig();
end

design = designLateralObserverGains(cfg);
fprintf("tau = %.4g | decay rate = %.3g 1/s | ISS gain = %.5g\n", ...
    design.tau, design.decayRate, design.issGain);
fprintf("largest vertex gain norm = %.4g | error bound at unit disturbance = %.4g\n", ...
    design.maxVertexGainNorm, design.errorBound);
fprintf("speed-rate domain: [%.3g, %.3g] m/s^2 | maximum LMI eigenvalue: %.4g\n", ...
    design.certificateDomain.speedRateRange, design.maximumCertificateEigenvalue);
fprintf("solver: %s\n", design.solverInfo);
