% runLateralObserverDesign Synthesize and report the lateral LPV gains.
% Solves twelve position/rate vertex LMIs and returns the solver result.
% No post-solve verification, sampling or automatic simulation is performed.
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
fprintf("tau = %.4g | slack scale = %.4g | decay rate = %.3g 1/s | ISS gain = %.5g\n", ...
    design.tau, design.slackScale, design.decayRate, design.issGain);
fprintf("largest vertex gain norm = %.4g | error bound at unit disturbance = %.4g\n", ...
    design.maxVertexGainNorm, design.errorBound);
fprintf("solver: %s\n", design.solverInfo);
