% runReplayObserverDesign: Synthesize the replay observer gains offline and
% exercise the designed observer on the synthetic replay scenario. The main
% LMI jointly certifies the high-rate flow over the full output polytope and
% the discrete timestamped pose correction; it needs YALMIP and SeDuMi on the
% MATLAB path. Set designOutputFolder to persist the design.
%
% Input:
%   designOutputFolder: optional workspace variable for the saved design
%
% Output:
%   design and simulation results in the workspace
setupVehicleLocalization();
assert(exist("sdpvar", "file") == 2 && exist("sedumi", "file") == 2, ...
    "Add YALMIP and SeDuMi to the MATLAB path before running the gain design.");
cfg = replayObserverConfig();
if exist("designOutputFolder", "var") && strlength(string(designOutputFolder)) > 0
    cfg.replayDesign.outputFolder = string(designOutputFolder);
end

design = designReplayObserverGains(cfg);
disp(design.summary);
simulation = simulateReplayObserverScenario(design.cfg);
fprintf("Position RMSE: %.4f m, yaw RMSE: %.4f deg, max information age: %.3f s\n", ...
    simulation.metrics.positionRmse, rad2deg(simulation.metrics.yawRmse), simulation.metrics.maxInformationAge);
