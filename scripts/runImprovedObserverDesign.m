% runImprovedObserverDesign Construct a continuous-mode ISS certificate.
% Select improvedObserverConfig("gnss") for the position-only GNSS mode.
% The default LiDAR synthesis requires YALMIP and the configured SDP solver.
projectFolder=fileparts(fileparts(mfilename('fullpath')));
addpath(projectFolder);
setupVehicleLocalization;
cfg=improvedObserverConfig("lidar");
design=designImprovedObserverGains(cfg);
fprintf('Continuous %s certificate: margin %.6g, rate %.6g /s.\n', ...
    cfg.mode,design.verification.uniformMargin,design.verification.rate);
