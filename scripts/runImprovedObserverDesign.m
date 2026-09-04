% runImprovedObserverDesign Synthesize the improved-observer gains.
% YALMIP and SeDuMi must already be on the MATLAB path. The design is left in
% the workspace only after every robust-LMI vertex combination has been checked.

projectFolder = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(projectFolder, "setupVehicleLocalization.m"));
cfg = improvedObserverConfig();
design = designImprovedObserverGains(cfg);

fprintf("Certified %d robust-LMI vertex combinations; lambda = %.6g.\n", ...
    design.verification.checkedVertexCount, design.lambda);
