function options=globalSolverOptions(solver)
% Load the global-design SDP dependencies and configure the selected solver.
    root=setupVehicleLocalization();
    external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')),genpath(fullfile(external,'sedumi')));
    assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','Add YALMIP before synthesis.');
    options=sdpsettings('solver',char(solver),'verbose',0);
    if solver=="sedumi",options.sedumi.stepdif=2;end
end
