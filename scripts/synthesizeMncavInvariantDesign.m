function [design,cfg]=synthesizeMncavInvariantDesign(cfg)
% Solve and verify temporary gains from this run's fully configured inputs.
% Returns the design and a ready-to-run configuration; reads/writes no gains
% file and always re-solves, including when cfg already contains a design.
    arguments
        cfg (1,1) struct=mncavFullObserverConfig()
    end
    root=setupVehicleLocalization();external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    design=localizationSupport.synthesize(cfg);
    cfg.invariantDesign=design;cfg.gainSource="fresh-lmi";
    fprintf('Four-vertex design: worst eigenvalue %.9g, GNSS heading %.9g/s, upstream rate %.9g/s.\n', ...
        design.verification.worstVertexEigenvalue,cfg.invariantObserver.theta/design.P(7,7), ...
        design.verification.yawRateGainPerSecond);
end
