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
    runtime=localizationSupport.runtime(cfg);
    fprintf('NOMINAL FUSION: %d vertices, worst %.9g; one SDP; noise proxy %.9g\n', ...
        runtime.continuousCertificate.verification.vertices,design.verification.worstVertexEigenvalue,design.meanSquareBound);
    fprintf('MOTION RESPONSE: required %d, nominal time constant %.3g s; operating-set invariance assumed, radius not certified\n', ...
        design.verification.motionResponse.required,design.verification.motionResponse.timeConstantS);
end
