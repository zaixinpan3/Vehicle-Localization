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
    fprintf('NOMINAL FUSION: %d vertices, worst %.9g; %d primary SDP solves; normalized noise energy %.9g\n', ...
        runtime.continuousCertificate.verification.vertices,design.verification.worstVertexEigenvalue, ...
        design.solveAudit.optimizationCalls-1,design.templateNoiseEnergy);
    fprintf('DIRECT YAW RATE: solved %.9g /s; verified normalized rate-error energy %.9g\n', ...
        design.verification.yawRateGainPerSecond,design.verification.rateNoiseEnergy);
    fprintf('MATCHED MOTION: GNSS bandwidth ratio %.9g; implicit Jacobian-weighted motion update.\n',cfg.invariantObserver.gnssBandwidthRatio);
end
