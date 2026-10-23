function [design,cfg]=synthesizeMncavInvariantDesign(cfg)
% Solve and verify temporary gains from this run's fully configured inputs.
% Returns the design and a ready-to-run configuration; reads/writes no gains
% file and always re-solves, including when cfg already contains a design.
    arguments
        cfg (1,1) struct=mncavFullObserverConfig()
    end
    root=setupVehicleLocalization();external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    design=invariantObserverSupport.synthesize(cfg);
    cfg.invariantDesign=design;cfg.gainSource="fresh-lmi";
    runtime=invariantObserverSupport.runtime(cfg);
    fprintf('INVARIANT DESIGN %d vertices, worst %.3g; kL %.6g kG %.6g auxiliary %s\n', ...
        runtime.continuousCertificate.verification.vertices,design.verification.worstVertexEigenvalue, ...
        design.kappaL,design.kappaG,mat2str(design.auxiliaryWeights.',5));
    joint=runtime.continuousCertificate.verification.jointSensor;
    fprintf('COMMON SENSOR DESIGN %d L + %d G + %d M controls; worst L %.9g G %s M %s\n', ...
        joint.lidarVertices,joint.gnssBernsteinVertices,joint.mixedVertices,joint.worstLidarEigenvalue, ...
        mat2str(joint.worstGnssEigenvalue,9),mat2str(joint.worstMixedEigenvalue,9));
    region=design.verification.regionalIss;
    fprintf('CONDITIONAL REGIONAL ISS radius %.9g, V level %.9g, alpha %.9g, additive input limit %.9g\n', ...
        region.normalizedRadius,region.lyapunovLevel,region.alpha,region.normalizedAdditiveInputLimit);
end
