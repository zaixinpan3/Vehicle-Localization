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
    fprintf('INVARIANT DESIGN %d vertices, worst %.3g; kL %.6g kG %.6g auxiliary baseline %s\n', ...
        runtime.continuousCertificate.verification.vertices,design.verification.worstVertexEigenvalue, ...
        design.kappaL,design.kappaG,mat2str(design.auxiliaryWeights.',5));
    if isfield(design,'motionGainSchedule')
        interval=design.verification.motionResponse.speedInterval;
        low=motionResponseSupport.weights(design,interval(1)^2);
        high=motionResponseSupport.weights(design,interval(2)^2);
        fprintf('SCHEDULED AUXILIARY weights at %.3g/%.3g m/s: %s / %s; nominal motion time constant %.3g s\n', ...
            interval(1),interval(2),mat2str(low.',5),mat2str(high.',5),design.verification.motionResponse.timeConstantS);
    end
    joint=runtime.continuousCertificate.verification.jointSensor;
    fprintf('COMMON SENSOR DESIGN %d L + %d G + %d M controls; worst L %.9g G %s M %s\n', ...
        joint.lidarVertices,joint.gnssBernsteinVertices,joint.mixedVertices,joint.worstLidarEigenvalue, ...
        mat2str(joint.worstGnssEigenvalue,9),mat2str(joint.worstMixedEigenvalue,9));
    region=design.verification.regionalIss;
    fprintf('CONDITIONAL REGIONAL ISS radius %.9g, V level %.9g, alpha %.9g, additive input limit %.9g\n', ...
        region.normalizedRadius,region.lyapunovLevel,region.alpha,region.normalizedAdditiveInputLimit);
end
