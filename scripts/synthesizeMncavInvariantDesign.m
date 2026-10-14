function design=synthesizeMncavInvariantDesign()
% Solve and independently verify current intrinsic/invariant observer gains.
    root=setupVehicleLocalization();external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    cfg=mncavFullObserverConfig(LoadDesign=false);design=invariantObserverSupport.synthesize(cfg);
    stored=struct('inputs',invariantObserverSupport.inputs(cfg),'design',design);
    f=fopen(fullfile(root,'config/mncavInvariantDesign.json'),'w');fprintf(f,'%s\n',jsonencode(stored,PrettyPrint=true));fclose(f);
    runtime=invariantObserverSupport.runtime(mncavFullObserverConfig());
    fprintf('INVARIANT DESIGN %d vertices, worst %.3g; kL %.6g kG %.6g auxiliary %s\n', ...
        runtime.continuousCertificate.verification.vertices,design.verification.worstVertexEigenvalue, ...
        design.kappaL,design.kappaG,mat2str(design.auxiliaryWeights.',5));
    joint=runtime.continuousCertificate.verification.jointSensor;
    fprintf('COMMON SENSOR DESIGN %d L + %d G controls, worst L %.9g G %.9g; motion floor %.3g m/s\n', ...
        joint.lidarVertices,joint.gnssBernsteinVertices,joint.worstLidarEigenvalue, ...
        joint.worstGnssEigenvalue,joint.minimumGnssCourseSpeed);
end
