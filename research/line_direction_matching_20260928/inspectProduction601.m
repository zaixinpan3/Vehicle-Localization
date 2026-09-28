function inspectProduction601()
% inspectProduction601 Verify the deployed frame and export its actual pairs.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/line_direction_matching_20260928/sources.mat');
    p=load('output/line_direction_matching_20260928/production/report.mat','report');c=p.report.calls(601,:);
    seed=c{1,{'predictedX','predictedY','predictedPsi'}};ref=c{1,{'referenceX','referenceY','referencePsi'}};cfg=distributionRegistrationConfig();
    [local,ids]=selectLocalProbabilityCloud(s.fixed,seed,cfg.localMapRadius);
    r=registerSemanticProbabilityCloud(local,s.sources{601},seed,cfg);assert(max(abs(r.poseXYTheta-c{1,{'x','y','psi'}}))<1e-7);
    pairs=r.correspondences;pairs.globalTarget=ids(pairs.target);pairs.sourceMean=s.sources{601}.components.mean(pairs.source,:);
    rot=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];pairs.targetReferenceBody=(local.components.mean(pairs.target,:)-ref(1:2))*rot;
    writetable(pairs,fullfile(dest,'frame601_pairs.csv'));writetable(r.classDiagnostics,fullfile(dest,'frame601_class_diagnostics.csv'));
    fid=fopen(fullfile(dest,'frame601_solver.json'),'w');fprintf(fid,'%s\n',jsonencode(struct('pyramid',r.pyramid,'lineDirection',cfg.lineDirection, ...
        'directionsUsed',nnz(pairs.lineDirectionUsed),'referenceUsedBySolver',false,'poseReproductionMaxAbs',max(abs(r.poseXYTheta-c{1,{'x','y','psi'}}))),PrettyPrint=true));fclose(fid);
end
