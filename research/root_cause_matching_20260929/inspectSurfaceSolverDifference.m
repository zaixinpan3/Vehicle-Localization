function inspectSurfaceSolverDifference()
% inspectSurfaceSolverDifference Locate real-scan disagreement with the direct model.
addpath(pwd); setupVehicleLocalization; addpath('research/root_cause_matching_20260929');
cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();poses=readFramePoseTable(fullfile(pwd,'data',mc.poseMatchCsvPath),301:399);s=matfile(fullfile(pwd,'data',mc.pointCloudMatPath));raw=s.pointClouds(1,301:399);
for j=1:numel(raw)
 k=300+j;[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(raw(j),cfg);g=p.diagnostics.ground;c=p.diagnostics.groundPointContext;R=p.probabilityCloud.projectionRotation;t=p.probabilityCloud.projectionTranslation;
 [a,da]=estimateCurbSurfaceMoments(g,c,R,t);[b,db]=estimateCurbBoundaryGeometry(g,c,R,t);err=max(abs(a.mean-b.mean),[],'all');
 if err>1e-8,fprintf('DIFFERENCE frame %d error %.10g\n',k,err);disp(da);disp(db);save('output/root_cause_matching_20260929/surface_solver_difference.mat','k','g','c','R','t','a','b','da','db');break;end
end

end
