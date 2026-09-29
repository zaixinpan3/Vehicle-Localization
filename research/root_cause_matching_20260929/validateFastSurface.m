function validateFastSurface()
% validateFastSurface Compare batched normal equations with the direct QR model.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;
    mc=featureMapBuildConfig();frames=[135 154 178 300 469 774 806 829 854 856 893 1028 1096 1170];poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),frames);rows=cell(numel(frames),5);
    for j=1:numel(frames)
        k=frames(j);raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(raw,cfg);
        g=p.diagnostics.ground;c=p.diagnostics.groundPointContext;R=p.probabilityCloud.projectionRotation;t=p.probabilityCloud.projectionTranslation;
        timer=tic;[a,da]=estimateCurbSurfaceMoments(g,c,R,t);slowMs=toc(timer)*1000;
        timer=tic;[b,db]=estimateCurbSurfaceMomentsFast(g,c,R,t);fastMs=toc(timer)*1000;
        error=max(abs(a.mean-b.mean),[],'all');same=isequal(da.accepted,db.accepted);assert(error<1e-7 && same);
        rows(j,:)={k,error,same,slowMs,fastMs};
    end
    results=cell2table(rows,VariableNames={'frame','maximumMeanDifference','sameDecisions','directMs','batchedMs'});writetable(results,fullfile(dest,'surface_solver_validation.csv'));disp(results);
end
