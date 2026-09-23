function cache=prepareMississippiLocalizationClouds(matchingFolder)
% prepareMississippiLocalizationClouds Recompute coarse horizons on recorded motion.
% Preparation is reusable across estimator controls; each horizon uses only
% current and preceding scans. Reference poses supply known tilt only.
    root=setupVehicleLocalization();
    saved=load(fullfile(matchingFolder,'report.mat'),'cfg','report');
    cfg=saved.cfg;cfg.sourceWindow=localizationSourceWindowConfig();calls=saved.report.calls;n=height(calls);sources=cell(n,1);
    mapCfg=featureMapBuildConfig();history=[];seconds=zeros(n,1);counts=zeros(n,1);
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),calls.frame.');
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));
    for first=1:50:n
        ids=first:min(n,first+49);block=store.pointClouds(1,calls.frame(ids));
        for k=ids
            timer=tic;[~,tilt]=poseRowToPlanarPose(poses(k,:));
            cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
            raw=perceiveCoarseProbabilityCloud(block(k-first+1),cfg.perception);
            motion=saved.report.deadReckoning{k,{'x','y','psi'}};
            [sources{k},history]=updateLocalizationSourceWindow(raw,calls.timeSeconds(k),motion,history,cfg.sourceWindow);
            seconds(k)=toc(timer);counts(k)=sources{k}.components.numComponents;
        end
        fprintf('Coarse source cache %d/%d\n',ids(end),n);
    end
    map=load(saved.report.metadata.sourceMap,'cloud');
    fixed=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    cache=struct('sources',{sources},'fixed',fixed,'calls',calls,'cfg',cfg, ...
        'seconds',seconds,'counts',counts,'clockModelId',saved.report.metadata.clockModelId);
end
