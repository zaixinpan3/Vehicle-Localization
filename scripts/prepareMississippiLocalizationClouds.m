function cache=prepareMississippiLocalizationClouds(matchingFolder,options)
% prepareMississippiLocalizationClouds Recompute coarse horizons on recorded motion.
% Preparation is reusable across estimator controls; each horizon uses only
% current and preceding scans. Raw LiDAR IMU supplies causal tilt; reference
% poses are not opened. Fixed installation compensation remains unchanged.
    arguments
        matchingFolder (1,1) string
        options.SensorFolder (1,1) string="output/mncav_wheel_only_20260916/sensors"
        options.TiltConfiguration (1,1) struct=lidarImuTiltConfig()
    end
    root=setupVehicleLocalization();
    saved=load(fullfile(matchingFolder,'report.mat'),'cfg','report');
    cfg=saved.cfg;cfg.sourceWindow=localizationSourceWindowConfig();calls=saved.report.calls;n=height(calls);sources=cell(n,1);
    currentSources=cell(n,1);
    mapCfg=featureMapBuildConfig();history=[];seconds=zeros(n,1);counts=zeros(n,1);
    tilt=prepareMississippiLidarTilt(calls.rosStamp,options.SensorFolder,Configuration=options.TiltConfiguration);
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));
    for first=1:50:n
        ids=first:min(n,first+49);block=store.pointClouds(1,calls.frame(ids));
        for k=ids
            timer=tic;
            cfg.perception.coarseProbabilityCloud.projectionRotation=tilt.rotation(:,:,k);
            raw=perceiveCoarseProbabilityCloud(block(k-first+1),cfg.perception);
            motion=saved.report.deadReckoning{k,{'x','y','psi'}};
            [sources{k},history,~,currentSources{k}]=updateLocalizationSourceWindow(raw,calls.timeSeconds(k),motion,history,cfg.sourceWindow);
            seconds(k)=toc(timer);counts(k)=sources{k}.components.numComponents;
        end
        fprintf('Coarse source cache %d/%d\n',ids(end),n);
    end
    map=load(saved.report.metadata.sourceMap,'cloud');
    % Keep map XYZ side information while retaining its exact XY geometry.
    fixed=map.cloud;
    cache=struct('sources',{sources},'currentSources',{currentSources},'fixed',fixed,'calls',calls,'cfg',cfg, ...
        'seconds',seconds,'counts',counts,'clockModelId',saved.report.metadata.clockModelId,'tilt',tilt);
end
