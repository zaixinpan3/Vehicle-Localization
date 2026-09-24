function cacheInputs(worker,workers)
% cacheInputs Fresh independent perception partitions; matching stays sequential.
    setupVehicleLocalization();maxNumCompThreads(2);
    out='output/fine_matching_20260919';if ~isfolder(out),mkdir(out);end
    baseline=load('output/matching_refinement_20260919/validated/recursive/report.mat');
    mapCfg=featureMapBuildConfig();posePath=fullfile('data',mapCfg.poseMatchCsvPath);
    n=height(baseline.report.calls);
    frames=(floor(n*(worker-1)/workers)+1):floor(n*worker/workers);
    poses=readFramePoseTable(posePath,frames);cfg=perceptionConfig('Mississippi',"offline");
    store=matfile('data/raw/MissisipiPointClouds.mat');
    fineClouds=cell(numel(frames),1);coarseClouds=fineClouds;selectedIndices=cell(numel(frames),3);
    perceptionSeconds=zeros(numel(frames),1);conversionSeconds=perceptionSeconds;counts=zeros(numel(frames),3);
    blockSize=20;destination=fullfile(out,sprintf('inputs_%d.mat',worker));
    for first=1:blockSize:numel(frames)
        last=min(numel(frames),first+blockSize-1);block=store.pointClouds(1,frames(first:last));
        for k=first:last
            frame=block(k-first+1);[~,tilt]=poseRowToPlanarPose(poses(k,:));
            cfg.coarseProbabilityCloud.projectionRotation=tilt;
            assert(abs(double(frame.timestamp)-poses.lidar_stamp_sec(k))<1e-5,'Frame timestamp mismatch.');
            timer=tic;result=perceiveFrame(frame,cfg);perceptionSeconds(k)=toc(timer);
            assert(result.executionMode=="offline" && isfield(result,'featureMasks'));
            coarseClouds{k}=result.probabilityCloud;
            timer=tic;fineClouds{k}=buildFineMatchingCloud(frame,result.featureMasks,cfg);conversionSeconds(k)=toc(timer);
            for j=1:numel(cfg.featureNames)
                selectedIndices{k,j}=find(result.featureMasks.(cfg.featureNames(j)));
                counts(k,j)=numel(selectedIndices{k,j});
            end
        end
        processed=last;
        save(destination,'frames','fineClouds','coarseClouds','selectedIndices','counts', ...
            'perceptionSeconds','conversionSeconds','cfg','processed','-v7.3');
        fprintf('Worker %d: %d/%d fine frames; latest frame %d, %.2f s\n',worker,last,numel(frames),frames(last),perceptionSeconds(last));
    end
end
