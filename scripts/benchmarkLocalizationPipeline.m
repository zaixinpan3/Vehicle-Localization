function report = benchmarkLocalizationPipeline(outputFolder, repetitions, mapFrameCount)
% benchmarkLocalizationPipeline Time raw-frame coarse perception through pose.
% Seven Mississippi scenes use current offline maps built from subsequent
% frames, excluding the query. Every timed call repeats perception and D2D.
% Disk loading, offline mapping and map export are preparation, reported
% separately. The map-pose differences are same-route consistency only.

    arguments
        outputFolder (1,1) string = "output/localization_pipeline_timing"
        repetitions (1,1) double {mustBeInteger,mustBePositive} = 20
        mapFrameCount (1,1) double {mustBeInteger,mustBePositive} = 30
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    frames = [120,260,350,550,700,900,1050];
    starts = [.5,-.4,deg2rad(2);-.5,.4,-deg2rad(2);0,0,0];
    mapCfg = featureMapBuildConfig(); mapCfg.logEnabled = false;
    pcfg = perceptionConfig("Mississippi");
    cfg = struct('perception',pcfg,'registration',distributionRegistrationConfig());
    matPath = fullfile(root,'data',mapCfg.pointCloudMatPath);
    posePath = fullfile(root,'data',mapCfg.poseMatchCsvPath);
    inputs = cell(numel(frames),1);
    preparation = zeros(numel(frames),5);
    for index = 1:numel(frames)
        frameIndex = frames(index);
        timer = tic;
        item.frame = loadPointCloudFrame(matPath,frameIndex);
        preparation(index,1) = 1000*toc(timer);
        row = readFramePoseTable(posePath,frameIndex);
        [item.pose,item.tilt] = poseRowToPlanarPose(row);
        mappingFrames = frameIndex+(1:mapFrameCount);
        poses = readFramePoseTable(posePath,mappingFrames);
        timer = tic;
        observations = collectFeatureObservations(matPath,mappingFrames,poses,pcfg,mapCfg);
        preparation(index,2) = toc(timer);
        timer = tic;
        map = buildSlidingWindowMap(observations,mapCfg);
        preparation(index,3) = toc(timer);
        timer = tic;
        item.mapCloud = temporalMapToProbabilityCloud(map);
        preparation(index,4) = 1000*toc(timer);
        preparation(index,5) = item.mapCloud.components.numComponents;
        item.frameIndex = frameIndex;
        item.mappingFrames = mappingFrames;
        inputs{index} = item;
        fprintf('Prepared frame %d: %d current-map Gaussians; query excluded.\n', ...
            frameIndex,item.mapCloud.components.numComponents);
    end
    save(fullfile(outputFolder,'inputs.mat'),'inputs','cfg','mapCfg','-v7.3');
    report.preparation = array2table([frames(:),preparation], ...
        'VariableNames',{'frame','loadFrameMs','offlinePerceptionSeconds', ...
        'offlineMapSeconds','mapExportMs','mapComponents'});
    % Warm every scene and initial condition twice, recording those calls
    % separately. No timeit batching hides a slow invocation.
    count = numel(frames)*size(starts,1);
    warm = cell(2*count,19);
    for index = 1:2*count
        scene = mod(floor((index-1)/3),numel(frames))+1;
        start = mod(index-1,3)+1;
        warm(index,:) = runCase(inputs{scene},starts(start,:),start,0,index,cfg);
    end
    rows = cell(count*repetitions,19);
    stream = RandStream('mt19937ar','Seed',20260906);
    index = 0;
    for repetition = 1:repetitions
        order = randperm(stream,count);
        for caseIndex = order
            index = index+1;
            scene = floor((caseIndex-1)/3)+1;
            start = mod(caseIndex-1,3)+1;
            rows(index,:) = runCase(inputs{scene},starts(start,:),start,repetition,index,cfg);
        end
    end
    names = {'frame','start','repetition','order','totalMs','perceptionMs', ...
        'registrationMs','overheadMs','accepted','reason','rank','iterations', ...
        'sourceComponents','mapComponents','x','y','psi','positionDifferenceM','yawDifferenceDeg'};
    report.calls = cell2table(rows,'VariableNames',names);
    report.warmup = cell2table(warm,'VariableNames',names);
    groups = ["all","accepted","rejected"];
    summary = cell(3,10);
    for index = 1:3
        selected = true(height(report.calls),1);
        if groups(index)=="accepted", selected=report.calls.accepted; end
        if groups(index)=="rejected", selected=~report.calls.accepted; end
        durations = report.calls.totalMs(selected);
        maximum = NaN;
        if ~isempty(durations), maximum=max(durations); end
        summary(index,:) = {groups(index),nnz(selected), ...
            percentile(durations,.5),percentile(durations,.95), ...
            percentile(durations,.99),maximum, ...
            mean(durations>100),mean(durations>150),mean(durations>200), ...
            mean(durations>250)};
    end
    report.summary = cell2table(summary,'VariableNames',{'group','calls', ...
        'medianMs','p95Ms','p99Ms','maximumMs','fractionOver100ms', ...
        'fractionOver150ms','fractionOver200ms','fractionOver250ms'});
    report.metadata = struct('matlabVersion',version,'frames',frames, ...
        'starts',starts,'repetitions',repetitions,'seed',20260906, ...
        'featureNames',pcfg.featureNames,'mapSchema',2,'mapFrameCount',mapFrameCount, ...
        'nativeKernel',which('perceptionKernelsMex'), ...
        'timedScope',"Preloaded raw frame and cached exported local map to accepted [X,Y,psi] event or explicit rejection", ...
        'excluded',"Disk loading; offline mapping/export; sensor scan acquisition; transport; ROS/queue scheduling; visualization", ...
        'limitation',"Seven same-route scenes and three starts, repeated on one desktop; empirical latency, not a hard real-time bound or independent accuracy test");
    writetable(report.calls,fullfile(outputFolder,'calls.csv'));
    writetable(report.warmup,fullfile(outputFolder,'warmup.csv'));
    writetable(report.preparation,fullfile(outputFolder,'preparation.csv'));
    writetable(report.summary,fullfile(outputFolder,'summary.csv'));
    fid = fopen(fullfile(outputFolder,'metadata.json'),'w');
    assert(fid>=0,'Cannot write benchmark metadata.');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report.metadata,PrettyPrint=true));
    save(fullfile(outputFolder,'report.mat'),'report');
    disp(report.summary);
end

function row = runCase(item,offset,start,repetition,index,cfg)
    cfg.perception.coarseProbabilityCloud.projectionRotation = item.tilt;
    timer = tic;
    [event,result] = localizeLidarFrame(item.frame,item.mapCloud,item.pose+offset, ...
        double(item.frameIndex),cfg);
    total = 1000*toc(timer);
    assert(result.accepted == ~isempty(event),'Acceptance/event mismatch.');
    if ~isempty(event)
        assert(isequal(size(event.pose),[1,3]) && all(isfinite(event.pose)), ...
            'An accepted event must carry a finite [X,Y,psi] pose.');
    end
    delta = result.poseXYTheta-item.pose;
    perception = 1000*result.perceptionSeconds;
    registration = 1000*result.registrationSeconds;
    row = {item.frameIndex,start,repetition,index,total,perception,registration, ...
        total-perception-registration,result.accepted,string(result.reason), ...
        result.observableRank,result.iterations,result.probabilityCloud.components.numComponents, ...
        item.mapCloud.components.numComponents,result.poseXYTheta(1),result.poseXYTheta(2), ...
        result.poseXYTheta(3),norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3))))};
end

function value = percentile(values,fraction)
% percentile Use a stated linear empirical quantile without a toolbox.
    values = sort(values(:));
    if isempty(values), value=NaN; return; end
    coordinate = 1+(numel(values)-1)*fraction;
    low = floor(coordinate); high = ceil(coordinate);
    value = values(low)+(coordinate-low)*(values(high)-values(low));
end
