function validation=validate_experiment()
% validate_experiment Verify temporal support, replay provenance and call path.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    folder=fullfile(root,'output/temporal_perception_20260922');
    old=load('output/lidar_origin_20260922/coarse_pipeline/matching/report.mat','report','cfg');
    new=load(fullfile(folder,'matching/report.mat'),'report','cfg');
    assert(isequaln(old.cfg.perception,new.cfg.perception) && isequaln(old.cfg.registration,new.cfg.registration));
    assert(isequal(old.report.deadReckoning,new.report.deadReckoning));
    w=new.report.sourceWindows;c=new.report.calls;
    assert(height(c)==1170 && all(w.minimumSupport(w.components>0)>=2));
    assert(~c.accepted(1) && ~c.directionalAccepted(1) && w.components(1)==0);
    assert(nnz(c.accepted)==1167 && nnz(c.directionalAccepted)==2);
    assert(~new.report.metadata.finePerceptionUsed && new.report.metadata.perceptionRerun);
    window=new.cfg.sourceWindow; % Reproduce this historical experiment explicitly.
    assert(window.maximumFrames==3 && window.maximumAgeSeconds==.25);
    s=load('output/frame959_matching_diagnosis_20260922/diagnostic.mat');h=[];
    for k=1:2
        [~,h]=updateLocalizationSourceWindow(s.history.clouds{k},s.history.time(k),s.history.motion(k,:),h,window);
    end
    [cloud,~,details]=updateLocalizationSourceWindow(s.history.clouds{3},s.history.time(3),s.history.motion(3,:),h,window);
    result=registerSemanticProbabilityCloud(s.fixed,cloud,s.seed,s.cfg.registration);
    assert(result.directionalAccepted && ~result.accepted && result.observableRank==2);
    for id=[100,142]
        poles=cloud.components.mean(cloud.components.semanticName=="pole",:);
        assert(all(vecnorm(poles-s.source.components.mean(id,:),2,2)>.75));
    end
    unchanged=registerSemanticProbabilityCloud(s.fixed,s.source,s.seed,s.cfg.registration);
    assert(max(abs(unchanged.poseXYTheta-s.results{1}.poseXYTheta))<1e-7);
    input=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',959);
    cfg=s.cfg;cfg.sourceWindow=window;
    profile clear;profile on;cleanup=onCleanup(@()profile('off'));
    [event,online]=localizeLidarFrame(input,s.fixed,s.seed,s.history.time(3),cfg,h,s.history.motion(3,:));
    profile off;trace=profile('info');clear cleanup;
    names=string({trace.FunctionTable.FunctionName});
    assert(any(contains(names,'perceiveCoarseProbabilityCloud')) && any(contains(names,'stableTracks')));
    forbidden=["refinePerceptionCandidates","buildFineFeatureMasks","buildSavedFeatureProbabilityCloud"];
    assert(~any(contains(names,forbidden)));
    assert(~isempty(event) && online.directionalAccepted);
    assert(max(abs(online.poseXYTheta-result.poseXYTheta))<1e-8);
    profile clear;
    save(fullfile(folder,'frame959_and_call_path.mat'),'cloud','result','details','online','trace');
    issues=cell(0,3);
    files=["config/localizationSourceWindowConfig.m","localization/updateLocalizationSourceWindow.m", ...
        "localization/registerSemanticProbabilityCloud.m","localization/registrationSupport.m", ...
        "localization/localizeLidarFrame.m","scripts/replayMississippiLocalization.m", ...
        "scripts/benchmarkLocalizationPipeline.m","scripts/measurePacedLocalizationPipeline.m", ...
        "scripts/runMncavFullObserverExperiment.m","tests/localizationSourceWindowTest.m"];
    helpers=dir(fullfile(dest,'*.m'));files=[files,string(fullfile({helpers.folder},{helpers.name}))];
    for file=files
        messages=checkcode(file,'-id','-config=factory');
        for k=1:numel(messages),issues(end+1,:)={file,messages(k).line,messages(k).message};end %#ok<AGROW>
    end
    writetable(cell2table(issues,VariableNames={'file','line','message'}),fullfile(dest,'code_analysis.csv'));
    assert(isempty(issues));tests=readtable(fullfile(dest,'tests.csv'));assert(all(tests.Passed));
    validation=struct('testsPassed',height(tests),'testsFailed',nnz(tests.Failed),'codeAnalyzerIssues',size(issues,1), ...
        'allFramesRequireRepeatedEvidence',true,'frame1Withheld',true,'coarseOnlyCallPathVerified',true, ...
        'perceptionParametersUnchanged',true,'matcherParametersUnchanged',true,'odometryUnchanged',true, ...
        'frame959SameSeedErrorM',norm(result.poseXYTheta(1:2)-s.reference(1:2)), ...
        'frame959SameSeedStatus',result.reason,'frame959RecursiveErrorM',c.positionErrorM(959), ...
        'frame959UnfilteredComponents',details.unfilteredComponentCount,'frame959StableComponents',details.componentCount, ...
        'frame959SingletonsRemoved',details.rejectedSingletons, ...
        'frame959SuspectPolesExcluded',true,'noInformationMultiplicationForRepeatedIdenticalScans',true, ...
        'windowMedianMs',median(w.windowMs),'windowP95Ms',prctile(w.windowMs,95),'windowMaximumMs',max(w.windowMs), ...
        'fiveFrameVariantSelected',false,'fullSequenceRuns',2,'sameDriveMap',true,'independentAccuracyClaimed',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));disp(validation);
end
