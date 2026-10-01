function validateRemoval()
% validateRemoval Verify removal against the recorded no-learning control.
% Replay fixed scenario-specific packets; no new registration is performed.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/remove_lidar_velocity_bias_20260930';if ~isfolder(out),mkdir(out);end
    b=load('output/lidar_observer_regression_20260930/inputs.mat','data','lateral','cfg','reference');
    previous=load('output/lidar_observer_regression_20260930/diagnostic.mat','controls','names');
    cfg=mncavFullObserverConfig();cfg.initialState=b.cfg.initialState;
    assert(~isfield(cfg,'bias'));
    estimate=runFullLocalizationObserver(b.data,struct(),cfg,LateralInputs=b.lateral);
    noLearning=previous.controls{previous.names=="disable_velocity_bias"};
    difference=max(abs(estimate.z-noLearning.z),[],'all');assert(difference==0);
    assert(~any(isfield(estimate.diagnostics, ...
        {'lidarLongitudinalVelocityBias','lidarVelocityBias','lidarBiasUpdates','motionBiasSource'})));
    changed=b.data;changed.lidar.pose(changed.lidar.time>60 & changed.lidar.valid,:)=100;
    replay=runFullLocalizationObserver(changed,struct(),cfg,LateralInputs=b.lateral);
    prefix=max(abs(estimate.z(estimate.time<=60,:)-replay.z(estimate.time<=60,:)),[],'all');assert(prefix==0);
    t=estimate.time;error=vecnorm(estimate.position-b.reference(:,1:2),2,2);
    after=t>=t(1)+2;paired=after & b.data.lidar.valid;selected=find(after);
    [peak,k]=max(error(after));k=selected(k);
    validation=struct('frames',numel(t),'pairedFrames',nnz(paired),'configurationKind',cfg.kind, ...
        'priorDisabledLearningMaximumStateDifference',difference,'futureMutationPrefixDifference',prefix, ...
        'removedConfigurationAndDiagnostics',true,'allStatesFinite',all(isfinite(estimate.z),'all'), ...
        'pairedPositionRmseM',rms(error(paired)),'after2SecondsPositionRmseM',rms(error(after)), ...
        'after2SecondsMaximumM',peak,'maximumFrame',k,'frame847ErrorM',error(847), ...
        'scope',"Removal regression on 1169 frozen LiDAR-only packets and saved current-vehicle motion/lateral inputs. Matches the preceding disabled-learning control exactly; not a new closed-loop matching run or an accuracy improvement claim.");
    files=["config/fullObserverConfig.m","config/mncavFullObserverConfig.m", ...
        "localization/runSynchronousLocalizationObserver.m","localization/runFullLocalizationObserver.m", ...
        "scripts/validateFullLocalizationObserver.m","scripts/validateSyntheticLocalizationCascade.m", ...
        "scripts/runMncavInterfaceCorrectionComparison.m","scripts/calibrateMncavMotionOutputPoint.m", ...
        "tests/localizationMotionInputTest.m","tests/synchronousLocalizationTest.m", ...
        "tests/fullLocalizationObserverTest.m","tests/gnssOutputPointTest.m", ...
        "research/remove_lidar_velocity_bias_20260930/validateRemoval.m"];
    findings=cell(0,4);
    for file=files
        messages=checkcode(char(file),'-config=factory','-id');
        for j=1:numel(messages)
            findings(end+1,:)={file,messages(j).line,string(messages(j).id),string(messages(j).message)}; %#ok<AGROW>
        end
    end
    analyzer=cell2table(findings,VariableNames={'file','line','id','message'});
    writetable(analyzer,fullfile(dest,'code_analyzer.csv'));
    validation.codeAnalyzerFiles=numel(files);validation.codeAnalyzerFindings=height(analyzer);
    assert(isempty(analyzer));
    fid=fopen(fullfile(dest,'replay_validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'replay.mat'),'estimate','cfg','validation');disp(validation);
end
