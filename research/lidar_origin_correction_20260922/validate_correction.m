function validation=validate_correction(selectedFolder)
% validate_correction Validate deployed calibration and recorded regressions.
    arguments
        selectedFolder (1,1) string
    end
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/lidar_origin_20260922';
    setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
    tests=runtests({'tests/lidarOriginCalibrationTest.m','tests/inspvaMappingPoseTest.m', ...
        'tests/geometricRegistrationTest.m','tests/localizationSourceWindowTest.m', ...
        'tests/wholePillarPerceptionTest.m','tests/coarseSemanticProbabilityCloudTest.m', ...
        'tests/heightProbabilityCloudTest.m','tests/receiverClockTest.m'});
    writetable(table(tests),fullfile(dest,'tests.csv'));assert(all([tests.Passed]));
    files=["config/lidarFrameCalibrationConfig.m","config/perceptionConfig.m","config/featureMapBuildConfig.m", ...
        "localization/fitLidarTranslationCalibration.m","localization/localizeLidarFrame.m", ...
        "scripts/prepareLidarCalibrationFrames.py","scripts/calibrateLidarReferencePoint.m", ...
        "scripts/calibrateLidarOriginFromScans.m","scripts/reprojectSavedFeatureObservations.m", ...
        "scripts/rebuildInspvaSavedFeatureMap.m","scripts/replayMississippiLocalization.m", ...
        "scripts/runMncavCoarseLocalizationExperiment.m","scripts/runMississippiMapMatchingExperiment.m", ...
        "scripts/runMncavFullLocalizationExperiment.m","scripts/runMncavZeroDelayExperiment.m", ...
        "tests/lidarOriginCalibrationTest.m","tests/coarseSemanticProbabilityCloudTest.m",string(mfilename('fullpath'))+".m"];
    issues=cell(0,3);
    for file=files(endsWith(files,'.m'))
        messages=checkcode(file,'-id','-config=factory');
        for k=1:numel(messages),issues(end+1,:)={file,messages(k).line,messages(k).message};end %#ok<AGROW>
    end
    writetable(cell2table(issues,VariableNames={'file','line','message'}),fullfile(dest,'code_analysis.csv'));
    profile=lidarFrameCalibrationConfig("Mississippi");mapCfg=featureMapBuildConfig();active=load(mapCfg.probabilityCloudPath,'cloud');
    assert(isequal(active.cloud.frameCalibration,profile));
    replay=load(fullfile(selectedFolder,'matching/report.mat'),'report','cfg');
    assert(isequal(replay.cfg.perception.frameCalibration,profile));
    assert(height(replay.report.calls)==1170 && ~replay.report.metadata.finePerceptionUsed);
    frames=[28,94,260,425,600,855,1137];checks=false(numel(frames),3);
    for k=1:numel(frames)
        input=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',frames(k));
        original=perceptionConfig("Mississippi");original.executionMode="offline";original.frameCalibration=lidarFrameCalibrationConfig();
        calibrated=original;calibrated.frameCalibration=profile;
        a=perceiveFrame(input,original);b=perceiveFrame(input,calibrated);
        checks(k,1)=isequal(a.featureMasks,b.featureMasks);
        original.executionMode="coarseProbabilityCloud";calibrated.executionMode="coarseProbabilityCloud";
        a=perceiveCoarseProbabilityCloud(input,original);b=perceiveCoarseProbabilityCloud(input,calibrated);
        checks(k,2)=isequal(a.sourceSummary.selectedHitCount,b.sourceSummary.selectedHitCount);
        checks(k,3)=isequal(a.sourceSummary.selectedSourceCellCount,b.sourceSummary.selectedSourceCellCount);
    end
    assert(all(checks,'all'));
    writetable(array2table([frames.',checks],VariableNames={'frame','sameFineMasks','sameSelectedHits','sameSelectedPillars'}),fullfile(dest,'perception_checks.csv'));
    old=load('output/mississippi_mapping_synchronized/probability_cloud.mat','cloud');
    mismatchRejected=false;
    try
        localizeLidarFrame(input,old.cloud,[0 0 0],0);
    catch exception
        assert(string(exception.identifier)=="VehicleLocalization:CalibrationMismatch",exception.message);mismatchRejected=true;
    end
    assert(mismatchRejected);
    validation=struct('testsPassed',nnz([tests.Passed]),'testsFailed',nnz([tests.Failed]), ...
        'codeAnalyzerIssues',size(issues,1),'mapOnlineCalibrationIdentical',true, ...
        'coarseOnlyFrames',height(replay.report.calls),'fineMasksPreserved',all(checks(:,1)), ...
        'pillarSelectionsPreserved',all(checks(:,2:3),'all'),'staleIdentityMapRejected',mismatchRejected, ...
        'selectedExperiment',selectedFolder,'activeMap',mapCfg.probabilityCloudPath,'calibration',profile);
    fid=fopen(fullfile(dest,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));save(fullfile(out,'final_tests.mat'),'tests','validation');disp(validation);
end
