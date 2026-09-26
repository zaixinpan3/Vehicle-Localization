function report=validateShaftSpeed()
% validateShaftSpeed: Verify exact recorded evidence, null controls, and tests.
% The serial evidence cache was compared with the frozen pre-optimization
% implementation separately; this check closes the link to the final backend.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    output=fullfile(root,'output','pillar_shaft_speed_20260926');
    if ~isfolder(output),mkdir(output);end
    diary(fullfile(output,'validation.log'));diary on
    diaryCleanup=onCleanup(@()diary('off'));
    previousRng=rng;rngCleanup=onCleanup(@()rng(previousRng));
    previousDataRoot=getenv('VEHICLE_LOCALIZATION_DATA_ROOT');
    environmentCleanup=onCleanup(@()setenv('VEHICLE_LOCALIZATION_DATA_ROOT',previousDataRoot));
    fprintf('Validation started %s\n',char(datetime('now','TimeZone','America/Chicago', ...
        'Format','yyyy-MM-dd HH:mm:ss XXX')));

    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'),'cases');
    serial=load(fullfile(output,'single_thread_evidence.mat'),'speedAssignments');
    proof=load(fullfile(output,'baseline_parity.mat'),'speedBaselineParity');
    frozen=proof.speedBaselineParity;
    assert(frozen.frames==117 && frozen.allModeFieldsExact && ...
        frozen.allAssignedFieldsExact && frozen.groupsExact, ...
        'The serial cache must first match the frozen baseline evidence.');
    assert(numel(data.cases)==117 && numel(serial.speedAssignments)==117);
    cfg=pillarShaftConfig();cfg.useNativeKernels=true;
    parityRows=cell(117,1);
    for k=1:117
        frame=data.cases{k};ids=unique(frame.pillarIds);c=cfg;
        c.columnMinimumScores=min(c.strongScore, ...
            c.minimumShapeProduct./max(double(frame.before.columnMaps.pointScore(ids)),eps));
        modes=findPillarShaftModes(frame.points,frame.pillarIds,frame.geometry,c);
        [assigned,groups]=assignPillarShaftSupport( ...
            frame.points,frame.pillarIds,frame.geometry,modes,c);
        expected=serial.speedAssignments{k};
        parityRows{k}=struct('frame',frame.frame,'occupiedPillars',numel(ids), ...
            'modesExact',isequaln(modes,expected.modes), ...
            'assignedExact',isequaln(assigned,expected.assigned), ...
            'groupsExact',isequaln(groups,expected.groups));
        if mod(k,20)==0,fprintf('Recorded evidence %d/117\n',k);end
    end
    parityTable=struct2table(vertcat(parityRows{:}));
    writetable(parityTable,fullfile(folder,'evidence_parity.csv'));

    expectedNull=readtable(fullfile(root,'research','pillar_shaft_20260926','uniform_controls.csv'));
    assert(isequal(expectedNull.seed,(1:32).'));
    geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
    nullRows=cell(32,1);
    for seed=1:32
        rng(seed,'twister');points=[.6*rand(800,2),-1+4*rand(800,1)];timer=tic;
        evidence=findPillarShaftModes(points,ones(800,1),geometry,cfg);
        nullRows{seed}=struct('seed',seed,'found',evidence.found, ...
            'score',evidence.score,'milliseconds',1000*toc(timer));
    end
    nullTable=struct2table(vertcat(nullRows{:}));
    writetable(nullTable,fullfile(folder,'uniform_controls.csv'));
    nullMasksExact=isequal(nullTable.found,logical(expectedNull.found));
    nullScoreError=max(abs(nullTable.score-expectedNull.score));

    setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(root,'data'));
    suites={'pillarShaftModesTest','pillarShaftExecutionTest','poleSubsetEvidenceTest', ...
        'poleDistributionEvidenceTest','pillarVerticalShapeTest','pillarRadialDistributionTest', ...
        'pillarDensityCoreTest','wholePillarPerceptionTest','coarseSemanticProbabilityCloudTest', ...
        'coarsePerceptionPerformanceTest','perceptionFeatureSelectionTest','pipelineRegressionTest'};
    paths=cellfun(@(name)fullfile(root,'tests',[name '.m']),suites,'UniformOutput',false);
    results=runtests(paths);
    testRows=table(string({results.Name}).',[results.Passed].',[results.Failed].', ...
        [results.Incomplete].',[results.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
    writetable(testRows,fullfile(folder,'tests.csv'));
    report=struct('nativeVersion',perceptionKernelsMex('version'), ...
        'configuredThreads',cfg.nativeThreads,'parityFrames',height(parityTable), ...
        'occupiedPillarsCompared',sum(parityTable.occupiedPillars), ...
        'modesExact',all(parityTable.modesExact), ...
        'assignedExact',all(parityTable.assignedExact), ...
        'groupsExact',all(parityTable.groupsExact), ...
        'serialCachePreviouslyMatchedFrozenBaseline',true, ...
        'nullScenes',height(nullTable),'nullRng','twister','nullAccepted',nnz(nullTable.found), ...
        'nullAcceptedSeeds',nullTable.seed(nullTable.found).', ...
        'nullMasksExact',nullMasksExact,'nullScoreMaximumDifference',nullScoreError, ...
        'testsPassed',nnz([results.Passed]),'testsFailed',nnz([results.Failed]), ...
        'testsIncomplete',nnz([results.Incomplete]),'expectedTestCount',161);
    save(fullfile(output,'validation.mat'),'results','report','parityTable','nullTable');
    fid=fopen(fullfile(folder,'validation.json'),'w');assert(fid>=0);
    fileCleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fileCleanup
    disp(report);
    assert(report.modesExact && report.assignedExact && report.groupsExact, ...
        'Optimized recorded evidence differs from the proven serial cache.');
    assert(nullMasksExact && nullScoreError<1e-12, ...
        'Null evidence differs from the prior CSV (allowing decimal export rounding).');
    assert(numel(results)==report.expectedTestCount,'Unexpected validation test count.');
    assertSuccess(results);
    clear diaryCleanup rngCleanup environmentCleanup
end
