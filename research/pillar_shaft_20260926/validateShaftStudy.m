function validateShaftStudy()
% validateShaftStudy: Backend parity, stochastic null controls and regressions.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    reference=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    indices=[1 3 6 12 20 31 51 65 86 104 109 117];maximumError=0;checked=0;
    for k=indices
        r=data.cases{k};ids=unique(r.pillarIds);select=ismember(ids,union(reference.records{k}.fineCells,reference.records{k}.baselineCells));
        c=pillarShaftConfig();c.columnMinimumScores=min(c.strongScore,c.minimumShapeProduct./max(double(r.before.columnMaps.pointScore(ids)),eps));
        a=findPillarShaftModes(r.points,r.pillarIds,r.geometry,c,select);c.useNativeKernels=true;
        b=findPillarShaftModes(r.points,r.pillarIds,r.geometry,c,select);
        assert(isequal(a.found,b.found));
        for field=fieldnames(a).'
            x=double(a.(field{1}));y=double(b.(field{1}));assert(isequal(isnan(x),isnan(y)));
            delta=max(abs(x-y),[],'all','omitnan');if isempty(delta) || isnan(delta),delta=0;end
            assert(delta<1e-8,'Native evidence differs from MATLAB.');maximumError=max(maximumError,delta);
        end
        checked=checked+nnz(select);
    end
    cfg=pillarShaftConfig();cfg.useNativeKernels=true;g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);rows=cell(32,1);
    for seed=1:32
        rng(seed);p=[.6*rand(800,2),-1+4*rand(800,1)];timer=tic;e=findPillarShaftModes(p,ones(800,1),g,cfg);
        rows{seed}=struct('seed',seed,'found',e.found,'score',e.score,'milliseconds',1000*toc(timer));
    end
    writetable(struct2table(vertcat(rows{:})),fullfile(folder,'uniform_controls.csv'));
    setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(root,'data'));
    suites={'pillarShaftModesTest','poleSubsetEvidenceTest','poleDistributionEvidenceTest', ...
        'pillarVerticalShapeTest','pillarRadialDistributionTest','pillarDensityCoreTest', ...
        'wholePillarPerceptionTest','coarseSemanticProbabilityCloudTest','coarsePerceptionPerformanceTest', ...
        'perceptionFeatureSelectionTest','pipelineRegressionTest'};
    paths=cellfun(@(s)fullfile(root,'tests',[s '.m']),suites,'UniformOutput',false);
    results=runtests(paths);testRows=table(string({results.Name}).',[results.Passed].',[results.Failed].',[results.Incomplete].',[results.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});writetable(testRows,fullfile(folder,'tests.csv'));
    save(fullfile(root,'output','pillar_shaft_20260926','validation.mat'),'results','indices','maximumError','checked');
    report=struct('parityFrames',numel(indices),'parityReferenceCells',checked,'maximumFieldError',maximumError, ...
        'nullScenes',32,'nullAccepted',nnz(cellfun(@(r)r.found,rows)), ...
        'testsPassed',nnz([results.Passed]),'testsFailed',nnz([results.Failed]),'testsIncomplete',nnz([results.Incomplete]));
    fid=fopen(fullfile(folder,'validation.json'),'w');clean=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(report);assertSuccess(results);
end
