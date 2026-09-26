function runPoleSubsetChecks()
% runPoleSubsetChecks: Persist targeted perception regression and code checks.
    root=setupVehicleLocalization();setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(root,'data'));
    suites={'poleSubsetEvidenceTest','poleDistributionEvidenceTest','pillarVerticalShapeTest', ...
        'pillarRadialDistributionTest','pillarDensityCoreTest','wholePillarPerceptionTest', ...
        'coarseSemanticProbabilityCloudTest','coarsePerceptionPerformanceTest', ...
        'perceptionFeatureSelectionTest','pipelineRegressionTest'};
    tests=cellfun(@(s)fullfile(root,'tests',[s '.m']),suites,'UniformOutput',false);
    results=runtests(tests);destination=fullfile(root,'research','pole_subset_20260925');
    rows=table(string({results.Name}).',[results.Passed].',[results.Failed].',[results.Incomplete].',[results.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
    writetable(rows,fullfile(destination,'tests.csv'));
    save(fullfile(root,'output','pole_subset_20260925','test_results.mat'),'results');
    assertSuccess(results);
    sources={'config/structuralPillarConfig.m','perception/offGroundFeatures/findPillarPoleSubsets.m', ...
        'perception/offGroundFeatures/analyzeStructuralPillars.m','perception/offGroundFeatures/detectPolePillars.m', ...
        'perception/perceptionNativeAvailable.m','scripts/buildPerceptionKernels.m', ...
        'tests/poleSubsetEvidenceTest.m','tests/wholePillarPerceptionTest.m'};
    research=dir(fullfile(destination,'*.m'));sources=[sources,cellfun(@(s)fullfile('research','pole_subset_20260925',s),{research.name},'UniformOutput',false)];
    count=zeros(numel(sources),1);
    for k=1:numel(sources)
        issues=checkcode(fullfile(root,sources{k}),'-config=factory');count(k)=numel(issues);
        for j=1:numel(issues),fprintf('%s L%d: %s\n',sources{k},issues(j).line,issues(j).message);end
    end
    writetable(table(string(sources(:)),count,'VariableNames',{'file','findings'}),fullfile(destination,'code_analysis.csv'));
    fprintf('%d/%d passed; %d Code Analyzer findings.\n',nnz([results.Passed]),numel(results),sum(count));
end
