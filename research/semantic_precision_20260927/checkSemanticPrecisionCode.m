function checkSemanticPrecisionCode()
% checkSemanticPrecisionCode: Factory analysis for implementation and studies.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    files={'config/perceptionConfig.m','config/semanticPillarPrecisionConfig.m', ...
        'perception/perceiveFrame.m','perception/pillarizePointCloud.m','perception/filterSemanticPillarCandidates.m','perception/scoreSemanticPillarModel.m', ...
        'perception/continuousPillarHeightSupport.m','perception/measureFacadeGroupEvidence.m', ...
        'perception/offGroundFeatures/analyzeStructuralPillars.m', ...
        'perception/measureSemanticPillarFeatures.m','perception/measureSemanticPointDistributions.m', ...
        'tests/semanticPillarPrecisionTest.m','tests/coarseSemanticProbabilityCloudTest.m'};
    local=dir(fullfile(folder,'*.m'));
    for k=1:numel(local),files{end+1}=fullfile('research','semantic_precision_20260927',local(k).name);end %#ok<AGROW>
    findings=cell(size(files));
    for k=1:numel(files),findings{k}=checkcode(fullfile(root,files{k}),'-id','-config=factory');end
    file=fopen(fullfile(folder,'code_analysis.json'),'w');finish=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(struct('files',{files},'findings',{findings}),PrettyPrint=true));
    for k=1:numel(files),if ~isempty(findings{k}),disp(files{k});disp(struct2table(findings{k}));end,end
    assert(all(cellfun(@isempty,findings)),'Review Code Analyzer findings.');
end
