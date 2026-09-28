function checkCurbProposalCode()
% checkCurbProposalCode: Factory analysis of the changed perception and audit files.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    files={'config/semanticPillarPrecisionConfig.m','perception/filterSemanticPillarCandidates.m', ...
        'perception/scoreSemanticPillarModel.m','perception/recoverCurbRawProposals.m', ...
        'perception/groundFeatures/analyzeGroundPillars.m','tests/mississippiCurbRecoveryTest.m'};
    files=[files,{'research/pillar_fine_alignment_20260926/measureFinePoleAlignment.m', ...
        'research/pole_reference_audit_20260928/auditFrame900.m'}];
    studies=dir(fullfile(folder,'*.m'));
    for k=1:numel(studies),files{end+1}=fullfile('research','curb_proposal_recovery_20260928',studies(k).name);end %#ok<AGROW>
    findings=cell(size(files));
    for k=1:numel(files),findings{k}=checkcode(fullfile(root,files{k}),'-id','-config=factory');end
    file=fopen(fullfile(folder,'code_analysis.json'),'w');finish=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(struct('files',{files},'findings',{findings}),PrettyPrint=true));
    for k=1:numel(files),if ~isempty(findings{k}),disp(files{k});disp(struct2table(findings{k}));end,end
    assert(all(cellfun(@isempty,findings)),'Factory Code Analyzer findings require review.');
end
