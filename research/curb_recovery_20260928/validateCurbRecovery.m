function validateCurbRecovery()
% validateCurbRecovery: Focused Mississippi contracts and factory analysis.
    root=setupVehicleLocalization();addpath(root);folder=fileparts(mfilename('fullpath'));
    r=runtests(fullfile(root,'tests','mississippiCurbRecoveryTest.m'));
    T=table(string({r.Name}).',[r.Passed].',[r.Failed].',[r.Incomplete].',[r.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
    writetable(T,fullfile(folder,'tests.csv'));assertSuccess(r);
    files={'config/semanticPillarPrecisionConfig.m','perception/filterSemanticPillarCandidates.m', ...
        'perception/scoreSemanticPillarModel.m','tests/mississippiCurbRecoveryTest.m'};
    if isfile(fullfile(root,'perception','measureCurbComponentSupport.m')),files{end+1}='perception/measureCurbComponentSupport.m';end
    studies=dir(fullfile(folder,'*.m'));
    for k=1:numel(studies),files{end+1}=fullfile('research','curb_recovery_20260928',studies(k).name);end %#ok<AGROW>
    findings=cell(size(files));
    for k=1:numel(files),findings{k}=checkcode(fullfile(root,files{k}),'-id','-config=factory');end
    file=fopen(fullfile(folder,'code_analysis.json'),'w');finish=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(struct('files',{files},'findings',{findings}),PrettyPrint=true));
    for k=1:numel(files),if ~isempty(findings{k}),disp(files{k});disp(struct2table(findings{k}));end,end
    assert(all(cellfun(@isempty,findings)),'Factory Code Analyzer findings require review.');
end
