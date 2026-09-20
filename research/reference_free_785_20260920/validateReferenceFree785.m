function validation=validateReferenceFree785(repoRoot)
% validateReferenceFree785 Check the isolated historical runtime and harness.
    previousFolder=pwd;previousPath=path;
    cleanup=onCleanup(@()restoreEnvironment(previousFolder,previousPath));
    output=fullfile(repoRoot,'output/reference_free_785_20260920');
    snapshot=fullfile(output,'historical_runtime');cd(snapshot);setupVehicleLocalization();
    assert(startsWith(which('registerSemanticProbabilityCloud'),snapshot));
    files={'gnssOutputPointTest.m','geometricRegistrationTest.m', ...
        'registrationInformationTest.m','repeatabilityRegistrationTest.m'};
    results=runtests(fullfile(snapshot,'tests',files));
    validation=struct('passed',nnz([results.Passed]),'failed',nnz([results.Failed]), ...
        'incomplete',nnz([results.Incomplete]),'testNames',string({results.Name}));
    assert(validation.failed==0 && validation.incomplete==0);
    folder=fullfile(repoRoot,'research/reference_free_785_20260920');
    sources=dir(fullfile(folder,'*.m'));findings=cell(numel(sources),1);
    for k=1:numel(sources)
        messages=checkcode(fullfile(folder,sources(k).name),'-config=factory');
        findings{k}=struct('file',sources(k).name,'messages',messages);
        assert(isempty(messages),'Code Analyzer findings in %s.',sources(k).name);
    end
    validation.codeAnalyzer=findings;
    fid=fopen(fullfile(output,'validation.json'),'w');fileCleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));
end

function restoreEnvironment(folder,searchPath)
    cd(folder);path(searchPath);
end
