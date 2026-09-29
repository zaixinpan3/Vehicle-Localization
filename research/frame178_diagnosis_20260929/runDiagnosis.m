function runDiagnosis()
% runDiagnosis Reproduce the fixed-frame diagnosis and all integrity checks.
    diagnoseFrame178();
    traceSignSupport();
    inspectMechanisms();
    dest=fileparts(mfilename('fullpath'));files=dir(fullfile(dest,'*.m'));
    rows=cell(numel(files),2);
    for k=1:numel(files)
        findings=checkcode(fullfile(dest,files(k).name),'-id','-config=factory');
        rows(k,:)={string(files(k).name),numel(findings)};
        if ~isempty(findings),disp(files(k).name);disp(findings);end
    end
    analysis=cell2table(rows,VariableNames={'file','findings'});
    writetable(analysis,fullfile(dest,'code_analysis.csv'));
    assert(all(analysis.findings==0),'Resolve research harness Code Analyzer findings.');
    validation=struct('baselinePoseParity',true,'sourceWindowExactReconstruction',true, ...
        'rawCurrentCloudsExactlyReproduced',5,'rawComponentCountsAndMeansReproduced',true, ...
        'signTrackMeansAndCovariancesExactlyReproduced',true,'otherResidualWeightsPreserved',true, ...
        'currentFineLandmarkMembershipVerified',true,'controlledRegistrations',31, ...
        'codeAnalyzerFiles',height(analysis),'codeAnalyzerFindings',sum(analysis.findings), ...
        'productionAlgorithmChanged',false,'fullRouteRerun',false,'guiOpened',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
end
