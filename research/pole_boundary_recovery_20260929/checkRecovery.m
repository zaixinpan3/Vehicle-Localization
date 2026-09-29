function checkRecovery()
% checkRecovery Check implementation and both point-processing backends.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    files={'perception/offGroundFeatures/recoverSplitPoleShaft.m', ...
        'perception/offGroundFeatures/classifyPillarPoleSupport.m', ...
        'config/pillarPoleDistributionConfig.m','tests/pillarPoleDistributionTest.m', ...
        fullfile(dest,'verifyRecovery.m'),fullfile(dest,'replayRecoveryRoute.m'), ...
        fullfile(dest,'validateRecovery.m'),fullfile(dest,'finishRecovery.m'),fullfile(dest,'checkRecovery.m')};
    findings=cell(size(files));
    for k=1:numel(files),findings{k}=checkcode(files{k},'-config=factory');assert(isempty(findings{k}));end
    cfg=perceptionConfig('Mississippi');cfg.executionBackend="matlab";rows=cell(0,3);
    for frameId=[685 820 856]
        frame=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',frameId);
        a=perceiveFrame(frame,cfg);native=cfg;native.executionBackend="native";b=perceiveFrame(frame,native);
        for k=1:numel(a.candidates.semanticNames)
            assert(isequal(a.candidates.pillarIndices{k},b.candidates.pillarIndices{k}));
        end
        rows(end+1,:)={frameId,true,numel(a.candidates.pillarIndices{a.candidates.semanticNames=="pole"})}; %#ok<AGROW>
    end
    writetable(cell2table(rows,VariableNames={'frame','backendAgreement','polePillars'}),fullfile(dest,'backend_checks.csv'));
    summary=struct('files',{files},'findings',{findings});
    fid=fopen(fullfile(dest,'code_analysis.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
end
