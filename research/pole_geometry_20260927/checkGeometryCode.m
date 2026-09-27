function checkGeometryCode()
% checkGeometryCode: Record Code Analyzer results for changed MATLAB sources.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));rows={};
    files=[dir(fullfile(folder,'*.m'));dir(fullfile(root,'tests','pillarPoleDistributionTest.m')); ...
        dir(fullfile(root,'config','pillarPoleDistributionConfig.m')); ...
        dir(fullfile(root,'config','structuralPillarConfig.m')); ...
        dir(fullfile(root,'perception','perceiveFrame.m')); ...
        dir(fullfile(root,'perception','offGroundFeatures','analyzeStructuralPillars.m')); ...
        dir(fullfile(root,'research','pole_precision_20260927','capturePrecisionTraining.m'))];
    names={'classifyPillarPoleSupport','loadPillarPoleModel','measurePillarPoleFeatures','measurePillarPoleMoments','scorePillarPoleModel'};
    for name=names,files=[files;dir(fullfile(root,'perception','offGroundFeatures',[name{1} '.m']))];end %#ok<AGROW>
    for name={'measurePoleAxisContext','measurePoleContinuousContext'}
        files=[files;dir(fullfile(root,'perception','offGroundFeatures','private',[name{1} '.m']))]; %#ok<AGROW>
    end
    for k=1:numel(files)
        file=fullfile(files(k).folder,files(k).name);issues=checkcode(file,'-id','-config=factory');
        for j=1:numel(issues)
            rows{end+1,1}=struct('file',string(erase(file,[root filesep])),'line',issues(j).line, ...
                'id',string(issues(j).id),'message',string(issues(j).message)); %#ok<AGROW>
        end
    end
    if isempty(rows),T=table(strings(0,1),zeros(0,1),strings(0,1),strings(0,1),'VariableNames',{'file','line','id','message'});
    else,T=struct2table(vertcat(rows{:}));end
    writetable(T,fullfile(folder,'code_analysis.csv'));disp(T);
end
