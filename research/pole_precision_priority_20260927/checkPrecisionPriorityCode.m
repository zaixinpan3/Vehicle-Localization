function checkPrecisionPriorityCode()
% checkPrecisionPriorityCode: Record factory Code Analyzer source findings.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));rows={};
    files=[dir(fullfile(folder,'*.m'));dir(fullfile(root,'config','pillarPoleDistributionConfig.m'))];
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
    assert(isempty(rows),'Review Code Analyzer findings before closure.');
end
