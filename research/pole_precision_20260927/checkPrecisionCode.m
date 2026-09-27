function T=checkPrecisionCode()
% checkPrecisionCode: Record factory Code Analyzer findings for this study.
    folder=fileparts(mfilename('fullpath'));files=dir(fullfile(folder,'*.m'));rows={};
    for k=1:numel(files)
        issues=checkcode(fullfile(folder,files(k).name),'-id','-config=factory');
        for j=1:numel(issues)
            rows{end+1,1}=struct('file',string(files(k).name),'line',issues(j).line, ...
                'id',string(issues(j).id),'message',string(issues(j).message)); %#ok<AGROW>
        end
    end
    T=table();if ~isempty(rows),T=struct2table(vertcat(rows{:}));end
    writetable(T,fullfile(folder,'code_analysis.csv'));disp(T);
end
