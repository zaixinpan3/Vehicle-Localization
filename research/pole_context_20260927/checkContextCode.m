function T=checkContextCode()
% checkContextCode: Inspect scoped implementation and study MATLAB sources.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    files={'config/pillarPoleValidationConfig.m', ...
        'perception/offGroundFeatures/measurePillarPoleSupport.m', ...
        'perception/offGroundFeatures/validatePillarPoleSupport.m', ...
        'tests/pillarPoleValidationTest.m'};
    scripts=dir(fullfile(folder,'*.m'));
    files=[files,cellfun(@(x)fullfile('research','pole_context_20260927',x),{scripts.name},'UniformOutput',false)];
    rows={};
    for k=1:numel(files)
        issues=checkcode(fullfile(root,files{k}),'-id','-config=factory');
        for j=1:numel(issues)
            rows{end+1,1}=struct('file',string(files{k}),'line',issues(j).line, ...
                'id',string(issues(j).id),'message',string(issues(j).message)); %#ok<AGROW>
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'code_analysis.csv'));disp(T);
end
