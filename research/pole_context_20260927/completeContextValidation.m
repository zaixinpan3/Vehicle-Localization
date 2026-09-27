function results=completeContextValidation()
% completeContextValidation: Replace the corrected fixture suite after a full run.
% The first range fixture accidentally crossed an extra pillar boundary. After
% correcting its translation, rerun that suite and retain the other full-run
% results. validateContextAlignment remains the clean full reproduction path.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_context_20260927');addpath(root);
    previous=load(fullfile(out,'tests.mat'),'results');results=previous.results;
    copyfile(fullfile(folder,'tests.csv'),fullfile(folder,'fixture_initial_tests.csv'));
    copyfile(fullfile(out,'tests.mat'),fullfile(out,'fixture_initial_tests.mat'));
    revised=runtests(fullfile(root,'tests','pillarPoleValidationTest.m'));
    assertSuccess(revised);
    [found,where]=ismember({revised.Name},{results.Name});assert(all(found));
    results(where)=revised;
    T=table(string({results.Name}).',[results.Passed].',[results.Failed].', ...
        [results.Incomplete].',[results.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
    writetable(T,fullfile(folder,'tests.csv'));save(fullfile(out,'tests.mat'),'results');
    assertSuccess(results);
end
