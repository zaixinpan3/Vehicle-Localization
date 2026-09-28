function validateCurbProposalRecovery()
% validateCurbProposalRecovery: Focused Mississippi contracts and factory analysis.
    root=setupVehicleLocalization();addpath(root);folder=fileparts(mfilename('fullpath'));
    r=runtests(fullfile(root,'tests','mississippiCurbRecoveryTest.m'));
    T=table(string({r.Name}).',[r.Passed].',[r.Failed].',[r.Incomplete].',[r.Duration].', ...
        'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
    writetable(T,fullfile(folder,'tests.csv'));assertSuccess(r);
    checkCurbProposalCode();
end
