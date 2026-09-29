function runPrototypeTests(label)
% runPrototypeTests Run regression suites with an isolated profile.
    root=setupVehicleLocalization();addpath(root);dest=fileparts(mfilename('fullpath'));
    sandbox=fullfile(root,'output/iterative_matching_20260929',label+"_test_config");if ~isfolder(sandbox),mkdir(sandbox);end
    model=fullfile(root,'output/iterative_matching_20260929',label+".mat");
    fid=fopen(fullfile(sandbox,'distributionRegistrationConfig.m'),'w');
    fprintf(fid,'function cfg=distributionRegistrationConfig()\n a=load(''%s'',''cfg'');cfg=a.cfg;\nend\n',model);fclose(fid);
    files=["canonicalPyramidTest","geometricRegistrationTest","distributionRegistrationTest", ...
        "repeatabilityRegistrationTest","lineDirectionRegistrationTest","positionAidedRegistrationTest", ...
        "registrationInformationTest","relativeHeightAssociationTest","localizationSourceWindowTest","softPointAssociationTest"];
    suite=testsuite(fullfile(root,'tests',files(1)+".m"));
    for f=files(2:end),suite=[suite,testsuite(fullfile(root,'tests',f+".m"))];end %#ok<AGROW>
    previous=pwd;cleanup=onCleanup(@()cd(previous));cd(sandbox);clear distributionRegistrationConfig
    active=distributionRegistrationConfig();expected=load(model,'cfg');assert(isequaln(active,expected.cfg));
    fprintf('ACTIVE PROFILE %s soft=%d from %s\n',label,isfield(active,'softPointAssociation'),which('distributionRegistrationConfig'));
    result=run(suite);save(fullfile(root,'output/iterative_matching_20260929',label+"_tests.mat"),'result');
    report=table(string({result.Name}).',[result.Passed].',[result.Failed].',[result.Incomplete].', ...
        VariableNames={'test','passed','failed','incomplete'});writetable(report,fullfile(dest,label+"_tests.csv"));
    disp(report(report.failed|report.incomplete,:));fprintf('PROFILE %s: %d passed, %d failed\n',label,nnz(report.passed),nnz(report.failed));
end
