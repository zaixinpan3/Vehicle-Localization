function checkFinalImplementation()
% checkFinalImplementation Validate the deployed solver and report actual checks.
    root=setupVehicleLocalization();addpath(root);dest=fileparts(mfilename('fullpath'));
    files=["canonicalPyramidTest","geometricRegistrationTest","distributionRegistrationTest", ...
        "repeatabilityRegistrationTest","lineDirectionRegistrationTest","positionAidedRegistrationTest", ...
        "registrationInformationTest","relativeHeightAssociationTest","localizationSourceWindowTest","softPointAssociationTest"];
    suite=testsuite(fullfile(root,'tests',files(1)+".m"));
    for f=files(2:end),suite=[suite,testsuite(fullfile(root,'tests',f+".m"))];end %#ok<AGROW>
    result=run(suite);report=table(string({result.Name}).',[result.Passed].',[result.Failed].',[result.Incomplete].', ...
        VariableNames={'test','passed','failed','incomplete'});writetable(report,fullfile(dest,'final_tests.csv'));
    assert(all(report.passed));
    checked=["config/distributionRegistrationConfig.m","localization/registerSemanticProbabilityCloud.m", ...
        "localization/prepareSemanticRegistrationGeometry.m","localization/sourceLineDirections.m", ...
        "localization/softPointAssociationTarget.m","localization/validateSoftPointAssociation.m", ...
        "tests/softPointAssociationTest.m","tests/relativeHeightAssociationTest.m","tests/canonicalPyramidTest.m"];
    counts=zeros(numel(checked),1);
    for k=1:numel(checked)
        findings=checkcode(fullfile(root,checked(k)),'-config=factory');counts(k)=numel(findings);
        if ~isempty(findings),disp(checked(k));disp(struct2table(findings));end
    end
    writetable(table(checked.',counts,VariableNames={'file','findings'}),fullfile(dest,'code_analysis.csv'));
    assert(~any(counts));fprintf('FINAL IMPLEMENTATION: %d tests passed; %d files clean.\n',height(report),numel(checked));
end
