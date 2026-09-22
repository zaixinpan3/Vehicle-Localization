function validation=validate_diagnosis()
% validate_diagnosis Check reproduction, control status, and unchanged runtime.
    dest=fileparts(mfilename('fullpath'));out=fullfile(pwd,'output/frame959_matching_diagnosis_20260922');
    s=load(fullfile(out,'diagnostic.mat'));t=load(fullfile(out,'iteration_trace.mat'));p=load(fullfile(out,'pole_audit.mat'));
    assert(s.summary.reproductionMaxAbs<1e-7);
    assert(max(abs(t.result.poseXYTheta-s.results{1}.poseXYTheta))<1e-10);
    assert(s.results{1}.accepted && abs(s.call.positionErrorM-.6928220515366847)<1e-9);
    assert(s.results{8}.accepted && s.controls.errorM(8)<.17);
    assert(p.results{3}.accepted && p.controls.errorM(3)<.17);
    assert(s.controls.errorM(17)>.65 && s.controls.errorM(13)>.65);
    assert(~s.results{5}.accepted && ~s.results{6}.accepted && s.results{5}.observableRank==2 && s.results{6}.observableRank==2);
    active=lidarFrameCalibrationConfig("Mississippi");assert(isequal(active,s.cfg.perception.frameCalibration));
    assert(s.summary.sourceWindow.frameCount==3 && s.source.components.numComponents==143);
    files=dir(fullfile(dest,'*.m'));issues=cell(0,3);
    for f=files.'
        messages=checkcode(fullfile(f.folder,f.name),'-id','-config=factory');
        for k=1:numel(messages),issues(end+1,:)={f.name,messages(k).line,messages(k).message};end %#ok<AGROW>
    end
    assert(isempty(issues));writetable(cell2table(issues,VariableNames={'file','line','message'}),fullfile(dest,'code_analysis.csv'));
    validation=struct('savedMatchReproduced',true,'instrumentationPreservesResult',true,'fullPoseAndDirectionalResultsDistinguished',true, ...
        'poleAblationRemovesMostOfPeak',true,'referenceMotionAndUniformPriorsDoNotRemovePeak',true, ...
        'activeCalibrationUnchanged',true,'diagnosticVariants',22,'matlabFilesChecked',numel(files),'codeAnalyzerIssues',size(issues,1), ...
        'productionAlgorithmModified',false,'completeSequenceRerun',false,'independentGroundTruthClaimed',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));disp(validation);
end
