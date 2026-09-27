function validatePrecisionStudy(prefix)
% validatePrecisionStudy: Physical/metric tests and all-row inference parity.
    if nargin<1,prefix='context';end
    root=setupVehicleLocalization();addpath(root);folder=fileparts(mfilename('fullpath'));addpath(folder);
    addpath(fullfile(root,'research','pole_context_20260927'));
    out=fullfile(root,'output','pole_precision_20260927');
    r=runtests({fullfile(folder,'precisionStudyTest.m'),fullfile(root,'tests','pillarFineAlignmentTest.m')});
    writetable(table(r),fullfile(folder,'feature_tests.csv'));assertSuccess(r);
    model=jsondecode(fileread(fullfile(folder,[prefix '_candidate_model.json'])));
    x=readmatrix(fullfile(out,[prefix '_model_replay_inputs.csv']));expected=readmatrix(fullfile(out,[prefix '_model_replay_expected.csv']));
    actual=scorePrecisionModel(x,model);error=max(abs(actual-expected));
    assert(error<1e-12,'MATLAB and training implementation disagree.');
    assert(isequal(actual>=model.threshold,expected>=model.threshold),'Selection threshold mismatch.');
    predictions=readtable(fullfile(folder,[prefix '_model_predictions.csv']),'TextType','string');
    rawChecks={};
    for dataset=["Mississippi","Downtown"]
        cfg=perceptionConfig(dataset);
        if dataset=="Mississippi",file='MissisipiPointClouds.mat';frames=[44 225 285 495 720 745 776 781 850 970 1100 1170];
        else,file='downTownPointClouds.mat';frames=[26 86 118 165 251 282 328 356 361 411 481 536];end
        source=matfile(fullfile(root,'data','raw',file));
        for frameId=frames
            frame=source.pointClouds(1,frameId);selected=detectPrecisionPillars(frame,cfg,model);
            eligible=predictions.dataset==dataset & predictions.frame==frameId;
            assert(any(eligible),'Replay frame must occur in frozen candidates.');
            expectedIds=sort(predictions.pillar(eligible & predictions.score>=model.threshold));
            assert(isequal(selected,expectedIds),'Raw-frame replay changed selected pillars.');
            rawChecks{end+1,1}=struct('dataset',dataset,'frame',frameId,'selected',numel(selected),'sameSelections',true); %#ok<AGROW>
        end
    end
    writetable(struct2table(vertcat(rawChecks{:})),fullfile(folder,[prefix '_raw_replay_checks.csv']));
    f=fopen(fullfile(folder,[prefix '_inference_validation.json']),'w');cleanup=onCleanup(@()fclose(f));
    fprintf(f,'%s\n',jsonencode(struct('rows',size(x,1),'features',size(x,2), ...
        'maximumAbsoluteProbabilityError',error,'sameSelections',true,'rawFrames',numel(rawChecks), ...
        'passedTests',nnz([r.Passed])),'PrettyPrint',true));
end
