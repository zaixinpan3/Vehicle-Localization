function report = evaluatePillarPerception(dataRoot, outputFolder)
% evaluatePillarPerception: Reproducible historical-fidelity and timing study.
% Diagnostic expansion frames exposed a split-pillar pole defect and informed
% its correction. Final checks use four frames not used to adjust this redesign;
% frames 500 and 800 had appeared in the earlier baseline study.
% Legacy outputs are a behavioral reference, not manually labeled truth.
% Timings are warmed medians of three runs in the active MATLAB session;
% file loading and plotting are excluded. No dataset or reference is modified.
    root=fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root,'setupVehicleLocalization.m'));
    if nargin<1 || strlength(string(dataRoot))==0
        dataRoot=fullfile(root,'data');
    end
    if nargin<2 || strlength(string(outputFolder))==0
        outputFolder=fullfile(root,'output','perception_redesign');
    end
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    frames=[260 300 326 370 450 550 700 850 1000 1150 150 600 900 1100 400 200 500 800 1050];
    names=["curb" "roadMarking" "pole"];
    featureRows=cell(numel(frames)*3,11);
    timingRows=cell(numel(frames),9);
    cfg=perceptionConfig(); offlineCfg=cfg; offlineCfg.executionMode="offline";
    legacyCfg=cfg; legacyCfg.executionMode="legacyFull";
    for k=1:numel(frames)
        dataset="mississippi"; file='MissisipiPointClouds.mat'; split="development";
        if k>=11 && k<=14, split="diagnosticExpansion"; end
        if k>=16, split="finalValidation"; end
        if k==15, dataset="downtown"; file='downTownPointClouds.mat'; end
        frame=loadPointCloudFrame(fullfile(dataRoot,'raw',file),frames(k));
        cfg=perceptionConfig(dataset);
        offlineCfg=cfg; offlineCfg.executionMode="offline";
        legacyCfg.offGroundFeatures.facadeDetectionEnabled=dataset=="downtown";
        legacy=perceiveFrame(frame,legacyCfg);
        online=perceiveFrame(frame,cfg);
        offline=perceiveFrame(frame,offlineCfg);
        for j=1:3
            reference=legacy.featureMasks.(names(j)); predicted=offline.featureMasks.(names(j));
            tp=nnz(reference & predicted); fp=nnz(~reference & predicted); fn=nnz(reference & ~predicted);
            precision=tp/max(tp+fp,1); recall=tp/max(tp+fn,1);
            if tp+fp==0, precision=NaN; end
            if tp+fn==0, recall=NaN; end
            featureRows{(k-1)*3+j,1}=dataset;
            featureRows((k-1)*3+j,2:11)={frames(k),split,names(j),nnz(reference),nnz(predicted),tp,fp,fn,precision,recall};
        end
        elapsed=zeros(3,3);
        for repeat=1:3
            timer=tic; perceiveFrame(frame,legacyCfg); elapsed(repeat,1)=toc(timer);
            timer=tic; perceiveFrame(frame,cfg); elapsed(repeat,2)=toc(timer);
            timer=tic; perceiveFrame(frame,offlineCfg); elapsed(repeat,3)=toc(timer);
        end
        time=median(elapsed,1);
        legacySize=whos('legacy'); onlineSize=whos('online'); offlineSize=whos('offline');
        timingRows(k,:)={dataset,frames(k),time(1),time(2),time(3),legacySize.bytes,onlineSize.bytes,offlineSize.bytes,online.probabilityCloud.components.numComponents};
        fprintf('%s %d: legacy %.3f, coarse %.3f, offline %.3f s; components %d\n',dataset,frames(k),time,online.probabilityCloud.components.numComponents);
    end
    report.features=cell2table(featureRows,'VariableNames',{'dataset','frameIndex','split','semanticName','legacyCount','offlineCount','tp','fp','fn','precision','recall'});
    report.timings=cell2table(timingRows,'VariableNames',{'dataset','frameIndex','legacySeconds','coarseSeconds','offlineSeconds','legacyBytes','coarseBytes','offlineBytes','numComponents'});
    report.referenceCommit="1ac86ba5c3c92021c83934f750ebe93546b36d8d";
    report.matlabVersion=string(version);
    report.description="Historical behavior fidelity; no manual ground truth or real-time guarantee.";
    writetable(report.features,fullfile(outputFolder,'feature_fidelity.csv'));
    writetable(report.timings,fullfile(outputFolder,'runtime_and_storage.csv'));
    save(fullfile(outputFolder,'evaluation.mat'),'report');
end
