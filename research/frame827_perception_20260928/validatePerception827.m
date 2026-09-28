function validatePerception827()
% validatePerception827 Check operating-point isolation and the inspected frame.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));addpath('research/pillar_fine_alignment_20260926');
    frame=loadPointCloudFrame(fullfile(root,'data/raw/MissisipiPointClouds.mat'),827);cfg=perceptionConfig('Mississippi');old=cfg;
    old.offGroundFeatures.pole.distributionValidation.minimumScore=.9371209151674477;
    before=perceiveFrame(frame,old);after=perceiveFrame(frame,cfg);
    assert(isequaln(before.sourceSummary,after.sourceSummary));assert(isequaln(before.candidates.geometry,after.candidates.geometry));
    for name=["curb","trafficSign"]
        k=after.candidates.semanticNames==name;assert(isequal(before.candidates.pillarIndices{k},after.candidates.pillarIndices{k}));
    end
    frozen=load('output/semantic_precision_20260927/mississippi_baseline.mat','frames','references','names');at=frozen.frames==827;
    rows={};
    for mode=1:2
        p=before;if mode==2,p=after;end
        for j=1:numel(frozen.names)
            name=frozen.names(j);ids=p.candidates.pillarIndices{p.candidates.semanticNames==name};
            metrics=measureFinePoleAlignment(frame,frozen.references{at,j},ids,p.candidates.geometry);metrics.mode=mode;metrics.feature=name;
            rows{end+1,1}=metrics; %#ok<AGROW>
        end
    end
    comparison=struct2table(vertcat(rows{:}));writetable(comparison,fullfile(dest,'frame827_comparison.csv'));
    fine=perceiveFrame(frame,perceptionConfig('Mississippi','offline'));
    for k=1:numel(frozen.names),assert(isequal(double(find(fine.featureMasks.(frozen.names(k)))),double(frozen.references{at,k}(:))));end
    files=["config/perceptionConfig.m","config/pillarPoleDistributionConfig.m","tests/pillarPoleDistributionTest.m", ...
        "research/frame827_perception_20260928/diagnosePerception827.m", ...
        "research/frame827_perception_20260928/measureCurbJointMoments.m", ...
        "research/frame827_perception_20260928/captureCurbJointMoments.m", ...
        "research/frame827_perception_20260928/replayPoleCalibration.m", ...
        "research/frame827_perception_20260928/validatePerception827.m"];
    issues=cell(size(files));for k=1:numel(files),issues{k}=checkcode(files(k),'-config=factory');end
    fid=fopen(fullfile(dest,'code_analysis.json'),'w');fprintf(fid,'%s\n',jsonencode(struct('files',files,'issues',{issues})));fclose(fid);
    disp(comparison(:,{'mode','feature','candidatePillarCount','extraPillarCount','coveredFinePointCount','finePointCountInRoi'}));
    fprintf('PERCEPTION827_VALIDATED\n');
end
