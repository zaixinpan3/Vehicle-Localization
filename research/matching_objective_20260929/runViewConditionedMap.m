function runViewConditionedMap()
% runViewConditionedMap Test acquisition-conditioned point-landmark moments.
% Offline fine observations and their acquisition origins define the map.
% Runtime conditioning uses the predicted sensor origin, never query labels.
% Even-index map observations permit a separately reported odd-query control.
    setupVehicleLocalization();out='output/matching_objective_20260929';
    original=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');
    [cloud,groups]=canonicalizeSemanticCloud(original.cloud,1.5,["pole","trafficSign"]);
    c=cloud.components;n=c.numComponents;
    cloud=struct('dimension',2,'coordinateFrame',original.cloud.coordinateFrame, ...
        'frameCalibration',original.cloud.frameCalibration);
    cloud.components=struct('mean',c.mean(:,1:2),'covariance',c.covariance(1:2,1:2,:), ...
        'mixtureWeight',c.mixtureWeight,'semanticName',c.semanticName,'numComponents',n);
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');f=a.featureData;
    observation=cell(n,1);acquisition=zeros(numel(f.frameIndices),3);
    for k=1:numel(f.frameIndices),acquisition(k,:)=poseRowToPlanarPose(f.framePoseTable(k,:));end
    for className=["pole","trafficSign"]
        ids=find(c.semanticName==className);row=find(string(f.featureNames)==className);
        for k=2:2:numel(f.frameIndices)
            p=double(f.pointsByFeatureFrame{row,k}(:,1:2));if isempty(p),continue;end
            distances=(p(:,1)-c.mean(ids,1).').^2+(p(:,2)-c.mean(ids,2).').^2;
            [d,j]=min(distances,[],2);assigned=ids(j);assigned(d>1.5^2)=0;
            for id=unique(assigned(assigned>0)).'
                q=p(assigned==id,:);if size(q,1)<3,continue;end
                mu=mean(q,1);delta=q-mu;S=delta.'*delta/size(q,1);
                observation{id}(end+1,:)=[k,acquisition(k,:),mu,S(1,1),S(1,2),S(2,2),size(q,1)]; %#ok<AGROW>
            end
        end
    end
    viewModel=struct('observations',{observation},'mapTraining',"even frames only", ...
        'bandwidth',5,'minimumEffectiveFrames',2,'varianceFloor',.0025,'maximumHeadingDifference',deg2rad(30));
    mapFile=fullfile(out,'viewConditioned_map.mat');save(mapFile,'cloud','viewModel','groups','-v7.3');
    cfg=distributionRegistrationConfig();cfg.pyramid.mapMergeRadius=0;
    for bandwidth=[5 10]
        viewModel.bandwidth=bandwidth;
        replayStudy('output/root_cause_matching_20260929/finalSurface_sources.mat',"viewMap"+bandwidth,cfg,mapFile,viewModel);
    end
end
