function cloud=buildViewConditionedLandmarkMap(base,featureData,cfg,observationIndices)
% buildViewConditionedLandmarkMap Retain per-acquisition point-landmark geometry.
% Inputs are an existing offline semantic map and registered fine observations
% from that map's acquisition drive. Optional observationIndices selects an
% explicit mapping subset for experiments; it never selects online queries.
% Same-class aliases are canonicalized once. Each acquisition contributes one
% mean and spatial scatter per landmark, irrespective of return density.
% Curb/facade components and all class masses are preserved. This product is
% XY only; it makes no claim of a joint, view-conditioned height distribution.
    if nargin<3,cfg=landmarkViewMapConfig();end
    validateLandmarkViewConfig(cfg);
    mappingSupport.validateSemanticProbabilityCloud(base);
    assert(all(isfield(featureData,{'pointsByFeatureFrame','featureNames','frameIndices','framePoseTable','frameCalibration'})), ...
        'VehicleLocalization:InvalidMapObservations','Require registered offline observations and their acquisition poses.');
    registrationSupport.validateRegistrationCalibration(base,featureData);
    nFrames=numel(featureData.frameIndices);
    if nargin<4,observationIndices=1:nFrames;end
    assert(isnumeric(observationIndices)&&isreal(observationIndices)&&isvector(observationIndices) && ...
        all(isfinite(observationIndices))&&all(observationIndices>=1&observationIndices<=nFrames&observationIndices==fix(observationIndices)) && ...
        numel(unique(observationIndices))==numel(observationIndices), ...
        'VehicleLocalization:InvalidMapObservationSelection','Select distinct mapping acquisition indices.');
    assert(height(featureData.framePoseTable)==nFrames && ...
        isequal(size(featureData.pointsByFeatureFrame),[numel(featureData.featureNames),nFrames]), ...
        'VehicleLocalization:InvalidMapObservations','Map observations and acquisition poses must align.');
    [canonical,groups]=canonicalizeSemanticCloud(base,cfg.mergeRadius,cfg.pointClasses);
    c=canonical.components;n=c.numComponents;
    cloud=struct('dimension',2,'frameCalibration',base.frameCalibration);
    if isfield(base,'coordinateFrame'),cloud.coordinateFrame=base.coordinateFrame;end
    if isfield(base,'clockModelId'),cloud.clockModelId=base.clockModelId;end
    cloud.components=struct('mean',c.mean(:,1:2),'covariance',c.covariance(1:2,1:2,:), ...
        'mixtureWeight',c.mixtureWeight,'semanticName',c.semanticName,'numComponents',n);
    observations=cell(n,1);acquisition=zeros(nFrames,3);
    for k=observationIndices,acquisition(k,:)=poseRowToPlanarPose(featureData.framePoseTable(k,:));end
    for name=cfg.pointClasses
        ids=find(c.semanticName==name);row=find(string(featureData.featureNames)==name);
        if isempty(ids)||isempty(row),continue;end
        for k=observationIndices
            p=double(featureData.pointsByFeatureFrame{row,k});
            assert(size(p,2)==3 && isreal(p)&&all(isfinite(p),'all'), ...
                'VehicleLocalization:InvalidMapObservations','Offline observations require finite registered XYZ.');
            if isempty(p),continue;end
            distances=(p(:,1)-c.mean(ids,1).').^2+(p(:,2)-c.mean(ids,2).').^2;
            [distance,j]=min(distances,[],2);assigned=ids(j);assigned(distance>cfg.maximumAssignmentDistance^2)=0;
            for id=unique(assigned(assigned>0)).'
                q=p(assigned==id,1:2);count=size(q,1);if count<cfg.minimumObservationPoints,continue;end
                mu=mean(q,1);delta=q-mu;S=delta.'*delta/count;
                observations{id}(end+1,:)=[acquisition(k,:),mu,S(1,1),S(1,2),S(2,2),count];
            end
        end
    end
    cloud.landmarkViews=struct('schemaVersion',1,'config',cfg,'observations',{observations}, ...
        'columns',["originX","originY","heading","meanX","meanY","varXX","covXY","varYY","pointCount"], ...
        'semantics',"equal-acquisition XY distributions conditioned on predicted viewing origin", ...
        'informationCalibrated',false);
    cloud.mapConstruction=struct('mappingFrameIndices',featureData.frameIndices(observationIndices), ...
        'originalComponentGroups',{groups},'selectedMappingAcquisitions',numel(observationIndices), ...
        'retainedLandmarkObservations',sum(cellfun(@(o)size(o,1),observations)), ...
        'onlineQueryLabelsUsed',false,'heightModel',"unavailable");
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end
