function [calibration, report] = fitLidarPitchCalibration(featureData, options)
% fitLidarPitchCalibration: Offline pitch-only frame increment from static features.
% Compare the first frame with later frames using fixed nearest-XY pairs.
% Fit one rotation about body Y by minimizing squared per-frame median Z
% residuals, with equal frame weight. This is a dataset calibration candidate,
% not a six-axis extrinsic solution or an online localization state. Recorded
% pose error and mounting error cannot be separated by this fit alone.
    arguments
        featureData (1,1) struct
        options.FeatureName (1,1) string = "curb"
        options.Identifier (1,1) string = "offlinePitchCandidate"
        options.MaximumPairDistance (1,1) double {mustBePositive} = .3
        options.MaximumPitchDegrees (1,1) double {mustBePositive} = 8
        options.MinimumForwardTravel (1,1) double {mustBePositive} = 1
        options.MaximumHeadingChangeDegrees (1,1) double {mustBePositive} = 5
    end
    if isfield(featureData,'frameCalibration')
        existing=validateLidarFrameCalibration(featureData.frameCalibration);
        assert(norm(existing.rotation-eye(3),'fro')<1e-10 && norm(existing.translation)<1e-10, ...
            'VehicleLocalization:CalibrationAlreadyApplied','Fit from identity-calibrated observations.');
    end
    feature=find(string(featureData.featureNames)==options.FeatureName,1);
    assert(~isempty(feature),'Requested calibration feature is absent.');
    poses=featureData.framePoseTable;
    [p0,r0]=poseGeometry(poses(1,:));
    reference=featureData.pointsByFeatureFrame{feature,1};
    pairs=cell(0,1); pairFrames=zeros(0,1); travels=zeros(0,1); counts=zeros(0,1);
    for j=2:height(poses)
        [position,rotation]=poseGeometry(poses(j,:));
        travel=(position-p0)*r0;
        heading=acos(max(-1,min(1,dot(r0(:,1),rotation(:,1)))));
        points=featureData.pointsByFeatureFrame{feature,j};
        if isempty(reference) || isempty(points) || abs(travel(1))<options.MinimumForwardTravel || ...
                heading>deg2rad(options.MaximumHeadingChangeDegrees), continue; end
        % Chunked search needs no Statistics and Machine Learning Toolbox.
        index=zeros(size(reference,1),1); distance=inf(size(index));
        for first=1:256:size(reference,1)
            selection=first:min(first+255,size(reference,1));
            [distance(selection),index(selection)]=min( ...
                (reference(selection,1)-points(:,1).').^2+(reference(selection,2)-points(:,2).').^2,[],2);
        end
        keep=distance<options.MaximumPairDistance^2;
        if nnz(keep)<10, continue; end
        pair=struct('reference',(reference(keep,:)-p0)*r0,'moving',(points(index(keep),:)-position)*rotation, ...
            'referenceRotation',r0,'movingRotation',rotation,'heightDifference',position(3)-p0(3));
        pairs{end+1,1}=pair; %#ok<AGROW>
        pairFrames(end+1,1)=featureData.frameIndices(j); %#ok<AGROW>
        travels(end+1,1)=travel(1); counts(end+1,1)=nnz(keep); %#ok<AGROW>
    end
    assert(numel(pairs)>=3,'VehicleLocalization:InsufficientCalibrationExcitation', ...
        'Need at least three translated, overlapping frame pairs with similar headings.');
    bound=deg2rad(options.MaximumPitchDegrees);
    objective=@(angle) mean(heightResidual(pairs,angle).^2);
    angle=fminbnd(objective,-bound,bound,optimset('TolX',1e-9,'Display','off'));
    assert(abs(angle)<.99*bound,'VehicleLocalization:CalibrationSearchBoundary', ...
        'Pitch estimate reached its allowed range; inspect frame and pose conventions.');
    calibration=lidarFrameCalibrationConfig();
    calibration.rotation=pitchRotation(angle); calibration.identifier=options.Identifier;
    before=heightResidual(pairs,0); after=heightResidual(pairs,angle);
    report=struct('pitchIncrementDegrees',-rad2deg(angle),'fittedAngleDegrees',rad2deg(angle), ...
        'referenceFrame',featureData.frameIndices(1),'feature',options.FeatureName, ...
        'parametersEstimated',"pitchOnly",'translationEstimated',false,'onlineState',"X,Y,psi", ...
        'rmsFrameMedianBeforeM',sqrt(mean(before.^2)),'rmsFrameMedianAfterM',sqrt(mean(after.^2)), ...
        'interpretation',"Offline calibration candidate; requires validation on frames not used in fitting.");
    report.pairs=table(pairFrames,travels,counts,before,after, ...
        'VariableNames',{'mapFrame','forwardTravelM','pairCount','medianBeforeM','medianAfterM'});
end

function residual=heightResidual(pairs,angle)
    correction=pitchRotation(angle); residual=zeros(numel(pairs),1);
    for j=1:numel(pairs)
        p=pairs{j};
        difference=p.moving*correction.'*p.movingRotation(3,:).'- ...
            p.reference*correction.'*p.referenceRotation(3,:).'+p.heightDifference;
        residual(j)=median(difference);
    end
end

function rotation=pitchRotation(angle)
    rotation=[cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
end

function [position,rotation]=poseGeometry(row)
    [pose,tilt,z]=poseRowToPlanarPose(row);
    yaw=[cos(pose(3)) -sin(pose(3)) 0;sin(pose(3)) cos(pose(3)) 0;0 0 1];
    position=[pose(1:2),z]; rotation=yaw*tilt;
end
