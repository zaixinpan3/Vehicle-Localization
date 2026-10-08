classdef lidarCalibrationSupport
% lidarCalibrationSupport Causal LiDAR tilt and offline frame calibration.
% Static methods estimate sensor-only roll/pitch from the LiDAR IMU, verify its
% provenance and fit offline pitch and translation calibration candidates.
% Example: tilt = lidarCalibrationSupport.estimateLidarImuTilt(imu,times).

    methods (Static)
        function tilt=estimateLidarImuTilt(imu,wheel,queryTime,cfg)
        % estimateLidarImuTilt Replay the streaming filter using past packets only.
        % imu arrivalTime/deviceTime, specificForce, angularVelocity; wheel
        % arrivalTime/speed. Query times are packet-arrival clock times, in seconds.
            if nargin<4,cfg=lidarImuTiltConfig();end
            queryTime=queryTime(:);n=numel(queryTime);
            assert(all(diff(queryTime)>0),'VehicleLocalization:InvalidTiltClock','Queries must increase.');
            assert(all(diff(imu.arrivalTime)>0) && all(diff(imu.deviceTime)>0) && ...
                all(diff(wheel.arrivalTime)>0),'VehicleLocalization:InvalidTiltClock','Sensor clocks must increase.');
            rotation=repmat(eye(3),1,1,n);angles=zeros(n,2);valid=false(n,1);aligned=valid;
            age=nan(n,1);sourceIndex=zeros(n,1);state=[];i=0;j=0;latest=eye(3);last=struct();
            for k=1:n
                while i<numel(imu.arrivalTime) && imu.arrivalTime(i+1)<=queryTime(k)
                    i=i+1;
                    while j<numel(wheel.arrivalTime) && wheel.arrivalTime(j+1)<=imu.arrivalTime(i),j=j+1;end
                    if j==0,continue;end
                    assert(imu.arrivalTime(i)-wheel.arrivalTime(j)<=cfg.maximumWheelAgeSeconds, ...
                        'VehicleLocalization:TiltWheelGap','Tilt compensation requires a current wheel sample.');
                    sample=struct('time',imu.deviceTime(i),'specificForce',imu.specificForce(i,:), ...
                        'angularVelocity',imu.angularVelocity(i,:));
                    [latest,state,last]=lidarCalibrationSupport.updateLidarImuTilt(sample,wheel.speed(j),state,cfg);
                end
                if ~isempty(state)
                    age(k)=queryTime(k)-imu.arrivalTime(i);
                    assert(age(k)<=cfg.maximumImuAgeSeconds,'VehicleLocalization:TiltImuGap','Latest IMU sample is stale.');
                    rotation(:,:,k)=latest;angles(k,:)=[last.roll,last.pitch];valid(k)=true;aligned(k)=last.aligned;sourceIndex(k)=i;
                end
            end
            tilt=struct('rotation',rotation,'angles',angles,'valid',valid,'aligned',aligned, ...
                'ageSeconds',age,'sourceIndex',sourceIndex,'time',queryTime,'configuration',cfg, ...
                'schemaVersion',1,'source',"front-ouster-raw-imu",'referencePoseUsed',false,'causal',true);
        end

        function [rotation,state,diagnostic]=updateLidarImuTilt(sample,speed,state,cfg)
        % updateLidarImuTilt One causal six-axis IMU update; no pose/reference input.
        % specificForce and angularVelocity are columns in stored point axes. time
        % is native IMU time. speed is a currently available wheel measurement.
        % Only stationary measurements initialize/update gyro bias. The gravity
        % direction is propagated on the unit sphere and corrected by specific force
        % minus the planar wheel-acceleration estimate. No yaw is observed/injected.
            if nargin<4,cfg=lidarImuTiltConfig();end
            f=sample.specificForce(:);w=sample.angularVelocity(:);t=sample.time;
            assert(numel(f)==3 && numel(w)==3 && all(isfinite([t;f;w;speed])), ...
                'VehicleLocalization:InvalidTiltImu','Require finite six-axis IMU and wheel speed.');
            stationary=abs(speed)<cfg.stationarySpeedMps && norm(w)<cfg.stationaryGyroRadps;
            if isempty(state)
                assert(stationary && norm(f)>cfg.gravityMps2/2, ...
                    'VehicleLocalization:TiltAlignmentRequired','Start with stationary six-axis IMU and measured wheel speed.');
                state=struct('time',t,'up',f/norm(f),'gyroBias',w,'biasCount',1, ...
                    'stationarySeconds',0,'speed',speed,'initialized',false);
                acceleration=0;
            else
                dt=t-state.time;
                assert(dt>0 && dt<=cfg.maximumImuStepSeconds, ...
                    'VehicleLocalization:TiltImuGap','IMU timestamps must increase without an excessive gap.');
                filtered=state.speed+(1-exp(-dt/cfg.speedTimeSeconds))*(speed-state.speed);
                acceleration=(filtered-state.speed)/dt;state.speed=filtered;
                if stationary
                    state.biasCount=state.biasCount+1;
                    state.gyroBias=state.gyroBias+(w-state.gyroBias)/state.biasCount;
                    state.stationarySeconds=state.stationarySeconds+dt;
                end
                angular=w-state.gyroBias;angle=norm(angular)*dt;
                if angle>0
                    axis=-angular/norm(angular);u=state.up;
                    state.up=u*cos(angle)+cross(axis,u)*sin(angle)+axis*dot(axis,u)*(1-cos(angle));
                end
                state.time=t;
                state.initialized=state.stationarySeconds>=cfg.minimumAlignmentSeconds;
            end
            angular=w-state.gyroBias;
            gravity=f-[acceleration;state.speed*angular(3);0];
            accepted=abs(norm(gravity)-cfg.gravityMps2)<=cfg.maximumGravityNormErrorMps2;
            if accepted
                measured=gravity/norm(gravity);
                if stationary && ~state.initialized
                    alpha=1/state.biasCount;
                elseif exist('dt','var')
                    alpha=1-exp(-dt/cfg.correctionTimeSeconds);
                else
                    alpha=0;
                end
                state.up=state.up+alpha*(measured-state.up);state.up=state.up/norm(state.up);
            end
            roll=atan2(state.up(2),state.up(3));pitch=atan2(-state.up(1),hypot(state.up(2),state.up(3)));
            cr=cos(roll);sr=sin(roll);cp=cos(pitch);sp=sin(pitch);
            rotation=[cp,sp*sr,sp*cr;0,cr,-sr;-sp,cp*sr,cp*cr];
            diagnostic=struct('roll',roll,'pitch',pitch,'aligned',state.initialized, ...
                'gravityAccepted',accepted,'gravityNormMps2',norm(gravity),'gyroBias',state.gyroBias);
        end

        function assertSensorOnlyLidarTilt(tilt)
        % assertSensorOnlyLidarTilt Reject legacy caches carrying reference-derived tilt.
            assert(isstruct(tilt) && all(isfield(tilt, ...
                {'schemaVersion','source','referencePoseUsed','causal','configuration'})) && ...
                tilt.schemaVersion==1 && ~tilt.referencePoseUsed && tilt.causal && ...
                ismember(string(tilt.source),["front-ouster-raw-imu","disabled"]), ...
                'VehicleLocalization:ReferenceTiltCache','Regenerate source inputs using sensor-only tilt; legacy reference-attitude caches are not allowed.');
        end

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

        function [calibration,report]=fitLidarTranslationCalibration(queryPoses,fixedPoses,matchedPoses,options)
        % fitLidarTranslationCalibration Estimate a planar stored-to-reference offset.
        % matchedPoses registers an unshifted query to an unshifted fixed-frame cloud
        % projected using fixedPoses. All poses are [map X, map Y, yaw radians].
        % Pair selection/registration is offline; no evaluation pose is used online.
        % Planar motion does not identify vertical translation or mounting rotation.
            arguments
                queryPoses (:,3) double {mustBeFinite,mustBeReal}
                fixedPoses (:,3) double {mustBeFinite,mustBeReal}
                matchedPoses (:,3) double {mustBeFinite,mustBeReal}
                options.HuberScaleM (1,1) double {mustBePositive}=.10
                options.MinimumPairs (1,1) double {mustBeInteger,mustBePositive}=8
                options.MinimumAngularExcitation (1,1) double {mustBePositive}=.08
            end
            n=size(queryPoses,1);
            assert(isequal(size(queryPoses),size(fixedPoses),size(matchedPoses)) && n>=options.MinimumPairs, ...
                'VehicleLocalization:InsufficientCalibrationPairs','Expected matching arrays with sufficient pairs.');
            A=zeros(2*n,2);delta=reshape((matchedPoses(:,1:2)-queryPoses(:,1:2)).',[],1);
            for k=1:n
                A(2*k-1:2*k,:)=rotation(queryPoses(k,3))-rotation(fixedPoses(k,3));
            end
            singular=svd(A);
            assert(min(singular)/sqrt(n)>=options.MinimumAngularExcitation, ...
                'VehicleLocalization:InsufficientCalibrationExcitation','Turning motion is required to estimate the offset.');
            offset=A\delta;
            for k=1:20
                residual=reshape(A*offset-delta,2,[]).';
                weight=min(1,options.HuberScaleM./max(vecnorm(residual,2,2),eps));
                rootWeight=repelem(sqrt(weight),2);
                offset=(A.*rootWeight)\(delta.*rootWeight);
            end
            calibration=lidarFrameCalibrationConfig();
            calibration.translation=[offset.',0];calibration.identifier="offlinePlanarReferencePointFit";
            errors=reshape(A*offset-delta,2,[]).';
            report=struct('pairs',n,'translationXYM',offset.','rotationEstimated',false,'verticalTranslationEstimated',false, ...
                'normalizedSingularValues',singular.'/sqrt(n),'huberScaleM',options.HuberScaleM, ...
                'beforeRmseM',sqrt(mean(sum(reshape(delta,2,[]).^2,1))), ...
                'afterRmseM',sqrt(mean(sum(errors.^2,2))),'pairResidualM',vecnorm(errors,2,2), ...
                'interpretation',"Effective stored-axis to recorded-INS-reference translation; not a surveyed mounting position");
        end
    end
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
    [pose,tilt,z]=poseSupport.poseRowToPlanarPose(row);
    yaw=[cos(pose(3)) -sin(pose(3)) 0;sin(pose(3)) cos(pose(3)) 0;0 0 1];
    position=[pose(1:2),z]; rotation=yaw*tilt;
end

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end
