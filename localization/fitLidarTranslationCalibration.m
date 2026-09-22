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

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end
