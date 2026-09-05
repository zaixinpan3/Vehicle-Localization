function [fixed, moving, details] = prepareSemanticRegistration(fixedCloud, movingCloud, cfg)
% prepareSemanticRegistration: Resolve height availability and its reference.
% A finite heightTranslation is moving-origin map Z in meters. Standard
% deviations model vertical translation/tilt uncertainty in the moving cloud.
% Auto uses the XY marginal if height or the reference is unavailable.
    fixed = mappingSupport.validateSemanticProbabilityCloud(fixedCloud);
    moving = mappingSupport.validateSemanticProbabilityCloud(movingCloud);
    calibrationStatus=validateRegistrationCalibration(fixedCloud,movingCloud);
    mode = "xy";
    translation = NaN;
    heightSd = 0.20;
    tiltSd = deg2rad(0.5);
    if isfield(cfg,'heightMode'), mode=string(cfg.heightMode); end
    if isfield(cfg,'heightTranslation'), translation=cfg.heightTranslation; end
    if isfield(cfg,'heightStandardDeviation'), heightSd=cfg.heightStandardDeviation; end
    if isfield(cfg,'tiltStandardDeviation'), tiltSd=cfg.tiltStandardDeviation; end
    assert(isscalar(mode) && ismember(mode,["auto","xy","xyz"]), 'Invalid height mode.');
    assert(isnumeric(translation) && isreal(translation) && isscalar(translation) && ...
        (isfinite(translation)||isnan(translation)), 'Invalid height translation.');
    assert(isnumeric(heightSd) && isreal(heightSd) && isscalar(heightSd) && isfinite(heightSd) && heightSd>=0 && ...
        isnumeric(tiltSd) && isreal(tiltSd) && isscalar(tiltSd) && isfinite(tiltSd) && tiltSd>=0, 'Invalid height/tilt uncertainty.');
    available = hasHeight(fixed) && hasHeight(moving);
    useHeight = mode~="xy" && available && isfinite(translation);
    assert(mode~="xyz" || useHeight, 'VehicleLocalization:HeightUnavailable', ...
        'XYZ matching requires height in both clouds and a finite heightTranslation.');
    dimension = 2+useHeight;
    fixed = projectSemanticProbabilityCloud(fixedCloud,dimension).components;
    moving = projectSemanticProbabilityCloud(movingCloud,dimension).components;
    details = struct('dimension',dimension,'heightUsed',useHeight, ...
        'calibrationStatus',calibrationStatus, ...
        'heightTranslation',translation,'heightStandardDeviation',heightSd, ...
        'tiltStandardDeviation',tiltSd,'heightReason',"enabled");
    if useHeight
        for k=1:moving.numComponents
            p=moving.mean(k,:);
            % First-order gravity-aligned roll/pitch perturbation Jacobian.
            jacobian=[0 p(3);-p(3) 0;p(2) -p(1)];
            moving.covariance(:,:,k)=moving.covariance(:,:,k)+tiltSd^2*(jacobian*jacobian.');
        end
        moving.covariance(3,3,:)=moving.covariance(3,3,:)+heightSd^2;
        moving.mean(:,3)=moving.mean(:,3)+translation;
    elseif mode=="xy"
        details.heightReason="explicitXY";
    elseif ~available
        details.heightReason="heightUnavailable";
    else
        details.heightReason="verticalReferenceUnavailable";
    end
end

function available=hasHeight(components)
    available=size(components.mean,2)==3 || ...
        (isfield(components,'heightAvailable') && all(components.heightAvailable));
end

function status = validateRegistrationCalibration(fixedCloud,movingCloud)
% validateRegistrationCalibration: Refuse known inconsistent frame transforms.
% Legacy clouds with no provenance remain usable only with an identity peer.
    fixedKnown=isfield(fixedCloud,'frameCalibration');
    movingKnown=isfield(movingCloud,'frameCalibration');
    fixed=lidarFrameCalibrationConfig(); moving=fixed;
    if fixedKnown, fixed=validateLidarFrameCalibration(fixedCloud.frameCalibration); end
    if movingKnown, moving=validateLidarFrameCalibration(movingCloud.frameCalibration); end
    same=norm(fixed.rotation-moving.rotation,'fro')<1e-8 && ...
        norm(fixed.translation-moving.translation)<1e-8;
    assert(same,'VehicleLocalization:CalibrationMismatch', ...
        'Map and source require the same frame calibration; rebuild the map with the selected transform.');
    status="unverifiedLegacy";
    if fixedKnown && movingKnown, status="verifiedTransform"; end
end
