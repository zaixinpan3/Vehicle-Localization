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
