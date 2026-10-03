function variance=predictLidarPoseErrorVariance(features,calibration)
% predictLidarPoseErrorVariance Predict uncentered conditional pose error.
% Variances are per horizontal axis (m^2) and heading (rad^2). The nonnegative
% intercept retains common errors invisible to within-scan residual scatter.
% This does not subtract a fitted bias from the actual pose measurement.
    arguments
        features (2,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}
        calibration (1,1) struct
    end
    assert(string(calibration.kind)=="conditional-pose-second-moment-v1", ...
        'VehicleLocalization:InvalidLidarCalibration','Unknown pose error calibration.');
    coefficients=calibration.coefficients;
    validateattributes(coefficients,{'double'},{'real','finite','nonnegative','size',[2,2]});
    variance=coefficients(:,1)+coefficients(:,2).*features;
    assert(all(variance>0),'VehicleLocalization:InvalidLidarCalibration', ...
        'Calibrated error second moments must be strictly positive.');
end
