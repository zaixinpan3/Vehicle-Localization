function features=lidarPoseErrorFeatures(model)
% lidarPoseErrorFeatures Conditional cluster-sandwich pose-error predictors.
% Evaluate at the fitted measurement, never at the observer prediction. Each
% matched distribution is one residual cluster. These predictors are NOT a
% calibrated covariance; a held-out empirical error calibration is required.
% Objective rescaling cancels between the normal inverse and cluster scores.
    arguments
        model (1,1) struct
    end
    m=evaluateLidarRegistrationResidual(model,model.anchorPose(:));
    J=m.jacobian;r=m.residual;H=J.'*J;inverse=pinv((H+H.')/2);
    count=numel(model.rowInfluence);width=numel(r)/count;
    assert(width==fix(width) && count>0,'VehicleLocalization:InvalidLidarResidualModel', ...
        'Residual clusters must correspond to matched distributions.');
    scores=zeros(3,count);
    for k=1:count
        ids=(k-1)*width+(1:width);scores(:,k)=J(ids,:).'*r(ids);
    end
    covariance=inverse*(scores*scores.')*inverse;
    features=max([trace(covariance(1:2,1:2))/2;covariance(3,3)],0);
end
