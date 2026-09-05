function result = registerSemanticProbabilityCloud(fixedCloud, movingCloud, initialPose, cfg)
% registerSemanticProbabilityCloud: Align planar semantic distributions.
% Maximize normalized Gaussian overlap using analytic derivatives, BFGS and
% Armijo line search at decreasing covariance smoothing scales. initialPose
% and poseXYTheta map moving-local XY to fixed-map XY, in meters/radians.
% The final curvature is a local observability diagnostic, NOT a calibrated
% pose information matrix. A rejected result must not be fused as a pose.
    if nargin < 4 || isempty(cfg)
        cfg = distributionRegistrationConfig();
    end
    [fixed, moving, heightDetails] = prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose = double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)), 'Expected finite initial [x y yaw].');
    assert(isscalar(cfg.yawLeverArm) && cfg.yawLeverArm>0 && isfinite(cfg.yawLeverArm), 'Invalid yaw scale.');
    scales = double(cfg.smoothingStandardDeviations(:).');
    assert(~isempty(scales) && all(isfinite(scales) & scales>=0) && scales(end)==0, 'Smoothing must end at zero.');
    bounds = double(cfg.maximumPoseCorrection(:));
    assert(numel(bounds)==3 && all(isfinite(bounds) & bounds>0), 'Invalid search bounds.');
    result = struct('accepted',false,'reason',"insufficientComponents", ...
        'poseXYTheta',initialPose,'initialPoseXYTheta',initialPose, ...
        'similarity',0,'initialSimilarity',0,'iterations',0,'converged',false, ...
        'scaledCurvature',nan(3),'curvatureEigenvalues',nan(3,1), ...
        'curvatureSemantics',"uncalibratedNegativeSimilarityHessian");
    result.height = heightDetails;
    common = intersect(unique(fixed.semanticName),unique(moving.semanticName));
    if nnz(ismember(fixed.semanticName,common) & fixed.mixtureWeight>0)<cfg.minimumComponents || ...
            nnz(ismember(moving.semanticName,common) & moving.mixtureWeight>0)<cfg.minimumComponents
        return;
    end
    % Recenter the map once to avoid cancellation at UTM-sized coordinates.
    fixed.mean(:,1:2) = fixed.mean(:,1:2)-initialPose(1:2);
    if heightDetails.heightUsed
        fixed.mean(:,3)=fixed.mean(:,3)-heightDetails.heightTranslation;
        moving.mean(:,3)=moving.mean(:,3)-heightDetails.heightTranslation;
    end
    rawFixed = fixed; rawMoving = moving;
    [fixed,moving] = balanceSemanticDistributions(rawFixed,rawMoving);
    parameterScale = [1;1;1/cfg.yawLeverArm];
    scaledBounds = bounds./parameterScale;
    q = zeros(3,1);
    normalization = sqrt(semanticGaussianOverlap(fixed,fixed,[0 0 0])* ...
        semanticGaussianOverlap(moving,moving,[0 0 0]));
    [f0,~] = objective(q,fixed,moving,initialPose(3),parameterScale,normalization);
    result.initialSimilarity = -f0;
    % Compare continuation with direct local refinement. Smoothing can merge
    % neighboring landmark modes; it must not discard a better fine-scale basin.
    bestValue = Inf; bestQ = q; bestConverged = false;
    for schedule = {scales, 0}
      q = zeros(3,1);
      for smoothing = schedule{1}
        f = smoothComponents(rawFixed,smoothing);
        m = smoothComponents(rawMoving,smoothing);
        [f,m] = balanceSemanticDistributions(f,m);
        normValue = sqrt(semanticGaussianOverlap(f,f,[0 0 0])*semanticGaussianOverlap(m,m,[0 0 0]));
        [value,gradient] = objective(q,f,m,initialPose(3),parameterScale,normValue);
        inverseHessian = eye(3);
        converged = false;
        for iteration = 1:cfg.maximumIterationsPerScale
            result.iterations = result.iterations+1;
            if norm(gradient,inf) <= cfg.gradientTolerance
                converged = true;
                break;
            end
            direction = -inverseHessian*gradient;
            if gradient.'*direction >= 0
                direction = -gradient;
                inverseHessian = eye(3);
            end
            direction = direction/max(1,norm(direction));
            step = 1;
            acceptedStep = false;
            for lineIteration = 1:24
                trial = max(-scaledBounds,min(scaledBounds,q+step*direction));
                displacement = trial-q;
                [trialValue,trialGradient] = objective(trial,f,m,initialPose(3),parameterScale,normValue);
                if gradient.'*displacement<0 && trialValue<=value+1e-4*(gradient.'*displacement)
                    acceptedStep = true;
                    break;
                end
                step = step/2;
            end
            if ~acceptedStep
                break;
            end
            y = trialGradient-gradient;
            ys = y.'*displacement;
            if ys > 1e-12*norm(y)*norm(displacement)
                v = eye(3)-displacement*y.'/ys;
                inverseHessian = v*inverseHessian*v.'+(displacement*displacement.')/ys;
            end
            q = trial; value = trialValue; gradient = trialGradient;
            if norm(displacement,inf)<=cfg.stepTolerance && norm(gradient,inf)<=10*cfg.gradientTolerance
                converged = true;
                break;
            end
        end
      end
      if value < bestValue
          bestValue = value; bestQ = q; bestConverged = converged;
      end
    end
    q = bestQ; value = bestValue; converged = bestConverged;
    result.converged = converged;
    result.poseXYTheta = initialPose+(q.*parameterScale).';
    result.poseXYTheta(3) = atan2(sin(result.poseXYTheta(3)),cos(result.poseXYTheta(3)));
    result.similarity = max(0,min(1,-value));
    curvature = zeros(3);
    for axis = 1:3
        dq = zeros(3,1); dq(axis)=1e-3;
        [~,gp] = objective(q+dq,fixed,moving,initialPose(3),parameterScale,normalization);
        [~,gm] = objective(q-dq,fixed,moving,initialPose(3),parameterScale,normalization);
        curvature(:,axis)=(gp-gm)/(2e-3);
    end
    curvature = (curvature+curvature.')/2;
    eigenvalues = eig(curvature);
    result.scaledCurvature = curvature;
    result.curvatureEigenvalues = eigenvalues;
    if any(abs(q)>=scaledBounds-1e-4)
        result.reason = "searchBoundary";
    elseif ~converged
        result.reason = "notConverged";
    elseif result.similarity<cfg.minimumSimilarity
        result.reason = "insufficientOverlap";
    elseif min(eigenvalues)<cfg.minimumScaledCurvature || min(eigenvalues)/max(eigenvalues)<cfg.minimumCurvatureRatio
        result.reason = "degenerateGeometry";
    else
        result.accepted = true;
        result.reason = "accepted";
    end
end

function components = smoothComponents(components, standardDeviation)
    components.covariance(1,1,:) = components.covariance(1,1,:)+standardDeviation^2;
    components.covariance(2,2,:) = components.covariance(2,2,:)+standardDeviation^2;
end

function [value,gradient] = objective(q,fixed,moving,yaw,scale,normalization)
    pose = (q.*scale).'; pose(3)=pose(3)+yaw;
    [energy,derivative] = semanticGaussianOverlap(fixed,moving,pose);
    value = -energy/max(normalization,realmin);
    gradient = -derivative(:).*scale/max(normalization,realmin);
end
