function result=registerWeightedNdtProbabilityCloud(fixedCloud,movingCloud,initialPose,cfg)
% registerWeightedNdtProbabilityCloud NDT overlap weighted by existing map mass.
% Integrate each source Gaussian against the existing same-class map GMM.
% Existing map mixtureWeight is used once, without additional repeatability.
% Optimize X/Y/yaw; source and map grids need not correspond. Distant Gaussian
% pairs have vanishing influence. Information is the observed
% objective curvature, not an empirically calibrated pose-error covariance.
    if nargin<4,cfg=weightedNdtRegistrationConfig();end
    assert(string(cfg.method)=="weightedNdt",'VehicleLocalization:InvalidRegistrationMethod', ...
        'Use weightedNdt registration.');
    [fixed,moving,height]=registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose=double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)),'Expected finite [X Y yaw].');
    validateattributes(cfg.yawLeverArm,{'double'},{'scalar','positive','finite'});
    validateattributes(cfg.maximumPoseCorrection,{'double'},{'numel',3,'positive','finite'});
    values=struct2array(cfg.ndt);assert(all(isfinite(values)&values>0),'Invalid NDT configuration.');
    assert(cfg.ndt.minimumInlierProbability<1 && cfg.ndt.minimumObservabilityRatio<1);
    fixed.mean(:,1:2)=fixed.mean(:,1:2)-initialPose(1:2);
    if height.heightUsed
        fixed.mean(:,3)=fixed.mean(:,3)-height.heightTranslation;
        moving.mean(:,3)=moving.mean(:,3)-height.heightTranslation;
    end
    height.strategy="jointGaussianOverlap";height.planarForce=height.heightUsed;
    model=semanticNdtSupport.prepare(fixed,moving,cfg.ndt);
    scale=diag([1 1 1/cfg.yawLeverArm]);bounds=cfg.maximumPoseCorrection(:)./diag(scale);
    result=emptyResult(initialPose,height);
    names=intersect(unique(fixed.semanticName),unique(moving.semanticName));
    if nnz(ismember(fixed.semanticName,names)&fixed.mixtureWeight>0)<cfg.minimumComponents || ...
            nnz(ismember(moving.semanticName,names)&moving.mixtureWeight>0)<cfg.minimumComponents
        return
    end
    [~,~,start]=semanticNdtSupport.evaluate(model,[0 0 initialPose(3)]);
    result.initialSimilarity=start.similarity;
    if start.matchedSources<cfg.minimumComponents
        result.reason="insufficientOverlap";result.correspondences=start.correspondences;return
    end
    options=optimoptions('fmincon','Algorithm','sqp','SpecifyObjectiveGradient',true, ...
        'Display','off','MaxIterations',cfg.maximumIterationsPerScale, ...
        'OptimalityTolerance',cfg.gradientTolerance,'StepTolerance',cfg.stepTolerance);
    fun=@(q) scaledObjective(q,eye(3),model,initialPose(3),scale);
    [q,~,flag,output]=fmincon(fun,zeros(3,1),[],[],[],[],-bounds,bounds,[],options);
    pose=[q(1:2).',initialPose(3)+q(3)/cfg.yawLeverArm];
    [physical,classH]=semanticNdtSupport.curvature(model,pose);
    [basis,rank,eigenvalues]=observableBasis(scale*physical*scale,cfg);
    if rank>0 && rank<3
        % Anchor unsupported corrections at the original prediction, then
        % solve again in the supported subspace with the same full objective.
        reduced=@(z) scaledObjective(z,basis,model,initialPose(3),scale);
        [z,~,flag,extra]=fmincon(reduced,basis.'*q,[basis;-basis],[bounds;bounds], ...
            [],[],[],[],[],options);
        q=basis*z;output.iterations=output.iterations+extra.iterations;
        pose=[q(1:2).',initialPose(3)+q(3)/cfg.yawLeverArm];
        [physical,classH]=semanticNdtSupport.curvature(model,pose);
        [basis,rank,eigenvalues]=observableBasis(scale*physical*scale,cfg);
    end
    [cost,gradient,details]=semanticNdtSupport.evaluate(model,pose);
    projector=basis*basis.';h=scale*physical*scale;
    result.poseXYTheta=initialPose+(scale*q).';
    result.poseXYTheta(3)=atan2(sin(result.poseXYTheta(3)),cos(result.poseXYTheta(3)));
    result.iterations=output.iterations;result.converged=flag>0;
    result.objectiveCost=cost;result.objectiveGradient=gradient;
    result.similarity=details.similarity;result.scaledCurvature=h;
    result.curvatureEigenvalues=eigenvalues;result.observableRank=rank;
    result.observableProjector=projector;
    result.physicalObservableProjector=scale*projector/scale;
    result.information=physical;
    result.directionalInformation=(scale\(projector*h*projector))/scale;
    result.directionalInformation=(result.directionalInformation+result.directionalInformation.')/2;
    result.correspondences=details.correspondences;
    result.matchedFraction=details.matchedSources/max(1,moving.numComponents);
    result.partialPoseAvailable=rank>0&&rank<3&&details.matchedSources>=cfg.minimumComponents;
    result.supportedConverged=result.converged && norm(projector*scale*gradient,inf)<10*cfg.gradientTolerance ...
        && norm((eye(3)-projector)*q,inf)<10*cfg.stepTolerance;
    result.classDiagnostics=classDiagnostics(model,details,classH,scale,cfg);
    if details.matchedSources<cfg.minimumComponents || result.matchedFraction<cfg.ndt.minimumMatchFraction || ...
            result.similarity<cfg.minimumSimilarity
        result.reason="insufficientOverlap";
    elseif any(abs(q)>=bounds-1e-4)
        result.reason="searchBoundary";
    elseif rank==0 || min(eigenvalues)<-cfg.minimumScaledCurvature
        result.reason="nonMinimumOrDegenerate";
    elseif any(result.classDiagnostics.observableCorrection>cfg.ndt.maximumClassCorrection)
        result.reason="inconsistentClasses";
    elseif ~result.converged || (rank<3 && ~result.supportedConverged)
        result.reason="notConverged";
    elseif rank<3
        result.reason="acceptedDirectional";result.directionalAccepted=true;
    else
        result.reason="accepted";result.accepted=true;
    end
end

function [cost,gradient]=scaledObjective(z,basis,model,yaw,scale)
    q=basis*z;pose=[q(1:2).',yaw+q(3)*scale(3,3)];
    [cost,physicalGradient]=semanticNdtSupport.evaluate(model,pose);
    cost=model.cfg.objectiveScale*cost;
    gradient=model.cfg.objectiveScale*basis.'*scale*physicalGradient;
end

function [basis,rank,eigenvalues]=observableBasis(h,cfg)
    [v,d]=eig((h+h.')/2,'vector');eigenvalues=d;
    keep=d>max(cfg.minimumScaledCurvature,cfg.ndt.minimumObservabilityRatio*max(d));
    basis=v(:,keep);rank=nnz(keep);
end

function diagnostics=classDiagnostics(model,details,hessians,scale,cfg)
    n=numel(model.groups);names=strings(n,1);correction=zeros(n,1);ranks=zeros(n,1);matches=zeros(n,1);
    for k=1:n
        names(k)=model.groups{k}.name;h=scale*hessians(:,:,k)*scale;
        [basis,ranks(k)]=observableBasis(h,cfg);
        if ranks(k)>0
            step=-basis*((basis.'*h*basis)\(basis.'*scale*details.classGradient(:,k)));
            correction(k)=norm(step);
        end
        matches(k)=nnz(details.correspondences.semanticName==names(k));
    end
    diagnostics=table(names,matches,ranks,correction, ...
        'VariableNames',{'semanticName','matchedComponents','observableRank','observableCorrection'});
end

function result=emptyResult(initial,height)
    result=struct('accepted',false,'reason',"insufficientComponents", ...
        'poseXYTheta',initial,'initialPoseXYTheta',initial,'similarity',0, ...
        'initialSimilarity',0,'iterations',0,'converged',false,'scaledCurvature',zeros(3), ...
        'curvatureEigenvalues',zeros(3,1),'height',height,'observableRank',0, ...
        'observableProjector',zeros(3),'physicalObservableProjector',zeros(3), ...
        'partialPoseAvailable',false,'directionalAccepted',false,'directionalInformation',zeros(3), ...
        'supportedConverged',false,'information',zeros(3),'informationCalibrated',false, ...
        'observableProjectorCoordinates',"q=diag(1,1,yawLeverArm)*deltaPose", ...
        'curvatureSemantics',"uncalibratedNdtMixtureObservedHessian", ...
        'similaritySemantics',"classBalancedSourceInlierProbability", ...
        'weightSemantics',"existingMapMixtureWeightOnce", ...
        'informationSemantics',"observedNegativeWeightedGaussianOverlapCurvature", ...
        'informationCoordinates',"additive map X,Y,psi; meters,radians", ...
        'objectiveCost',NaN,'objectiveGradient',zeros(3,1),'matchedFraction',0, ...
        'correspondences',table(zeros(0,1),zeros(0,1),strings(0,1),zeros(0,1), ...
            'VariableNames',{'source','target','semanticName','inlierProbability'}), ...
        'classDiagnostics',table(strings(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
            'VariableNames',{'semanticName','matchedComponents','observableRank','observableCorrection'}));
end
