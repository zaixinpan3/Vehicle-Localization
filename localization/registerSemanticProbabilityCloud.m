function result = registerSemanticProbabilityCloud(fixedCloud,movingCloud,initialPose,cfg,positionAid,additionalSeeds)
% registerSemanticProbabilityCloud: Semantic Gaussian geometry registration.
% The default GICP-style residuals use both covariances. Elongated ground
% components constrain their normal direction; poles constrain horizontal XY.
% Stored map mixture weights are priors in same-class Gaussian association.
% They already contain temporal stability; do not multiply repeatability again.
% Source temporal stability scales influence after semantic class balancing.
% Ground-line tangents bound association but do not create pose information.
% Height conditions correspondence compatibility, not the planar pose force.
% A partially observable solution is reported but never accepted as full SE(2).
    if nargin<4, cfg=distributionRegistrationConfig(); end
    if nargin<5,positionAid=[];end
    if nargin<6,additionalSeeds=zeros(0,3);end
    solve=@(pose) solveGeometry(fixedCloud,movingCloud,pose,cfg);
    result=selectPositionAidedRegistration(solve,initialPose,positionAid,cfg,additionalSeeds);
end

function result=solveGeometry(fixedCloud,movingCloud,initialPose,cfg)
% Each hypothesis is solved using exactly the same LiDAR-only objective.
    assert(string(cfg.method)=="geometricD2D",'VehicleLocalization:InvalidRegistrationMethod', ...
        'Use geometricD2D registration.');
    initialPose=double(initialPose(:).');gcfg=cfg.geometric;
    model=prepareSemanticRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg);
    f=model.fixed;m=model.moving;height=model.height;
    result=struct('accepted',false,'reason',"insufficientComponents", ...
        'poseXYTheta',initialPose,'initialPoseXYTheta',initialPose,'similarity',0, ...
        'initialSimilarity',0,'iterations',0,'converged',false,'scaledCurvature',zeros(3), ...
        'curvatureEigenvalues',zeros(3,1),'height',height,'observableRank',0, ...
        'observableProjector',zeros(3),'partialPoseAvailable',false, ...
        'physicalObservableProjector',zeros(3),'directionalAccepted',false, ...
        'directionalInformation',zeros(3),'supportedConverged',false, ...
        'observableProjectorCoordinates',"scaled correction q=diag(1,1,yawLeverArm)*deltaPose", ...
        'curvatureSemantics',"uncalibratedGaussianGeometryNormalMatrix", ...
        'similaritySemantics',"sourceClassBalancedGaussianCompatibilityWithCoverage", ...
        'mapWeightSource',"storedMixtureWeight", ...
        'weightSemantics',"mapMixtureAssociationPriorAndSourceClassBalancedQualityTimesTemporalStability", ...
        'information',zeros(3), ...
        'informationSemantics',"robustCompositeGaussianGaussNewton", ...
        'informationCoordinates',"additive map X,Y,psi; meters,radians", ...
        'informationCalibrated',false);
    shared=intersect(unique(f.semanticName),unique(m.semanticName));
    if nnz(ismember(f.semanticName,shared))<cfg.minimumComponents || ...
            nnz(ismember(m.semanticName,shared))<cfg.minimumComponents, return; end
    scale=[1;1;1/cfg.yawLeverArm]; bounds=cfg.maximumPoseCorrection(:)./scale;
    q=zeros(3,1); converged=false;
    for iteration=1:cfg.maximumIterationsPerScale
        pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
        system=model.linearize(pose,scale);
        if iteration==1, result.initialSimilarity=system.similarity; end
        if system.numPairs<cfg.minimumComponents, break; end
        [step,~,rank]=observableStep(system.H,system.gradient,gcfg.minimumObservabilityRatio);
        if rank==0, break; end
        if norm(step,inf)<cfg.stepTolerance
            converged=true; break;
        end
        step=step/max(1,norm(step)); accepted=false;
        for lineSearch=0:15
            trial=max(-bounds,min(bounds,q+step*2^(-lineSearch)));
            trialPose=[trial(1:2).',initialPose(3)+trial(3)*scale(3)];
            cost=model.frozenCost(system,trialPose);
            if cost<system.cost-1e-12
                q=trial; accepted=true; break;
            end
        end
        if ~accepted
            converged=norm(step,inf)<10*cfg.stepTolerance; break;
        end
    end
    pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
    system=model.linearize(pose,scale);
    [finalStep,projector,rank,eigenvalues]=observableStep(system.H,system.gradient,gcfg.minimumObservabilityRatio);
    % Preserve the initial prediction in unsupported directions, rather than
    % silently replacing them with a drift accumulated through changing pairs.
    if rank<3
        q=projector*q;
        pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
        system=model.linearize(pose,scale);
        [finalStep,projector,rank,eigenvalues]=observableStep(system.H,system.gradient,gcfg.minimumObservabilityRatio);
    end
    result.poseXYTheta=initialPose+(q.*scale).';
    result.poseXYTheta(3)=atan2(sin(result.poseXYTheta(3)),cos(result.poseXYTheta(3)));
    result.iterations=iteration; result.converged=converged;
    result.similarity=system.similarity; result.scaledCurvature=system.H;
    result.curvatureEigenvalues=eigenvalues; result.observableRank=rank;
    result.observableProjector=projector;
    result.physicalObservableProjector=diag(scale)*projector/diag(scale);
    result.supportedConverged=converged && norm(finalStep,inf)<10*cfg.stepTolerance ...
        && norm((eye(3)-projector)*q,inf)<10*cfg.stepTolerance;
    result.partialPoseAvailable=rank>0 && rank<3 && system.numPairs>=cfg.minimumComponents;
    % q = diag(1,1,yawLeverArm) * deltaPose. Undo this numerical
    % conditioning before exporting physical pose information. The normal
    % matrix sums J_i' W_i J_i, where W_i contains summed source/map scatter,
    % class-balanced source quality and final robust influence. Map priors
    % determine associations; their mass is not a second residual multiplier.
    % Keep cross terms and genuine null directions; add no diagonal prior.
    inverseScale=diag(1./scale);
    result.information=inverseScale*((system.H+system.H.')/2)*inverseScale;
    % Project in solver coordinates before transforming the bilinear form.
    % Directions excluded by the observability threshold must not leak back
    % through small positive eigenvalues of the raw normal matrix.
    supported=projector*((system.H+system.H.')/2)*projector;
    result.directionalInformation=inverseScale*supported*inverseScale;
    result.directionalInformation=(result.directionalInformation+result.directionalInformation.')/2;
    result.correspondences=struct2table(system.pairs);
    result.matchedFraction=system.numPairs/max(1,m.numComponents);
    result.classDiagnostics=classDiagnostics(system,gcfg);
    result.height.medianConditionalResidual=median(system.pairs.heightResidual,'omitnan');
    if system.numPairs<cfg.minimumComponents || result.matchedFraction<gcfg.minimumMatchFraction || ...
            system.similarity<cfg.minimumSimilarity
        result.reason="insufficientOverlap";
    elseif any(abs(q)>=bounds-1e-4)
        result.reason="searchBoundary";
    elseif rank==0
        result.reason="degenerateGeometry";
    elseif any(result.classDiagnostics.informationWeightedCorrection>gcfg.maximumClassCorrection)
        result.reason="inconsistentClasses";
    elseif ~converged || (rank<3 && ~result.supportedConverged)
        result.reason="notConverged";
    elseif rank<3
        result.directionalAccepted=true; result.reason="acceptedDirectional";
    else
        result.accepted=true; result.reason="accepted";
    end
end

function [step,projector,rank,eigenvalues]=observableStep(h,gradient,ratio)
    [v,d]=eig((h+h.')/2,'vector'); eigenvalues=d;
    keep=d>max(1e-8,ratio*max(d)); rank=nnz(keep);
    projector=v(:,keep)*v(:,keep).';
    step=-v(:,keep)*((v(:,keep).'*gradient)./d(keep));
end

function diagnostics=classDiagnostics(system,cfg)
    classes=unique(system.pairs.semanticName); correction=zeros(numel(classes),1);
    ranks=zeros(numel(classes),1); matches=zeros(numel(classes),1);weighted=zeros(numel(classes),1);
    for c=1:numel(classes)
        h=zeros(3); gradient=zeros(3,1); selected=find(system.pairs.semanticName==classes(c));
        for j=selected.'
            a=system.J(:,:,j); w=system.weights(j)*system.robust(j);
            h=h+w*(a.'*a); gradient=gradient+w*a.'*system.residual(:,j);
        end
        [step,~,ranks(c)]=observableStep(h,gradient,cfg.minimumObservabilityRatio);
        correction(c)=norm(step); matches(c)=numel(selected);
        % Measure disagreement in supported geometry. A large Newton step
        % in a weak class direction must not veto the other classes. The
        % largest eigenvalue keeps this engineering score in scaled meters;
        % it is not a chi-square statistic or a calibrated confidence.
        weighted(c)=sqrt(max(0,step'*h*step)/max(max(eig(h)),eps));
    end
    diagnostics=table(classes,matches,ranks,correction,weighted, ...
        'VariableNames',{'semanticName','matchedComponents','observableRank','observableCorrection','informationWeightedCorrection'});
end
