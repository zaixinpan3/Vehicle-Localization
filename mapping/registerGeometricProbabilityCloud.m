function result = registerGeometricProbabilityCloud(fixedCloud,movingCloud,initialPose,cfg)
% registerGeometricProbabilityCloud: Semantic Gaussian geometry registration.
% GICP-style distribution residuals use both covariances. Elongated ground
% components constrain their normal direction; poles constrain horizontal XY.
% Mixture volume/count mass never becomes a geometric correspondence weight.
% Height conditions correspondence compatibility, not the planar pose force.
% A partially observable solution is reported but never accepted as full SE(2).
    if nargin<4, cfg=distributionRegistrationConfig(); end
    [f,m,height]=prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose=double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)),'Expected finite [x y yaw].');
    gcfg=cfg.geometric;
    validateParameters(cfg,gcfg);
    originalF=validateSemanticProbabilityCloud(fixedCloud);
    originalM=validateSemanticProbabilityCloud(movingCloud);
    f.quality=quality(originalF); m.quality=quality(originalM);
    % Height/tilt uncertainty belongs to the compatibility calculation. Keep
    % the planar metric identical when only height mode/reference changes.
    planarMoving=projectSemanticProbabilityCloud(movingCloud,2);
    m.planarCovariance=planarMoving.components.covariance;
    f.mean(:,1:2)=f.mean(:,1:2)-initialPose(1:2);
    if height.heightUsed
        f.mean(:,3)=f.mean(:,3)-height.heightTranslation;
        m.mean(:,3)=m.mean(:,3)-height.heightTranslation;
    end
    height.strategy="conditionalDistributionCompatibility";
    height.planarForce=false;
    result=struct('accepted',false,'reason',"insufficientComponents", ...
        'poseXYTheta',initialPose,'initialPoseXYTheta',initialPose,'similarity',0, ...
        'initialSimilarity',0,'iterations',0,'converged',false,'scaledCurvature',zeros(3), ...
        'curvatureEigenvalues',zeros(3,1),'height',height,'observableRank',0, ...
        'observableProjector',zeros(3),'partialPoseAvailable',false, ...
        'curvatureSemantics',"uncalibratedGaussianGeometryNormalMatrix", ...
        'similaritySemantics',"meanClassGaussianResidualCompatibilityWithCoverage");
    shared=intersect(unique(f.semanticName),unique(m.semanticName));
    if nnz(ismember(f.semanticName,shared))<cfg.minimumComponents || ...
            nnz(ismember(m.semanticName,shared))<cfg.minimumComponents, return; end
    [f.normal,f.majorVariance,f.lineEligible]=normalGeometry(f,gcfg);
    if height.heightUsed, f=prepareConditionalHeight(f); end
    scale=[1;1;1/cfg.yawLeverArm]; bounds=cfg.maximumPoseCorrection(:)./scale;
    q=zeros(3,1); converged=false;
    for iteration=1:cfg.maximumIterationsPerScale
        pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
        system=linearize(f,m,pose,gcfg,scale);
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
            cost=frozenCost(system,m,trialPose,gcfg);
            if cost<system.cost-1e-12
                q=trial; accepted=true; break;
            end
        end
        if ~accepted
            converged=norm(step,inf)<10*cfg.stepTolerance; break;
        end
    end
    pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
    system=linearize(f,m,pose,gcfg,scale);
    [~,projector,rank,eigenvalues]=observableStep(system.H,system.gradient,gcfg.minimumObservabilityRatio);
    % Preserve the initial prediction in unsupported directions, rather than
    % silently replacing them with a drift accumulated through changing pairs.
    if rank<3
        q=projector*q;
        pose=[q(1:2).',initialPose(3)+q(3)*scale(3)];
        system=linearize(f,m,pose,gcfg,scale);
        [~,projector,rank,eigenvalues]=observableStep(system.H,system.gradient,gcfg.minimumObservabilityRatio);
    end
    result.poseXYTheta=initialPose+(q.*scale).';
    result.poseXYTheta(3)=atan2(sin(result.poseXYTheta(3)),cos(result.poseXYTheta(3)));
    result.iterations=iteration; result.converged=converged;
    result.similarity=system.similarity; result.scaledCurvature=system.H;
    result.curvatureEigenvalues=eigenvalues; result.observableRank=rank;
    result.observableProjector=projector;
    result.correspondences=system.pairs;
    result.matchedFraction=system.numPairs/max(1,m.numComponents);
    result.classDiagnostics=classDiagnostics(system,gcfg);
    result.height.medianConditionalResidual=median(system.pairs.heightResidual,'omitnan');
    if system.numPairs==0 || result.matchedFraction<gcfg.minimumMatchFraction || ...
            system.similarity<cfg.minimumSimilarity
        result.reason="insufficientOverlap";
    elseif any(abs(q)>=bounds-1e-4)
        result.reason="searchBoundary";
    elseif rank<3
        result.reason="degenerateGeometry";
        result.partialPoseAvailable=system.numPairs>=cfg.minimumComponents;
    elseif system.numPairs<cfg.minimumComponents
        result.reason="insufficientOverlap";
    elseif any(result.classDiagnostics.observableCorrection>gcfg.maximumClassCorrection)
        result.reason="inconsistentClasses";
    elseif ~converged
        result.reason="notConverged";
    else
        result.accepted=true; result.reason="accepted";
    end
end

function system=linearize(f,m,pose,cfg,scale)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    means=m.mean(:,1:2)*r.'+pose(1:2);
    n=m.numComponents;
    jacobian=zeros(2,3,n); residual=zeros(2,n); precision=zeros(2,2,n);
    source=zeros(n,1); target=zeros(n,1); weights=zeros(n,1); qvalue=zeros(n,1);
    zResidual=nan(n,1); rows=0;
    for i=1:n
        candidates=find(f.semanticName==m.semanticName(i) & f.quality>0);
        if isempty(candidates) || m.quality(i)<=0, continue; end
        ground=ismember(m.semanticName(i),["curb","roadMarking"]);
        if ground, candidates=candidates(f.lineEligible(candidates)); end
        if isempty(candidates), continue; end
        cm=r*m.planarCovariance(:,:,i)*r.';
        delta=means(i,:)-f.mean(candidates,1:2);
        cf=f.covariance(1:2,1:2,candidates);
        a=reshape(cf(1,1,:),[],1)+cm(1,1)+cfg.noiseStandardDeviation^2;
        b=reshape(cf(1,2,:),[],1)+cm(1,2);
        d=reshape(cf(2,2,:),[],1)+cm(2,2)+cfg.noiseStandardDeviation^2;
        if ground
            normal=f.normal(candidates,:);
            dn=sum(delta.*normal,2); tangent=[-normal(:,2),normal(:,1)];
            dt=sum(delta.*tangent,2);
            variance=a.*normal(:,1).^2+2*b.*normal(:,1).*normal(:,2)+d.*normal(:,2).^2;
            distance=dn.^2./variance+dt.^2./(f.majorVariance(candidates)+cfg.maximumMatchDistance^2);
            valid=abs(dn)<=cfg.maximumMatchDistance & ...
                abs(dt)<=3*sqrt(f.majorVariance(candidates))+cfg.maximumMatchDistance;
        else
            determinant=a.*d-b.^2;
            distance=(d.*delta(:,1).^2-2*b.*delta(:,1).*delta(:,2)+a.*delta(:,2).^2)./determinant;
            % Gaussian shape compatibility discourages diffuse components
            % from capturing a compact landmark solely through large variance.
            detF=reshape(cf(1,1,:),[],1).*reshape(cf(2,2,:),[],1)-reshape(cf(1,2,:),[],1).^2;
            detM=det(cm);
            distance=distance+max(0,log(determinant./(4*sqrt(detF*detM))));
            valid=sum(delta.^2,2)<=cfg.maximumMatchDistance^2;
        end
        dz=nan(numel(candidates),1);
        if size(f.mean,2)==3
            [dz,zVariance]=conditionalHeightResidual(f,m,i,candidates,means(i,:),pose(3));
            valid=valid & abs(dz)<=cfg.heightCompatibilitySigma*sqrt(zVariance);
        end
        distance(~valid)=Inf;
        [best,j]=min(distance);
        if ~isfinite(best), continue; end
        k=candidates(j); rows=rows+1;
        if ground
            p=f.normal(k,:)/sqrt(variance(j));
            whiten=[p;0 0];
        else
            whiten=chol([a(j) b(j);b(j) d(j)],'lower')\eye(2);
        end
        positionJacobian=[eye(2),r*[-m.mean(i,2);m.mean(i,1)]];
        jacobian(:,:,rows)=whiten*positionJacobian*diag(scale);
        residual(:,rows)=whiten*delta(j,:).'; precision(:,:,rows)=whiten;
        source(rows)=i; target(rows)=k; zResidual(rows)=dz(j);
        weights(rows)=m.quality(i)*f.quality(k);
        qvalue(rows)=sum(residual(:,rows).^2);
    end
    source=source(1:rows); target=target(1:rows); weights=weights(1:rows);
    qvalue=qvalue(1:rows); residual=residual(:,1:rows); jacobian=jacobian(:,:,1:rows);
    names=m.semanticName(source); classes=intersect(unique(f.semanticName),unique(m.semanticName));
    similarity=0;
    for name=classes.'
        selected=names==name; possible=m.semanticName==name & m.quality>0;
        weights(selected)=weights(selected)/max(sum(weights(selected)),eps)/max(1,numel(classes));
        coverage=nnz(selected)/max(1,nnz(possible));
        similarity=similarity+coverage*sum(weights(selected).*exp(-qvalue(selected)/2));
    end
    robust=1./(1+qvalue/cfg.robustStandardizedDistance^2);
    h=zeros(3); gradient=zeros(3,1);
    for j=1:rows
        a=jacobian(:,:,j); w=weights(j)*robust(j);
        h=h+w*(a.'*a); gradient=gradient+w*a.'*residual(:,j);
    end
    pairs=table(source,target,names,qvalue,zResidual(1:rows), ...
        'VariableNames',{'source','target','semanticName','squaredStandardizedResidual','heightResidual'});
    system=struct('H',h,'gradient',gradient,'numPairs',rows,'pairs',pairs, ...
        'J',jacobian,'residual',residual,'weights',weights,'robust',robust, ...
        'precision',precision(:,:,1:rows),'targetMean',f.mean(target,1:2), ...
        'cost',sum(weights.*cfg.robustStandardizedDistance^2.*log1p(qvalue/cfg.robustStandardizedDistance^2)), ...
        'similarity',similarity);
end

function cost=frozenCost(system,m,pose,cfg)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    delta=m.mean(system.pairs.source,1:2)*r.'+pose(1:2)-system.targetMean;
    q=zeros(system.numPairs,1);
    for i=1:system.numPairs, q(i)=sum((system.precision(:,:,i)*delta(i,:).').^2); end
    cost=sum(system.weights.*cfg.robustStandardizedDistance^2.*log1p(q/cfg.robustStandardizedDistance^2));
end

function [step,projector,rank,eigenvalues]=observableStep(h,gradient,ratio)
    [v,d]=eig((h+h.')/2,'vector'); eigenvalues=d;
    keep=d>max(1e-8,ratio*max(d)); rank=nnz(keep);
    projector=v(:,keep)*v(:,keep).';
    step=-v(:,keep)*((v(:,keep).'*gradient)./d(keep));
end

function diagnostics=classDiagnostics(system,cfg)
    classes=unique(system.pairs.semanticName); correction=zeros(numel(classes),1);
    ranks=zeros(numel(classes),1); matches=zeros(numel(classes),1);
    for c=1:numel(classes)
        h=zeros(3); gradient=zeros(3,1); selected=find(system.pairs.semanticName==classes(c));
        for j=selected.'
            a=system.J(:,:,j); w=system.weights(j)*system.robust(j);
            h=h+w*(a.'*a); gradient=gradient+w*a.'*system.residual(:,j);
        end
        [step,~,ranks(c)]=observableStep(h,gradient,cfg.minimumObservabilityRatio);
        correction(c)=norm(step); matches(c)=numel(selected);
    end
    diagnostics=table(classes,matches,ranks,correction, ...
        'VariableNames',{'semanticName','matchedComponents','observableRank','observableCorrection'});
end

function [normal,major,eligible]=normalGeometry(f,cfg)
    normal=zeros(f.numComponents,2); major=zeros(f.numComponents,1); eligible=false(f.numComponents,1);
    for i=1:f.numComponents
        [v,d]=eig(f.covariance(1:2,1:2,i),'vector');
        [minor,k]=min(d); major(i)=max(d); normal(i,:)=v(:,k).';
        eligible(i)=major(i)>=cfg.minimumLineAnisotropy*minor;
    end
end

function q=quality(c)
    q=ones(c.numComponents,1);
    if isfield(c,'semanticProbability'), q=double(c.semanticProbability(:)); end
    if isfield(c,'occupancyProbability'), q=q.*double(c.occupancyProbability(:)); end
    if isfield(c,'supportAmplitude')
        q=double(c.supportAmplitude(:));
        assert(numel(q)==c.numComponents && all(isfinite(q)&q>=0), ...
            'VehicleLocalization:InvalidSemanticQuality','Invalid support amplitude.');
        for name=unique(c.semanticName).'
            keep=c.semanticName==name; q(keep)=q(keep)/max(max(q(keep)),eps);
        end
    end
    assert(numel(q)==c.numComponents && all(isfinite(q)&q>=0&q<=1), ...
        'VehicleLocalization:InvalidSemanticQuality','Invalid semantic/occupancy probability.');
    q(c.mixtureWeight<=0)=0;
end

function f=prepareConditionalHeight(f)
    n=f.numComponents; f.heightSlope=zeros(n,2); f.conditionalVariance=zeros(n,1);
    for k=1:n
        c=f.covariance(:,:,k); beta=c(1:2,1:2)\c(1:2,3);
        f.heightSlope(k,:)=beta.';
        f.conditionalVariance(k)=max(0,c(3,3)-c(3,1:2)*beta);
    end
end

function [residual,variance]=conditionalHeightResidual(f,m,source,targets,meanXY,yaw)
% Target conditional height at the moving mean; propagate full XYZ scatter.
    r=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
    cm=r*m.covariance(:,:,source)*r.';
    beta=f.heightSlope(targets,:);
    residual=f.mean(targets,3)+sum((meanXY-f.mean(targets,1:2)).*beta,2)-m.mean(source,3);
    direction=[-beta,ones(numel(targets),1)];
    variance=max(1e-10,f.conditionalVariance(targets)+sum((direction*cm).*direction,2));
end

function validateParameters(cfg,g)
    assert(isscalar(cfg.yawLeverArm)&&isfinite(cfg.yawLeverArm)&&cfg.yawLeverArm>0);
    assert(numel(cfg.maximumPoseCorrection)==3&&all(isfinite(cfg.maximumPoseCorrection)&cfg.maximumPoseCorrection>0));
    values=struct2array(g); assert(all(isfinite(values)&values>0),'Invalid geometric configuration.');
    assert(g.minimumObservabilityRatio<1&&g.minimumMatchFraction<=1&&g.minimumLineAnisotropy>1);
end
