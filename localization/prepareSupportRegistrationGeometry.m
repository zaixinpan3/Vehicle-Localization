function model=prepareSupportRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg)
% prepareSupportRegistrationGeometry Continuous partial-support Gaussian model.
% The target coordinates are relative to initialPose XY. Geometry information
% uses spatial scatter and class-balanced weights, not calibrated pose noise.
% Every semantic class uses the same latent sliding displacement and angular
% factor. Anisotropy continuously relaxes long-axis position and supplies yaw.
    [f,m,height]=registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose=double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)),'Expected finite [x y yaw].');
    gcfg=cfg.geometric;gcfg.support=cfg.support;
    validateParameters(cfg,gcfg);
    m.quality=quality(m);
    f.viewReliability=ones(f.numComponents,1);
    if isfield(fixedCloud.components,'viewReliability')
        reliability=double(fixedCloud.components.viewReliability(:));
        assert(numel(reliability)==f.numComponents && isreal(reliability)&& ...
            all(isfinite(reliability)&reliability>=0&reliability<=1), ...
            'VehicleLocalization:InvalidViewReliability','Map view reliability must be in [0,1].');
        f.viewReliability=reliability;
    end

    m.temporalStability=ones(m.numComponents,1);
    if isfield(movingCloud.components,'temporalStability')
        m.temporalStability=double(movingCloud.components.temporalStability(:));
        assert(numel(m.temporalStability)==m.numComponents && isreal(m.temporalStability) && ...
            all(isfinite(m.temporalStability)&m.temporalStability>=0&m.temporalStability<=1), ...
            'VehicleLocalization:InvalidTemporalStability','Temporal stability must be in [0,1].');
        m.quality(m.temporalStability==0)=0;
    end
    % Height/tilt uncertainty belongs to the compatibility calculation. Keep
    % the planar metric identical when only height mode/reference changes.
    m.planarCovariance=movingCloud.components.covariance(1:2,1:2,:);
    [f,m]=supportRegistrationGeometry(f,m,gcfg.support);
    f.planarCovariance=f.covariance(1:2,1:2,:);
    f.mean(:,1:2)=f.mean(:,1:2)-initialPose(1:2);
    if height.heightUsed
        f.mean(:,3)=f.mean(:,3)-height.heightTranslation;
        m.mean(:,3)=m.mean(:,3)-height.heightTranslation;
    end
    height.strategy="conditionalDistributionCompatibility";
    height.planarForce=false;
    if height.heightUsed, f=prepareConditionalHeight(f); end
    groups=correspondenceGroups(f,m);
    association=[];
    if isfield(cfg,'softPointAssociation')
        association=cfg.softPointAssociation;
        validateSoftPointAssociation(association);
    end
    relative=[];
    if isfield(cfg,'relativeHeight') && cfg.relativeHeight.enabled
        assert(~height.heightUsed,'VehicleLocalization:ConflictingHeightModes', ...
            'Choose relative-height evidence or externally referenced XYZ compatibility.');
        initial=linearize(f,m,[0 0 initialPose(3)],gcfg,[1;1;1/cfg.yawLeverArm],groups,[],association);
        relative=prepareRelativeHeightAssociation(fixedCloud,movingCloud,initialPose,initial.pairs,cfg.relativeHeight);
        height.relativeAssociation=relative.details;
    end
    model=struct('fixed',f,'moving',m,'height',height,'origin',initialPose(1:2));
    model.linearize=@(pose,scale) linearize(f,m,pose,gcfg,scale,groups,relative,association);
    model.squaredResidual=@(system,pose) squaredResidual(system,m,pose);
    model.frozenCost=@(system,pose) sum(system.weights.*gcfg.robustStandardizedDistance^2.* ...
        log1p(squaredResidual(system,m,pose)/gcfg.robustStandardizedDistance^2));
end

function groups=correspondenceGroups(f,m)
% Class membership and positive-mass targets are invariant during one solve.
    names=intersect(unique(f.semanticName),unique(m.semanticName));
    groups=repmat(struct('source',[],'target',[],'priorCost',[]),numel(names),1);
    for k=1:numel(names)
        groups(k).source=find(m.semanticName==names(k) & m.quality>0);
        keep=f.semanticName==names(k) & f.mixtureWeight>0;
        groups(k).target=find(keep);
        prior=f.mixtureWeight(keep);
        % Subtracting a common log mass preserves the MAP assignment while
        % making global map-weight scaling immaterial. No new stability score.
        groups(k).priorCost=-2*(log(prior)-log(max(prior)));
    end
end

function system=linearize(f,m,pose,cfg,scale,groups,relative,association)
% Compare each semantic class in a matrix, preserving first-target tie order.
% Workspace scales with one class rather than the full all-class product.
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    means=m.mean(:,1:2)*r.'+pose(1:2);
    n=m.numComponents;
    rotated=pagemtimes(pagemtimes(r,m.planarCovariance),r.');
    chosen=zeros(n,1); chosenZ=nan(n,1); chosenHeightCost=zeros(n,1);
    softMean=zeros(n,2);softCov=zeros(2,2,n);softUsed=false(n,1);softCount=ones(n,1);
    for group=groups.'
        source=group.source; targets=group.target;
        if isempty(source) || isempty(targets), continue; end
        % Bound temporary pair matrices for larger externally supplied maps.
        allSource=source; blockSize=max(1,floor(65536/numel(targets)));
        for first=1:blockSize:numel(allSource)
            source=allSource(first:min(first+blockSize-1,numel(allSource)));
            dx=means(source,1).'-f.mean(targets,1);
            dy=means(source,2).'-f.mean(targets,2);
            cf=f.planarCovariance(:,:,targets)+f.slidingCovariance(:,:,targets);
            cm=rotated(:,:,source);
            a=reshape(cf(1,1,:),[],1)+reshape(cm(1,1,:),1,[])+cfg.noiseStandardDeviation^2;
            b=reshape(cf(1,2,:),[],1)+reshape(cm(1,2,:),1,[]);
            d=reshape(cf(2,2,:),[],1)+reshape(cm(2,2,:),1,[])+cfg.noiseStandardDeviation^2;
            determinant=a.*d-b.^2;
            positionCost=(d.*dx.^2-2*b.*dx.*dy+a.*dy.^2)./determinant;
            sourceAxis=m.supportTangent(source,:)*r.';
            angular=(f.orientationNormal(targets,:)*sourceAxis.').^2.* ...
                (f.axisConfidence(targets)*m.axisConfidence(source).')./ ...
                (cfg.support.angularFloor^2+f.angularVariance(targets)+m.angularVariance(source).');
            tangentOffset=-dx.*f.supportNormal(targets,2)+dy.*f.supportNormal(targets,1);
            distance=positionCost+angular+tangentOffset.^2.*f.slidingFraction(targets)/cfg.maximumMatchDistance^2;
            % A common covariance-aware gate admits sliding along an extended
            % cloud without switching any class to a line model.
            ga=9*a+cfg.maximumMatchDistance^2;gb=9*b;gd=9*d+cfg.maximumMatchDistance^2;
            valid=(gd.*dx.^2-2*gb.*dx.*dy+ga.*dy.^2)<=ga.*gd-gb.^2;
            dz=nan(size(distance));
            if size(f.mean,2)==3
                for k=1:numel(source)
                    [dz(:,k),zVariance]=conditionalHeightResidual(f,m,source(k),targets,means(source(k),:),pose(3));
                    valid(:,k)=valid(:,k) & abs(dz(:,k))<=cfg.heightCompatibilitySigma*sqrt(zVariance);
                end
            end
            heightCost=zeros(size(distance));
            if ~isempty(relative) && relative.details.enabled
                [heightCost,dz]=relative.cost(source,targets,pose);
                distance=distance+heightCost;
            end
            distance=distance+group.priorCost;
            distance(~valid)=Inf;
            [best,index]=min(distance,[],1);
            keep=isfinite(best); selected=source(keep);
            chosen(selected)=targets(index(keep));
            linear=index(keep)+(find(keep)-1)*numel(targets);
            chosenZ(selected)=dz(linear);
            chosenHeightCost(selected)=heightCost(linear);
            if ~isempty(association)
                for col=find(keep)
                    sourceId=source(col);
                    localAssociation=association;
                    localAssociation.temperature=association.temperature* ...
                        max(1e-6,1-f.slidingFraction(targets(index(col))))^2;
                    [mu,scatter,count]=softPointAssociationTarget( ...
                        f.mean(targets,1:2),f.planarCovariance(:,:,targets), ...
                        distance(:,col),group.priorCost+heightCost(:,col),index(col),localAssociation);
                    softMean(sourceId,:)=mu;softCov(:,:,sourceId)=scatter;
                    softUsed(sourceId)=true;softCount(sourceId)=count;
                end
            end
        end
    end
    source=find(chosen); target=chosen(source); rows=numel(source);
    cf=f.planarCovariance(:,:,target);targetMean=f.mean(target,1:2);
    soft=softUsed(source);cf(:,:,soft)=softCov(:,:,source(soft));targetMean(soft,:)=softMean(source(soft),:);
    slidingCoverage=max(0,1-m.supportMajor(source)./f.supportMajor(target)).^cfg.support.coveragePower;
    slidingCoverage=max(slidingCoverage,f.slidingFraction(target).^cfg.support.directionPower);
    partial=partialSupport(f,m,means,chosen,cfg.support);
    slidingCoverage(partial(source))=1;
    cf=cf+f.slidingCovariance(:,:,target).*reshape(slidingCoverage,1,1,[]);
    directionScale=sqrt(f.axisConfidence(target).*m.axisConfidence(source)./ ...
        (cfg.support.angularFloor^2+f.angularVariance(target)+m.angularVariance(source)));
    [residual,jacobian,precision]=supportRegistrationResiduals( ...
        m.mean(source,1:2),m.planarCovariance(:,:,source),targetMean,cf, ...
        m.supportTangent(source,:),f.orientationNormal(target,:),directionScale,pose,cfg.noiseStandardDeviation);
    shapeUsed=directionScale>0;
    jacobian=jacobian.*reshape(scale,1,3,1);
    weights=m.quality(source);
    qvalue=sum(residual.^2,1).'; zResidual=chosenZ(source);
    names=m.semanticName(source); classes=intersect(unique(f.semanticName),unique(m.semanticName));
    similarity=0;
    for name=classes.'
        selected=names==name; possible=m.semanticName==name & m.quality>0;
        weights(selected)=weights(selected)/max(sum(weights(selected)),eps)/max(1,numel(classes));
        % Apply after class balancing: sparse temporal evidence must not be
        % normalized back to the influence of a fully repeated class.
        weights(selected)=weights(selected).*m.temporalStability(source(selected));
        % View coverage is an absolute target-support factor. Applying it
        % after class balancing avoids restoring an unsupported class's force.
        weights(selected)=weights(selected).*f.viewReliability(target(selected));
        coverage=nnz(selected)/max(1,nnz(possible));
        similarity=similarity+coverage*sum(weights(selected).*exp(-qvalue(selected)/2));
    end
    robust=1./(1+qvalue/cfg.robustStandardizedDistance^2);
    stacked=reshape(permute(jacobian,[1 3 2]),[],3);
    % Preserve a column even when exactly one correspondence remains.
    rowWeights=repelem(weights.*robust,size(residual,1),1);
    h=stacked.'*(stacked.*rowWeights);
    gradient=stacked.'*(residual(:).*rowWeights);
    pairs=struct('source',source,'target',target,'semanticName',names, ...
        'squaredStandardizedResidual',qvalue,'heightResidual',zResidual(1:rows), ...
        'heightAssociationCost',chosenHeightCost(source), ...
        'mapMixtureWeight',f.mixtureWeight(target),'temporalStability',m.temporalStability(source), ...
        'weight',weights,'robustWeight',weights.*robust, ...
        'shapeRotationUsed',shapeUsed,'shapeResidual',residual(3,:).', ...
        'slidingFraction',slidingCoverage,'partialSupport',partial(source));
    pairs.associationComponents=softCount(source);
    pairs.sourceMeanXY=m.mean(source,1:2);
    pairs.targetMeanXY=targetMean;
    pairs.targetCovarianceXY=[reshape(cf(1,1,:),[],1),reshape(cf(1,2,:),[],1),reshape(cf(2,2,:),[],1)];
    system=struct('H',h,'gradient',gradient,'numPairs',rows,'pairs',pairs, ...
        'J',jacobian,'residual',residual,'weights',weights,'robust',robust, ...
        'precision',precision(:,:,1:rows),'targetMean',targetMean, ...
        'targetCovariance',cf,'noiseStandardDeviation',cfg.noiseStandardDeviation, ...
        'sourceTangent',m.supportTangent(source,:),'targetNormal',f.orientationNormal(target,:),'directionScale',directionScale, ...
        'cost',sum(weights.*cfg.robustStandardizedDistance^2.*log1p(qvalue/cfg.robustStandardizedDistance^2)), ...
        'similarity',similarity);
end

function q=squaredResidual(system,m,pose)
% Freeze associations and weights, but rotate source scatter at every trial.
    residual=supportRegistrationResiduals(m.mean(system.pairs.source,1:2), ...
        m.planarCovariance(:,:,system.pairs.source),system.targetMean, ...
        system.targetCovariance,system.sourceTangent,system.targetNormal,system.directionScale,pose,system.noiseStandardDeviation);
    q=sum(residual.^2,1).';
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
    values=struct2array(rmfield(g,'support')); assert(all(isfinite(values)&values>0),'Invalid geometric configuration.');
    assert(g.minimumObservabilityRatio<1&&g.minimumMatchFraction<=1);
end

function partial=partialSupport(f,m,means,chosen,cfg)
% Repeated off-center patches retain surface support without center attraction.
    partial=false(m.numComponents,1);
    for target=unique(chosen(chosen>0)).'
        ids=find(chosen==target);if numel(ids)<3,continue;end
        normal=f.supportNormal(target,:);tangent=[-normal(2),normal(1)];
        positions=means(ids,:)*tangent.';normalPositions=means(ids,:)*normal.';
        neighbors=abs(positions-positions.')<=cfg.consensusRadius;
        quality=m.quality(ids).*m.temporalStability(ids);
        mass=neighbors*quality;mass(sum(neighbors,2)<2)=0;
        [support,best]=max(mass);if support<=0,continue;end
        coherent=neighbors(best,:).';
        if support<=sum(quality(~coherent)),continue;end
        center=sum(normalPositions(coherent).*quality(coherent))/sum(quality(coherent));
        compatible=abs(normalPositions-center)<=cfg.normalConsensus;
        partial(ids(~coherent & compatible))=true;
    end
end
