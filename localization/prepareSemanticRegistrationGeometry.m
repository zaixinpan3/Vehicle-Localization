function model=prepareSemanticRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg)
% prepareSemanticRegistrationGeometry Shared Gaussian association and residual model.
% The target coordinates are relative to initialPose XY. Geometry information
% uses spatial scatter and class-balanced weights, not calibrated pose noise.
% Dispatch supportD2D and full-overlap models before the legacy implementation.
% Legacy curb/facade neighborhoods add line-direction yaw information without
% inventing a line-tangent position residual.
    if string(cfg.method)=="supportD2D"
        model=prepareSupportRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg);
        return;
    end
    if string(cfg.method)=="anisotropicD2D"
        model=prepareAnisotropicRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg);
        return;
    end
    [f,m,height]=registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose=double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)),'Expected finite [x y yaw].');
    gcfg=cfg.geometric;
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
    m.lineTangent=zeros(m.numComponents,2);m.lineDirectionValid=false(m.numComponents,1);
    m.lineDirectionSigma=ones(m.numComponents,1);
    if isfield(cfg,'lineDirection') && cfg.lineDirection.enabled
        [m.lineTangent,m.lineDirectionValid,scatter]=registrationSupport.sourceLineDirections(m,cfg.lineDirection);
        m.lineDirectionSigma(:)=cfg.lineDirection.standardDeviation;
        if isfield(cfg.lineDirection,'scatterScale')
            assert(isscalar(cfg.lineDirection.scatterScale) && isreal(cfg.lineDirection.scatterScale) && ...
                isfinite(cfg.lineDirection.scatterScale) && cfg.lineDirection.scatterScale>=0, ...
                'VehicleLocalization:InvalidLineDirectionConfiguration','Scatter scale must be finite and nonnegative.');
            m.lineDirectionSigma=hypot(m.lineDirectionSigma,cfg.lineDirection.scatterScale*scatter);
        end
    end
    m.planarCovariance=movingCloud.components.covariance(1:2,1:2,:);
    f.mean(:,1:2)=f.mean(:,1:2)-initialPose(1:2);
    if height.heightUsed
        f.mean(:,3)=f.mean(:,3)-height.heightTranslation;
        m.mean(:,3)=m.mean(:,3)-height.heightTranslation;
    end
    height.strategy="conditionalDistributionCompatibility";
    height.planarForce=false;
    [f.normal,f.majorVariance,f.lineEligible]=normalGeometry(f,gcfg);
    f.surfaceEligible=false(f.numComponents,1);
    if isfield(cfg,'partialSign'),validatePartialSign(cfg.partialSign);end
    if isfield(cfg,'partialSign') && cfg.partialSign.enabled && isfield(f,'intrinsicCovariance')
        S=f.intrinsicCovariance;
        assert(isnumeric(S)&&isreal(S)&&size(S,1)==2&&size(S,2)==2&&size(S,3)==f.numComponents && ...
            all(isfinite(S),'all')&&all(abs(S-permute(S,[2 1 3]))<1e-9,'all'), ...
            'VehicleLocalization:InvalidIntrinsicShape','Intrinsic XY scatter must be finite, symmetric and aligned.');
        f.partialSignConfig=cfg.partialSign;
        for id=find(f.semanticName=="trafficSign").'
            [v,e]=eig(f.intrinsicCovariance(:,:,id),'vector');[minor,j]=min(e);major=max(e);
            assert(minor>=-1e-10,'VehicleLocalization:InvalidIntrinsicShape','Intrinsic scatter must be positive semidefinite.');
            if major>=cfg.partialSign.minimumVariance && major>=cfg.partialSign.minimumAnisotropy*max(minor,1e-8)
                f.normal(id,:)=v(:,j).';f.majorVariance(id)=major;f.surfaceEligible(id)=true;
            end
        end
    end
    if height.heightUsed, f=prepareConditionalHeight(f); end
    groups=correspondenceGroups(f,m);
    association=[];
    if isfield(cfg,'softPointAssociation')
        association=cfg.softPointAssociation;
        registrationSupport.validateSoftPointAssociation(association);
    end
    relative=[];
    if isfield(cfg,'relativeHeight') && cfg.relativeHeight.enabled
        assert(~height.heightUsed,'VehicleLocalization:ConflictingHeightModes', ...
            'Choose relative-height evidence or externally referenced XYZ compatibility.');
        initial=linearize(f,m,[0 0 initialPose(3)],gcfg,[1;1;1/cfg.yawLeverArm],groups,[],association);
        relative=registrationSupport.prepareRelativeHeightAssociation(fixedCloud,movingCloud,initialPose,initial.pairs,cfg.relativeHeight);
        height.relativeAssociation=relative.details;
    end
    model=struct('fixed',f,'moving',m,'height',height,'origin',initialPose(1:2));
    model.linearize=@(pose,scale) linearize(f,m,pose,gcfg,scale,groups,relative,association);
    model.squaredResidual=@(system,pose) squaredResidual(system,m,pose);
    model.frozenCost=@(system,pose) sum(system.weights.*gcfg.robustStandardizedDistance^2.* ...
        log1p(squaredResidual(system,m,pose)/gcfg.robustStandardizedDistance^2));
end

function validatePartialSign(cfg)
    fields=["minimumAnisotropy","minimumVariance","consensusRadius","maximumNormalDifference"];
    valid=isstruct(cfg)&&isscalar(cfg)&&all(isfield(cfg,[fields,"enabled"]));
    if valid
        valid=isscalar(cfg.enabled)&&islogical(cfg.enabled);
        for field=fields
            value=cfg.(field);valid=valid&&isnumeric(value)&&isscalar(value)&&isreal(value)&&isfinite(value)&&value>0;
        end
        valid=valid&&cfg.minimumAnisotropy>1;
    end
    assert(valid,'VehicleLocalization:InvalidPartialSignConfig','Require a logical switch and finite positive partial-sign geometry scales.');
end

function partial=partialSignSources(f,m,means,chosen)
% A coherent cluster retains center constraints; displaced portions constrain
% only the known panel normal. Repetition establishes existence, not center.
    partial=false(m.numComponents,1);cfg=f.partialSignConfig;
    for target=find(f.surfaceEligible).'
        ids=find(chosen==target);
        if numel(ids)<3,continue;end
        normal=f.normal(target,:);tangent=[-normal(2),normal(1)];
        positions=means(ids,:)*tangent.';normalPositions=means(ids,:)*normal.';
        neighbors=abs(positions-positions.')<=cfg.consensusRadius;
        quality=m.quality(ids).*m.temporalStability(ids);
        mass=neighbors*quality;mass(sum(neighbors,2)<2)=0;
        [support,best]=max(mass);if support<=0,continue;end
        coherent=neighbors(best,:).';
        if support<=sum(quality(~coherent)),continue;end
        center=sum(normalPositions(coherent).*quality(coherent))/sum(quality(coherent));
        delta=abs(normalPositions-center);
        compatible=delta<=cfg.maximumNormalDifference;
        partial(ids(~coherent & compatible))=true;
    end
end

function groups=correspondenceGroups(f,m)
% Class membership and eligible targets are invariant during one solve.
    names=intersect(unique(f.semanticName),unique(m.semanticName));
    groups=repmat(struct('source',[],'target',[],'priorCost',[],'line',false),numel(names),1);
    for k=1:numel(names)
        groups(k).line=any(names(k)==["curb","facade"]);
        groups(k).source=find(m.semanticName==names(k) & m.quality>0);
        keep=f.semanticName==names(k) & f.mixtureWeight>0;
        if groups(k).line, keep=keep & f.lineEligible; end
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
    chosen=zeros(n,1); chosenZ=nan(n,1); chosenHeightCost=zeros(n,1);lineSource=false(n,1);
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
            cf=f.covariance(1:2,1:2,targets);
            cm=rotated(:,:,source);
            a=reshape(cf(1,1,:),[],1)+reshape(cm(1,1,:),1,[])+cfg.noiseStandardDeviation^2;
            b=reshape(cf(1,2,:),[],1)+reshape(cm(1,2,:),1,[]);
            d=reshape(cf(2,2,:),[],1)+reshape(cm(2,2,:),1,[])+cfg.noiseStandardDeviation^2;
            if group.line
                normal=f.normal(targets,:);
                dn=dx.*normal(:,1)+dy.*normal(:,2);
                dt=-dx.*normal(:,2)+dy.*normal(:,1);
                variance=a.*normal(:,1).^2+2*b.*normal(:,1).*normal(:,2)+d.*normal(:,2).^2;
                distance=dn.^2./variance+dt.^2./(f.majorVariance(targets)+cfg.maximumMatchDistance^2);
                tangent=m.lineTangent(source,:)*r.';
                angular=(normal*tangent.').^2.*(double(m.lineDirectionValid(source))./m.lineDirectionSigma(source).^2).';
                distance=distance+angular;
                valid=abs(dn)<=cfg.maximumMatchDistance & ...
                    abs(dt)<=3*sqrt(f.majorVariance(targets))+cfg.maximumMatchDistance;
            else
                determinant=a.*d-b.^2;
                distance=(d.*dx.^2-2*b.*dx.*dy+a.*dy.^2)./determinant;
                detF=reshape(cf(1,1,:),[],1).*reshape(cf(2,2,:),[],1)-reshape(cf(1,2,:),[],1).^2;
                detM=reshape(cm(1,1,:).*cm(2,2,:)-cm(1,2,:).*cm(2,1,:),1,[]);
                distance=distance+max(0,log(determinant./(4*sqrt(detF*detM))));
                valid=dx.^2+dy.^2<=cfg.maximumMatchDistance^2;
            end
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
            lineSource(selected)=group.line;
            if ~group.line && ~isempty(association)
                for col=find(keep)
                    sourceId=source(col);
                    [mu,scatter,count]=registrationSupport.softPointAssociationTarget( ...
                        f.mean(targets,1:2),f.covariance(1:2,1:2,targets), ...
                        distance(:,col),group.priorCost+heightCost(:,col),index(col),association);
                    softMean(sourceId,:)=mu;softCov(:,:,sourceId)=scatter;
                    softUsed(sourceId)=true;softCount(sourceId)=count;
                end
            end
        end
    end
    if isfield(f,'partialSignConfig')
        partial=partialSignSources(f,m,means,chosen);
        lineSource(partial)=true;softUsed(partial)=false;
    end
    source=find(chosen); target=chosen(source); rows=numel(source);
    cm=rotated(:,:,source); cf=f.covariance(1:2,1:2,target);targetMean=f.mean(target,1:2);
    soft=softUsed(source);cf(:,:,soft)=softCov(:,:,source(soft));targetMean(soft,:)=softMean(source(soft),:);
    a=reshape(cf(1,1,:)+cm(1,1,:),[],1)+cfg.noiseStandardDeviation^2;
    b=reshape(cf(1,2,:)+cm(1,2,:),[],1);
    d=reshape(cf(2,2,:)+cm(2,2,:),[],1)+cfg.noiseStandardDeviation^2;
    line=lineSource(source); point=~line;
    w11=zeros(rows,1); w12=w11; w21=w11; w22=w11;
    normal=f.normal(target(line),:);
    variance=a(line).*normal(:,1).^2+2*b(line).*normal(:,1).*normal(:,2)+d(line).*normal(:,2).^2;
    w11(line)=normal(:,1)./sqrt(variance); w12(line)=normal(:,2)./sqrt(variance);
    % Explicit 2-by-2 Cholesky whitening for positive definite summed scatter.
    l11=sqrt(a(point)); l21=b(point)./l11; l22=sqrt(d(point)-l21.^2);
    w11(point)=1./l11; w21(point)=-l21./l11./l22; w22(point)=1./l22;
    precision=zeros(2,2,rows);
    precision(1,1,:)=w11; precision(1,2,:)=w12;
    precision(2,1,:)=w21; precision(2,2,:)=w22;
    yawDerivative=m.mean(source,1:2)*[r(:,2),-r(:,1)].';
    jacobian=zeros(2,3,rows);
    jacobian(1,1,:)=w11*scale(1); jacobian(1,2,:)=w12*scale(2);
    jacobian(2,1,:)=w21*scale(1); jacobian(2,2,:)=w22*scale(2);
    jacobian(1,3,:)=(w11.*yawDerivative(:,1)+w12.*yawDerivative(:,2))*scale(3);
    jacobian(2,3,:)=(w21.*yawDerivative(:,1)+w22.*yawDerivative(:,2))*scale(3);
    delta=means(source,:)-targetMean;
    residual=[w11.*delta(:,1)+w12.*delta(:,2),w21.*delta(:,1)+w22.*delta(:,2)].';
    tangent=m.lineTangent(source,:);normals=f.normal(target,:);
    directionValid=line & m.lineDirectionValid(source);
    directionScale=double(directionValid)./m.lineDirectionSigma(source);
    rotatedTangent=tangent*r.';
    directionResidual=sum(rotatedTangent.*normals,2).*directionScale;
    if any(directionValid)
        residual(3,:)=directionResidual.';
        jacobian(3,3,:)=sum((tangent*[r(:,2),-r(:,1)].').*normals,2).*directionScale*scale(3);
    end
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
        'lineDirectionUsed',directionValid,'lineDirectionResidual',directionResidual);
    pairs.associationComponents=softCount(source);
    pairs.partialSignSurface=f.surfaceEligible(target) & line;
    pairs.sourceMeanXY=m.mean(source,1:2);
    pairs.targetMeanXY=targetMean;
    pairs.targetCovarianceXY=[reshape(cf(1,1,:),[],1),reshape(cf(1,2,:),[],1),reshape(cf(2,2,:),[],1)];
    system=struct('H',h,'gradient',gradient,'numPairs',rows,'pairs',pairs, ...
        'J',jacobian,'residual',residual,'weights',weights,'robust',robust, ...
        'precision',precision(:,:,1:rows),'targetMean',targetMean, ...
        'directionNormal',normals,'directionScale',directionScale, ...
        'cost',sum(weights.*cfg.robustStandardizedDistance^2.*log1p(qvalue/cfg.robustStandardizedDistance^2)), ...
        'similarity',similarity);
end

function q=squaredResidual(system,m,pose)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    delta=m.mean(system.pairs.source,1:2)*r.'+pose(1:2)-system.targetMean;
    p=system.precision;
    rx=reshape(p(1,1,:),[],1).*delta(:,1)+reshape(p(1,2,:),[],1).*delta(:,2);
    ry=reshape(p(2,1,:),[],1).*delta(:,1)+reshape(p(2,2,:),[],1).*delta(:,2);
    direction=sum((m.lineTangent(system.pairs.source,:)*r.').*system.directionNormal,2).*system.directionScale;
    q=rx.^2+ry.^2+direction.^2;
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
    supportValidateParameters(cfg,gcfg);
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
    groups=supportCorrespondenceGroups(f,m);
    association=[];
    if isfield(cfg,'softPointAssociation')
        association=cfg.softPointAssociation;
        registrationSupport.validateSoftPointAssociation(association);
    end
    relative=[];
    if isfield(cfg,'relativeHeight') && cfg.relativeHeight.enabled
        assert(~height.heightUsed,'VehicleLocalization:ConflictingHeightModes', ...
            'Choose relative-height evidence or externally referenced XYZ compatibility.');
        initial=supportLinearize(f,m,[0 0 initialPose(3)],gcfg,[1;1;1/cfg.yawLeverArm],groups,[],association);
        relative=registrationSupport.prepareRelativeHeightAssociation(fixedCloud,movingCloud,initialPose,initial.pairs,cfg.relativeHeight);
        height.relativeAssociation=relative.details;
    end
    model=struct('fixed',f,'moving',m,'height',height,'origin',initialPose(1:2));
    model.linearize=@(pose,scale) supportLinearize(f,m,pose,gcfg,scale,groups,relative,association);
    model.squaredResidual=@(system,pose) supportSquaredResidual(system,m,pose);
    model.frozenCost=@(system,pose) sum(system.weights.*gcfg.robustStandardizedDistance^2.* ...
        log1p(supportSquaredResidual(system,m,pose)/gcfg.robustStandardizedDistance^2));
end

function groups=supportCorrespondenceGroups(f,m)
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

function system=supportLinearize(f,m,pose,cfg,scale,groups,relative,association)
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
                    [mu,scatter,count]=registrationSupport.softPointAssociationTarget( ...
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
    [residual,jacobian,precision]=registrationSupport.supportRegistrationResiduals( ...
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

function q=supportSquaredResidual(system,m,pose)
% Freeze associations and weights, but rotate source scatter at every trial.
    residual=registrationSupport.supportRegistrationResiduals(m.mean(system.pairs.source,1:2), ...
        m.planarCovariance(:,:,system.pairs.source),system.targetMean, ...
        system.targetCovariance,system.sourceTangent,system.targetNormal,system.directionScale,pose,system.noiseStandardDeviation);
    q=sum(residual.^2,1).';
end

function supportValidateParameters(cfg,g)
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

function model=prepareAnisotropicRegistrationGeometry(fixedCloud,movingCloud,initialPose,cfg)
% prepareAnisotropicRegistrationGeometry Full Gaussian association and residual model.
% The target coordinates are relative to initialPose XY. Geometry information
% uses spatial scatter and class-balanced weights, not calibrated pose noise.
% Every semantic class uses normalized Gaussian overlap with the same complete
% covariance metric. Anisotropy controls position and shape forces continuously.
    [f,m,height]=registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    initialPose=double(initialPose(:).');
    assert(numel(initialPose)==3 && all(isfinite(initialPose)),'Expected finite [x y yaw].');
    gcfg=cfg.geometric;
    anisotropicValidateParameters(cfg,gcfg);
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
    f.planarCovariance=f.covariance(1:2,1:2,:);
    f.mean(:,1:2)=f.mean(:,1:2)-initialPose(1:2);
    if height.heightUsed
        f.mean(:,3)=f.mean(:,3)-height.heightTranslation;
        m.mean(:,3)=m.mean(:,3)-height.heightTranslation;
    end
    height.strategy="conditionalDistributionCompatibility";
    height.planarForce=false;
    if height.heightUsed, f=prepareConditionalHeight(f); end
    groups=supportCorrespondenceGroups(f,m);
    association=[];
    if isfield(cfg,'softPointAssociation')
        association=cfg.softPointAssociation;
        registrationSupport.validateSoftPointAssociation(association);
    end
    relative=[];
    if isfield(cfg,'relativeHeight') && cfg.relativeHeight.enabled
        assert(~height.heightUsed,'VehicleLocalization:ConflictingHeightModes', ...
            'Choose relative-height evidence or externally referenced XYZ compatibility.');
        initial=anisotropicLinearize(f,m,[0 0 initialPose(3)],gcfg,[1;1;1/cfg.yawLeverArm],groups,[],association);
        relative=registrationSupport.prepareRelativeHeightAssociation(fixedCloud,movingCloud,initialPose,initial.pairs,cfg.relativeHeight);
        height.relativeAssociation=relative.details;
    end
    model=struct('fixed',f,'moving',m,'height',height,'origin',initialPose(1:2));
    model.linearize=@(pose,scale) anisotropicLinearize(f,m,pose,gcfg,scale,groups,relative,association);
    model.squaredResidual=@(system,pose) anisotropicSquaredResidual(system,m,pose);
    model.frozenCost=@(system,pose) sum(system.weights.*gcfg.robustStandardizedDistance^2.* ...
        log1p(anisotropicSquaredResidual(system,m,pose)/gcfg.robustStandardizedDistance^2));
end

function system=anisotropicLinearize(f,m,pose,cfg,scale,groups,relative,association)
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
            cf=f.planarCovariance(:,:,targets);
            cm=rotated(:,:,source);
            a=reshape(cf(1,1,:),[],1)+reshape(cm(1,1,:),1,[])+cfg.noiseStandardDeviation^2;
            b=reshape(cf(1,2,:),[],1)+reshape(cm(1,2,:),1,[]);
            d=reshape(cf(2,2,:),[],1)+reshape(cm(2,2,:),1,[])+cfg.noiseStandardDeviation^2;
            determinant=a.*d-b.^2;
            positionCost=(d.*dx.^2-2*b.*dx.*dy+a.*dy.^2)./determinant;
            noise=cfg.noiseStandardDeviation^2/2;
            detF=(reshape(cf(1,1,:),[],1)+noise).*(reshape(cf(2,2,:),[],1)+noise)-reshape(cf(1,2,:),[],1).^2;
            detM=(reshape(cm(1,1,:),1,[])+noise).*(reshape(cm(2,2,:),1,[])+noise)-reshape(cm(1,2,:),1,[]).^2;
            distance=positionCost+max(0,log(determinant)-log(4)-.5*(log(detF)+log(detM)));
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
                    [mu,scatter,count]=registrationSupport.softPointAssociationTarget( ...
                        f.mean(targets,1:2),f.planarCovariance(:,:,targets), ...
                        distance(:,col),group.priorCost+heightCost(:,col),index(col),association);
                    softMean(sourceId,:)=mu;softCov(:,:,sourceId)=scatter;
                    softUsed(sourceId)=true;softCount(sourceId)=count;
                end
            end
        end
    end
    source=find(chosen); target=chosen(source); rows=numel(source);
    cf=f.planarCovariance(:,:,target);targetMean=f.mean(target,1:2);
    soft=softUsed(source);cf(:,:,soft)=softCov(:,:,source(soft));targetMean(soft,:)=softMean(source(soft),:);
    [residual,jacobian,precision,shapeUsed]=registrationSupport.gaussianRegistrationResiduals( ...
        m.mean(source,1:2),m.planarCovariance(:,:,source),targetMean,cf,pose,cfg.noiseStandardDeviation);
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
        'shapeRotationUsed',shapeUsed,'shapeResidual',sqrt(sum(residual(3:4,:).^2,1)).');
    pairs.associationComponents=softCount(source);
    pairs.sourceMeanXY=m.mean(source,1:2);
    pairs.targetMeanXY=targetMean;
    pairs.targetCovarianceXY=[reshape(cf(1,1,:),[],1),reshape(cf(1,2,:),[],1),reshape(cf(2,2,:),[],1)];
    system=struct('H',h,'gradient',gradient,'numPairs',rows,'pairs',pairs, ...
        'J',jacobian,'residual',residual,'weights',weights,'robust',robust, ...
        'precision',precision(:,:,1:rows),'targetMean',targetMean, ...
        'targetCovariance',cf,'noiseStandardDeviation',cfg.noiseStandardDeviation, ...
        'cost',sum(weights.*cfg.robustStandardizedDistance^2.*log1p(qvalue/cfg.robustStandardizedDistance^2)), ...
        'similarity',similarity);
end

function q=anisotropicSquaredResidual(system,m,pose)
% Freeze associations and weights, but rotate source scatter at every trial.
    residual=registrationSupport.gaussianRegistrationResiduals(m.mean(system.pairs.source,1:2), ...
        m.planarCovariance(:,:,system.pairs.source),system.targetMean, ...
        system.targetCovariance,pose,system.noiseStandardDeviation);
    q=sum(residual.^2,1).';
end

function anisotropicValidateParameters(cfg,g)
    assert(isscalar(cfg.yawLeverArm)&&isfinite(cfg.yawLeverArm)&&cfg.yawLeverArm>0);
    assert(numel(cfg.maximumPoseCorrection)==3&&all(isfinite(cfg.maximumPoseCorrection)&cfg.maximumPoseCorrection>0));
    values=struct2array(g); assert(all(isfinite(values)&values>0),'Invalid geometric configuration.');
    assert(g.minimumObservabilityRatio<1&&g.minimumMatchFraction<=1);
end

function [f,m]=supportRegistrationGeometry(f,m,cfg)
% supportRegistrationGeometry Estimate geometric support without class rules.
% A latent displacement along an elongated target marginalizes partial-view
% center drift. Neighborhood moments supply orientation at extended support.
    values=[cfg.slidingPower,cfg.angularFloor,cfg.scatterScale,cfg.neighborhoodRadius, ...
        cfg.minimumSpan,cfg.directionPower,cfg.coveragePower,cfg.consensusRadius,cfg.normalConsensus,cfg.maximumGap];
    assert(isreal(values)&&all(isfinite(values)&values>0)&&cfg.angularFloor<pi/2 && ...
        isscalar(cfg.directionResolution)&&isreal(cfg.directionResolution)&& ...
        isfinite(cfg.directionResolution)&&cfg.directionResolution>=0, ...
        'VehicleLocalization:InvalidSupportGeometry','Require finite positive support geometry scales.');
    count=f.numComponents;f.slidingCovariance=zeros(2,2,count);
    if isfield(f,'intrinsicCovariance')
        C=f.intrinsicCovariance;
        assert(isnumeric(C)&&isreal(C)&&size(C,1)==2&&size(C,2)==2&&size(C,3)==count && ...
            all(isfinite(C),'all')&&all(abs(C-permute(C,[2 1 3]))<1e-9,'all'), ...
            'VehicleLocalization:InvalidIntrinsicShape','Require aligned finite symmetric intrinsic scatter.');
    end
    f.supportNormal=zeros(count,2);f.orientationNormal=zeros(count,2);f.axisConfidence=zeros(count,1);
    f.angularVariance=zeros(count,1);f.slidingFraction=zeros(count,1);f.supportMajor=zeros(count,1);
    for k=1:count
        C=f.covariance(1:2,1:2,k);
        if isfield(f,'intrinsicCovariance') && any(f.intrinsicCovariance(:,:,k),'all')
            C=f.intrinsicCovariance(:,:,k);
        end
        [~,~,~,intrinsicConfidence]=geometry(C);
        orientation=orientationMoment(f,k,C,cfg);
        [axis,minor,major,confidence]=geometry(orientation);
        f.supportNormal(k,:)=[-axis(2),axis(1)];
        f.orientationNormal(k,:)=f.supportNormal(k,:);
        f.axisConfidence(k)=intrinsicConfidence*confidence^(cfg.directionPower/2)*major/(major+cfg.directionResolution^2);
        f.angularVariance(k)=cfg.scatterScale^2*minor/major;
        f.slidingFraction(k)=intrinsicConfidence*confidence;f.supportMajor(k)=major;
        variance=intrinsicConfidence*(major-minor)*((major/minor)^cfg.slidingPower-1);
        f.slidingCovariance(:,:,k)=variance*(axis.'*axis);
    end
    count=m.numComponents;m.supportTangent=zeros(count,2);
    m.axisConfidence=zeros(count,1);m.angularVariance=zeros(count,1);m.supportMajor=zeros(count,1);
    for k=1:count
        C=m.planarCovariance(:,:,k);
        m.supportMajor(k)=max(eig(C));
        C=orientationMoment(m,k,C,cfg);
        [axis,minor,major,confidence]=geometry(C);
        m.supportTangent(k,:)=axis;m.axisConfidence(k)=confidence^(cfg.directionPower/2)*major/(major+cfg.directionResolution^2);
        m.angularVariance(k)=cfg.scatterScale^2*minor/major;
    end
end

function [axis,minor,major,confidence]=geometry(C)
    [v,e]=eig((C+C.')/2,'vector');[major,k]=max(e);minor=max(min(e),1e-6);
    assert(min(e)>=-1e-10,'VehicleLocalization:InvalidIntrinsicShape','Scatter must be positive semidefinite.');
    major=max(major,minor);axis=v(:,k).';
    confidence=((major-minor)/(major+minor))^2;
end

function C=orientationMoment(c,k,C,cfg)
% Use the same support-scale rule on both sides of a correspondence.
    if 12*max(eig(C))>=cfg.minimumSpan^2,return;end
    eligible=c.mixtureWeight>0;
    if isfield(c,'quality'),eligible=eligible & c.quality>0;end
    if isfield(c,'viewReliability'),eligible=eligible & c.viewReliability>0;end
    ids=c.semanticName==c.semanticName(k) & eligible & ...
        sum((c.mean(:,1:2)-c.mean(k,1:2)).^2,2)<=cfg.neighborhoodRadius^2+64*eps(max(1,cfg.neighborhoodRadius^2));
    points=unique(c.mean(ids,1:2),'rows');
    delta=permute(points,[1 3 2])-permute(points,[3 1 2]);
    links=sum(delta.^2,3)<=cfg.maximumGap^2+64*eps(max(1,cfg.maximumGap^2));
    [~,anchor]=min(sum((points-c.mean(k,1:2)).^2,2));
    reached=false(size(points,1),1);reached(anchor)=true;
    while true
        expanded=any(links(:,reached),2);
        if isequal(expanded,reached),break;end
        reached=expanded;
    end
    points=points(reached,:);
    if size(points,1)>=3
        centered=points-mean(points,1);neighborhood=centered.'*centered/size(points,1);
        [axis,~,~,~]=geometry(neighborhood);span=centered*axis.';
        if max(span)-min(span)>=cfg.minimumSpan-64*eps(max(1,cfg.minimumSpan)),C=neighborhood;end
    end
end
