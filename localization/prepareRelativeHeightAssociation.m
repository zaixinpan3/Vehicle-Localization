function model=prepareRelativeHeightAssociation(fixedCloud,movingCloud,origin,initialPairs,cfg)
% prepareRelativeHeightAssociation Infer a common Z datum from curb evidence.
% Height changes candidate costs only. The retained SE(2) pose residual and
% information remain planar. Heights are current-scan observations; the XY
% cloud can still contain five acquisitions. No reference altitude is used.
    values=[cfg.minimumAnchors,cfg.minimumAnchorSpan,cfg.maximumAnchorDistance, ...
        cfg.noiseStandardDeviation,cfg.tiltStandardDeviation,cfg.weight,cfg.maximumPenalty];
    assert(all(isfinite(values))&&all(values>0)&&cfg.minimumAnchors>=2 && ...
        cfg.minimumAnchors==fix(cfg.minimumAnchors),'VehicleLocalization:InvalidHeightConfiguration', ...
        'Relative-height bounds must be positive, with at least two anchors.');
    assert(isscalar(string(cfg.candidateModel)) && ismember(string(cfg.candidateModel),["marginal","conditional"]), ...
        'VehicleLocalization:InvalidHeightConfiguration','Use marginal or conditional height evidence.');
    f=registrationSupport.getHeightEvidence(fixedCloud);
    m=registrationSupport.getHeightEvidence(movingCloud);
    m.semanticName=string(movingCloud.components.semanticName);
    f.mean(:,1:2)=f.mean(:,1:2)-origin(1:2);
    n=numel(f.available);f.slope=zeros(n,2);f.conditionalVariance=zeros(n,1);
    for k=find(f.available).'
        covariance=f.covariance(:,:,k);beta=covariance(1:2,1:2)\covariance(1:2,3);
        f.slope(k,:)=beta.';
        f.conditionalVariance(k)=max(0,covariance(3,3)-covariance(3,1:2)*beta);
    end
    details=struct('enabled',false,'reason',"insufficientHeightAnchors", ...
        'anchorCount',0,'anchorSpan',0,'offset',NaN,'offsetScatter',NaN, ...
        'sourceHeightCount',nnz(m.available),'mapHeightCount',nnz(f.available), ...
        'verticalReference',"mapCurbConsensus",'referenceAltitudeUsed',false, ...
        'planarForce',false,'weight',cfg.weight,'candidateModel',string(cfg.candidateModel));
    model=struct('details',details,'cost',[]);
    s=initialPairs.source;t=initialPairs.target;
    select=initialPairs.semanticName=="curb" & f.available(t) & m.available(s);
    s=s(select);t=t(select);
    r=rotation(origin(3));world=m.mean(s,1:2)*r.';
    delta=world-f.mean(t,1:2);
    % The XY normal residual selects local curb support without demanding
    % agreement along an extended line component's tangent.
    distance=zeros(numel(s),1);
    for k=1:numel(s)
        [v,d]=eig(f.covariance(1:2,1:2,t(k)),'vector');[~,small]=min(d);
        distance(k)=abs(delta(k,:)*v(:,small));
    end
    [~,order]=sort(distance);s=s(order);t=t(order);world=world(order,:);distance=distance(order);
    [~,uniqueTarget]=unique(t,'stable');
    use=uniqueTarget(distance(uniqueTarget)<=cfg.maximumAnchorDistance);
    s=s(use);t=t(use);world=world(use,:);
    dz=f.mean(t,3)+sum((world-f.mean(t,1:2)).*f.slope(t,:),2)-m.mean(s,3);
    if numel(dz)<cfg.minimumAnchors,return;end
    center=median(dz);scatter=1.4826*median(abs(dz-center));
    inlier=abs(dz-center)<=3*max(scatter,cfg.noiseStandardDeviation);
    dz=dz(inlier);world=world(inlier,:);
    details.anchorCount=numel(dz);
    if isempty(dz),model.details=details;return;end
    details.anchorSpan=norm(max(world,[],1)-min(world,[],1));
    details.offset=median(dz);
    % Scatter includes common anchor/model disagreement and is not divided
    % by the number of correlated curb observations.
    details.offsetScatter=1.4826*median(abs(dz-details.offset));
    details.enabled=numel(dz)>=cfg.minimumAnchors && details.anchorSpan>=cfg.minimumAnchorSpan;
    if details.enabled,details.reason="enabled";end
    model.details=details;
    if details.enabled
        model.cost=@(source,target,pose) candidateCost(f,m,source,target,pose,details,cfg);
    end
end

function [cost,residual]=candidateCost(f,m,source,target,pose,details,cfg)
    r=rotation(pose(3));mean=m.mean(source,1:2)*r.'+pose(1:2);
    beta=f.slope(target,:);mapVariance=f.conditionalVariance(target);
    if string(cfg.candidateModel)=="marginal"
        % A narrow vertical structure does not define a reliable Z surface
        % over XY. Do not extrapolate its height from a small lateral offset.
        beta(:)=0;mapVariance=reshape(f.covariance(3,3,target),[],1);
    end
    dx=mean(:,1).'-f.mean(target,1);dy=mean(:,2).'-f.mean(target,2);
    residual=f.mean(target,3)+beta(:,1).*dx+beta(:,2).*dy-m.mean(source,3).'-details.offset;
    r3=blkdiag(r,1);c=pagemtimes(pagemtimes(r3,m.covariance(:,:,source)),r3.');
    xx=reshape(c(1,1,:),1,[]);xy=reshape(c(1,2,:),1,[]);yy=reshape(c(2,2,:),1,[]);
    xz=reshape(c(1,3,:),1,[]);yz=reshape(c(2,3,:),1,[]);zz=reshape(c(3,3,:),1,[]);
    variance=mapVariance+zz+beta(:,1).^2.*xx+beta(:,2).^2.*yy ...
        +2*beta(:,1).*beta(:,2).*xy-2*beta(:,1).*xz-2*beta(:,2).*yz ...
        +cfg.noiseStandardDeviation^2+details.offsetScatter^2 ...
        +cfg.tiltStandardDeviation^2*sum(m.mean(source,1:2).^2,2).';
    q=residual.^2./max(variance,eps);
    cost=cfg.weight*cfg.maximumPenalty*q./(cfg.maximumPenalty+q);
    available=f.available(target)&m.available(source).' & ismember(m.semanticName(source),cfg.semanticNames).';
    cost(~available)=0;residual(~available)=NaN;
end

function r=rotation(yaw)
    r=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
end
