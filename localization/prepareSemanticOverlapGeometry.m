function model=prepareSemanticOverlapGeometry(fixedCloud,movingCloud,pose,cfg)
% prepareSemanticOverlapGeometry Prepare support axes for landmark overlap.
% Means are relative to the current prediction. This prepares mixture geometry;
% it does not choose correspondences or construct a pose optimization problem.
    arguments
        fixedCloud (1,1) struct
        movingCloud (1,1) struct
        pose (1,3) double {mustBeReal,mustBeFinite}
        cfg (1,1) struct = semanticGeometryConfig()
    end
    [f,m,height]=registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    m.quality=quality(m);
    f.viewReliability=ones(f.numComponents,1);
    if isfield(fixedCloud.components,'viewReliability')
        reliability=double(fixedCloud.components.viewReliability(:));
        assert(numel(reliability)==f.numComponents && isreal(reliability) && ...
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
    m.planarCovariance=movingCloud.components.covariance(1:2,1:2,:);
    [f,m]=supportGeometry(f,m,cfg.support);
    f.planarCovariance=f.covariance(1:2,1:2,:);
    f.mean(:,1:2)=f.mean(:,1:2)-pose(1:2);
    model=struct('fixed',f,'moving',m,'height',height,'origin',pose(1:2));
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

function [f,m]=supportGeometry(f,m,cfg)
% supportGeometry Estimate geometric support without class rules.
% Neighborhood moments supply the axes used by the optional orientation kernel.
    values=[cfg.angularFloor,cfg.scatterScale,cfg.neighborhoodRadius, ...
        cfg.minimumSpan,cfg.directionPower,cfg.maximumGap];
    assert(isreal(values)&&all(isfinite(values)&values>0)&&cfg.angularFloor<pi/2 && ...
        isscalar(cfg.directionResolution)&&isreal(cfg.directionResolution)&& ...
        isfinite(cfg.directionResolution)&&cfg.directionResolution>=0, ...
        'VehicleLocalization:InvalidSupportGeometry','Require finite positive support geometry scales.');
    count=f.numComponents;
    if isfield(f,'intrinsicCovariance')
        C=f.intrinsicCovariance;
        assert(isnumeric(C)&&isreal(C)&&size(C,1)==2&&size(C,2)==2&&size(C,3)==count && ...
            all(isfinite(C),'all')&&all(abs(C-permute(C,[2 1 3]))<1e-9,'all'), ...
            'VehicleLocalization:InvalidIntrinsicShape','Require aligned finite symmetric intrinsic scatter.');
    end
    f.orientationNormal=zeros(count,2);f.axisConfidence=zeros(count,1);
    f.angularVariance=zeros(count,1);
    for k=1:count
        C=f.covariance(1:2,1:2,k);
        if isfield(f,'intrinsicCovariance') && any(f.intrinsicCovariance(:,:,k),'all')
            C=f.intrinsicCovariance(:,:,k);
        end
        [~,~,~,intrinsicConfidence]=geometry(C);
        orientation=orientationMoment(f,k,C,cfg);
        [axis,minor,major,confidence]=geometry(orientation);
        f.orientationNormal(k,:)=[-axis(2),axis(1)];
        f.axisConfidence(k)=intrinsicConfidence*confidence^(cfg.directionPower/2)*major/(major+cfg.directionResolution^2);
        f.angularVariance(k)=cfg.scatterScale^2*minor/major;
    end
    count=m.numComponents;m.supportTangent=zeros(count,2);
    m.axisConfidence=zeros(count,1);m.angularVariance=zeros(count,1);
    for k=1:count
        C=m.planarCovariance(:,:,k);
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
% Use the same neighborhood rule for map and source component axes.
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
