function [features,names]=measurePillarMomentContext(stats,dims,query)
% measurePillarMomentContext: Multiscale complete-population descriptors.
% Parallel-axis merging preserves every member of neighboring pillars.
% Conditional covariance is moment algebra, not a fitted object or plane.
% Horizontal shape, concentration and within/between height variation help
% distinguish compact shafts, vertical walls and ground discontinuities.
    at=double(stats.pillarIndices);number=prod(dims);
    count=zeros(dims);count(at)=stats.count;occupied=count>0;
    mu=zeros(number,3);
    if ~isempty(at),mu(at,:)=stats.meanXYZ-stats.meanXYZ(1,:);end
    cv=zeros(number,6);cv(at,:)=stats.covarianceXYZ;
    first=mu.*count(:);pairs=[1 1;1 2;2 2;1 3;2 3;3 3];
    second=zeros(number,6);
    for k=1:6
        second(at,k)=stats.count.*(stats.covarianceXYZ(:,k)+ ...
            mu(at,pairs(k,1)).*mu(at,pairs(k,2)));
    end
    columns=["count","cells","centerCountFraction","effectiveCells", ...
        "verticalVariance","withinVerticalVariance","betweenVerticalVariance", ...
        "withinVerticalFraction","centerHeightContrast","horizontalVariance", ...
        "horizontalAnisotropy","horizontalMinorVariance","horizontalMajorVariance", ...
        "transverseVariance","transverseAnisotropy","transverseMinorVariance", ...
        "transverseMajorVariance","verticalResidualVariance","verticalResidualFraction"];
    features=zeros(numel(query),0);names=strings(1,0);
    for radius=[0 1 2 4 7]
        kernel=ones(2*radius+1);total=conv2(count,kernel,'same');
        cells=conv2(double(occupied),kernel,'same');
        squares=conv2(count.^2,kernel,'same');n=reshape(total(query),[],1);
        means=zeros(numel(query),3);covariance=zeros(numel(query),6);
        for k=1:3
            merged=conv2(reshape(first(:,k),dims),kernel,'same');
            means(:,k)=reshape(merged(query),[],1)./max(n,1);
        end
        for k=1:6
            merged=conv2(reshape(second(:,k),dims),kernel,'same');
            covariance(:,k)=reshape(merged(query),[],1)./max(n,1)- ...
                means(:,pairs(k,1)).*means(:,pairs(k,2));
        end
        if radius==0,covariance=cv(query,:);end
        xx=max(covariance(:,1),0);xy=covariance(:,2);yy=max(covariance(:,3),0);
        xz=covariance(:,4);yz=covariance(:,5);zz=max(covariance(:,6),0);
        [trace,anisotropy,minor,major]=horizontalShape(xx,xy,yy);
        [transverse,tAnisotropy,tMinor,tMajor]=horizontalShape( ...
            max(0,xx-xz.^2./max(zz,eps)),xy-xz.*yz./max(zz,eps), ...
            max(0,yy-yz.^2./max(zz,eps)));
        % Resolve the 2D population eigensystem without an ill-conditioned
        % inverse. Unidentifiable directions contribute no explained mass.
        angle=.5*atan2(2*xy,xx-yy);
        along=cos(angle).*xz+sin(angle).*yz;
        across=-sin(angle).*xz+cos(angle).*yz;
        explained=zeros(size(zz));known=major>1e-14;
        explained(known)=along(known).^2./major(known);
        known=minor>max(1e-14,major*1e-10);
        explained(known)=explained(known)+across(known).^2./minor(known);
        residual=max(0,zz-explained);
        residual(residual<max(1e-12,zz*1e-10))=0;
        withinMap=conv2(reshape(count(:).*cv(:,6),dims),kernel,'same');
        within=reshape(withinMap(query),[],1)./max(n,1);
        values=[n,reshape(cells(query),[],1),reshape(count(query),[],1)./max(n,1), ...
            n.^2./max(reshape(squares(query),[],1),1),zz,within,max(0,zz-within), ...
            min(1,within./max(zz,eps)),mu(query,3)-means(:,3),trace,anisotropy,minor,major, ...
            transverse,tAnisotropy,tMinor,tMajor,residual,min(1,residual./max(zz,eps))];
        values(n==0,:)=0;values(~isfinite(values))=0;
        features=[features,values];names=[names,"r"+radius+"_"+columns]; %#ok<AGROW>
    end
    features=floor(features*1e8+.5)/1e8;
end

function [trace,anisotropy,minor,major]=horizontalShape(xx,xy,yy)
    trace=xx+yy;difference=min(trace,sqrt((xx-yy).^2+4*xy.^2));
    degenerate=trace<1e-12;trace(degenerate)=0;difference(degenerate)=0;
    anisotropy=difference./max(trace,eps);
    minor=max(0,(trace-difference)/2);major=max(0,(trace+difference)/2);
end
