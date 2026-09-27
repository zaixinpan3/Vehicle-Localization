function f=measurePoleContext(points,structural,h)
% measurePoleContext: Continuous multiscale XY shape around a supported axis.
% No XY raster, frame identity, world position or reference label is used.
    p=double(points);d=p(:,1:2)-h.axisXY-(p(:,3)-h.axisZ).*h.slopeXY;
    z=p(:,3);r2=sum(d.^2,2);
    selected=structural & r2<=1.5^2 & z>=h.minimumZ & z<=h.maximumZ;
    d=d(selected,:);z=z(selected);r2=r2(selected);
    f=struct('supportHeight',h.supportHeight,'longestHeight',h.longestSupportedHeight, ...
        'meanRatio',h.meanRatio,'massRatio',h.massRatio,'supportedCount',h.supportedCount, ...
        'tilt',h.tilt,'radialRms',h.radialRms,'maximumStd',h.maximumStd, ...
        'aspect',h.aspect,'isolation',h.isolation,'acceptedCount',h.acceptedCount);
    n=numel(z);f.contextCount=n;
    if n==0,return;end
    for radius=[.06 .10 .15 .25 .50 .75 1 1.5]
        key=sprintf('fraction%03d',round(100*radius));f.(key)=nnz(r2<=radius^2)/n;
    end
    core=r2<=.25^2;zc=sort(z(core));f.coreMinimumZ=min(zc);f.coreMaximumZ=max(zc);
    f.coreHeight=max(zc)-min(zc);f.meanGap=mean(diff(zc));f.maximumGap=max(diff(zc));
    f.gapVariation=std(diff(zc))/max(f.meanGap,eps);
    for scale=[.15 .30 .45 .60]
        tag=sprintf('%02d',round(100*scale));w=exp(-r2/(2*scale^2));total=sum(w);
        centroid=sum(w.*d,1)/total;second=(d.'*(w.*d))/total;
        eigenvalues=sort(eig(second));curvature=1-eigenvalues/scale^2;
        f.(['peakFloor' tag])=min(curvature);f.(['peakCeiling' tag])=max(curvature);
        f.(['peakShift' tag])=norm(centroid)/scale;
        f.(['anisotropy' tag])=1-eigenvalues(1)/max(eigenvalues(2),eps);
        f.(['density' tag])=total/(2*pi*scale^2);
        f.(['centralFraction' tag])=nnz(core)/max(total,1);
    end
    bounds=linspace(min(zc),max(zc)+eps(max(zc)),5);centers=nan(4,2);rms=zeros(4,1);count=rms;
    for k=1:4
        member=core & z>=bounds(k) & z<=bounds(k+1);count(k)=nnz(member);
        if count(k)>0,centers(k,:)=mean(d(member,:),1);rms(k)=sqrt(mean(r2(member)));end
    end
    good=count>0;f.minimumQuarterCount=min(count);f.maximumQuarterFraction=max(count)/max(sum(count),1);
    f.quarterCenterSpread=max(vecnorm(centers(good,:),2,2));
    f.quarterCenterStep=max(vecnorm(diff(centers(good,:),1,1),2,2));
    if isempty(f.quarterCenterStep),f.quarterCenterStep=0;end
    f.quarterRmsMaximum=max(rms);f.quarterRmsVariation=std(rms);
    ring=d(r2>.25^2,:);angle=atan2(ring(:,2),ring(:,1));
    sectors=histcounts(angle,linspace(-pi,pi,9));f.ringConcentration=max(sectors)/max(sum(sectors),1);
end
