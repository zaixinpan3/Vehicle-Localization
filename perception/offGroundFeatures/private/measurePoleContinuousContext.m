function f=measurePoleContinuousContext(points,structural,h,limitToShaftHeight,wanted)
% measurePoleContinuousContext: Measure continuous height support at metric probes.
% Probes overlap and follow the fitted axis; they do not assign an XY grid.
    if nargin<4,limitToShaftHeight=true;end
    if nargin<5,wanted=strings(0,1);allFeatures=true;else,allFeatures=false;end
    wanted=string(wanted);
    q=double(points);d=q(:,1:2)-h.axisXY-(q(:,3)-h.axisZ).*h.slopeXY;
    z=q(:,3);use=structural;
    if limitToShaftHeight,use=use & z>=h.minimumZ-.25 & z<=h.maximumZ+.25;end
    d=d(use,:);z=z(use);r2=sum(d.^2,2);f=struct();
    for radius=[.12 .18 .25]
        for context=[.45 .60 .75]
            tag=sprintf('%02d_%02d',round(100*radius),round(100*context));
            if ~allFeatures && ~any(ismember(["height","longest","mean","mass"]+tag,wanted)),continue;end
            [height,longest,meanRatio,massRatio]=densitySupport(z,r2<=radius^2,r2<=context^2);
            f.(['height' tag])=height;f.(['longest' tag])=longest;
            f.(['mean' tag])=meanRatio;f.(['mass' tag])=massRatio;
        end
    end
    center=NaN;
    for offset=[.45 .90]
        tag=sprintf('%02d',round(100*offset));
        if ~allFeatures && ~any(ismember(["probeFraction","probeMaximum","probeSum", ...
                "probePointScore","probePairMaximum","probeOpposingMinimum"]+tag,wanted)),continue;end
        if isnan(center),center=countSupport(z(r2<=.20^2));end
        angle=(0:7)'*pi/4;probes=offset*[cos(angle),sin(angle)];height=zeros(8,1);
        for k=1:8,height(k)=countSupport(z(sum((d-probes(k,:)).^2,2)<=.20^2));end
        pairs=height(1:4)+height(5:8);
        f.(['probeFraction' tag])=center/max(center+sum(height),eps);
        f.(['probeMaximum' tag])=max(height)/max(center,eps);
        f.(['probeSum' tag])=sum(height)/max(center,eps);
        f.(['probePointScore' tag])=min(max(2*center-pairs,0))/max(max(2*center-pairs),eps);
        f.(['probePairMaximum' tag])=max(pairs)/max(center,eps);
        f.(['probeOpposingMinimum' tag])=max(min(height(1:4),height(5:8)))/max(center,eps);
    end
end

function total=countSupport(z)
    if numel(z)<3,total=0;return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);
    count=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    total=sum(diff(p).*(count(1:end-1)>=3));
end

function [height,longest,meanRatio,massRatio]=densitySupport(z,core,context)
    z=z(context);core=core(context);height=0;longest=0;meanRatio=0;massRatio=0;
    if nnz(core)<3,return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);a=double(core);
    c=cumsum(accumarray(g,[a;-a]));t=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    c=c(1:end-1);t=t(1:end-1);w=diff(p);ratio=c./max(t,1);ok=c>=3 & ratio>.6;
    height=sum(w(ok));if height==0,return;end
    edge=diff([false;ok;false]);longest=max(p(edge==-1)-p(edge==1));
    meanRatio=sum(w(ok).*ratio(ok))/height;massRatio=sum(w(ok).*c(ok))/sum(w(ok).*t(ok));
end
