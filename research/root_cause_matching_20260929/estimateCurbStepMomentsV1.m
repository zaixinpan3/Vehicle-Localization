function [moments,details]=estimateCurbStepMomentsV1(ground,context,rotation,translation)
% estimateCurbStepMoments Fit a continuous height discontinuity inside a pillar.
% A local plane absorbs road slope; a binary change point locates the curb.
% Candidate locations are observed transverse-coordinate midpoints, not a grid.
% This research estimator changes geometry only, never a detection mask.
    moments=ground.moments;ids=find(ground.curbCellMask);centers=moments.mean(ids,:);
    xyz=double(context.groundPoints)*rotation.'+translation;
    rows=cell(0,8);
    for k=1:numel(ids)
        near=sum((centers-centers(k,:)).^2,2)<=2.4^2;
        if nnz(near)<3,continue;end
        local=centers(near,:)-mean(centers(near,:),1);[v,e]=eig(local.'*local,'vector');[major,j]=max(e);
        if major<4*max(min(e),1e-4),continue;end
        tangent=v(:,j);normal=[-tangent(2);tangent(1)];
        delta=xyz(:,1:2)-centers(k,:);region=sum(delta.^2,2)<=1.6^2; q=xyz(region,:);delta=delta(region,:);
        t=delta*tangent;n=delta*normal;keep=abs(t)<=1.2 & abs(n)<=.9;q=q(keep,:);t=t(keep);n=n(keep);
        if numel(n)<20||max(n)-min(n)<.6,continue;end
        X=[ones(size(t)),t,n];if rcond(X.'*X)<1e-5,continue;end
        residual=q(:,3)-X*(X\q(:,3));base=sum(residual.^2);
        ordered=sort(unique(n));candidates=(ordered(1:end-1)+ordered(2:end))/2;candidates=candidates(abs(candidates)<=.35);
        if numel(candidates)>48,candidates=candidates(unique(round(linspace(1,numel(candidates),48))));end
        if isempty(candidates),continue;end
        proposed=[centers(k,:)+candidates*normal.',repmat(moments.meanZ(ids(k)),numel(candidates),1)];
        stored=(proposed-translation)*rotation;bins=floor((stored(:,1:2)-ground.cellOrigin)./ground.cellSize)+1;
        [row,col]=ind2sub(ground.cellMapSize,ids(k));candidates=candidates(bins(:,1)==col & bins(:,2)==row);
        if isempty(candidates),continue;end
        H=double(n>=candidates.');count=sum(H);valid=count>=5 & count<=numel(n)-5;
        G=H-X*(X\H);denominator=sum(G.^2);height=(residual.'*G)./max(denominator,eps);
        gain=height.^2.*denominator;gain(~valid | abs(height)<.04 | abs(height)>.30)=-Inf;
        [best,j]=max(gain);if ~isfinite(best),continue;end
        fraction=best/max(base,eps);noise=sqrt(max(0,base-best)/numel(n));shift=candidates(j);
        accepted=fraction>=.25 && noise<=.06;
        if accepted,moments.mean(ids(k),:)=centers(k,:)+shift*normal.';end
        rows(end+1,:)={ids(k),shift*normal(1),shift*normal(2),height(j),fraction,noise,accepted,numel(n)}; %#ok<AGROW>
    end
    details=cell2table(rows,VariableNames={'pillar','dx','dy','stepHeight','explainedFraction','residualStd','accepted','points'});
end
