function [moments,details]=estimateCurbSurfaceMomentsV1(ground,context,rotation,translation)
% estimateCurbSurfaceMoments Fit two road planes joined by a vertical curb face.
% Unlike vertical least squares on a binary step, a return on the face is
% measured horizontally to the face, and need not lie on either road plane.
    moments=ground.moments;ids=find(ground.curbCellMask);centers=moments.mean(ids,:);
    xyz=double(context.groundPoints)*rotation.'+translation;rows=cell(0,8);
    for k=1:numel(ids)
        near=sum((centers-centers(k,:)).^2,2)<=2.4^2;if nnz(near)<3,continue;end
        local=centers(near,:)-mean(centers(near,:),1);[v,e]=eig(local.'*local,'vector');[major,j]=max(e);
        if major<4*max(min(e),1e-4),continue;end
        tangent=v(:,j);normal=[-tangent(2);tangent(1)];if centers(k,:)*normal<0,normal=-normal;end
        delta=xyz(:,1:2)-centers(k,:);region=sum(delta.^2,2)<=1.6^2;q=xyz(region,:);delta=delta(region,:);
        t=delta*tangent;n=delta*normal;keep=abs(t)<=1.2 & abs(n)<=.9;q=q(keep,:);t=t(keep);n=n(keep);
        if numel(n)<20||max(n)-min(n)<.6,continue;end
        X=[ones(size(t)),t,n];if rcond(X.'*X)<1e-5,continue;end
        z=q(:,3);residual=z-X*(X\z);base=sum(residual.^2);
        ordered=sort(unique(n));candidates=(ordered(1:end-1)+ordered(2:end))/2;candidates=candidates(abs(candidates)<=.7);
        if numel(candidates)>64,candidates=candidates(unique(round(linspace(1,numel(candidates),64))));end
        best=Inf;shift=NaN;height=NaN;faceCount=0;
        for edge=candidates.'
            side=double(n>=edge);A=[X,side];outer=abs(n-edge)>=.06;
            if nnz(side(outer))<5||nnz(~side(outer))<5||rcond(A(outer,:).'*A(outer,:))<1e-5,continue;end
            beta=A(outer,:)\z(outer);h=beta(4);if abs(h)<.04||abs(h)>.30,continue;end
            for iteration=1:2
                residual=z-X*beta(1:3);distance=n-edge;vertical=residual/h;
                costs=[residual.^2+max(distance,0).^2, (residual-h).^2+min(distance,0).^2, ...
                    distance.^2+h^2*(max(-vertical,0).^2+max(vertical-1,0).^2)];
                [loss,assignment]=min(costs,[],2);
                if iteration==1
                    use=assignment~=3;B=[X,double(assignment==2)];
                    if nnz(assignment==1)<5||nnz(assignment==2)<5||rcond(B(use,:).'*B(use,:))<1e-5,break;end
                    beta=B(use,:)\z(use);h=beta(4);if abs(h)<.04||abs(h)>.30,break;end
                end
            end
            value=sum(loss);
            if value<best && abs(h)>=.04 && abs(h)<=.30,best=value;shift=edge;height=h;faceCount=nnz(assignment==3);end
        end
        if ~isfinite(best),continue;end
        fraction=1-best/max(base,eps);noise=sqrt(best/numel(n));proposed=[centers(k,:)+shift*normal.',moments.meanZ(ids(k))];
        stored=(proposed-translation)*rotation;bins=floor((stored(1:2)-ground.cellOrigin)./ground.cellSize)+1;[row,col]=ind2sub(ground.cellMapSize,ids(k));
        accepted=fraction>=.25 && noise<=.06 && bins(1)==col && bins(2)==row && faceCount>=3;
        if accepted,moments.mean(ids(k),:)=proposed(1:2);end
        rows(end+1,:)={ids(k),shift*normal(1),shift*normal(2),height,fraction,noise,accepted,numel(n)}; %#ok<AGROW>
    end
    details=cell2table(rows,VariableNames={'pillar','dx','dy','stepHeight','explainedFraction','residualStd','accepted','points'});
end
