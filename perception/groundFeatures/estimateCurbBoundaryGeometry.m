function [moments,details]=estimateCurbBoundaryGeometry(ground,context,rotation,translation,cfg)
% estimateCurbBoundaryGeometry Locate a curb face within an already selected pillar.
% Returned XY centers are boundary-model estimates, not full-pillar means.
% Classification, counts and the input empirical moments are not mutated.
% Unlike vertical least squares on a binary step, a return on the face is
% measured horizontally to the face, and need not lie on either road plane.
    if nargin<5,cfg=curbBoundaryGeometryConfig();end
    moments=ground.moments;ids=find(ground.curbCellMask);centers=moments.mean(ids,:);
    xyz=double(context.groundPoints)*rotation.'+translation;rows=cell(0,8);
    for k=1:numel(ids)
        near=sum((centers-centers(k,:)).^2,2)<=cfg.directionRadius^2;if nnz(near)<3,continue;end
        local=centers(near,:)-mean(centers(near,:),1);[v,e]=eig(local.'*local,'vector');[major,j]=max(e);
        if major<cfg.minimumAnisotropy*max(min(e),1e-4),continue;end
        tangent=v(:,j);normal=[-tangent(2);tangent(1)];if centers(k,:)*normal<0,normal=-normal;end
        delta=xyz(:,1:2)-centers(k,:);region=sum(delta.^2,2)<=cfg.supportRadius^2;q=xyz(region,:);delta=delta(region,:);
        t=delta*tangent;n=delta*normal;keep=abs(t)<=cfg.tangentHalfWidth & abs(n)<=cfg.normalHalfWidth;q=q(keep,:);t=t(keep);n=n(keep);
        if numel(n)<cfg.minimumPoints||max(n)-min(n)<cfg.minimumNormalSpan,continue;end
        X=[ones(size(t)),t,n];if rcond(X.'*X)<1e-5,continue;end
        z=q(:,3);residual=z-X*(X\z);base=sum(residual.^2);
        ordered=sort(unique(n));candidates=(ordered(1:end-1)+ordered(2:end))/2;candidates=candidates(abs(candidates)<=cfg.maximumBoundaryShift);
        if numel(candidates)>cfg.maximumCandidates,candidates=candidates(unique(round(linspace(1,numel(candidates),cfg.maximumCandidates))));end
        if isempty(candidates),continue;end
        distance=n-candidates.';side=double(distance>=0);outer=abs(distance)>=cfg.faceHalfWidth;
        [beta,valid]=batchFit(X,side,distance,outer,z);
        valid=valid & sum(side.*outer,1)>=5 & sum((1-side).*outer,1)>=5 & abs(beta(4,:))>=cfg.minimumHeight & abs(beta(4,:))<=cfg.maximumHeight;
        [loss,assignment]=surfaceDistance(X,z,distance,beta);
        use=assignment~=3;secondSide=double(assignment==2);
        [second,refit]=batchFit(X,secondSide,distance,use,z);
        refit=refit & sum(assignment==1,1)>=5 & sum(assignment==2,1)>=5;
        valid(refit)=valid(refit) & abs(second(4,refit))>=cfg.minimumHeight & abs(second(4,refit))<=cfg.maximumHeight;
        beta(:,refit)=second(:,refit);
        [updated,secondAssignment]=surfaceDistance(X,z,distance,beta);
        loss(:,refit)=updated(:,refit);assignment(:,refit)=secondAssignment(:,refit);
        value=sum(loss,1);value(~valid)=Inf;[best,j]=min(value);
        shift=candidates(j);height=beta(4,j);faceCount=nnz(assignment(:,j)==3);
        if ~isfinite(best),continue;end
        fraction=1-best/max(base,eps);noise=sqrt(best/numel(n));proposed=[centers(k,:)+shift*normal.',moments.meanZ(ids(k))];
        stored=(proposed-translation)*rotation;bins=floor((stored(1:2)-ground.cellOrigin)./ground.cellSize)+1;[row,col]=ind2sub(ground.cellMapSize,ids(k));
        accepted=fraction>=cfg.minimumExplainedFraction && noise<=cfg.maximumResidualStd && bins(1)==col && bins(2)==row && faceCount>=cfg.minimumFacePoints;
        if accepted,moments.mean(ids(k),:)=proposed(1:2);end
        rows(end+1,:)={ids(k),shift*normal(1),shift*normal(2),height,fraction,noise,accepted,numel(n)}; %#ok<AGROW>
    end
    details=cell2table(rows,VariableNames={'pillar','dx','dy','stepHeight','explainedFraction','residualStd','accepted','points'});
end

function [beta,valid]=batchFit(X,side,distance,use,z)
% Batch the small normal systems; reject the same poorly conditioned systems.
    n=size(X,1);count=size(side,2);A=zeros(n,5,count);A(:,1:3,:)=repmat(X,1,1,count);
    A(:,4,:)=reshape(side,n,1,count);A(:,5,:)=reshape(distance.*side,n,1,count);
    At=permute(A,[2 1 3]);weighted=A.*reshape(use,n,1,count);
    normal=pagemtimes(At,weighted);rhs=pagemtimes(At,reshape(z.*use,n,1,count));
    valid=false(1,count);
    for j=1:count,valid(j)=rcond(normal(:,:,j))>=1e-5;end
    beta=zeros(5,count);
    if any(valid),beta(:,valid)=reshape(pagemldivide(normal(:,:,valid),rhs(:,:,valid)),5,[]);end
end
function [loss,assignment]=surfaceDistance(X,z,distance,beta)
    residual=z-X*beta(1:3,:);h=beta(4,:);vertical=(residual+beta(3,:).*distance)./max(abs(h),eps).*sign(h);
    road=residual.^2+max(distance,0).^2;
    upper=(residual-h-beta(5,:).*distance).^2+min(distance,0).^2;
    face=distance.^2+h.^2.*(max(-vertical,0).^2+max(vertical-1,0).^2);
    [loss,assignment]=min(cat(3,road,upper,face),[],3);
end
