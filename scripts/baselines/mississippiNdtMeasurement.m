function measurement=mississippiNdtMeasurement(map,source,pose,cellSize,noise)
% Experimental planar NDT on deterministic samples of the shared Gaussians.
% Five equal sigma points reproduce each component's mean/covariance. NDT
% ignores semantic labels and component masses. It is an adapted front-end
% control, not a raw-scan NDT or a complete external localization system.
    measurement=[];if source.components.numComponents==0,return;end
    cfg=overlapGradientConfig();local=registrationSupport.selectLocalProbabilityCloud(map,pose.',cfg.localMapRadius);if local.components.numComponents==0,return;end
    local=registrationSupport.conditionSemanticMapOnView(local,pose.');[f,m]=registrationSupport.prepareSemanticRegistration(local,source,cfg.geometry);
    fixed=points(f)-pose(1:2).';moving=points(m);if size(fixed,1)<6 || size(moving,1)<6,return;end
    [q,stats]=matchScans(lidarScan(moving),lidarScan(fixed),InitialPose=[0,0,pose(3)],CellSize=cellSize,MaxIterations=80);
    H=(stats.Hessian+stats.Hessian.')/2;[U,E]=eig(H);ev=diag(E);keep=ev>1e-10*max(1,max(abs(ev)));if ~any(keep) || any(~isfinite(q)),return;end
    measurement=struct('pose',[pose(1:2)+q(1:2).';q(3)],'basis',U(:,keep),'covariance',noise,'rank',nnz(keep));
end
function p=points(c)
    p=zeros(5*c.numComponents,2);
    for k=1:c.numComponents
        [U,E]=eig((c.covariance(:,:,k)+c.covariance(:,:,k).')/2);A=U*diag(sqrt(max(diag(E),0)*2.5));
        p((k-1)*5+(1:5),:)=c.mean(k,:)+[0,0;A(:,1).';-A(:,1).';A(:,2).';-A(:,2).'];
    end
end
