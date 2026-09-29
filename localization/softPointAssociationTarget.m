function [mu,covariance,count]=softPointAssociationTarget(means,covariances,cost,priorCost,closest,cfg)
% softPointAssociationTarget Moment-match nearby ambiguous map components.
% Compatibility costs include priors and optional height penalties. Temper only
% the geometry,
% preserve that nongeometric evidence, and restrict support around the original MAP target.
% Between-component scatter prevents an ambiguous mean becoming a sharp anchor.
% Inputs are prevalidated by prepareSemanticRegistrationGeometry.
    nearby=sum((means-means(closest,:)).^2,2)<=cfg.radius^2 & isfinite(cost);
    ids=find(nearby);
    value=(cost(ids)-priorCost(ids))/cfg.temperature+priorCost(ids);
    value=value-min(value);
    keep=value<=9;ids=ids(keep);value=value(keep);
    probability=exp(-.5*value);probability=probability/sum(probability);
    [confidence,mode]=max(probability);
    if confidence>=cfg.minimumPosterior && ids(mode)==closest
        mu=means(closest,:);covariance=covariances(:,:,closest);count=1;return;
    end
    mu=sum(means(ids,:).*probability,1);covariance=zeros(2);count=numel(ids);
    for k=1:count
        delta=means(ids(k),:)-mu;
        covariance=covariance+probability(k)*(covariances(:,:,ids(k))+delta.'*delta);
    end
end
