function [fixed,indices]=selectLocalProbabilityCloud(map,seed,radius)
% selectLocalProbabilityCloud Select map components without changing their priors.
    c=map.components;keep=sum((c.mean(:,1:2)-seed(1:2)).^2,2)<=radius^2;
    fixed=map;fixed.components.mean=c.mean(keep,:);
    fixed.components.covariance=c.covariance(:,:,keep);
    fixed.components.numComponents=nnz(keep);indices=find(keep);
    for name=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,name),fixed.components.(name)=c.(name)(keep,:);end
    end
end
