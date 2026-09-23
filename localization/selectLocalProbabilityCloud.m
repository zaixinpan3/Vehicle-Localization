function [fixed,indices]=selectLocalProbabilityCloud(map,seed,radius)
% selectLocalProbabilityCloud Select map components without changing their priors.
    c=map.components;keep=sum((c.mean(:,1:2)-seed(1:2)).^2,2)<=radius^2;
    % A local subcloud retains original priors, not whole-map normalization
    % claims such as totalMass/classTotalMass for components outside the crop.
    fixed=struct('components',struct());
    for name=["frameCalibration","coordinateFrame","dimension"]
        if isfield(map,name),fixed.(name)=map.(name);end
    end
    fixed.components.mean=c.mean(keep,:);
    fixed.components.covariance=c.covariance(:,:,keep);
    fixed.components.numComponents=nnz(keep);indices=find(keep);
    for name=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability", ...
            "supportAmplitude","meanXYZ","heightAvailable","componentId"]
        if isfield(c,name),fixed.components.(name)=c.(name)(keep,:);end
    end
    if isfield(c,'covarianceXYZ'),fixed.components.covarianceXYZ=c.covarianceXYZ(:,:,keep);end
    if isfield(map,'heightEvidence')
        e=map.heightEvidence;fixed.heightEvidence=e;
        fixed.heightEvidence.mean=e.mean(keep,:);
        fixed.heightEvidence.covariance=e.covariance(:,:,keep);
        fixed.heightEvidence.available=e.available(keep);
    end
end
