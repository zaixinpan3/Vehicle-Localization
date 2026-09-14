function [cloud,roundTripError]=buildSavedFeatureProbabilityCloud(data,index,pose,cfg)
% buildSavedFeatureProbabilityCloud Aggregate cached fine points for D2D.
% Use the same output resolution and covariance safeguards as the coarse
% cloud, but label this adapter explicitly as a fine-point input product.
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    means=zeros(0,2);covariances=zeros(2,2,0);names=strings(0,1);counts=zeros(0,1);roundTripError=0;
    for j=1:numel(data.featureNames)
        globalPoints=double(data.pointsByFeatureFrame{j,index}(:,1:2));
        p=(globalPoints-pose(1:2))*r;
        if isempty(p), continue; end
        roundTripError=max(roundTripError,max(abs(p*r.'+pose(1:2)-globalPoints),[],'all'));
        valid=all(isfinite(p),2)&p(:,1)>=cfg.xMin&p(:,1)<cfg.xMax&p(:,2)>=cfg.yMin&p(:,2)<cfg.yMax;
        p=p(valid,:);if isempty(p),continue;end
        [~,~,group]=unique(floor((p-[cfg.xMin cfg.yMin])/cfg.resolution),'rows');
        for g=1:max(group)
            points=p(group==g,:); mu=mean(points,1);centered=points-mu;
            c=centered.'*centered/size(points,1)+cfg.regularizationVariance*eye(2);
            [v,e]=eig((c+c.')/2,'vector');e=max(cfg.minCovarianceEigenvalue,min(cfg.maxCovarianceEigenvalue,e));
            c=v*diag(e)*v.';c=(c+c.')/2;
            means(end+1,:)=mu;covariances(:,:,end+1)=c;names(end+1,1)=string(data.featureNames(j));counts(end+1,1)=size(points,1); %#ok<AGROW>
        end
    end
    assert(roundTripError<1e-8,'Global-to-local reconstruction failed.');
    n=numel(counts);assert(n>0);
    components=struct('semanticName',names,'mean',means,'covariance',covariances, ...
        'mixtureWeight',ones(n,1)/n,'numComponents',n,'semanticProbability',ones(n,1), ...
        'occupancyProbability',min(1,counts/cfg.occupancySaturationPointCount));
    cloud=struct('components',components,'dimension',2,'coordinateFrame',"gravityAlignedLocalXY", ...
        'frameCalibration',data.frameCalibration,'sourceRepresentation',"cachedFinePoints",'sourcePointCounts',counts);
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end
