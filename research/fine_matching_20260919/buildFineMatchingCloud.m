function cloud=buildFineMatchingCloud(frame,masks,cfg)
% buildFineMatchingCloud Bin fresh point-level feature masks for D2D diagnosis.
% Use the production output lattice, empirical population moments, covariance
% bounds and occupancy transfer. Binary accepted fine labels have probability
% one. Apply the same stored-point calibration and known tilt as coarse input.
% No reference XY/yaw, ring index or map observation enters this conversion.
    g=cfg.coarseProbabilityCloud; calibration=cfg.frameCalibration;
    xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
    points=(xyz*calibration.rotation.'+calibration.translation)*g.projectionRotation.';
    points=points+g.projectionTranslation;
    means=zeros(0,2); covariance=zeros(2,2,0); names=strings(0,1); counts=zeros(0,1);
    classes=validatePerceptionFeatureNames(cfg.featureNames); selectedCounts=zeros(numel(classes),1);
    dims=ceil(([g.xMax g.yMax]-[g.xMin g.yMin])/g.resolution);
    for k=1:numel(classes)
        mask=masks.(classes(k));
        assert(islogical(mask) && numel(mask)==size(xyz,1),'Fine masks must index the original frame.');
        selected=mask(:) & all(isfinite(points),2);p=points(selected,1:2);
        bins=floor((p-[g.xMin g.yMin])/g.resolution);
        keep=all(bins>=0,2) & bins(:,1)<dims(1) & bins(:,2)<dims(2);
        p=p(keep,:); bins=bins(keep,:); selectedCounts(k)=size(p,1);
        [cells,~,group]=unique(bins,'rows');
        for cellId=1:size(cells,1)
            q=p(group==cellId,:); count=size(q,1);
            if count<g.minimumPointsPerComponent,continue;end
            mu=mean(q,1);d=q-mu; scatter=(d.'*d)/count+g.regularizationVariance*eye(2);
            [v,values]=eig(scatter,'vector');
            values=min(g.maxCovarianceEigenvalue,max(g.minCovarianceEigenvalue,values));
            means(end+1,:)=mu;covariance(:,:,end+1)=v*diag(values)*v.'; %#ok<AGROW>
            names(end+1,1)=classes(k);counts(end+1,1)=count; %#ok<AGROW>
        end
    end
    occupancy=1-exp(-counts/g.occupancySaturationPointCount);
    components=struct('mean',means,'covariance',covariance,'semanticName',names, ...
        'numComponents',numel(counts),'mixtureWeight',occupancy/max(sum(occupancy),eps), ...
        'count',counts,'semanticProbability',ones(size(counts)),'occupancyProbability',occupancy);
    cloud=struct('components',components,'dimension',2,'coordinateFrame',"sensorLocalXY", ...
        'classificationStage',"finePointMasks",'frameCalibration',calibration, ...
        'sourceSummary',struct('semanticName',classes(:),'selectedPointCount',selectedCounts));
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end
