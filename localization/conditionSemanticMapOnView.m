function [cloud,details]=conditionSemanticMapOnView(cloud,predicted)
% conditionSemanticMapOnView Evaluate a frozen map at the predicted origin.
% Relative kernel weights determine mean and within-plus-between scatter.
% Absolute nearest-view distance continuously reduces influence where the
% mapping drive supplies little view support. It is not a pose observation.
% The solver freezes these map moments for its solve. Reported information
% is conditional on this view and is not a calibrated or independent posterior.
    details=struct('enabled',false,'conditionedComponents',0,'queryOrigin',predicted, ...
        'informationCalibrated',false);
    if ~isfield(cloud,'landmarkViews'),return;end
    validateattributes(predicted,{'numeric'},{'vector','numel',3,'finite','real'});
    predicted=double(predicted(:).');model=cloud.landmarkViews;cfg=model.config;
    validateLandmarkViewConfig(cfg);
    n=cloud.components.numComponents;
    assert(model.schemaVersion==1 && iscell(model.observations)&&numel(model.observations)==n, ...
        'VehicleLocalization:InvalidLandmarkViews','Map view arrays must align with component indices.');
    cloud.components.viewReliability=ones(n,1);modeled=ismember(cloud.components.semanticName,cfg.pointClasses);
    cloud.components.viewReliability(modeled)=0;
    for id=find(modeled).'
        o=model.observations{id};if isempty(o),continue;end
        assert(isnumeric(o)&&isreal(o)&&size(o,2)==9&&all(isfinite(o),'all'), ...
            'VehicleLocalization:InvalidLandmarkViews','Require finite acquisition-origin and Gaussian observation rows.');
        yaw=atan2(sin(o(:,3)-predicted(3)),cos(o(:,3)-predicted(3)));
        o=o(abs(yaw)<=cfg.maximumHeadingDifference,:);if isempty(o),continue;end
        d2=sum((o(:,1:2)-predicted(1:2)).^2,2);minimum=min(d2);
        w=exp(-.5*(d2-minimum)/cfg.bandwidth^2);w=w/sum(w);
        mu=sum(o(:,4:5).*w,1);delta=o(:,4:5)-mu;
        S=[sum(w.*o(:,6)),sum(w.*o(:,7));sum(w.*o(:,7)),sum(w.*o(:,8))]+delta.'*(delta.*w);
        [V,E]=eig((S+S.')/2,'vector');S=V*diag(max(E,cfg.varianceFloor))*V.';
        cloud.components.mean(id,:)=mu;cloud.components.covariance(:,:,id)=(S+S.')/2;
        cloud.components.viewReliability(id)=exp(-.5*minimum/cfg.coverageScale^2);
        details.conditionedComponents=details.conditionedComponents+1;
    end
    details.enabled=true;details.queryOrigin=predicted;
    details.modeledComponents=nnz(modeled);
    details.effectiveSupport=sum(cloud.components.viewReliability(modeled));
    cloud.landmarkViewConditioning=details;
end
