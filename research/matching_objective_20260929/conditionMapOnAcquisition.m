function cloud=conditionMapOnAcquisition(cloud,predicted,model)
% conditionMapOnAcquisition Condition map point distributions on viewing origin.
% The geometric samples remain fixed map coordinates; origin only weights them.
    smooth=isfield(model,'coverageScale');
    if smooth,cloud.components.viewReliability=ones(cloud.components.numComponents,1);end
    for id=1:cloud.components.numComponents
        originalWeight=cloud.components.mixtureWeight(id);
        if isfield(model,'unsupportedPolicy') && model.unsupportedPolicy=="omit" && ...
                ismember(cloud.components.semanticName(id),["pole","trafficSign"])
            cloud.components.mixtureWeight(id)=0;
        end
        if smooth && ismember(cloud.components.semanticName(id),["pole","trafficSign"]),cloud.components.viewReliability(id)=0;end
        o=model.observations{id};if size(o,1)<model.minimumEffectiveFrames,continue;end
        d2=sum((o(:,2:3)-predicted(1:2)).^2,2);
        yaw=atan2(sin(o(:,4)-predicted(3)),cos(o(:,4)-predicted(3)));
        valid=abs(yaw)<=model.maximumHeadingDifference & d2<=9*model.bandwidth^2;
        if smooth,valid=abs(yaw)<=model.maximumHeadingDifference;end
        if nnz(valid)<model.minimumEffectiveFrames,continue;end
        o=o(valid,:);d2=d2(valid);w=exp(-.5*(d2-min(d2))/model.bandwidth^2);w=w/sum(w);
        if 1/sum(w.^2)<model.minimumEffectiveFrames,continue;end
        cloud.components.mixtureWeight(id)=originalWeight;
        if smooth,cloud.components.viewReliability(id)=exp(-.5*min(d2)/model.coverageScale^2);end
        mu=sum(o(:,5:6).*w,1);delta=o(:,5:6)-mu;
        S=[sum(w.*o(:,7)),sum(w.*o(:,8));sum(w.*o(:,8)),sum(w.*o(:,9))]+delta.'*(delta.*w);
        if isfield(model,'meanModel') && model.meanModel=="localLinear" && 1/sum(w.^2)>=4
            origin=sum(o(:,2:3).*w,1);x=o(:,2:3)-origin;
            [V,E]=eig(x.'*(x.*w),'vector');[variance,k]=max(E);axis=V(:,k);t=x*axis;q=(predicted(1:2)-origin)*axis;
            if variance>.1 && q>=min(t) && q<=max(t)
                slope=(t.'*(delta.*w))/variance;mu=mu+q*slope;
            end
        end
        [V,E]=eig((S+S.')/2,'vector');S=V*diag(max(E,model.varianceFloor))*V.';
        if ~isfield(model,'updateMean')||model.updateMean,cloud.components.mean(id,:)=mu;end
        if ~isfield(model,'updateCovariance')||model.updateCovariance,cloud.components.covariance(:,:,id)=S;end
    end
end
