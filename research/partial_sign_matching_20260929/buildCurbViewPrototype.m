function buildCurbViewPrototype()
% buildCurbViewPrototype Offline orientation observations, separate from point views.
    setupVehicleLocalization;map=load('output/mississippi_mapping_calibrated/view_conditioned_cloud.mat','cloud');cloud=map.cloud;
    d=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');data=d.featureData;
    c=cloud.components;ids=find(c.semanticName=="curb");row=find(string(data.featureNames)=="curb");normal=zeros(numel(ids),2);
    for j=1:numel(ids),[v,e]=eig(c.covariance(:,:,ids(j)),'vector');[~,k]=min(e);normal(j,:)=v(:,k).';end
    observations=cell(c.numComponents,1);
    for k=1:1170
        p=data.pointsByFeatureFrame{row,k};if isempty(p),continue;end
        dx=p(:,1)-c.mean(ids,1).';dy=p(:,2)-c.mean(ids,2).';
        distance=dx.^2+dy.^2;dn=dx.*normal(:,1).'+dy.*normal(:,2).';distance(abs(dn)>.3 | distance>25)=Inf;
        [best,index]=min(distance,[],2);assigned=ids(index);assigned(~isfinite(best))=0;
        pose=poseRowToPlanarPose(data.framePoseTable(k,:));
        for id=unique(assigned(assigned>0)).'
            q=p(assigned==id,1:2);if size(q,1)<4,continue;end
            mu=mean(q,1);delta=q-mu;S=delta.'*delta/size(q,1);e=eig(S);
            if max(e)<.05 || max(e)<9*max(min(e),1e-8),continue;end
            observations{id}(end+1,:)=[pose,mu,S(1,1),S(1,2),S(2,2),size(q,1)];
        end
    end
    cloud.curbViews=observations;
    save('output/partial_sign_matching_20260929/curb_view_map.mat','cloud','-v7.3');
    fprintf('Curb observations %d\n',sum(cellfun(@(o)size(o,1),observations)));
end
