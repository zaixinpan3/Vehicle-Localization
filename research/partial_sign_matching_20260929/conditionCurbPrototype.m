function cloud=conditionCurbPrototype(cloud,pose,bandwidth,mode)
% conditionCurbPrototype Research-only conditional target normal directions.
    c=cloud.components;
    for id=find(c.semanticName=="curb").'
        o=cloud.curbViews{id};if isempty(o),continue;end
        yaw=atan2(sin(o(:,3)-pose(3)),cos(o(:,3)-pose(3)));o=o(abs(yaw)<pi/6,:);if isempty(o),continue;end
        d2=sum((o(:,1:2)-pose(1:2)).^2,2);if min(d2)>100,continue;end
        w=exp(-.5*(d2-min(d2))/bandwidth^2);w=w/sum(w);
        S=[sum(w.*o(:,6)),sum(w.*o(:,7));sum(w.*o(:,7)),sum(w.*o(:,8))];
        [v,e]=eig(S,'vector');[~,order]=sort(e);v=v(:,order);
        old=sort(eig(c.covariance(:,:,id)));c.covariance(:,:,id)=v*diag(old)*v.';
        if mode=="mean",c.mean(id,:)=sum(o(:,4:5).*w,1);end
    end
    cloud.components=c;
end
