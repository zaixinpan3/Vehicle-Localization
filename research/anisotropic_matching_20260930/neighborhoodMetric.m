function covariance=neighborhoodMetric(c,radius)
% neighborhoodMetric Prototype coherent support moments shared by all classes.
% The compatibility kernel links overlapping Gaussian supports, not labels
% from fine perception. Means remain the original association coordinates.
    covariance=c.covariance(1:2,1:2,:);original=covariance;
    for i=1:c.numComponents
        ids=find(c.semanticName==c.semanticName(i) & sum((c.mean(:,1:2)-c.mean(i,1:2)).^2,2)<=radius^2);
        delta=c.mean(ids,1:2)-c.mean(i,1:2);S=original(:,:,ids)+original(:,:,i);
        a=reshape(S(1,1,:),[],1);b=reshape(S(1,2,:),[],1);d=reshape(S(2,2,:),[],1);
        q=(d.*delta(:,1).^2-2*b.*delta(:,1).*delta(:,2)+a.*delta(:,2).^2)./(a.*d-b.^2);
        w=exp(-q/5);w=w/sum(w);mu=sum(delta.*w,1);centered=delta-mu;
        covariance(:,:,i)=sum(original(:,:,ids).*reshape(w,1,1,[]),3)+centered.'*(centered.*w);
    end
end
