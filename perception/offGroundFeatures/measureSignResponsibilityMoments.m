function stats=measureSignResponsibilityMoments(points,pillarIds,intensity,selected,threshold,softness)
% measureSignResponsibilityMoments Traffic-sign sufficient statistics of whole pillars.
% Each return contributes to its pillar's sign distribution in proportion to
% r(I)=1/(1+exp(-(I-threshold)/softness)), the sign posterior of a two-class
% intensity model in which intensity is independent of position within each
% class. The sums of r, r*p and r*p*p' are the expected sufficient statistics of
% the sign component (an EM M-step with intensity-only responsibilities): no
% return is classified or discarded, and sum(r) is the expected number of sign
% returns. Dim returns of the post, foliage or clutter therefore barely move
% the sign moments. SELECTED lists the pillar ids to evaluate. The result has the
% fields of aggregatePillarStatistics for those pillars: count is the soft mass
% sum(r); covarianceXYZ is packed as [xx xy yy xz yz zz].
    arguments
        points (:,3) double
        pillarIds (:,1)
        intensity (:,1) double
        selected (:,1) double
        threshold (1,1) double {mustBeFinite}
        softness (1,1) double {mustBePositive,mustBeFinite}
    end
    keep=ismember(double(pillarIds),selected) & isfinite(intensity) & all(isfinite(points),2);
    [ids,~,group]=unique(double(pillarIds(keep)));n=numel(ids);
    p=points(keep,:);r=1./(1+exp(-(intensity(keep)-threshold)/softness));
    mass=accumarray(group,r,[n 1]);
    mu=zeros(n,3);
    for a=1:3,mu(:,a)=accumarray(group,r.*p(:,a),[n 1])./max(mass,realmin);end
    centered=p-mu(group,:);pairs=[1 1;1 2;2 2;1 3;2 3;3 3];covariance=zeros(n,6);
    for k=1:6
        covariance(:,k)=accumarray(group,r.*centered(:,pairs(k,1)).*centered(:,pairs(k,2)),[n 1])./max(mass,realmin);
    end
    stats=struct('pillarIndices',int32(ids),'count',mass,'meanXYZ',mu, ...
        'covarianceXYZ',covariance,'covarianceOrder',"xx xy yy xz yz zz");
end
