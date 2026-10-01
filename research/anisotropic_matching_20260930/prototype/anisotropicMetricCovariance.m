function covariance=anisotropicMetricCovariance(covariance,exponent)
% anisotropicMetricCovariance Continuous uncertainty along incomplete support.
% Preserve minor-axis variance and inflate major variance by the eigenvalue
% ratio^(exponent-1). Exponent one uses measured scatter without inflation.
% This is a geometric metric, not calibrated pose/sensor uncertainty.
    if exponent==1,return;end
    for k=1:size(covariance,3)
        [v,d]=eig(covariance(:,:,k),'vector');[minor,i]=min(d);[major,j]=max(d);
        if i==j,continue;end
        d(j)=major*(major/minor)^(exponent-1);
        covariance(:,:,k)=v*diag(d)*v.';
    end
end
