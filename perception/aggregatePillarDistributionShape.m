function summary=aggregatePillarDistributionShape(points,pillarIds)
% aggregatePillarDistributionShape: Higher moments of complete pillars.
% Normalized centered moments describe asymmetry and tail concentration.
% Relative-height population fractions describe the complete distribution,
% without height voxels, object fits, neighborhoods or point decisions.
    [ids,~,group]=unique(pillarIds(:));n=numel(ids);
    names=["heightSkewness","heightKurtosis","radialKurtosis", ...
        "heightMeanFraction","heightVarianceFraction","heightLowerQuarterFraction", ...
        "heightMiddleHalfFraction","heightUpperQuarterFraction"];
    values=zeros(n,numel(names));
    if n>0
        count=accumarray(group,1,[n 1]);meanXYZ=zeros(n,3);
        for axis=1:3
            meanXYZ(:,axis)=accumarray(group,double(points(:,axis)),[n 1])./count;
        end
        delta=double(points)-meanXYZ(group,:);z=delta(:,3);
        variance=accumarray(group,z.^2,[n 1])./count;
        third=accumarray(group,z.^3,[n 1])./count;
        fourth=accumarray(group,z.^4,[n 1])./count;
        radial=sum(delta(:,1:2).^2,2);
        radialSecond=accumarray(group,radial,[n 1])./count;
        radialFourth=accumarray(group,radial.^2,[n 1])./count;
        lower=accumarray(group,double(points(:,3)),[n 1],@min);
        upper=accumarray(group,double(points(:,3)),[n 1],@max);span=upper-lower;
        normalized=(double(points(:,3))-lower(group))./max(span(group),1e-10);
        values=[third./max(variance.^1.5,1e-15),fourth./max(variance.^2,1e-20), ...
            radialFourth./max(radialSecond.^2,1e-20), ...
            (meanXYZ(:,3)-lower)./max(span,1e-10),variance./max(span.^2,1e-20), ...
            accumarray(group,double(normalized<.25),[n 1])./count, ...
            accumarray(group,double(normalized>=.25 & normalized<=.75),[n 1])./count, ...
            accumarray(group,double(normalized>.75),[n 1])./count];
        values(span<1e-10,[1 2 4:8])=0;
        values(~isfinite(values))=0;
    end
    summary=struct('pillarIndices',int32(ids),'names',names,'values',values);
end
