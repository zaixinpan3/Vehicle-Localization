function summary=aggregatePillarDistributionShape(points,pillarIds)
% aggregatePillarDistributionShape: Higher moments of complete pillars.
% Normalized centered moments describe asymmetry and tail concentration.
% Relative-height population fractions describe the complete distribution,
% without height voxels, object fits, neighborhoods or point decisions.
    [ids,~,group]=unique(pillarIds(:));n=numel(ids);
    names=["heightSkewness","heightKurtosis","radialKurtosis", ...
        "heightMeanFraction","heightVarianceFraction","heightLowerQuarterFraction", ...
        "heightMiddleHalfFraction","heightUpperQuarterFraction", ...
        "heightInterquartileFraction","heightTrimmedSpanFraction", ...
        "heightMedianFraction","heightQuantileAsymmetry","heightMaximumGapFraction", ...
        "heightDecileEntropy","heightDecileMaximumFraction","heightDecileOccupancyFraction", ...
        "heightRadialCorrelation","transverseRadius90Meters", ...
        "transverseRadiusMaximumMeters","transverseKurtosis","heightSquaredTransverseCoupling"];
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
        % Order statistics and empirical-CDF concentration summarize whole
        % populations. No point is selected/rejected and no height voxel
        % topology or per-height object support leaves this aggregation.
        sorted=sortrows([group,normalized],[1 2]);start=cumsum([1;count(1:end-1)]);
        quantiles=zeros(n,5);probabilities=[.1 .25 .5 .75 .9];
        for k=1:5
            position=(count-1)*probabilities(k);low=floor(position);high=ceil(position);
            quantiles(:,k)=sorted(start+low,2).*(high-position)+ ...
                sorted(start+high,2).*(position-low);
            exact=low==high;quantiles(exact,k)=sorted(start(exact)+low(exact),2);
        end
        gaps=diff(sorted(:,2));same=diff(sorted(:,1))==0;
        maximumGap=accumarray(sorted(find(same),1),gaps(same),[n 1],@max,0);
        normalizedBins=floor(normalized*1e10+.5)/1e10;
        bins=min(10,max(1,floor(normalizedBins*10)+1));
        histogram=accumarray([group,bins],1,[n 10])./count;
        entropy=-sum(histogram.*log(max(histogram,eps)),2)/log(10);
        radialCentered=radial-radialSecond(group);
        radialVariance=accumarray(group,radialCentered.^2,[n 1])./count;
        coupling=accumarray(group,z.*radialCentered,[n 1])./count;
        correlation=zeros(n,1);
        known=variance>1e-14 & radialVariance>max(1e-20,radialSecond.^2*1e-12);
        correlation(known)=coupling(known)./sqrt(variance(known).*radialVariance(known));
        more=[quantiles(:,4)-quantiles(:,2),quantiles(:,5)-quantiles(:,1), ...
            quantiles(:,3),(quantiles(:,5)+quantiles(:,1)-2*quantiles(:,3))./ ...
            max(quantiles(:,5)-quantiles(:,1),1e-10),maximumGap,entropy, ...
            max(histogram,[],2),sum(histogram>0,2)./min(count,10),min(1,max(-1,correlation))];
        more(span<1e-10,:)=0;values=[values,more];
        xz=accumarray(group,delta(:,1).*z,[n 1])./count;
        yz=accumarray(group,delta(:,2).*z,[n 1])./count;
        beta=[xz,yz]./max(variance,1e-14);
        transverse=delta(:,1:2)-beta(group,:).*z;
        radiusSquared=sum(transverse.^2,2);
        radialMean=accumarray(group,radiusSquared,[n 1])./count;
        radialFourth=accumarray(group,radiusSquared.^2,[n 1])./count;
        sortedRadius=sortrows([group,sqrt(radiusSquared)],[1 2]);
        position=(count-1)*.9;low=floor(position);high=ceil(position);
        percentile=sortedRadius(start+low,2).*(high-position)+sortedRadius(start+high,2).*(position-low);
        exact=low==high;percentile(exact)=sortedRadius(start(exact)+low(exact),2);
        maximum=accumarray(group,sqrt(radiusSquared),[n 1],@max,0);
        zSquared=z.^2-variance(group);
        varianceSquared=accumarray(group,zSquared.^2,[n 1])./count;
        crossX=accumarray(group,zSquared.*transverse(:,1),[n 1])./count;
        crossY=accumarray(group,zSquared.*transverse(:,2),[n 1])./count;
        coupling=zeros(n,1);known=varianceSquared>1e-14 & radialMean>1e-12;
        coupling(known)=hypot(crossX(known),crossY(known))./sqrt(varianceSquared(known).*radialMean(known));
        values=[values,percentile,maximum,radialFourth./max(radialMean.^2,1e-20),min(1,coupling)];
        values(~isfinite(values))=0;
    end
    summary=struct('pillarIndices',int32(ids),'names',names,'values',values);
end
