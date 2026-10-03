function [features,names]=measureDowntownDistributionContext(maps,ids)
% measureDowntownDistributionContext: Distribution-only structural context.
% Merge whole-pillar first and second moments in 3-by-3 and 5-by-5 windows.
% Radiometric conditional moments and ground-relative bounds remain summary
% features; this function has no access to raw points or semantic labels.
    features=zeros(numel(ids),0);names=strings(1,0);
    stats=maps.statistics;at=double(stats.pillarIndices);
    [found,~]=ismember(double(ids),at);assert(all(found));
    radiation=maps.radiometry;
    [found,radiometryRows]=ismember(double(ids),double(radiation.pillarIndices));assert(all(found));
    names(end+1)="radiometryFiniteCount";
    features(:,end+1)=radiation.finiteIntensityCount(radiometryRows);
    summaryNames=["count","fraction","minimumZ","meanZ","maximumZ","height", ...
        "widthX","widthY","varXX","covXY","varYY","covXZ","covYZ","varZZ", ...
        "verticalVarianceFraction","radialVariance","tilt"];
    for channel=["reflective","nonreflective"]
        value=radiation.(channel);cv=value.covarianceXYZ;span=value.maximumXYZ-value.minimumXYZ;
        vertical=cv(:,6)./max(cv(:,1)+cv(:,3)+cv(:,6),eps);
        radial=max(0,cv(:,1)+cv(:,3)-(cv(:,4).^2+cv(:,5).^2)./max(cv(:,6),eps));
        tilt=hypot(cv(:,4),cv(:,5))./max(cv(:,6),eps);
        data=[value.count,value.fraction,value.minimumXYZ(:,3),value.meanXYZ(:,3), ...
            value.maximumXYZ(:,3),span(:,3),span(:,1:2),cv,vertical,radial,tilt];
        data(value.count==0,:)=0;
        features=[features,data(radiometryRows,:)]; %#ok<AGROW>
        names=[names,channel+"_"+summaryNames]; %#ok<AGROW>
    end
    ground=radiation.ground;
    groundColumns=sort(string(fieldnames(ground))).';
    for key=groundColumns
        features(:,end+1)=ground.(key)(radiometryRows); %#ok<AGROW>
        names(end+1)="ground_"+key; %#ok<AGROW>
    end
    for radius=[0 1 2 4]
        prefix="r"+radius+"_";known=ground.(prefix+"count")>0;
        nearestKnown=ground.(prefix+"nearestCount")>0;
        for channel=["reflective","nonreflective"]
            value=radiation.(channel);present=value.count>0;
            differences=[value.minimumXYZ(:,3)-ground.(prefix+"meanZ"), ...
                value.meanXYZ(:,3)-ground.(prefix+"meanZ"), ...
                value.maximumXYZ(:,3)-ground.(prefix+"meanZ"), ...
                value.minimumXYZ(:,3)-ground.(prefix+"minimumZ"), ...
                value.minimumXYZ(:,3)-ground.(prefix+"nearestMeanZ")];
            differences(~(known & present),1:4)=0;
            differences(~(nearestKnown & present),5)=0;
            features=[features,differences(radiometryRows,:)]; %#ok<AGROW>
            names=[names,channel+"_ground_"+prefix+ ...
                ["minimumClearance","meanClearance","maximumClearance","lowerGroundClearance","nearestClearance"]]; %#ok<AGROW>
        end
    end
    for channel=["whole","nonreflective"]
        source=stats;if channel=="nonreflective",source=radiation.nonreflective;end
        [values,columns]=pooledShape(source,at,maps.mapSize,double(ids));
        features=[features,values];names=[names,channel+"_"+columns]; %#ok<AGROW>
    end
    features(~isfinite(features))=0;
    features=floor(features*1e8+.5)/1e8;
end

function [features,names]=pooledShape(stats,at,dims,query)
% Parallel-axis moment merging keeps all members of every source pillar.
    number=prod(dims);count=zeros(dims);count(at)=stats.count;
    occupied=count>0;mu=zeros(number,3);mu(at,:)=stats.meanXYZ;
    first=mu.*count(:);pairs=[1 1;1 2;2 2;1 3;2 3;3 3];
    second=zeros(number,6);
    for k=1:6
        second(at,k)=stats.count.*(stats.covarianceXYZ(:,k)+ ...
            stats.meanXYZ(:,pairs(k,1)).*stats.meanXYZ(:,pairs(k,2)));
    end
    features=zeros(numel(query),0);names=strings(1,0);
    for radius=0:2
        kernel=ones(2*radius+1);total=conv2(count,kernel,'same');
        cells=conv2(double(occupied),kernel,'same');means=zeros(numel(query),3);
        covariance=zeros(numel(query),6);n=reshape(total(query),[],1);
        for k=1:3
            merged=conv2(reshape(first(:,k),dims),kernel,'same');
            means(:,k)=reshape(merged(query),[],1)./max(n,1);
        end
        for k=1:6
            merged=conv2(reshape(second(:,k),dims),kernel,'same');
            covariance(:,k)=reshape(merged(query),[],1)./max(n,1)-means(:,pairs(k,1)).*means(:,pairs(k,2));
        end
        [shape,shapeNames]=eigenShape(covariance,n);
        values=[n,reshape(cells(query),[],1),means(:,3),covariance,shape];
        values(n==0,:)=0;
        features=[features,values]; %#ok<AGROW>
        names=[names,"r"+radius+"_"+["count","cells","meanZ", ...
            "varXX","covXY","varYY","covXZ","covYZ","varZZ",shapeNames]]; %#ok<AGROW>
    end
end

function [values,names]=eigenShape(packed,count)
    names=["largestVariance","middleVariance","smallestVariance", ...
        "principalVarianceFraction","linearity","planarity","scattering", ...
        "normalVerticalComponent","principalVerticalComponent","eigenEntropy"];
    values=zeros(size(packed,1),numel(names));
    for k=find(count>1).'
        c=packed(k,:);matrix=[c(1) c(2) c(4);c(2) c(3) c(5);c(4) c(5) c(6)];
        [vectors,lambda]=eig((matrix+matrix.')/2,'vector');
        [lambda,order]=sort(max(real(lambda),0),'descend');
        if sum(lambda)<=eps,continue;end
        largest=max(lambda(1),eps);fraction=lambda/sum(lambda);
        tolerance=1e-10*max(1,largest);
        normalVertical=0;principalVertical=0;
        if lambda(2)-lambda(3)>tolerance,normalVertical=abs(vectors(3,order(3)));end
        if lambda(1)-lambda(2)>tolerance,principalVertical=abs(vectors(3,order(1)));end
        values(k,:)=[lambda.',fraction(1),(lambda(1)-lambda(2))/largest, ...
            (lambda(2)-lambda(3))/largest,lambda(3)/largest, ...
            normalVertical,principalVertical, ...
            -sum(fraction.*log(max(fraction,eps)))];
    end
end
