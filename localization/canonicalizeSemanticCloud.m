function [cloud,groups]=canonicalizeSemanticCloud(cloud,radius,classes)
% canonicalizeSemanticCloud Merge same-class point components closer than radius.
% Greedy grouping in descending weight order: a component joins the first
% group whose weighted centroid lies within radius, otherwise it starts one.
% Each group becomes one Gaussian by moment matching (weighted mean, within
% plus between scatter, summed weights), so class mass and the first two
% moments of the mixture are preserved. Quality fields become weighted means,
% counts maxima, frame masks unions; identifiers and any other per-component
% field follow the heaviest member. Line classes (curb, facade) are never
% merged. A radius of zero returns the cloud unchanged. groups lists the
% original member indices of every output component in output order.
    arguments
        cloud (1,1) struct
        radius (1,1) double {mustBeNonnegative,mustBeFinite}
        classes (1,:) string=["pole","trafficSign"]
    end
    c=cloud.components;n=c.numComponents;groups=num2cell((1:n).');
    if radius<=0 || n<2,return;end
    % Only relative weights matter, so a common scale of the stored prior
    % cannot change the grouping or the merged statistics.
    names=string(c.semanticName(:));w=double(c.mixtureWeight(:));w=max(w/max(max(w),realmin),eps);
    groups=cell(0,1);
    for name=unique(names).'
        idx=find(names==name);
        if ~ismember(name,classes) || numel(idx)<2,groups=[groups;num2cell(idx)];continue;end %#ok<AGROW>
        [~,order]=sort(w(idx),'descend');idx=idx(order);xy=c.mean(idx,1:2);
        centroid=zeros(numel(idx),2);mass=zeros(numel(idx),1);member=cell(numel(idx),1);count=0;
        for i=1:numel(idx)
            j=0;
            if count>0
                [d,j]=min(sum((centroid(1:count,:)-xy(i,:)).^2,2));
                if d>radius^2,j=0;end
            end
            if j>0
                centroid(j,:)=(centroid(j,:)*mass(j)+xy(i,:)*w(idx(i)))/(mass(j)+w(idx(i)));
                mass(j)=mass(j)+w(idx(i));member{j}(end+1,1)=idx(i);
            else
                count=count+1;centroid(count,:)=xy(i,:);mass(count)=w(idx(i));member{count}=idx(i);
            end
        end
        groups=[groups;member(1:count)]; %#ok<AGROW>
    end
    [~,order]=sort(cellfun(@min,groups));groups=groups(order);
    m=numel(groups);heavy=zeros(m,1);merged=false(m,1);
    for g=1:m
        [~,j]=max(w(groups{g}));heavy(g)=groups{g}(j);merged(g)=numel(groups{g})>1;
    end
    % Every output component starts as its heaviest member; merged groups are
    % then overwritten with their moment-matched statistics.
    out=struct();
    for field=string(fieldnames(c)).'
        v=c.(field);
        if field=="numComponents",out.(field)=m;
        elseif ndims(v)==3 && size(v,3)==n,out.(field)=v(:,:,heavy);
        elseif ismatrix(v) && size(v,1)==n,out.(field)=v(heavy,:);
        else,out.(field)=v;
        end
    end
    fields=string(fieldnames(c)).';
    for g=find(merged).'
        i=groups{g};wi=w(i);total=sum(wi);
        out.mean(g,:)=sum(c.mean(i,:).*wi,1)/total;
        out.covariance(:,:,g)=momentMatch(c.covariance(:,:,i),c.mean(i,:),out.mean(g,:),wi);
        for field=intersect(["mixtureWeight","classMixtureWeight","mass","referenceMass"],fields)
            out.(field)(g)=sum(double(c.(field)(i)));
        end
        for field=intersect(["temporalStability","semanticProbability","occupancyProbability"],fields)
            out.(field)(g)=sum(double(c.(field)(i)).*wi)/total;
        end
        for field=intersect(["repeatability","detectionFrameCount"],fields)
            out.(field)(g)=max(double(c.(field)(i)));
        end
        if isfield(c,'detectionFrameMask'),out.detectionFrameMask(g,:)=any(c.detectionFrameMask(i,:),1);end
        if isfield(c,'heightAvailable')
            % The XYZ moments must marginalize exactly to the XY moments, so a
            % merged component carries height only when every member does.
            if all(c.heightAvailable(i))
                out.heightAvailable(g)=true;mu=sum(c.meanXYZ(i,:).*wi,1)/total;mu(1:2)=out.mean(g,1:2);
                S=momentMatch(c.covarianceXYZ(:,:,i),c.meanXYZ(i,:),mu,wi);S(1:2,1:2)=out.covariance(:,:,g);
                out.meanXYZ(g,:)=mu;out.covarianceXYZ(:,:,g)=S;
            else
                out.heightAvailable(g)=false;out.meanXYZ(g,:)=[out.mean(g,1:2),0];
                out.covarianceXYZ(:,:,g)=blkdiag(out.covariance(:,:,g),1);
            end
        end
    end
    cloud.components=out;
    if isfield(cloud,'heightEvidence')
        h=cloud.heightEvidence;e=h;e.mean=h.mean(heavy,:);e.covariance=h.covariance(:,:,heavy);e.available=h.available(heavy);
        for g=find(merged).'
            i=groups{g};wi=w(i);e.available(g)=all(h.available(i));
            if e.available(g)
                mu=sum(h.mean(i,:).*wi,1)/sum(wi);e.mean(g,:)=mu;e.covariance(:,:,g)=momentMatch(h.covariance(:,:,i),h.mean(i,:),mu,wi);
            else
                e.mean(g,:)=0;e.covariance(:,:,g)=eye(size(h.covariance,1));
            end
        end
        cloud.heightEvidence=e;
    end
end

function S=momentMatch(covariance,means,mu,w)
% Weighted within-plus-between scatter about mu over the leading dimensions.
    d=size(covariance,1);S=zeros(d);
    for k=1:numel(w)
        delta=(means(k,1:d)-mu(1:d)).';S=S+w(k)*(covariance(:,:,k)+delta*delta.');
    end
    S=S/sum(w);S=(S+S.')/2;
end
