function [cloud,groups]=canonicalizeSemanticCloud(cloud,radius,classes)
% canonicalizeSemanticCloud Merge same-class point components closer than radius.
% Greedy single-linkage grouping by weight; each group becomes one Gaussian by
% moment matching (mean = weighted mean, covariance = within + between scatter,
% weights summed). Quality fields are weighted means, counts are maxima, masks
% are unions, identifiers follow the heaviest member. Line classes are left
% untouched. radius <= 0 returns the cloud unchanged.
    arguments
        cloud (1,1) struct
        radius (1,1) double {mustBeNonnegative}
        classes (1,:) string=["pole","trafficSign","facade"]
    end
    c=cloud.components;n=c.numComponents;names=string(c.semanticName(:));
    w=double(c.mixtureWeight(:));w=max(w,eps);
    groups=num2cell((1:n).');
    if radius>0
        groups={};
        for name=unique(names).'
            idx=find(names==name);
            if ~ismember(name,classes),groups=[groups;num2cell(idx)];continue;end %#ok<AGROW>
            [~,order]=sort(w(idx),'descend');idx=idx(order);centroids=zeros(0,2);members={};mass=zeros(0,1);
            for i=idx.'
                d=inf;j=0;
                if ~isempty(centroids),[d,j]=min(vecnorm(centroids-c.mean(i,1:2),2,2));end
                if d<=radius
                    centroids(j,:)=(centroids(j,:)*mass(j)+c.mean(i,1:2)*w(i))/(mass(j)+w(i));mass(j)=mass(j)+w(i);members{j}(end+1)=i;
                else
                    centroids(end+1,:)=c.mean(i,1:2);mass(end+1,1)=w(i);members{end+1}=i; %#ok<AGROW>
                end
            end
            groups=[groups;cellfun(@(m)m(:),members(:),UniformOutput=false)]; %#ok<AGROW>
        end
        % Restore the original order by first member so downstream indices stay stable.
        [~,order]=sort(cellfun(@min,groups));groups=groups(order);
    end
    m=numel(groups);heavy=zeros(m,1);
    for g=1:m,[~,j]=max(w(groups{g}));heavy(g)=groups{g}(j);end
    out=struct();
    for field=string(fieldnames(c)).'
        v=c.(field);
        switch field
            case "numComponents",out.(field)=m;
            case {"mean","meanXYZ"},out.(field)=weightedMean(v,groups,w);
            case "covariance",S=momentMatch(v,c.mean,groups,w);out.(field)=(S+permute(S,[2 1 3]))/2;
            case "covarianceXYZ",out.(field)=momentMatch(v,c.meanXYZ,groups,w);
            case {"mixtureWeight","classMixtureWeight","mass","referenceMass"},out.(field)=cellfun(@(g)sum(double(v(g))),groups);
            case {"temporalStability","semanticProbability","occupancyProbability"}
                out.(field)=cellfun(@(g)sum(double(v(g)).*w(g))/sum(w(g)),groups);
            case {"repeatability","detectionFrameCount"},out.(field)=cellfun(@(g)max(double(v(g))),groups);
            case {"heightAvailable"},out.(field)=cellfun(@(g)any(v(g)),groups);
            case {"detectionFrameMask"},out.(field)=cell2mat(cellfun(@(g)any(v(g,:),1),groups,UniformOutput=false));
            otherwise
                if size(v,1)==n && ndims(v)==2,out.(field)=v(heavy,:);
                elseif ndims(v)==3 && size(v,3)==n,out.(field)=v(:,:,heavy);
                else,out.(field)=v;end
        end
    end
    if isfield(c,'heightAvailable') && isfield(c,'meanXYZ')
        % The XYZ moments must marginalize exactly to the XY moments, so a
        % group carries height only when every member does.
        for g=1:m
            members=groups{g};
            if all(c.heightAvailable(members))
                out.heightAvailable(g)=true;
                out.meanXYZ(g,:)=weightedMean(c.meanXYZ,{members},w);
                S=momentMatch(c.covarianceXYZ,c.meanXYZ,{members},w);out.covarianceXYZ(:,:,g)=(S+S.')/2;
                out.meanXYZ(g,1:2)=out.mean(g,1:2);out.covarianceXYZ(1:2,1:2,g)=out.covariance(:,:,g);
            else
                out.heightAvailable(g)=false;out.meanXYZ(g,:)=[out.mean(g,1:2),0];
                out.covarianceXYZ(:,:,g)=blkdiag(out.covariance(:,:,g),1);
            end
        end
    end
    cloud.components=out;
    if isfield(cloud,'heightEvidence')
        h=cloud.heightEvidence;e=struct('mean',zeros(m,3),'covariance',zeros(3,3,m),'available',false(m,1));
        for g=1:m
            members=groups{g};e.available(g)=all(h.available(members));
            if e.available(g)
                e.mean(g,:)=weightedMean(h.mean,{members},w);S=momentMatch(h.covariance,h.mean,{members},w);e.covariance(:,:,g)=(S+S.')/2;
            end
        end
        for field=setdiff(string(fieldnames(h)).',["mean","covariance","available"]),e.(field)=h.(field);end
        cloud.heightEvidence=e;
    end
end

function mu=weightedMean(v,groups,w)
    mu=zeros(numel(groups),size(v,2));
    for g=1:numel(groups),i=groups{g};mu(g,:)=sum(v(i,:).*w(i),1)/sum(w(i));end
end

function S=momentMatch(cov,means,groups,w)
    d=size(cov,1);S=zeros(d,d,numel(groups));
    for g=1:numel(groups)
        i=groups{g};mu=sum(means(i,:).*w(i),1)/sum(w(i));acc=zeros(d);
        for k=i(:).'
            delta=(means(k,:)-mu).';acc=acc+w(k)*(cov(:,:,k)+delta*delta.');
        end
        S(:,:,g)=acc/sum(w(i));
    end
end
