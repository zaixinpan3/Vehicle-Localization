function [seeds,aliases]=enumerateMapAliasSeeds(map,source,seed,options)
% enumerateMapAliasSeeds Alias displacements from local map self-similarity.
% Two same-class point components closer than MaximumSeparation let a source
% object lock onto either of them, so the registration cost has a second
% basin displaced by their offset. The weighted pairwise offsets of the local
% map are clustered and returned as seeds seed -/+ delta at the seed yaw.
% Only classes present in the source count; line classes are excluded because
% their tangential aliases are already unobservable.
    arguments
        map (1,1) struct
        source (1,1) struct
        seed (1,3) double {mustBeFinite}
        options.SearchRadius (1,1) double=40
        options.MinimumSeparation (1,1) double=0.25
        options.MaximumSeparation (1,1) double=1.5
        options.ClusterRadius (1,1) double=0.15
        options.MinimumWeightRatio (1,1) double=0.05
        options.MaximumAliases (1,1) double=3
        options.PointClasses (1,:) string=["pole","trafficSign","facade"]
    end
    c=map.components;names=string(c.semanticName(:));
    present=intersect(intersect(unique(names),unique(string(source.components.semanticName(:)))),options.PointClasses);
    near=vecnorm(c.mean(:,1:2)-seed(1:2),2,2)<=options.SearchRadius;
    offsets=zeros(0,2);weights=zeros(0,1);classes=strings(0,1);
    for name=present(:).'
        idx=find(near & names==name & c.mixtureWeight(:)>0);
        if numel(idx)<2,continue;end
        w=c.mixtureWeight(idx);keep=w>=options.MinimumWeightRatio*max(w);idx=idx(keep);w=w(keep);
        m=c.mean(idx,1:2);
        for i=1:numel(idx)-1
            d=m(i+1:end,:)-m(i,:);r=vecnorm(d,2,2);
            sel=find(r>options.MinimumSeparation & r<=options.MaximumSeparation);
            offsets=[offsets;d(sel,:)];weights=[weights;w(i)*w(i+sel)];classes=[classes;repmat(name,numel(sel),1)]; %#ok<AGROW>
        end
    end
    % Greedy clustering of offsets up to sign; the returned aliases are the
    % cluster centroids ordered by summed pair weight.
    [weights,order]=sort(weights,'descend');offsets=offsets(order,:);classes=classes(order);
    centroids=zeros(0,2);mass=zeros(0,1);members=zeros(0,1);label=strings(0,1);
    for i=1:numel(weights)
        d=offsets(i,:);assigned=false;
        for j=1:size(centroids,1)
            if norm(d-centroids(j,:))<=options.ClusterRadius || norm(d+centroids(j,:))<=options.ClusterRadius
                if norm(d+centroids(j,:))<norm(d-centroids(j,:)),d=-d;end
                centroids(j,:)=(centroids(j,:)*mass(j)+d*weights(i))/(mass(j)+weights(i));
                mass(j)=mass(j)+weights(i);members(j)=members(j)+1;assigned=true;break;
            end
        end
        if ~assigned
            centroids(end+1,:)=d;mass(end+1,1)=weights(i);members(end+1,1)=1;label(end+1,1)=classes(i); %#ok<AGROW>
        end
    end
    [mass,order]=sort(mass,'descend');centroids=centroids(order,:);members=members(order);label=label(order);
    keep=1:min(options.MaximumAliases,size(centroids,1));
    aliases=table(centroids(keep,:),mass(keep),members(keep),label(keep),VariableNames={'offsetXY','pairWeight','pairs','class'});
    seeds=zeros(0,3);
    for j=keep
        seeds(end+1,:)=[seed(1:2)-centroids(j,:),seed(3)]; %#ok<AGROW>
        seeds(end+1,:)=[seed(1:2)+centroids(j,:),seed(3)]; %#ok<AGROW>
    end
end
