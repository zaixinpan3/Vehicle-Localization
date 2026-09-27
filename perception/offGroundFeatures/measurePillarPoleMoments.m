function f=measurePillarPoleMoments(points,lower,cellSize,h)
% measurePillarPoleMoments: Joint XYZ moments of a whole owner and its shaft.
% Continuous polynomial expectations retain height-dependent lateral shape.
% There are no spatial bins or sub-pillar ownership labels in this descriptor.
    p=double(points);xy=2*(p(:,1:2)-lower)./cellSize-1;
    z=max(-1,min(1,2*(p(:,3)-h.axisZ)/max(h.maximumZ-h.minimumZ,.5)));
    r=p(:,1:2)-h.axisXY-(p(:,3)-h.axisZ).*h.slopeXY;
    core=sum(r.^2,2)<=.25^2 & p(:,3)>=h.minimumZ & p(:,3)<=h.maximumZ;
    persistent powers names
    if isempty(powers)
        powers=zeros(34,3);names=cell(1,83);index=0;
        for i=0:4
            for j=0:4-i
                for k=0:4-i-j
                    if i+j+k==0,continue;end
                    index=index+1;powers(index,:)=[i j k];name=sprintf('moment%d%d%d',i,j,k);
                    names{2*index-1}=name;names{2*index}=['core_' name];
                end
            end
        end
        for dimension=1:3
            for j=1:5,names{68+(dimension-1)*5+j}=sprintf('quantile%d_%d',dimension,j);end
        end
    end
    v=[xy,z];px=ones(size(v,1),5);py=px;pz=px;
    for d=1:4
        px(:,d+1)=px(:,d).*v(:,1);py(:,d+1)=py(:,d).*v(:,2);pz(:,d+1)=pz(:,d).*v(:,3);
    end
    values=px(:,powers(:,1)+1).*py(:,powers(:,2)+1).*pz(:,powers(:,3)+1);
    moments=[mean(values,1);mean(values(core,:),1)];quantiles=quantile(v,[.1 .25 .5 .75 .9],1);
    f=cell2struct(num2cell([moments(:).',quantiles(:).']),names,2);
end
