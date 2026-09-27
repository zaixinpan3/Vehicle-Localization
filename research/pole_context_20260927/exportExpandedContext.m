function T=exportExpandedContext()
% exportExpandedContext: Diagnose width growth beyond a cropped cylinder.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'),'cases');
    previous=load(fullfile(root,'output','pole_context_20260927','development.mat'),'cache','T');
    rows={};
    for k=1:numel(previous.cache)
        old=previous.cache{k};c=data.cases{find(cellfun(@(v)v.frame==old.frame,data.cases),1)};
        for j=1:numel(old.rows)
            f=old.rows{j};h=old.hypotheses(f.hypothesis);p=double(c.points);
            d=p(:,1:2)-h.axisXY-(p(:,3)-h.axisZ).*h.slopeXY;r=vecnorm(d,2,2);
            selected=p(:,3)>=h.minimumZ & p(:,3)<=h.maximumZ;
            for radius=[.25 .30 .35 .40 .50 .60 .75]
                tag=sprintf('%02d',round(100*radius));use=selected & r<=radius;
                covar=cov(d(use,:));e=sort(max(eig(covar),0));
                f.(['std' tag])=sqrt(e(2));f.(['aspect' tag])=sqrt(e(2)/max(e(1),eps));
                f.(['rms' tag])=sqrt(mean(r(use).^2));f.(['count' tag])=nnz(use);
                f.(['coreFraction' tag])=nnz(selected & r<=.25)/nnz(use);
                f.(['shift' tag])=norm(mean(d(use,:),1));
                qq=prctile(r(use),[50 75 90 95]);
                percentiles=[50 75 90 95];
                for v=1:4,f.(sprintf('radius%d_%s',percentiles(v),tag))=qq(v);end
            end
            own=selected & c.pillarIds==f.pillar;
            f.ownerMedianRadius=median(r(own));f.ownerCoreFraction=nnz(own & r<=.15)/max(nnz(own),1);
            [a,b]=ind2sub(c.geometry.mapSize,f.pillar);
            lower=c.geometry.origin+([b a]-1).*c.geometry.cellSize;
            at=h.axisXY+([h.minimumZ;h.maximumZ]-h.axisZ).*h.slopeXY;
            f.axisOwnerDistance=min(vecnorm(max(max(lower-at,at-lower-c.geometry.cellSize),0),2,2));
            rows{end+1,1}=f; %#ok<AGROW>
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'expanded_features.csv'));
end
