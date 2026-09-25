function T=summarizePolePointDistributions()
% summarizePolePointDistributions: Order statistics of reference pillar returns.
% Reference labels describe agreement, not independently annotated ground truth.
    root=setupVehicleLocalization(); folder=fullfile(root,'output','pole_distribution_20260925');
    data=load(fullfile(folder,'point_distributions.mat'),'records'); rows={};
    for k=1:numel(data.records)
        r=data.records{k}; xyz=double(r.points); ids=double(r.pillarIds); g=r.geometry;
        [uniqueIds,~,group]=unique(ids); counts=accumarray(group,1);
        minimum=accumarray(group,xyz(:,3),[],@min); maximum=accumarray(group,xyz(:,3),[],@max);
        evaluated=counts>=6 & maximum-minimum>=0.6;
        [fraction,height,isolation,peak,coreCount]=computePillarDensityCore(xyz,ids,g,0.1,0.15,0.6,evaluated);
        targets=find(evaluated & (ismember(uniqueIds,r.baselineCells)|ismember(uniqueIds,r.fineCells)|ismember(uniqueIds,r.currentCells)));
        for j=targets.'
            id=uniqueIds(j); own=ids==id; p=xyz(own,:); c=peak(j,:);
            distance=vecnorm(xyz(:,1:2)-c,2,2); core=xyz(distance<=0.15,:);
            z=sort(core(:,3)); gaps=diff(z); robust=quantile(z,[0.05 0.25 0.75 0.95]);
            fineOwn=xyz(own & r.finePointMask,:); fineNear=xyz(distance<=0.6 & r.finePointMask,:);
            if size(fineNear,1)>=3
                fz=sort(fineNear(:,3)); fspan=(max(fz)-min(fz)); fgap=max(diff(fz));
                a=[ones(numel(fz),1) fineNear(:,3)]\fineNear(:,1:2);
                ftilt=atand(norm(a(2,:))); frad=sqrt(mean(sum((fineNear(:,1:2)-[ones(numel(fz),1) fineNear(:,3)]*a).^2,2)));
            else
                fspan=NaN; fgap=NaN; ftilt=NaN; frad=NaN;
            end
            rows(end+1,:)={r.frame,id,size(p,1),mean(vecnorm(p(:,1:2),2,2)), ...
                ismember(id,r.baselineCells),ismember(id,r.fineCells),ismember(id,r.currentCells), ...
                fraction(j),height(j),isolation(j),coreCount(j), ...
                robust(4)-robust(1),robust(3)-robust(2),max(gaps), ...
                max(gaps)/max((max(z)-min(z)),eps),size(fineOwn,1),size(fineNear,1),fspan,fgap,ftilt,frad}; %#ok<AGROW>
        end
    end
    T=cell2table(rows,'VariableNames',{'frame','pillar','count','range','baseline','fine','current', ...
        'coreFraction','coreHeight','isolation','coreCount','robustHeight','interquartileHeight', ...
        'maximumGap','gapRatio','fineOwnCount','fineNearCount','fineHeight','fineGap','fineTilt','fineRadialStd'});
    writetable(T,fullfile(root,'research','pole_distribution_20260925','reference_distributions.csv'));
    groups={T.fine, T.baseline & ~T.fine, T.current & ~T.baseline & ~T.fine};
    for j=1:3
        fprintf('Group %d: %d pillars\n',j,nnz(groups{j}));
        disp(varfun(@median,T(groups{j},[8:15 18:21])));
    end
end
