function auditPoleRunExpansion()
% auditPoleRunExpansion: Hold a passing axis fixed while relaxing local contrast.
% The prior qualifying subrun still exists, but only maximal runs are scored.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    r=data.cases{find(cellfun(@(v)v.frame==71,data.cases),1)};
    target=7035;e=r.after.columnMaps.poleSubset;j=find(e.pillarIndices==target);
    assert(e.found(j));cfg=structuralPillarConfig(.6);cfg=cfg.pole.subset;
    [row,col]=ind2sub(r.geometry.mapSize,target);
    [rr,cc]=ind2sub(r.geometry.mapSize,double(r.pillarIds));
    reach=ceil(cfg.neighborhoodRadius./r.geometry.cellSize);
    selected=abs(rr-row)<=reach(2) & abs(cc-col)<=reach(1);
    q=double(r.points(selected,:));owned=double(r.pillarIds(selected))==target;
    distance=vecnorm(q(:,1:2)-e.axisXY(j,:)-(q(:,3)-e.axisZ(j)).*e.slopeXY(j,:),2,2);
    radius=e.radius(j);core=find(distance<=radius);[z,order]=sort(q(core,3));core=core(order);
    inner=arrayfun(@(zz)nnz(abs(z-zz)<=cfg.heightHalfWindow),z);
    outerZ=q(distance<=2*radius,3);
    outer=arrayfun(@(zz)nnz(abs(outerZ-zz)<=cfg.heightHalfWindow),z);
    contrast=3*inner./max(outer-inner,1);gap=cfg.maximumGap+cfg.rangeGapScale*norm(e.axisXY(j,:));
    rows={};baseline=[];relaxed=[];
    for threshold=[4 3]
        valid=inner>=cfg.minimumWindowPoints & contrast>=threshold;
        idx=core(valid);zz=q(idx,3);cuts=[0;find(diff(zz)>gap);numel(zz)];
        for k=1:numel(cuts)-1
            part=idx(cuts(k)+1:cuts(k+1));heights=q(part,3);
            if min(heights)>e.minimumZ(j)+1e-10 || max(heights)<e.maximumZ(j)-1e-10,continue;end
            record=measure(q,part,owned,distance,e,j,radius,threshold);
            record.interval="maximal";rows{end+1}=record; %#ok<AGROW>
            if threshold==4,baseline=part;else,relaxed=part;end
        end
    end
    assert(~isempty(baseline) && all(ismember(baseline,relaxed)));
    original=measure(q,baseline,owned,distance,e,j,radius,3);
    original.interval="retainedOldSubrun";rows{end+1}=original;
    T=struct2table(vertcat(rows{:}));
    assert(height(T)==3 && T.peakContrast(1)>=cfg.minimumPeakContrast);
    assert(T.peakContrast(2)<cfg.minimumPeakContrast && T.peakContrast(3)>=cfg.minimumPeakContrast);
    assert(T.points(2)>T.points(1) && T.points(3)==T.points(1));
    assert(all(T.points>=cfg.minimumPoints & T.ownPoints>=cfg.minimumOwnPoints));
    assert(all(T.height>=cfg.minimumHeight & T.robustHeight>=cfg.minimumRobustHeight));
    assert(all(T.ownHeight>=cfg.minimumOwnHeight & T.radialRms<=cfg.maximumRadialRms));
    assert(all(T.maximumGap<=T.gapLimit));
    writetable(T,fullfile(folder,'run_expansion.csv'));disp(T);
end

function record=measure(q,part,owned,distance,e,j,radius,threshold)
    z=q(part,3);own=part(owned(part));inHeight=q(:,3)>=min(z) & q(:,3)<=max(z);
    residual=q(inHeight,1:2)-e.axisXY(j,:)-(q(inHeight,3)-e.axisZ(j)).*e.slopeXY(j,:);
    directions=[1 0;0 1;1 1;1 -1;-1 0;0 -1;-1 -1;-1 1];
    directions=directions./vecnorm(directions,2,2);
    center=nnz(sum(residual.^2,2)<=radius^2);side=0;
    for k=1:8,side=max(side,nnz(sum((residual-radius*directions(k,:)).^2,2)<=radius^2));end
    record=struct('frame',71,'pillar',7035,'contrastThreshold',threshold, ...
        'interval',"",'points',numel(part),'ownPoints',numel(own), ...
        'minimumZ',min(z),'maximumZ',max(z),'height',max(z)-min(z), ...
        'robustHeight',diff(quantile(z,[.05 .95])), ...
        'ownHeight',max(q(own,3))-min(q(own,3)), ...
        'maximumGap',max(diff(z)),'gapLimit',.5+.005*norm(e.axisXY(j,:)), ...
        'radialRms',sqrt(mean(distance(part).^2)), ...
        'centerCount',center,'sideCount',side,'peakContrast',center/max(side,1));
end
