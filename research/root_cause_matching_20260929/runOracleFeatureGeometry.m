function runOracleFeatureGeometry()
% runOracleFeatureGeometry DIAGNOSTIC ONLY: original fine labels replace centers.
% Fixed selected output cells retain their covariances/counts and fallback
% centers. This uses unavailable online labels and is never a deployable model.
    root=setupVehicleLocalization();out='output/root_cause_matching_20260929';
    base=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');
    refs=cell(1170,3);names=["curb","pole","trafficSign"];
    for j=1:4
        a=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',j),'frames','selectedIndices','cfg');
        for c=1:3,refs(a.frames,c)=a.selectedIndices(:,string(a.cfg.featureNames)==names(c));end
    end
    mc=featureMapBuildConfig();store=matfile(fullfile(root,'data',mc.pointCloudMatPath));allClouds=cell(1170,3);frames=[];first=0;
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);old=base.currentClouds{k};means=old.components.mean;
        for c=1:3
            ids=double(refs{k,c});xyz=double([raw.x(ids),raw.y(ids),raw.z(ids)])*old.projectionRotation.'+old.projectionTranslation;
            g=old.geometry;bin=floor((xyz(:,1:2)-[g.xMin g.yMin])/g.resolution)+1;
            valid=all(bin>=1,2)&bin(:,1)<=g.dims(2)&bin(:,2)<=g.dims(1);bin=bin(valid,:);xyz=xyz(valid,:);
            cellIds=sub2ind(g.dims,bin(:,2),bin(:,1));count=accumarray(cellIds,1,[g.numCells 1]);
            mu=[accumarray(cellIds,xyz(:,1),[g.numCells 1]),accumarray(cellIds,xyz(:,2),[g.numCells 1])]./max(count,1);
            selected=find(old.components.semanticName==names(c));outputIds=double(old.components.cellLinIdx(selected));
            selected=selected(count(outputIds)>0);means(selected,:)=mu(double(old.components.cellLinIdx(selected)),:);
        end
        for variant=1:3
            cloud=old;mask=true(size(means,1),1);
            if variant==1,mask=cloud.components.semanticName=="curb";elseif variant==2,mask=cloud.components.semanticName~="curb";end
            cloud.components.mean(mask,:)=means(mask,:);cloud.components.meanXYZ(mask,1:2)=means(mask,:);allClouds{k,variant}=cloud;
        end
    end
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');wc=localizationSourceWindowConfig();
    for variant=1:3
        currentClouds=allClouds(:,variant);sources=cell(1170,1);history=[];
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);end
        label="oracleFeature"+variant;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');rootCauseReplay(file,label,distributionRegistrationConfig());
    end
end
