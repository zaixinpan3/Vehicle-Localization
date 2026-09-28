function auditMap601()
% auditMap601 Audit matched map support and the effect of curb support removal.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame601_matching_diagnosis_20260928';
    s=load(fullfile(out,'diagnostic.mat'));g=load(fullfile(out,'geometry.mat'));
    loaded=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');
    pmap=load('output/mississippi_mapping_calibrated/probability_cloud_map.mat','probabilityCloudMap');
    observations=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');
    c=loaded.cloud.components;target=1046;map=pmap.probabilityCloudMap.canonicalMap;
    layer=map.layers(map.classLabels=="pole");component=layer.components(layer.componentIds==erase(c.componentId(target),"pole:"));
    tile=layer.tiles(all(vertcat(layer.tiles.ownerTile)==component.ownerTile,2));frames=str2double(tile.observationBlockIds)+1;counts=component.blockEffectiveCounts;
    support=table(frames,counts,VariableNames={'frame','effectivePoints'});writetable(support,fullfile(dest,'map_pole_support.csv'));
    data=observations.featureData;channel=find(string(data.featureNames)=="pole");R=[cos(s.ref(3)) -sin(s.ref(3));sin(s.ref(3)) cos(s.ref(3))];
    rows=cell(0,7);
    for f=1:numel(data.pointsByFeatureFrame(channel,:))
        points=double(data.pointsByFeatureFrame{channel,f});if isempty(points),continue;end
        near=vecnorm(points(:,1:2)-c.mean(target,:),2,2)<1;
        if any(near)
            mu=mean(points(near,1:2),1);body=(mu-s.ref(1:2))*R;
            rows(end+1,:)={f,nnz(near),mu(1),mu(2),body(1),body(2),norm(mu-c.mean(target,:))}; %#ok<AGROW>
        end
    end
    pointsByFrame=cell2table(rows,VariableNames={'frame','pointCount','meanX','meanY','reference601BodyX','reference601BodyY','distanceToMapCenterM'});
    writetable(pointsByFrame,fullfile(dest,'map_pole_observed_centers.csv'));
    mapSummary=struct('globalTarget',target,'componentId',c.componentId(target),'mean',c.mean(target,:), ...
        'covariance',c.covariance(:,:,target),'repeatability',c.repeatability(target),'mixtureWeight',c.mixtureWeight(target), ...
        'supportedFramesOverHalfPoint',nnz(counts>.5),'firstSupportedFrame',min(frames(counts>.5)),'lastSupportedFrame',max(frames(counts>.5)), ...
        'currentFineGlobalCenter',g.poleSummary.nearbyFinePoleCenter(1:2)*R.'+s.ref(1:2));
    fid=fopen(fullfile(dest,'map_pole_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(mapSummary,PrettyPrint=true));fclose(fid);
    % More curb support is diagnostic: do not relax deployed selection here.
    restored=replaceClass(s.source,g.unfiltered,"curb");
    restoredOracle=replaceClass(g.oracle,g.unfiltered,"curb");
    inputs={restored,restoredOracle};names=["restore_curb_only","restore_curb_and_map_center_pole_oracle"];
    rows=cell(2,7);results=cell(2,1);
    for k=1:2
        r=registerSemanticProbabilityCloud(s.fixed,inputs{k},s.seed,s.cfg.registration);results{k}=r;d=r.poseXYTheta-s.ref;b=d(1:2)*R;
        rows(k,:)={names(k),r.accepted,r.reason,norm(d(1:2)),b(1),b(2),rad2deg(atan2(sin(d(3)),cos(d(3))))};
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','reason','errorM','forwardErrorM','leftErrorM','yawErrorDeg'});
    writetable(controls,fullfile(dest,'support_controls.csv'));disp(controls);
    geometry=cell(0,7);
    for k=1:2
        cloud=s.source;if k==2,cloud=g.unfiltered;end
        for j=1:cloud.components.numComponents
            geometry(end+1,:)={k,j,cloud.components.semanticName(j),cloud.components.mean(j,1),cloud.components.mean(j,2), ...
                cloud.components.temporalStability(j),cloud.components.detectionFrameCount(j)}; %#ok<AGROW>
        end
    end
    geometry=cell2table(geometry,VariableNames={'production1Unfiltered2','source','class','x','y','temporalStability','detections'});
    writetable(geometry,fullfile(dest,'source_geometry.csv'));
    % Export matched map means, normal directions, and support extents.
    ids=unique(s.pairs.globalTarget);rows=cell(numel(ids),10);
    for k=1:numel(ids)
        id=ids(k);cov=R.'*c.covariance(:,:,id)*R;[v,d]=eig(cov,'vector');[minor,j]=min(d);normal=v(:,j);body=(c.mean(id,:)-s.ref(1:2))*R;
        rows(k,:)={id,c.semanticName(id),body(1),body(2),normal(1),normal(2),sqrt(minor),sqrt(max(d)),c.mixtureWeight(id),rad2deg(atan2(normal(1),-normal(2)))};
    end
    targets=cell2table(rows,VariableNames={'globalTarget','class','x','y','normalX','normalY','minorStd','majorStd','weight','tangentAngleDeg'});
    writetable(targets,fullfile(dest,'map_targets.csv'));disp(targets);disp(mapSummary);disp(pointsByFrame(pointsByFrame.frame>=595&pointsByFrame.frame<=605,:));
    save(fullfile(out,'map_audit.mat'),'mapSummary','pointsByFrame','support','targets','restored','restoredOracle','results','controls','geometry');
end
function out=replaceClass(a,b,name)
    out=a;ac=a.components;bc=b.components;ka=ac.semanticName~=name;kb=bc.semanticName==name;
    c=struct();
    for field=["mean","semanticName","mixtureWeight","semanticProbability","occupancyProbability","temporalStability"]
        c.(field)=[ac.(field)(ka,:);bc.(field)(kb,:)];
    end
    c.covariance=cat(3,ac.covariance(:,:,ka),bc.covariance(:,:,kb));c.numComponents=size(c.mean,1);out.components=c;
    if isfield(out,'heightEvidence'),out=rmfield(out,'heightEvidence');end
end
