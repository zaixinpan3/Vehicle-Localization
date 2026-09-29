function traceSignSupport()
% traceSignSupport Audit original point membership, without changing perception.
    root=setupVehicleLocalization(); dest=fileparts(mfilename('fullpath'));
    s=load('output/frame178_diagnosis_20260929/diagnostic.mat');
    d=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');
    ids=load('output/root_cause_matching_20260929/map_point_indices.mat','indices');
    fd=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');
    od=load('output/line_direction_matching_20260928/sources.mat','motion');
    mc=featureMapBuildConfig();store=matfile(fullfile(root,'data',mc.pointCloudMatPath));raw=store.pointClouds(1,174:178);
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    signClass=find(string(fd.featureData.featureNames)=="trafficSign");
    rows=cell(0,20);points=cell(0,9);signIds=find(s.source.components.semanticName=="trafficSign");
    state=struct('mean',zeros(0,2),'covariance',zeros(2,2,0),'count',zeros(0,1));
    mapBody=(s.fixed.components.mean(1074,:)-s.ref(1:2))*rot(s.ref(3));
    for k=174:178
        [reference,tilt]=poseRowToPlanarPose(fd.featureData.framePoseTable(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(raw(k-173),cfg);cloud=p.probabilityCloud;
        assert(isequaln(cloud,d.currentClouds{k}));c=cloud.components;
        maps=p.diagnostics.offGround.columnMaps;context=p.diagnostics.offGroundPointContext;
        selected=find(p.diagnostics.offGround.trafficSignCellMask);
        mu=maps.moments.mean(selected,:);g=cloud.geometry;
        col=floor((mu(:,1)-g.xMin)/g.resolution)+1;row=floor((mu(:,2)-g.yMin)/g.resolution)+1;
        group=sub2ind(g.dims,row,col);
        delta=od.motion(k,:)-od.motion(178,:);rt=rot(delta(3));shift=delta(1:2)*rot(od.motion(178,3));
        refDelta=reference-s.ref;rref=rot(refDelta(3));tref=refDelta(1:2)*rot(s.ref(3));
        currentIds=find(c.semanticName=="trafficSign");
        transportedMean=c.mean(currentIds,:)*rt.'+shift;
        transportedCov=pagemtimes(pagemtimes(rt,c.covariance(:,:,currentIds)),rt.');
        [state,assignment]=associate(state,transportedMean,transportedCov);
        for jj=1:numel(currentIds)
            id=currentIds(jj);
            member=ismember(double(context.pointPillarLinIdx),selected(group==double(c.cellLinIdx(id))));
            q=context.points(member,:);original=context.pointIndices(member);
            fine=ismember(original,ids.indices{signClass,k});
            local=q*cloud.projectionRotation.'+cloud.projectionTranslation;
            assert(size(q,1)==c.count(id));assert(max(abs(mean(local(:,1:2),1)-c.mean(id,:)))<1e-9);
            transported=local(:,1:2)*rt.'+shift;oracle=local(:,1:2)*rref.'+tref;
            center=mean(transported,1);fineCenter=mean(transported(fine,:),1);
            rows(end+1,:)={k,id,assignment(jj),size(q,1),nnz(fine),mean(fine),center(1),center(2),fineCenter(1),fineCenter(2), ...
                mean(local(:,3)),min(local(:,3)),max(local(:,3)),c.semanticProbability(id),c.occupancyProbability(id), ...
                norm(center-mapBody),norm(fineCenter-mapBody),mean(oracle(:,1))-center(1),mean(oracle(:,2))-center(2),c.cellLinIdx(id)}; %#ok<AGROW>
            for j=1:size(q,1)
                points(end+1,:)={k,id,original(j),fine(j),transported(j,1),transported(j,2),local(j,3),oracle(j,1),oracle(j,2)}; %#ok<AGROW>
            end
        end
    end
    tracks=cell2table(rows,VariableNames={'frame','component','track','points','finePoints','fineFraction','transportedX','transportedY','fineX','fineY','meanZ','minZ','maxZ','semanticProbability','occupancy','targetDistance','fineTargetDistance','oracleMotionDeltaX','oracleMotionDeltaY','outputCell'});
    pointTable=cell2table(points,VariableNames={'frame','component','originalIndex','fineSign','x178','y178','z','oracleX178','oracleY178'});
    lookup=zeros(numel(state.count),1);
    for id=find(state.count>=2).'
        [distance,j]=min(sum((s.source.components.mean(signIds,:)-state.mean(id,:)).^2,2));
        assert(distance<1e-18);lookup(id)=signIds(j);
        assert(norm(state.covariance(:,:,id)-s.source.components.covariance(:,:,signIds(j)),'fro')<1e-9);
    end
    tracks.pooledSource=lookup(tracks.track);
    t=tracks(tracks.pooledSource==22,:);assert(height(t)==5 && isequal(t.frame,(174:178).'));
    assert(max(abs(mean(t{:,{'transportedX','transportedY'}},1)-s.source.components.mean(22,:)))<1e-9);
    writetable(tracks,fullfile(dest,'sign_acquisitions.csv'));writetable(pointTable,fullfile(dest,'sign_points.csv'));disp(tracks);
    fine=fd.featureData.pointsByFeatureFrame{signClass,178};fineBody=(fine(:,1:2)-s.ref(1:2))*rot(s.ref(3));
    localFine=fineBody(vecnorm(fineBody-mapBody,2,2)<1.5,:);
    map=load('output/mississippi_mapping_calibrated/view_conditioned_cloud.mat','cloud');
    observations=map.cloud.landmarkViews.observations{1074};[distance,i]=min(vecnorm(observations(:,1:2)-s.ref(1:2),2,2));
    assert(distance<1e-8 && size(localFine,1)==observations(i,9));
    assert(norm(mean(localFine,1)-(observations(i,4:5)-s.ref(1:2))*rot(s.ref(3)))<1e-8);
    summary=struct('mapSignGlobalComponent',1074,'mapCenterBody',mapBody,'frame178AllFineSignCount',size(fine,1), ...
        'frame178LocalFineSignCount',size(localFine,1),'frame178LocalFineMeanBody',mean(localFine,1), ...
        'frame178LocalFineBoundsBody',[min(localFine,[],1);max(localFine,[],1)], ...
        'frame178LocalFineToConditionalMapM',norm(mean(localFine,1)-mapBody),'source22Mean',s.source.components.mean(22,:), ...
        'source22AllFramesFineFraction',sum(t.finePoints)/sum(t.points));
    fid=fopen(fullfile(dest,'sign_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);
end
function [state,assignment]=associate(state,mu,covariance)
% Exact class-local copy of the production greedy association and moment update.
    cfg=localizationSourceWindowConfig();n=size(mu,1);assignment=zeros(n,1);nt=size(state.mean,1);
    if nt>0
        dx=state.mean(:,1)-mu(:,1).';dy=state.mean(:,2)-mu(:,2).';
        xx=reshape(state.covariance(1,1,:),[],1)+reshape(covariance(1,1,:),1,[])+cfg.associationNoiseStandardDeviation^2;
        xy=reshape(state.covariance(1,2,:),[],1)+reshape(covariance(1,2,:),1,[]);
        yy=reshape(state.covariance(2,2,:),[],1)+reshape(covariance(2,2,:),1,[])+cfg.associationNoiseStandardDeviation^2;
        distance=(yy.*dx.^2-2*xy.*dx.*dy+xx.*dy.^2)./(xx.*yy-xy.^2);
        eligible=find(dx.^2+dy.^2<=cfg.maximumAssociationDistance^2 & distance<=cfg.maximumStandardizedDistance^2);
        [~,order]=sort(distance(eligible));used=false(nt,1);
        for index=eligible(order).'
            [i,j]=ind2sub(size(distance),index);
            if ~used(i)&&assignment(j)==0,assignment(j)=i;used(i)=true;end
        end
    end
    for j=1:n
        id=assignment(j);
        if id==0,nt=nt+1;id=nt;state.mean(id,:)=0;state.covariance(:,:,id)=zeros(2);state.count(id,1)=0;end
        state.count(id)=state.count(id)+1;fraction=1/state.count(id);delta=mu(j,:)-state.mean(id,:);
        state.covariance(:,:,id)=(1-fraction)*state.covariance(:,:,id)+fraction*covariance(:,:,j)+fraction*(1-fraction)*(delta.'*delta);
        state.mean(id,:)=state.mean(id,:)+fraction*delta;assignment(j)=id;
    end
end
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
