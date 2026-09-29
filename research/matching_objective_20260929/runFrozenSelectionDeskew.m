function runFrozenSelectionDeskew()
% runFrozenSelectionDeskew Isolate geometric compensation from changed detection.
    root=setupVehicleLocalization();addpath(fullfile(root,'research/root_cause_matching_20260929'));
    dest=fileparts(mfilename('fullpath'));out='output/matching_objective_20260929';
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    timing=load('output/root_cause_matching_20260929/point_timing.mat');old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    phases=[0 .05 .1];allCurrent=cell(1170,3);allSources=allCurrent;histories=cell(3,1);wc=localizationSourceWindowConfig();frames=[];first=0;rows=cell(1170*3,5);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);[~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(raw,cfg);twist=zeros(1,3);
        if k>1
            dt=calls.timeSeconds(k)-calls.timeSeconds(k-1);yaw=atan2(sin(motion(k,3)-motion(k-1,3)),cos(motion(k,3)-motion(k-1,3)));r=rotation(motion(k-1,3));translation=(motion(k,1:2)-motion(k-1,1:2))*r;
            V=eye(2);if abs(yaw)>1e-8,a=sin(yaw)/yaw;b=2*sin(yaw/2)^2/yaw;V=[a -b;b a];end
            twist=[(V\translation.').'/dt,yaw/dt];
        end
        R=p.probabilityCloud.projectionRotation;t=p.probabilityCloud.projectionTranslation;g0=p.diagnostics.ground;o0=p.diagnostics.offGround;gc=p.diagnostics.groundPointContext;oc=p.diagnostics.offGroundPointContext;
        [gx,gy]=ind2sub(g0.cellMapSize([2 1]),gc.groundCellLinIdx);groundCells=sub2ind(g0.cellMapSize,gy,gx);
        times=double(timing.pointTimes(:,:,k));gt=accumarray(groundCells,times(gc.groundOriginalPointIdx),[numel(g0.moments.count),1],@mean,0);detail=p.diagnostics.curbBoundaryFit;
        for j=1:3
            corrected=deskewStoredFrame(raw,times,twist,phases(j),cfg.frameCalibration,tilt);xyz=double([corrected.x(:),corrected.y(:),corrected.z(:)]);g=g0;o=o0;
            g.moments=aggregatePlanarCellMoments(xyz(gc.groundOriginalPointIdx,:),groundCells,numel(g0.moments.count),R,t,true);
            keep=detail.accepted;ids=detail.pillar(keep);angle=twist(3)*(gt(ids)-phases(j));delta=[detail.dx(keep),detail.dy(keep)];
            if ~isempty(ids),g.moments.mean(ids,:)=g.moments.mean(ids,:)+[cos(angle).*delta(:,1)-sin(angle).*delta(:,2),sin(angle).*delta(:,1)+cos(angle).*delta(:,2)];end
            o.columnMaps.moments=aggregatePlanarCellMoments(xyz(oc.pointIndices,:),oc.pointPillarLinIdx,numel(o0.columnMaps.moments.count),R,t,true);o.columnMaps.trafficSignMoments=o.columnMaps.moments;
            if ~isequal(g.moments.count,g0.moments.count)||~isequal(o.columnMaps.moments.count,o0.columnMaps.moments.count),save(fullfile(out,'frozen_count_failure.mat'),'g','g0','o','o0','gc','oc','k','j');error('Count mismatch ground %d off %d',nnz(g.moments.count~=g0.moments.count),nnz(o.columnMaps.moments.count~=o0.columnMaps.moments.count));end
            cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cc.projectionRotation=R;cc.projectionTranslation=t;cc.frameCalibration=p.probabilityCloud.frameCalibration;
            cloud=buildCoarseSemanticProbabilityCloud(g,o,cc);cloud=applyCurbBoundaryScatter(cloud,cfg.curbBoundary);allCurrent{k,j}=cloud;
            [allSources{k,j},histories{j}]=updateLocalizationSourceWindow(cloud,calls.timeSeconds(k),motion(k,:),histories{j},wc);
            rows((j-1)*1170+k,:)={k,phases(j),true,p.probabilityCloud.components.numComponents,cloud.components.numComponents};
        end
        if mod(k,100)==0,fprintf('Frozen selection deskew %d/1170\n',k);end
    end
    writetable(cell2table(rows,VariableNames={'frame','phase','unchangedPillarMembers','originalComponents','compensatedComponents'}),fullfile(dest,'frozen_deskew_audit.csv'));
    for j=1:3
        label="frozenDeskew"+round(1000*phases(j));currentClouds=allCurrent(:,j);sources=allSources(:,j);phase=phases(j);file=fullfile(out,label+"_sources.mat");save(file,'currentClouds','sources','phase','cfg','-v7.3');
        replayStudy(file,label,distributionRegistrationConfig(),fullfile('output/root_cause_matching_20260929',"deskew"+round(1000*phase)+"_map.mat"));
    end
end
function R=rotation(yaw)
    R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
end
