function testMissingPoleMaximum()
% testMissingPoleMaximum Oracle owner-selection intervention, never production.
% Select only reference owners of the inspected pole; retain whole-pillar moments.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/current_matching_20260929/replay.mat');d=a.maximum;last=d.frame;
    s=load('output/line_direction_matching_20260928/sources.mat','fixed','motion');
    mapCfg=featureMapBuildConfig();mapData=load(mapCfg.probabilityCloudPath,'cloud');
    s.fixed=registrationSupport.projectSemanticProbabilityCloud(mapData.cloud,2);
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    refs=load('output/pole_precision_20260927/mississippi.mat','frames','references');
    mc=featureMapBuildConfig();poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    hist={[],[],[]};sources=cell(3,1);rows=cell(0,9);scoreRows=cell(0,7);
    targetFrame=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),last);
    targetIds=refs.references{refs.frames==last}.pointIndices;
    target=mean(double([targetFrame.x(targetIds),targetFrame.y(targetIds)]),1);
    targetPose=calls{last,{'referenceX','referenceY','referencePsi'}};
    targetWorld=transform(target,targetPose);
    for k=last-4:last
        frame=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(frame,cfg);og=p.diagnostics.offGround;maps=og.columnMaps;stats=maps.statistics;
        ids=double(refs.references{refs.frames==k}.pointIndices(:));xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
        pose=calls{k,{'referenceX','referenceY','referencePsi'}};
        keep=vecnorm(transform(xyz(ids,1:2),pose)-targetWorld,2,2)<1;
        ids=ids(keep);bins=floor((xyz(ids,1:2)-maps.origin)./[maps.dx maps.dy])+1;
        owners=unique(sub2ind(maps.mapSize,bins(:,2),bins(:,1)));owners=owners(maps.pillarCounts(owners)>0);
        [rr,ccol]=ind2sub(maps.mapSize,owners);center=maps.origin+([ccol rr]-.5).*[maps.dx maps.dy];
        geometry=p.candidates.geometry;bins=floor((center-geometry.origin)./geometry.cellSize)+1;
        globalOwners=sub2ind(geometry.mapSize,bins(:,2),bins(:,1));
        for owner=owners.'
            j=find(stats.pillarIndices==owner);q=xyz(ids,:);
            rows(end+1,:)={k,owner,globalOwners(owners==owner),numel(ids),stats.count(j),maps.pillarZRange(owner),max(q(:,3))-min(q(:,3)),maps.poleValidation.found(j),maps.poleValidation.score(j)}; %#ok<AGROW>
        end
        if k==last
            ctx=p.diagnostics.offGroundPointContext;
            structural=true(size(ctx.points,1),1);
            if isfield(ctx.pointAttributes,'intensity'),v=ctx.pointAttributes.intensity;structural=~(isfinite(v)&v>cfg.offGroundFeatures.trafficSignIntensityThreshold);end
            pointScore=maps.pointScore;lineScore=maps.lineScore;
            if isfield(ctx,'sourcePillarGeometry')
                original=ctx.sourcePillarGeometry;g=ctx.pillarGeometry;offset=round((g.origin-original.origin)./g.cellSize);
                rr=(1:g.mapSize(1))+offset(2);cc=(1:g.mapSize(2))+offset(1);
                support=zeros(original.mapSize,'single');occupied=false(original.mapSize);
                support(rr,cc)=maps.supportEvidence;occupied(rr,cc)=maps.occupiedMask;
                [fp,fl]=buildPillarShapeScores(support,occupied,cfg.offGroundFeatures,g.cellSize(1),g.cellSize(2));
                pointScore=fp(rr,cc);lineScore=fl(rr,cc);
            end
            [~,diag]=classifyPillarPoleSupport(ctx.points,ctx.pointPillarLinIdx,ctx.pillarGeometry,maps.poleValidationProposals, ...
                cfg.offGroundFeatures.pole.validation,structural,pointScore,lineScore,cfg.offGroundFeatures.pole.distributionValidation);
            for j=find(ismember(diag.owners,owners)).'
                h=diag.hypotheses(diag.hypothesisIds(j));scoreRows(end+1,:)={last,diag.owners(j),globalOwners(owners==diag.owners(j)),diag.scores(j),h.supportHeight,h.radialRms,h.isolation}; %#ok<AGROW>
            end
            save(fullfile('output/current_matching_20260929','pole_diagnostic.mat'),'p','diag','ids','owners');
        end
        cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cal=validateLidarFrameCalibration(cfg.frameCalibration);
        if ~isfield(cc,'projectionTranslation'),cc.projectionTranslation=[0 0 0];end
        cc.projectionTranslation=cc.projectionTranslation+cal.translation*cc.projectionRotation.';
        cc.projectionRotation=cc.projectionRotation*cal.rotation;cc.frameCalibration=cal;
        for variant=1:3
            modified=og;
            if variant==2||variant==3,modified.poleCellMask(owners)=true;modified.poleProbability(owners)=.8+.1*(variant-1);end
            cloud=buildCoarseSemanticProbabilityCloud(p.diagnostics.ground,modified,cc);
            if variant==1,assert(isequaln(cloud.components,p.probabilityCloud.components));end
            [sources{variant},hist{variant}]=updateLocalizationSourceWindow(cloud,calls.timeSeconds(k),s.motion(k,:),hist{variant},a.wc);
        end
    end
    assert(isequaln(sources{1}.components,d.source.components));
    writetable(cell2table(rows,VariableNames={'frame','owner','globalOwner','frameReferencePoints','offGroundPoints','wholePillarHeight','frameReferenceHeight','selected','acceptedScore'}),fullfile(dest,'pole_owners.csv'));
    writetable(cell2table(scoreRows,VariableNames={'frame','owner','globalOwner','score','supportHeight','radialRms','isolation'}),fullfile(dest,'pole_scores.csv'));
    fixed=selectLocalProbabilityCloud(s.fixed,d.predicted,a.reg.localMapRadius);results=cell(3,2);rows=cell(0,9);
    for variant=1:3
        for merge=1:2
            reg=a.reg;if merge==2,reg.pyramid.mapMergeRadius=0;end
            r=registerSemanticProbabilityCloud(fixed,sources{variant},d.predicted,reg);results{variant,merge}=r;
            e=r.poseXYTheta-d.reference;
            rows(end+1,:)={variant,reg.pyramid.mapMergeRadius,nnz(sources{variant}.components.semanticName=="pole"),nnz(r.correspondences.semanticName=="pole"),r.accepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.pyramid.refinementShiftM}; %#ok<AGROW>
        end
    end
    controls=cell2table(rows,VariableNames={'baseline1Oracle09_2Oracle10_3','mapMergeRadius','sourcePoles','matchedPoles','accepted','reason','errorM','yawErrorDeg','fineShiftM'});disp(controls);
    writetable(controls,fullfile(dest,'oracle_pole_controls.csv'));
    save('output/current_matching_20260929/pole_controls.mat','sources','results','controls');
end
function xy=transform(xy,pose)
    R=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];xy=xy*R.'+pose(1:2);
end
