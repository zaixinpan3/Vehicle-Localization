function inspectPoleLocation()
% inspectPoleLocation Compare an accepted shaft axis with its whole-pillar mean.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));mc=featureMapBuildConfig();
    cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;frames=[175 178 806 850 854 856 893 894];
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),frames);map=load(mc.probabilityCloudPath,'cloud');f=map.cloud.components;
    old=load('output/line_direction_matching_20260928/production/report.mat','report');rows=cell(0,10);
    for j=1:numel(frames)
        k=frames(j);raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(raw,cfg);off=p.diagnostics.offGround;m=off.columnMaps;e=m.poleValidation;
        ref=old.report.calls{k,{'referenceX','referenceY','referencePsi'}};R=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];fixed=(f.mean(f.semanticName=="pole",:)-ref(1:2))*R;
        for id=find(off.poleCellMask).'
            i=find(e.pillarIndices==id);axis=[e.axisXY(i,:),e.axisZ(i)]*p.probabilityCloud.projectionRotation.'+p.probabilityCloud.projectionTranslation;
            whole=m.moments.mean(id,:);[oldDistance,target]=min(vecnorm(fixed-whole,2,2));newDistance=norm(axis(1:2)-fixed(target,:));
            rows(end+1,:)={k,id,whole(1),whole(2),axis(1),axis(2),oldDistance,newDistance,e.score(i),e.radialRms(i)}; %#ok<AGROW>
        end
    end
    t=cell2table(rows,VariableNames={'frame','pillar','wholeX','wholeY','axisX','axisY','wholeMapDistance','axisMapDistance','score','radialRms'});writetable(t,fullfile(dest,'pole_location.csv'));disp(t);
end
