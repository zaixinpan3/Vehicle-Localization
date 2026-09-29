function inspectCurbPosition(selectedFrames)
% inspectCurbPosition Diagnostic only: compare whole-pillar and frozen fine centers.
    if nargin<1,selectedFrames=[178 806 854];end
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;
    mc=featureMapBuildConfig();frameIds=selectedFrames;poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),frameIds);
    for j=1:numel(frameIds)
        frame=frameIds(j);ref=[];
        for worker=1:4
            a=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',worker),'frames','selectedIndices','cfg');at=find(a.frames==frame);
            if ~isempty(at),ref=a.selectedIndices{at,string(a.cfg.featureNames)=="curb"};break;end
        end
        raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),frame);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(raw,cfg);g=p.diagnostics.ground;xyz=double([raw.x(ref),raw.y(ref),raw.z(ref)]);assert(size(xyz,2)==3);
        bins=floor((xyz(:,1:2)-g.cellOrigin)./g.cellSize)+1;valid=all(bins>=1,2)&bins(:,1)<=g.cellMapSize(2)&bins(:,2)<=g.cellMapSize(1);xyz=xyz(valid,:);bins=bins(valid,:);
        ids=sub2ind(g.cellMapSize,bins(:,2),bins(:,1));mu=aggregatePlanarCellMoments(xyz,ids,prod(g.cellMapSize),p.probabilityCloud.projectionRotation,p.probabilityCloud.projectionTranslation);
        selected=find(g.curbCellMask & reshape(mu.count>0,g.cellMapSize));a=g.moments.mean(selected,:);b=mu.mean(selected,:);
        t=table(selected,a(:,1),a(:,2),b(:,1),b(:,2),b(:,1)-a(:,1),b(:,2)-a(:,2),VariableNames={'pillar','wholeX','wholeY','fineX','fineY','dx','dy'});
        writetable(t,fullfile(dest,"curb_positions_"+frame+".csv"));disp(frame);disp(t);
    end
end
