function [x,names]=measureWideFacadeEvidence(frame,cfg,mainOffGround)
% measureWideFacadeEvidence: Global line context on an extended 0.6 m lattice.
% Keep the published ROI unchanged. Context pillars share its exact phase;
% only the observation extent grows to cover the original reference context.
% No fine inference, reference lookup, finer XY cells or height bins are used.
    ids=find(mainOffGround.facadeCellMask);m=mainOffGround.columnMaps;[row,col]=ind2sub(m.mapSize,ids);
    centers=m.origin+([col row]-.5).*[m.dx m.dy];x=zeros(numel(ids),0);names=strings(1,0);
    if isempty(ids),return;end
    cfg.voxel.gridDims=[168 168];cfg.featureNames="facade";cfg.semanticPrecision.enabled=false;
    cfg.coarseProbabilityCloud.storeDiagnostics=true;
    settings=[1 3;2 3;1 6];
    for k=1:size(settings,1)
        cfg.offGroundFeatures.fineShapeScoreNeighborhoodRadiusCells=settings(k,1);
        cfg.offGroundFeatures.facadeContinuousSupportMinimumPoints=settings(k,2);
        if k==1
            context=perceiveFrame(frame,cfg);off=context.diagnostics.offGround;
            widePillars=context.diagnostics.offGroundPointContext;
        else
            structural=cfg.offGroundFeatures;structural.useNativeKernels=perceptionNativeAvailable(cfg.executionBackend);
            cloud=cfg.coarseProbabilityCloud;cloud.semanticNames="facade";
            off=analyzeStructuralPillars(widePillars,structural,cloud);
        end
        c=off.columnMaps;
        bins=round((centers-c.origin)./[c.dx c.dy]+.5);valid=all(bins>=1 & bins<=[c.mapSize(2),c.mapSize(1)],2);
        target=zeros(numel(ids),1);target(valid)=sub2ind(c.mapSize,bins(valid,2),bins(valid,1));
        rows=zeros(numel(ids),3);rows(valid,:)=[double(off.facadeCellMask(target(valid))), ...
            double(c.facadeLineScore(target(valid))),double(c.pillarZRange(target(valid)))];
        [group,groupNames]=measureFacadeGroupEvidence(off,widePillars);
        [has,at]=ismember(target,find(off.facadeCellMask));expanded=zeros(numel(ids),size(group,2));expanded(has,:)=group(at(has),:);
        x=[x,rows,expanded];names=[names,"wide"+k+"_"+["selected","lineScore","height",groupNames]]; %#ok<AGROW>
    end
    x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
