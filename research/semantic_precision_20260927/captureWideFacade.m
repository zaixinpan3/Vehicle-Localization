function captureWideFacade()
% captureWideFacade: Global context features joined only by coarse cell ID.
    root=setupVehicleLocalization();out=fullfile(root,'output','semantic_precision_20260927');
    baseline=load(fullfile(out,'downtown_baseline.mat'),'frames');
    cfg=perceptionConfig('Downtown');cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;
    file=fopen(fullfile(out,'facade_wide_features.csv'),'w');finish=onCleanup(@()fclose(file));
    source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));
    for k=1:numel(baseline.frames)
        frame=source.pointClouds(1,baseline.frames(k));p=perceiveFrame(frame,cfg);
        [x,names]=measureWideFacadeEvidence(frame,cfg,p.diagnostics.offGround);
        if k==1,fprintf(file,'frame,pillar,%s\n',strjoin(names,','));end
        off=p.diagnostics.offGround;local=find(off.facadeCellMask);m=off.columnMaps;[row,col]=ind2sub(m.mapSize,local);g=p.candidates.geometry;
        xy=m.origin+([col row]-.5).*[m.dx m.dy];bins=floor((xy-g.origin)./g.cellSize)+1;ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
        values=[repmat(baseline.frames(k),numel(ids),1),ids,x];
        if ~isempty(values),fprintf(file,[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'],values.');end
        if mod(k,20)==0||k==numel(baseline.frames),fprintf('Wide facade %d/%d\n',k,numel(baseline.frames));end
    end
end
