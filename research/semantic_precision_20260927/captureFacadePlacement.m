function captureFacadePlacement()
% captureFacadePlacement: Freeze line-relative descriptors without labels.
    root=setupVehicleLocalization();out=fullfile(root,'output','semantic_precision_20260927');s=load(fullfile(out,'downtown_baseline.mat'),'frames');
    cfg=perceptionConfig('Downtown');cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;
    source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));file=fopen(fullfile(out,'facade_placement_features.csv'),'w');finish=onCleanup(@()fclose(file));
    for k=1:numel(s.frames)
        p=perceiveFrame(source.pointClouds(1,s.frames(k)),cfg);off=p.diagnostics.offGround;[x,names]=measureFacadePlacementEvidence(off);
        if k==1,fprintf(file,'frame,pillar,%s\n',strjoin(names,','));end
        ids=find(off.facadeCellMask);m=off.columnMaps;[row,col]=ind2sub(m.mapSize,ids);g=p.candidates.geometry;
        xy=m.origin+([col row]-.5).*[m.dx m.dy];bins=floor((xy-g.origin)./g.cellSize)+1;public=sub2ind(g.mapSize,bins(:,2),bins(:,1));values=[repmat(s.frames(k),numel(ids),1),public,x];
        if ~isempty(values),fprintf(file,[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'],values.');end
    end
    fprintf('Facade placement complete: %d frames.\n',numel(s.frames));
end
