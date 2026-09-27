function captureFacadeGroups()
% captureFacadeGroups: Add line-level evidence without changing frozen labels.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));out=fullfile(root,'output','semantic_precision_20260927');
    baseline=load(fullfile(out,'downtown_baseline.mat'),'frames');
    cfg=perceptionConfig('Downtown');cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;
    file=fopen(fullfile(out,'facade_group_features.csv'),'w');closeFile=onCleanup(@()fclose(file));
    source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));
    for k=1:numel(baseline.frames)
        frame=source.pointClouds(1,baseline.frames(k));p=perceiveFrame(frame,cfg);off=p.diagnostics.offGround;
        [x,names]=measureFacadeGroupEvidence(off,p.diagnostics.offGroundPointContext);
        if k==1,fprintf(file,'frame,pillar,%s\n',strjoin(names,','));end
        local=find(off.facadeCellMask);[row,col]=ind2sub(off.columnMaps.mapSize,local);m=off.columnMaps;g=p.candidates.geometry;
        xy=m.origin+([col row]-.5).*[m.dx m.dy];bins=floor((xy-g.origin)./g.cellSize)+1;ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
        values=[repmat(baseline.frames(k),numel(ids),1),ids,x];
        if ~isempty(values),fprintf(file,[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'],values.');end
        if mod(k,25)==0 || k==numel(baseline.frames),fprintf('Facade groups %d/%d\n',k,numel(baseline.frames));end
    end
    findings=checkcode(fullfile(root,'perception','measureFacadeGroupEvidence.m'),'-id','-config=factory');
    output=fopen(fullfile(folder,'facade_group_code_analysis.json'),'w');finish=onCleanup(@()fclose(output));fprintf(output,'%s\n',jsonencode(findings,PrettyPrint=true));
end
