function captureCurbGroups()
% captureCurbGroups: Add topology descriptors using audited raw support rows.
    root=setupVehicleLocalization();out=fullfile(root,'output','semantic_precision_20260927');
    for dataset=["Mississippi","Downtown"]
        label=lower(dataset);reference=load(fullfile(out,label+"_baseline.mat"),'frames');
        sourceFile='MissisipiPointClouds.mat';if dataset=="Downtown",sourceFile='downTownPointClouds.mat';end
        source=matfile(fullfile(root,'data','raw',sourceFile));
        old=readtable(fullfile(out,label+"_curb_features.csv"));rawNames=string(old.Properties.VariableNames);rawNames=rawNames(startsWith(rawNames,"raw_"));
        cfg=perceptionConfig(dataset);cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;cfg.featureNames="curb";
        file=fopen(fullfile(out,label+"_curb_group_features.csv"),'w');closeFile=onCleanup(@()fclose(file));lastBlock=-1;
        for k=1:numel(reference.frames)
            frameId=reference.frames(k);blockId=floor((frameId-1)/50);
            if blockId~=lastBlock
                first=blockId*50+1;last=min(first+49,max(reference.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
            end
            p=perceiveFrame(block(frameId-first+1),cfg);g=p.diagnostics.ground;local=find(g.curbCellMask);
            [row,col]=ind2sub(g.cellMapSize,local);geometry=p.candidates.geometry;
            xy=g.cellOrigin+([col row]-.5).*g.cellSize;bins=floor((xy-geometry.origin)./geometry.cellSize)+1;ids=sub2ind(geometry.mapSize,bins(:,2),bins(:,1));
            table=old(old.frame==frameId,:);[valid,at]=ismember(ids,table.pillar);assert(all(valid)&&height(table)==numel(ids));
            [x,names]=measureCurbGroupEvidence(g,table{at,cellstr(rawNames)},rawNames);
            if k==1,fprintf(file,'frame,pillar,%s\n',strjoin(names,','));end
            values=[repmat(frameId,numel(ids),1),ids,x];
            if ~isempty(values),fprintf(file,[repmat('%.12g,',1,size(values,2)-1),'%.12g\n'],values.');end
            if mod(k,100)==0||k==numel(reference.frames),fprintf('%s curb groups %d/%d\n',dataset,k,numel(reference.frames));end
        end
        clear closeFile
    end
end
