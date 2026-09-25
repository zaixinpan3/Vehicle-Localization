function captureWholePillarDescriptors()
% captureWholePillarDescriptors: Frozen teacher labels and whole-pillar inputs.
% Diagnostic dataset only. Uses the 0.6 m runtime, with no fine detector calls.
    root=setupVehicleLocalization();
    folder=fullfile(root,'output','coarse_lattice_20260924');
    base=load(fullfile(folder,'baseline_sequence.mat'),'records','geometry');
    cfg=perceptionConfig(); cfg.featureNames="pole";
    cfg.coarseProbabilityCloud.storeDiagnostics=true;
    store=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    names=["frame","cell","isBase","range","n","h","hStd","tilt","rStd", ...
        "line","point","coreFrac","coreHeight","isoAll60","coreAllCount", ...
        "meanLocalX","meanLocalY","peakLocalX","peakLocalY", ...
        "minimumLocalX","minimumLocalY","maximumLocalX","maximumLocalY", ...
        "meanZ","minimumZ","maximumZ","cxx","cxy","cyy","cxz","cyz","czz", ...
        "maximumIntensity","maximumReflectivity"];
    file=fullfile(folder,'whole_pillar_descriptors_all.csv');
    fid=fopen(file,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',strjoin(names,','));
    fmt=[repmat('%.12g,',1,numel(names)-1) '%.12g\n'];
    for first=1:50:1170
        frames=first:min(1170,first+49); block=store.pointClouds(1,frames);
        for f=frames
            p=perceiveFrame(block(f-first+1),cfg);
            maps=p.diagnostics.offGround.columnMaps; stats=maps.statistics;
            g=p.candidates.geometry; local=double(stats.pillarIndices);
            [r,c]=ind2sub(maps.mapSize,local);
            lower=maps.origin+([c r]-1).*[maps.dx maps.dy];
            bins=round((lower-g.origin)./g.cellSize)+1;
            ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
            b=base.records{f}; assert(b.frame==f);
            baseline=double(b.pillarIndices{b.semanticNames=="pole"});
            [br,bc]=ind2sub(base.geometry.mapSize,baseline);
            xy=base.geometry.origin+([bc br]-0.5).*base.geometry.cellSize;
            bb=floor((xy-g.origin)./g.cellSize)+1;
            inside=all(bb>=1,2)&bb(:,1)<=g.mapSize(2)&bb(:,2)<=g.mapSize(1);
            labels=unique(sub2ind(g.mapSize,bb(inside,2),bb(inside,1)));
            cv=stats.covarianceXYZ;
            slope=cv(:,4:5)./max(cv(:,6),eps);
            radial=sqrt(max(0,cv(:,1)+cv(:,3)-sum(cv(:,4:5).^2,2)./max(cv(:,6),eps)));
            h=stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3);
            peak=[maps.corePeakX(local) maps.corePeakY(local)];
            values=[repmat(f,numel(ids),1),ids,ismember(ids,labels),vecnorm(peak,2,2), ...
                stats.count,h,sqrt(cv(:,6)),atand(vecnorm(slope,2,2)),radial, ...
                double(maps.lineScore(local)),double(maps.pointScore(local)), ...
                maps.coreFraction(local),maps.coreHeight(local),maps.coreIsolation(local),maps.corePointCount(local), ...
                stats.meanXYZ(:,1:2)-lower,peak-lower,stats.minimumXYZ(:,1:2)-lower,stats.maximumXYZ(:,1:2)-lower, ...
                stats.meanXYZ(:,3),stats.minimumXYZ(:,3),stats.maximumXYZ(:,3),cv, ...
                stats.intensity.maximum,stats.reflectivity.maximum];
            keep=stats.count>=6 & h>=1;
            fprintf(fid,fmt,values(keep,:).');
        end
        fprintf('Whole-pillar descriptors %d/1170\n',frames(end));
    end
end
