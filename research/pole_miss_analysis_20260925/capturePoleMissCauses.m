function capturePoleMissCauses()
% capturePoleMissCauses: Trace reference-cell losses with original point identities.
% Fine output is an algorithmic reference, never manual truth or detector input.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    raw=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    fine=load(fullfile(root,'output','coarse_lattice_20260924','fine_pole_pillars.mat'));
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    frames=cellfun(@(r)r.frame,raw.records);block=source.pointClouds(1,frames);
    cfg=structuralPillarConfig(.6);cfg.useNativeKernels=true;subset=cfg;
    subset.pole.detector="subset";subset.pole.probabilityEvidence="subset";
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    tables=cell(numel(frames),1);cases=tables;maximumScoreError=0;
    for k=1:numel(frames)
        r=raw.records{k};frame=block(k);index=double(r.originalIndices);
        p=double([frame.x(index),frame.y(index),frame.z(index)]);ids=double(r.pillarIds);
        grid=struct('points',p,'pointPillarLinIdx',ids,'pointAttributes',struct(),'pillarGeometry',r.geometry);
        before=analyzeStructuralPillars(grid,cfg,cloud);after=analyzeStructuralPillars(grid,subset,cloud);
        assert(isequal(double(find(before.poleCellMask)),double(sort(r.currentCells(:)))),'Default replay differs from cached reference.');
        occupied=unique(ids);targets=union(double(r.baselineCells),double(r.fineCells));selected=ismember(occupied,targets);
        [traced,trace]=tracePoleSubsetRejections(p,ids,r.geometry,cfg.pole.subset,selected);
        native=after.columnMaps.poleSubset;
        assert(isequal(traced.found(selected),native.found(selected)));
        error=max(abs(traced.score(selected)-native.score(selected)));if isempty(error),error=0;end
        assert(error<1e-9);maximumScoreError=max(maximumScoreError,error);
        t=struct2table(trace);t.frame=repmat(r.frame,height(t),1);
        t.baseline=ismember(occupied,r.baselineCells);t.fine=ismember(occupied,r.fineCells);
        t.previous=before.poleCellMask(occupied);t.subsetFound=native.found;t.subsetAccepted=after.poleCellMask(occupied);
        t.defaultCore=before.candidates.coreMask(occupied);
        m=before.columnMaps;s=m.statistics;c=cfg.pole;covariance=s.covarianceXYZ;
        t.range=vecnorm(s.meanXYZ(:,1:2),2,2);
        t.wholeTilt=atand(vecnorm(covariance(:,4:5)./max(covariance(:,6),eps),2,2));
        t.wholeRadialStd=sqrt(max(0,covariance(:,1)+covariance(:,3)-sum(covariance(:,4:5).^2,2)./max(covariance(:,6),eps)));
        t.coreFraction=m.coreFraction(occupied);t.coreHeight=m.coreHeight(occupied);
        t.coreIsolation=m.coreIsolation(occupied);t.coreCount=m.corePointCount(occupied);
        t.failCount=s.count<c.minimumPoints;t.failHeight=t.ownSpan<c.minimumHeight;
        t.failHeightStd=covariance(:,6)<c.minimumHeightStd^2;t.failTilt=t.wholeTilt>c.maximumTiltDegrees;
        t.failRadial=t.wholeRadialStd>c.maximumRadialStd;t.failCoreFraction=t.coreFraction<c.minimumCoreFraction;
        t.failCoreHeight=t.coreHeight<c.minimumCoreHeight;t.failIsolation=t.coreIsolation<c.minimumCoreIsolation;
        t.failCoreCount=t.coreCount<c.minimumCorePoints;
        t.failEligibility=~before.candidates.eligibleMask(occupied);
        t.failLine=before.columnMaps.lineScore(occupied)>c.maximumLineScore;
        t.densityCoreEvaluated=~t.failCount & ~t.failHeight;
        t.defaultFootprintDrop=t.defaultCore & ~t.previous;
        t.subsetFootprintDrop=t.subsetFound & ~t.subsetAccepted;
        accepted=find(after.poleCellMask);[ay,ax]=ind2sub(r.geometry.mapSize,accepted);
        [ty,tx]=ind2sub(r.geometry.mapSize,occupied);
        if isempty(accepted),t.nearestAcceptedCellMeters=inf(size(occupied));
        else,t.nearestAcceptedCellMeters=.6*sqrt(min((tx-ax.').^2+(ty-ay.').^2,[],2));end
        t.fineOriginalOwnCount=zeros(height(t),1);t.fineOffGroundOwnCount=t.fineOriginalOwnCount;
        t.fineGroundRemovedCount=t.fineOriginalOwnCount;t.fineOutsideFilterCount=t.fineOriginalOwnCount;
        t.fineOriginalOwnHeight=t.fineOriginalOwnCount;t.fineOffGroundOwnHeight=t.fineOriginalOwnCount;
        t.fineAxisNearestAccepted=nan(height(t),1);t.fineAxisOverlapHeight=nan(height(t),1);
        originalFine=double(fine.fine([fine.fine.frame]==r.frame).polePoints(:));
        finePoints=double([frame.x(originalFine),frame.y(originalFine),frame.z(originalFine)]);
        bins=floor((finePoints(:,1:2)-r.geometry.origin)./r.geometry.cellSize)+1;
        valid=all(bins>=1 & bins<=[r.geometry.mapSize(2),r.geometry.mapSize(1)],2);
        fineIds=zeros(size(originalFine));fineIds(valid)=sub2ind(r.geometry.mapSize,bins(valid,2),bins(valid,1));
        retained=pillarizePointCloud(frame,pillarGridConfig());
        off=ismember(originalFine,index);kept=ismember(originalFine,double(retained.pointIndices));
        for j=find(t.fine).'
            own=fineIds==occupied(j);z=finePoints(own,3);zoff=finePoints(own & off,3);
            t.fineOriginalOwnCount(j)=nnz(own);t.fineOffGroundOwnCount(j)=nnz(own & off);
            t.fineGroundRemovedCount(j)=nnz(own & kept & ~off);t.fineOutsideFilterCount(j)=nnz(own & ~kept);
            if ~isempty(z),t.fineOriginalOwnHeight(j)=max(z)-min(z);end
            if ~isempty(zoff),t.fineOffGroundOwnHeight(j)=max(zoff)-min(zoff);end
            % Diagnostic association only: fit fine-labeled points near this
            % owner, then compare actual shaft axes at a common height.
            center=median(finePoints(own,1:2),1);
            near=vecnorm(finePoints(:,1:2)-center,2,2)<=.4;
            q=finePoints(near,:);if size(q,1)<3,continue;end
            z0=median(q(:,3));design=[ones(size(q,1),1),q(:,3)-z0];beta=design\q(:,1:2);
            candidate=ismember(native.pillarIndices,accepted);
            axes=native.axisXY(candidate,:)+(z0-native.axisZ(candidate)).*native.slopeXY(candidate,:);
            if isempty(axes),continue;end
            distance=vecnorm(axes-beta(1,:),2,2);[best,which]=min(distance);
            t.fineAxisNearestAccepted(j)=best;keptRows=find(candidate);b=keptRows(which);
            t.fineAxisOverlapHeight(j)=max(0,min(max(q(:,3)),native.maximumZ(b))-max(min(q(:,3)),native.minimumZ(b)));
        end
        t=t(selected,:);assert(all(ismember(targets,occupied)),'Missing reference pillar needs explicit absent-row handling.');
        tables{k}=t;
        cases{k}=struct('frame',r.frame,'geometry',r.geometry,'points',p,'pillarIds',ids, ...
            'originalIndices',index,'finePoints',finePoints,'fineIds',fineIds, ...
            'fineOffGround',off,'before',before,'after',after);
        if mod(k,20)==0,fprintf('Pole miss trace %d/%d\n',k,numel(frames));end
    end
    T=vertcat(tables{:});writetable(T,fullfile(folder,'reference_cell_causes.csv'));
    out=fullfile(root,'output','pole_miss_analysis_20260925');if ~isfolder(out),mkdir(out);end
    save(fullfile(out,'cases.mat'),'cases','cfg','subset','maximumScoreError','-v7.3');
    fid=fopen(fullfile(folder,'trace_validation.json'),'w');cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(struct('frames',numel(frames),'referenceCells',height(T), ...
        'traceMatchesNative',true,'maximumScoreDifference',maximumScoreError,'defaultReplayMatches',true),PrettyPrint=true));
    disp(T(T.fine & ~T.subsetAccepted,{'frame','pillarIndices','stage','fineOriginalOwnCount', ...
        'fineGroundRemovedCount','fineOriginalOwnHeight','fineOffGroundOwnHeight', ...
        'nearestAcceptedCellMeters','fineAxisNearestAccepted','fineAxisOverlapHeight'}));
end
