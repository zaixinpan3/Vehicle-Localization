function report=validatePoleSubsetIntegration()
% validatePoleSubsetIntegration: Recorded invariants, backend parity and cost.
    root=setupVehicleLocalization(); out=fullfile(root,'output','coarse_lattice_20260924');
    before=load(fullfile(out,'distributionEvidenceFull_sequence.mat'),'records');
    after=load(fullfile(out,'subsetFull_sequence.mat'),'records');
    assert(numel(before.records)==1170 && numel(after.records)==1170);
    common=0; added=0; removed=0;
    for k=1:1170
        a=before.records{k};b=after.records{k};
        assert(a.frame==b.frame && isequal(a.sourceSummary,b.sourceSummary));
        assert(isequal(a.roadCells,b.roadCells) && isequal(a.curbCells,b.curbCells) && isequal(a.signCells,b.signCells));
        assert(isequal(a.curbEnergy,b.curbEnergy) && isequal(a.signProbability,b.signProbability));
        pole=find(a.cloudSemanticNames=="pole"); aa=a.components.semanticId~=pole;bb=b.components.semanticId~=pole;
        for name={'semanticId','count','mean','covariance','meanXYZ','semanticProbability','occupancyProbability'}
            av=a.components.(name{1});bv=b.components.(name{1});
            if strcmp(name{1},'covariance')
                assert(isequaln(av(:,:,aa),bv(:,:,bb)),'Non-pole covariance changed.');
            else
                assert(isequaln(av(aa,:),bv(bb,:)),'Non-pole components changed.');
            end
        end
        common=common+numel(intersect(a.poleCells,b.poleCells));
        added=added+numel(setdiff(b.poleCells,a.poleCells));removed=removed+numel(setdiff(a.poleCells,b.poleCells));
    end
    cfg=perceptionConfig(); experimental=poleSubsetExperimentConfig(cfg);
    frames=1:10:1170;source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));block=source.pointClouds(1,frames);
    elapsed=zeros(numel(frames),2);
    for warmup=1:3
        perceiveFrame(block(1),cfg);perceiveFrame(block(1),experimental);
    end
    for k=1:numel(frames)
        configs={cfg,experimental};sequence=1:2;if mod(k,2)==0,sequence=2:-1:1;end
        for j=sequence
            timer=tic; result=perceiveFrame(block(k),configs{j});elapsed(k,j)=toc(timer);
            if j==1
                stored=before.records{frames(k)};
                assert(isequal(result.candidates.pillarIndices,stored.pillarIndices),'Default masks changed.');
                for name=fieldnames(stored.components).'
                    assert(isequaln(result.probabilityCloud.components.(name{1}),stored.components.(name{1})),'Default output changed.');
                end
            end
        end
    end
    raw=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    structural=structuralPillarConfig(.6);structural.useNativeKernels=true;subset=structural;
    subset.pole.detector="subset";subset.pole.probabilityEvidence="subset";
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    maximumScoreError=0;
    for k=1:numel(raw.records)
        r=raw.records{k};grid=struct('points',double(r.points),'pointPillarLinIdx',r.pillarIds, ...
            'pointAttributes',struct(),'pillarGeometry',r.geometry);
        a=analyzeStructuralPillars(grid,structural,cloud);b=analyzeStructuralPillars(grid,subset,cloud);
        assert(isequaln(a.columnMaps.statistics,b.columnMaps.statistics) && isequaln(a.columnMaps.moments,b.columnMaps.moments));
        if ismember(k,[1 13 60 90 117])
            expected=findPillarPoleSubsets(grid.points,r.pillarIds,r.geometry,subset.pole.subset);
            actual=b.columnMaps.poleSubset;assert(isequal(actual.found,expected.found));
            error=max(abs(actual.score-expected.score));assert(error<1e-9);maximumScoreError=max(maximumScoreError,error);
        end
    end
    report=struct('fullFrames',1170,'unchangedNonPoleChannels',true,'unchangedSegmentationCounts',true, ...
        'commonPoleCells',common,'addedPoleCells',added,'removedPoleCells',removed, ...
        'defaultUnchangedFrames',numel(frames),'allPointMomentChecks',numel(raw.records), ...
        'backendParityRawFrames',5,'maximumBackendScoreError',maximumScoreError, ...
        'defaultMedianMs',1000*median(elapsed(:,1)),'subsetMedianMs',1000*median(elapsed(:,2)), ...
        'medianPairedIncreaseMs',1000*median(elapsed(:,2)-elapsed(:,1)));
    destination=fullfile(root,'research','pole_subset_20260925');
    writetable(table(frames(:),1000*elapsed(:,1),1000*elapsed(:,2),'VariableNames',{'frame','defaultMs','subsetMs'}),fullfile(destination,'paired_timing.csv'));
    fid=fopen(fullfile(destination,'integration_validation.json'),'w');clean=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end
