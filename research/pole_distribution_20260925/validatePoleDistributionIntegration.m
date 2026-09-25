function report=validatePoleDistributionIntegration()
% validatePoleDistributionIntegration: Whole-drive invariants and paired timing.
    root=setupVehicleLocalization(); folder=fullfile(root,'output','coarse_lattice_20260924');
    before=load(fullfile(folder,'metricCoreFull_sequence.mat'),'records');
    after=load(fullfile(folder,'distributionEvidenceFull_sequence.mat'),'records');
    assert(numel(before.records)==1170 && numel(after.records)==1170);
    probabilityChanges=0; weightChange=zeros(1170,1); poleMassBefore=weightChange; poleMassAfter=weightChange;
    for k=1:1170
        a=before.records{k}; b=after.records{k};
        assert(isequal(a.pillarIndices,b.pillarIndices),'Candidate masks changed.');
        assert(isequal(a.roadCells,b.roadCells) && isequal(a.curbCells,b.curbCells) && isequal(a.signCells,b.signCells));
        for name={'semanticId','count','mean','covariance','meanXYZ'}
            assert(isequaln(a.components.(name{1}),b.components.(name{1})),'All-point Gaussian geometry changed.');
        end
        probabilityChanges=probabilityChanges+nnz(a.poleProbability~=b.poleProbability);
        weightChange(k)=sum(abs(a.components.mixtureWeight-b.components.mixtureWeight));
        channel=find(a.cloudSemanticNames=="pole"); selected=a.components.semanticId==channel;
        poleMassBefore(k)=sum(a.components.mixtureWeight(selected)); poleMassAfter(k)=sum(b.components.mixtureWeight(selected));
    end
    cfg=perceptionConfig(); oldCfg=cfg; oldCfg.offGroundFeatures.pole.probabilityEvidence="shape";
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    frames=1:10:1170; elapsed=zeros(numel(frames),2);
    block=source.pointClouds(1,frames);
    for warmup=1:3
        perceiveCoarseProbabilityCloud(block(1),cfg); perceiveCoarseProbabilityCloud(block(1),oldCfg);
    end
    for k=1:numel(frames)
        configurations={oldCfg,cfg}; sequence=1:2; if mod(k,2)==0, sequence=2:-1:1; end
        for j=sequence
            timer=tic; perceiveCoarseProbabilityCloud(block(k),configurations{j}); elapsed(k,j)=toc(timer);
        end
    end
    report=struct('frames',1170,'unchangedCandidateMasks',true,'unchangedGaussianGeometry',true, ...
        'changedPoleProbabilities',probabilityChanges,'meanL1MixtureWeightChange',mean(weightChange), ...
        'meanPoleMassBefore',mean(poleMassBefore),'meanPoleMassAfter',mean(poleMassAfter), ...
        'pairedTimingFrames',numel(frames),'oldMedianMs',1000*median(elapsed(:,1)), ...
        'newMedianMs',1000*median(elapsed(:,2)),'medianPairedDifferenceMs',1000*median(elapsed(:,2)-elapsed(:,1)));
    destination=fullfile(root,'research','pole_distribution_20260925');
    fid=fopen(fullfile(destination,'integration_validation.json'),'w');clean=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    writetable(table(frames(:),1000*elapsed(:,1),1000*elapsed(:,2),'VariableNames',{'frame','shapeMs','distributionMs'}),fullfile(destination,'paired_timing.csv'));
    disp(report);
end
