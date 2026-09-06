function report = evaluateStructuralPerception(dataRoot,outputDirectory,baselineRoot)
% evaluateStructuralPerception: Reproduce facade/sign restoration comparisons.
% baselineRoot is an immutable export of commit 8914140 with the same optional
% MEX kernel. Inputs stay under dataRoot. Reports contain counts/agreement and
% warm coarse timings; legacy masks are a software baseline, not ground truth.
    root=fileparts(fileparts(mfilename('fullpath')));
    originalPath=path; cleanup=onCleanup(@() path(originalPath));
    assert(isfolder(baselineRoot),'An immutable baseline export is required.');
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    dataRoot=string(dataRoot); outputDirectory=string(outputDirectory);
    rows=struct([]); timingRows=struct([]);
    for dataset=["Missisipi","downTown"]
        for frameIndex=[100 200 300 400 500]
            frame=loadPointCloudFrame(fullfile(dataRoot,'raw',dataset+'PointClouds.mat'),frameIndex);
            activateRoot(baselineRoot);
            baselineCfg=perceptionConfig(); baselineCfg.offGroundFeatures.facadeDetectionEnabled=dataset=="downTown";
            baselineCfg.executionMode="offline"; before=perceiveFrame(frame,baselineCfg);
            baselineCfg.executionMode="legacyFull"; legacy=perceiveFrame(frame,baselineCfg);
            baselineCfg.executionMode="coarseProbabilityCloud";
            beforeTime=measure(frame,baselineCfg);
            activateRoot(root);
            cfg=perceptionConfig(dataset); cfg.executionMode="offline";
            after=perceiveFrame(frame,cfg);
            cfg.executionMode="coarseProbabilityCloud";
            afterTime=measure(frame,cfg);
            for name=after.candidates.semanticNames.'
                fine=after.featureMasks.(name); reference=legacy.featureMasks.(name);
                audit=after.refinement.(name);
                candidate=false(size(fine)); candidate(audit.candidatePointIndices)=true;
                unchanged=NaN;
                if ismember(name,["curb","roadMarking","pole"])
                    unchanged=nnz(xor(fine,before.featureMasks.(name)));
                end
                row=struct('dataset',dataset,'frameIndex',frameIndex,'feature',name, ...
                    'legacyPoints',nnz(reference),'candidatePoints',nnz(candidate),'finePoints',nnz(fine), ...
                    'candidateLegacyIntersection',nnz(candidate&reference),'fineLegacyIntersection',nnz(fine&reference), ...
                    'changedFromPreviousModernPoints',unchanged, ...
                    'gaussians',nnz(after.probabilityCloud.components.semanticName==name));
                rows=[rows;row]; %#ok<AGROW>
            end
            for repeat=1:numel(afterTime)
                timingRows=[timingRows;struct('dataset',dataset,'frameIndex',frameIndex,'repeat',repeat, ...
                    'baselineMs',1000*beforeTime(repeat),'restoredMs',1000*afterTime(repeat))]; %#ok<AGROW>
            end
            fprintf('%s %d: coarse %.2f -> %.2f ms\n',dataset,frameIndex,1000*median(beforeTime),1000*median(afterTime));
        end
    end
    report=struct('quality',struct2table(rows),'timing',struct2table(timingRows));
    writetable(report.quality,fullfile(outputDirectory,'feature_counts.csv'));
    writetable(report.timing,fullfile(outputDirectory,'coarse_timings.csv'));
end

function activateRoot(root)
    folders={'config','perception',fullfile('perception','groundSegmentation'), ...
        fullfile('perception','groundFeatures'),fullfile('perception','offGroundFeatures'), ...
        fullfile('perception','semanticProduct')};
    for k=1:numel(folders), addpath(fullfile(root,folders{k})); end
    assert(startsWith(string(which('perceiveFrame')),string(root)), 'Unexpected perception source.');
end

function elapsed=measure(frame,cfg)
    perceiveFrame(frame,cfg); perceiveFrame(frame,cfg);
    elapsed=zeros(11,1);
    for k=1:numel(elapsed)
        start=tic; perceiveFrame(frame,cfg); elapsed(k)=toc(start);
    end
end
