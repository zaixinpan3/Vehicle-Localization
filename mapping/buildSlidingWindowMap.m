function map = buildSlidingWindowMap(featureData, cfg)
% buildSlidingWindowMap: Schedule frame ingestion; fit uniquely owned tiles.
% Windows describe acquisition batches, not independent maps. Their union is
% ingested once and canonicalMap owns all geometry. Offline inference runs one
% spatial tile plus halo at a time. This implementation retains the compacted
% observations in memory; it is not an out-of-core storage implementation.
% Optional featureData.observationBlockIds contains one block ID per frame.
    frames=double(featureData.frameIndices(:).');
    windows=schedule(frames,cfg.batchFrameCount,cfg.batchFrameStride);
    batches=repmat(struct('batchIndex',0,'frameIndices',[],'ingestedFrameIndices',[]),numel(windows),1);
    visited=zeros(1,0);
    for j=1:numel(windows)
        batches(j)=struct('batchIndex',j,'frameIndices',windows{j}, ...
            'ingestedFrameIndices',setdiff(windows{j},visited,'stable'));
        visited=union(visited,windows{j});
    end
    [~,ingestionOrder]=ismember([batches.ingestedFrameIndices],frames);
    names=string(featureData.featureNames(:));
    assert(size(featureData.pointsByFeatureFrame,1)==numel(names) && ...
        size(featureData.pointsByFeatureFrame,2)==numel(frames),'Feature/frame dimensions disagree.');
    gmmCfg=temporalStabilityMapConfig();
    if isfield(cfg,'temporalMap'), gmmCfg=cfg.temporalMap; end
    gmmCfg.classes=names; gmmCfg.logEnabled=mappingSupport.isLogEnabled(cfg,"logEnabled");
    points=cell(numel(names)*numel(frames),1); labels=points; frameIds=points; sourceIds=points; blockIds=points;
    explicitBlocks=isfield(featureData,'observationBlockIds');
    if explicitBlocks
        assert(numel(featureData.observationBlockIds)==numel(frames),'One observation block ID is required per frame.');
    end
    entry=0;
    for f=ingestionOrder
        for c=1:numel(names)
            entry=entry+1; xyz=double(featureData.pointsByFeatureFrame{c,f});
            if isempty(xyz), xyz=zeros(0,3); end
            assert(size(xyz,2)==3,'Registered observations must retain XYZ.');
            valid=all(isfinite(xyz(:,1:2)),2) & (isfinite(xyz(:,3)) | isnan(xyz(:,3)));
            rows=find(valid); xyz=xyz(valid,:); n=size(xyz,1);
            points{entry}=xyz; labels{entry}=repmat(names(c),n,1);
            frameIds{entry}=repmat(frames(f),n,1);
            sourceIds{entry}=string(frames(f))+":"+names(c)+":"+string(rows);
            if explicitBlocks, blockIds{entry}=repmat(string(featureData.observationBlockIds(f)),n,1); end
        end
    end
    inputFrames=vertcat(frameIds{:});
    if explicitBlocks
        observations=struct('frameId',inputFrames,'observationBlockId',vertcat(blockIds{:}), ...
            'sourceId',vertcat(sourceIds{:}));
    else
        validateattributes(gmmCfg.observationBlockSize,{'numeric'},{'scalar','positive','integer','finite'});
        validateattributes(gmmCfg.blockOriginFrame,{'numeric'},{'scalar','integer','finite'});
        observations=struct('frameId',inputFrames, ...
            'observationBlockId',floor((inputFrames-gmmCfg.blockOriginFrame)/gmmCfg.observationBlockSize), ...
            'sourceId',vertcat(sourceIds{:}));
    end
    canonical=buildTemporalStabilityGmmMap(vertcat(points{:}),vertcat(labels{:}),observations,gmmCfg);
    if isfield(featureData,'frameCalibration'), canonical.frameCalibration=featureData.frameCalibration; end
    map=struct('mapType',"scheduledRepeatedObservationGaussianField",'schemaVersion',2, ...
        'queryFunction',"queryTemporalStabilityGmmMap",'queryBatchFusion',"canonicalOwnership", ...
        'canonicalMap',canonical,'batchMaps',batches,'batchFrameWindows',{windows}, ...
        'frameIndices',frames,'sourceFrameCount',numel(frames),'featureNames',names, ...
        'batchFrameCount',cfg.batchFrameCount,'batchFrameStride',cfg.batchFrameStride, ...
        'spatialDimension',3,'heightModel',"conditionalGaussianGivenXY",'config',cfg);
    if isfield(featureData,'frameSummaryTable'), map.frameSummaryTable=featureData.frameSummaryTable; end
    if isfield(featureData,'frameCalibration'), map.frameCalibration=featureData.frameCalibration; end
    counts=arrayfun(@(layer) numel(layer.components),canonical.layers).';
    published=arrayfun(@(layer) nnz(layer.componentPublished),canonical.layers).';
    masses=arrayfun(@(layer) layer.totalMass,canonical.layers).';
    map.layerSummaryTable=table(names,counts(:),published(:),masses(:), ...
        'VariableNames',{'classLabel','componentCount','publishedComponentCount','totalMass'});
end

function windows=schedule(frames,count,stride)
    validateattributes(count,{'numeric'},{'scalar','integer','positive','finite'});
    validateattributes(stride,{'numeric'},{'scalar','integer','positive','finite'});
    assert(stride<=count,'buildSlidingWindowMap:UncoveredFrames','Window stride must not exceed window count.');
    assert(~isempty(frames) && all(isfinite(frames)) && all(frames==floor(frames)) && all(diff(frames)==1), ...
        'buildSlidingWindowMap:InvalidFrames','Frame indices must be contiguous finite integers.');
    if numel(frames)<=count, windows={frames}; return; end
    last=numel(frames)-count+1; starts=unique([1:stride:last,last]);
    windows=arrayfun(@(i) frames(i:i+count-1),starts,'UniformOutput',false).';
end
