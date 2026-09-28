function diagnosePerception827()
% diagnosePerception827 Trace precise reference owners through coarse gates.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame827_perception_20260928';
    addpath('research/pillar_fine_alignment_20260926');
    frame=loadPointCloudFrame(fullfile(root,'data/raw/MissisipiPointClouds.mat'),827);
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    cfg.offGroundFeatures.pole.distributionValidation.minimumScore=.9371209151674477;
    cfg.semanticPrecision.modelFiles.curb="mississippiCurbPillarPrecisionModel.json";
    p=perceiveFrame(frame,cfg);unfilteredCfg=cfg;unfilteredCfg.semanticPrecision.enabled=false;raw=perceiveFrame(frame,unfilteredCfg);
    cache=load('output/semantic_precision_20260927/mississippi_baseline.mat','frames','references','names');
    at=cache.frames==827;fine=double(cache.references{at,cache.names=="curb"}(:));
    g=raw.diagnostics.ground;gc=raw.diagnostics.groundPointContext;final=p.diagnostics.ground;
    geom=struct('origin',g.cellOrigin,'cellSize',g.cellSize,'mapSize',g.cellMapSize);
    [~,d]=measureFinePoleAlignment(frame,fine,find(g.curbCellMask),geom);
    counts=accumarray(d.finePointPillarIds,1,[prod(g.cellMapSize) 1]);
    pool=g.curbCellMask | g.energyMaps.rawExtractedMask | reshape(counts>0,g.cellMapSize);
    input=g;input.curbCellMask=pool;[x,names,ids]=measureSemanticPillarFeatures(input,struct(),"curb");
    [support,supportNames]=measureSemanticPointDistributions(input,struct(),"curb",gc,struct());
    [r,c]=ind2sub(g.cellMapSize,ids);xy=g.cellOrigin+([c r]-.5).*g.cellSize;
    bins=floor((xy-p.candidates.geometry.origin)./p.candidates.geometry.cellSize)+1;globalIds=sub2ind(p.candidates.geometry.mapSize,bins(:,2),bins(:,1));
    scores=nan(prod(g.cellMapSize),1);scores(final.precision.candidateIds)=final.precision.scores;
    tableOut=table(ids,globalIds,xy(:,1),xy(:,2),counts(ids),g.curbCellMask(ids),final.curbCellMask(ids),scores(ids), ...
        'VariableNames',{'localPillar','pillar','x','y','finePointCount','proposal','selected','score'});
    tableOut=[tableOut,array2table([x support],VariableNames=[names supportNames])];
    writetable(tableOut,fullfile(dest,'curb_cells.csv'));
    points=gc.groundPoints;[col,row]=ind2sub(g.cellMapSize([2 1]),gc.groundCellLinIdx);
    pointIds=sub2ind(g.cellMapSize,row,col);original=gc.groundOriginalPointIdx;
    pointTable=table(points(:,1),points(:,2),points(:,3),pointIds,original,ismember(original,fine), ...
        'VariableNames',{'x','y','z','localPillar','originalIndex','reference'});
    writetable(pointTable,fullfile(out,'ground_points.csv'));
    fprintf('Reference ground %d/%d; proposed points %d; final points %d\n',nnz(ismember(fine,original)),numel(fine),sum(counts(g.curbCellMask)),sum(counts(final.curbCellMask)));
    save(fullfile(out,'frame827.mat'),'p','raw','frame','cfg','fine','counts','-v7.3');
    fprintf('PERCEPTION827_DIAGNOSIS_COMPLETED\n');
end
