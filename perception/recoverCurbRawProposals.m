function ground=recoverCurbRawProposals(ground,originalMask,model)
% recoverCurbRawProposals: Reconsider geometry suppressed by road adjacency.
% Recover only omitted raw proposals with strong distribution evidence or a
% slender connected chain. All statistics and topology use whole 0.6 m cells.
    assert(all(abs(ground.cellSize-.6)<1e-12),'perception:SemanticModelLattice', ...
        'Curb recovery requires 0.6 m whole pillars.');
    if ~isfield(ground.energyMaps,'rawExtractedMask'),return;end
    rule=model.recovery;parameters=ground.curbProbabilityParameters;
    pool=ground.energyMaps.rawExtractedMask & ~originalMask ...
        & ground.energyMaps.totalBase>=rule.minimumBaseEnergy ...
        & ground.energyMaps.total>=parameters.minimumEnergy;
    if ~any(pool,'all'),return;end
    proposals=ground;proposals.curbCellMask=pool;
    [features,names,ids]=measureSemanticPillarFeatures(proposals,struct(),"curb");
    assert(isequal(names(:),string(model.featureNames(:))), ...
        'perception:SemanticFeatureSchema','Curb recovery feature schema mismatch.');
    score=scoreSemanticPillarModel(features,model,model.decisionThreshold);
    accepted=score>=model.decisionThreshold;
    weak=false(size(pool));weak(ids(score>=rule.weak))=true;
    anchor=false(size(pool));anchor(ids(score>=rule.anchor))=true;
    components=bwconncomp(weak,8);supported=false(size(pool));
    for k=1:components.NumObjects
        members=components.PixelIdxList{k};count=numel(members);seeds=nnz(anchor(members));
        if seeds<rule.seeds || seeds/count<rule.seedFraction,continue;end
        [row,col]=ind2sub(size(pool),members);xy=[col(:),row(:)].*ground.cellSize;
        centered=xy-mean(xy,1);covariance=(centered.'*centered)/count;
        spread=hypot(covariance(1,1)-covariance(2,2),2*covariance(1,2));
        total=trace(covariance);small=max(0,(total-spread)/2);large=max(0,(total+spread)/2);
        values=[sqrt(small),2*sqrt(3*large)+ground.cellSize(1),spread/max(total,1e-12)];
        values=floor(values*rule.geometryScale+.5)/rule.geometryScale;
        if values(1)<=rule.width && values(2)>=rule.length && values(3)>=rule.minimumAnisotropy
            supported(members)=true;
        end
    end
    accepted=accepted | supported(ids);chosen=ids(accepted);
    ground.curbCellMask(chosen)=true;
    normalized=(double(ground.energyMaps.total(chosen))-parameters.minimumEnergy)/max(1-parameters.minimumEnergy,eps);
    normalized=min(1,max(0,normalized));
    ground.curbProbability(chosen)=single(parameters.minimumProbability+(1-parameters.minimumProbability)*normalized);
    ground.recovery=struct('candidateIds',ids,'scores',score,'accepted',accepted);
end
