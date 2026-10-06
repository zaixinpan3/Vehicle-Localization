function [ground,offGround]=classifyDowntownPillarStatistics(ground,offGround,names,cfg)
% classifyDowntownPillarStatistics: Select pillars using distribution summaries.
% Only branch rasters and whole-pillar statistics are read. Fine geometry,
% source point labels and reference data are unavailable to this classifier.
    if any(names=="curb")
        original=ground.curbCellMask;occupied=ground.stats.countMap>0;
        radius=ceil(cfg.curbHaloMeters/cfg.spacing);
        candidates=imdilate(original,ones(2*radius+1)) & occupied;
        maps=ground.energyMaps;
        extension=maps.total>=cfg.curbExtensionMinimumTotalEnergy & maps.totalBase>=cfg.curbExtensionMinimumBaseEnergy & ...
            maps.heightStepMeters>=cfg.curbExtensionMinimumHeightStep & ...
            maps.linearityComponentCenterEvidence>=cfg.curbExtensionMinimumCenterEvidence;
        if isfield(maps,'relativeHeightMeters')
            extension=extension & maps.relativeHeightMeters<=cfg.curbExtensionMaximumRelativeHeight;
        end
        horizontal=ceil(cfg.curbExtensionMeters/cfg.spacing);vertical=ceil(cfg.curbExtensionHalfWidthMeters/cfg.spacing);
        candidates=candidates | (extension & occupied & imdilate(original,ones(2*vertical+1,2*horizontal+1)));
        ground.curbCellMask=candidates;
        ground.curbProbability=single(candidates.*max(.5,double(ground.curbProbability)));
    end
    if cfg.spacing<.6 && any(ismember(names,["pole","facade"]))
        occupied=offGround.columnMaps.occupiedMask;
        gate=[];
        if any(names=="pole") && isfield(cfg,'poleDistributionGate') && cfg.poleDistributionGate.enabled
            gate=filterDowntownPoleDistribution(offGround.columnMaps,cfg.poleDistributionGate);
            offGround.poleCellMask=offGround.poleCellMask & gate;
        end
        for name=["pole","facade"]
            halo=cfg.structuralHaloMeters;
            if name=="facade",halo=cfg.facadeHaloMeters;end
            radius=ceil(halo/cfg.spacing);
            mask=imdilate(offGround.(name+"CellMask"),ones(2*radius+1)) & occupied;
            if name=="pole" && ~isempty(gate) && ...
                    isfield(cfg.poleDistributionGate,'requireHaloSupport') && cfg.poleDistributionGate.requireHaloSupport
                % A neighbor may propose a member, but cannot supply its
                % missing distribution evidence after seed filtering.
                mask=mask & gate;
            end
            offGround.(name+"CellMask")=mask;
            offGround.(name+"Probability")=single(mask.*max(.5,double(offGround.(name+"Probability"))));
        end
    end
    if cfg.spacing<.6 && any(names=="trafficSign") && isfield(cfg,'signMinimumReflectiveMeanClearance')
        radiation=offGround.columnMaps.radiometry;
        known=radiation.ground.r4_count>0;
        clearance=radiation.reflective.meanXYZ(:,3)-radiation.ground.r4_meanZ;
        keep=~known | clearance>=cfg.signMinimumReflectiveMeanClearance;
        mask=false(size(offGround.trafficSignCellMask));mask(double(radiation.pillarIndices))=keep;
        offGround.trafficSignCellMask=offGround.trafficSignCellMask & mask;
        offGround.trafficSignProbability(~offGround.trafficSignCellMask)=0;
    end
    if ~cfg.useModels
        if isfield(cfg,'precisionFilter') && cfg.precisionFilter.enabled
            [ground,offGround]=applyDowntownPrecisionRules(ground,offGround,names,cfg.precisionFilter.rules);
        end
        offGround=synchronizeStructuralMasks(offGround);
        return;
    end
    assert(abs(cfg.spacing-.6)<1e-12,'perception:DowntownModelLattice', ...
        'Downtown statistical forests require the 0.6 m native pillar lattice.');
    structuralFeatures=[];structuralNames=strings(1,0);structuralIds=[];
    for name=string(names(:)).'
        if ~isfield(cfg.modelFiles,name),continue;end
        if name=="curb"
            [x,featureNames,ids]=measureDowntownPillarStatistics(ground,offGround,name);
        else
            if isempty(structuralNames)
                [structuralFeatures,structuralNames,structuralIds]=measureDowntownPillarStatistics(ground,offGround,name);
            end
            x=structuralFeatures;featureNames=structuralNames;ids=structuralIds;
        end
        model=readModel(fullfile(cfg.modelDirectory,cfg.modelFiles.(name)));
        assert(isequal(featureNames(:),string(model.featureNames(:))), ...
            'perception:DowntownFeatureSchema','Statistical feature schema changed.');
        scores=scoreSemanticPillarModel(x,model,model.decisionThreshold);
        if name=="curb",branch=ground;else,branch=offGround;end
        eligibility=branch.(name+"CellMask");
        mask=false(size(eligibility));mask(ids(scores>=model.decisionThreshold))=true;
        if name=="trafficSign",mask=mask & eligibility;end
        probability=zeros(size(mask));probability(ids)=scores;probability(~mask)=0;
        branch.(name+"CellMask")=mask;branch.(name+"Probability")=single(probability);
        branch.statisticalClassification.(name)=struct('pillarIndices',ids,'scores',scores, ...
            'threshold',model.decisionThreshold,'featureNames',featureNames);
        if name=="curb",ground=branch;else,offGround=branch;end
    end
    offGround=synchronizeStructuralMasks(offGround);
end

function branch=synchronizeStructuralMasks(branch)
% Diagnostic aliases describe the same final candidates as public masks.
    if ~isfield(branch,'columnMaps'),return;end
    branch.columnMaps.trafficSignCellMask=branch.trafficSignCellMask;
    branch.candidates.candidateMask=branch.poleCellMask;
    branch.facade.mask=branch.facadeCellMask;
    branch.facade.lineMap(~branch.facadeCellMask)=0;
    branch.facade.pillarLinIdx=find(branch.facadeCellMask);
    branch.facade.pillarLineIdx=double(branch.facade.lineMap(branch.facade.pillarLinIdx));
end

function model=readModel(filename)
    persistent paths models
    if isempty(paths),paths=strings(0,1);models={};end
    index=find(paths==filename,1);
    if isempty(index)
        model=jsondecode(fileread(filename));model.leaf=logical(model.leaf);
        paths(end+1,1)=string(filename);models{end+1,1}=model;
    else
        model=models{index};
    end
end
