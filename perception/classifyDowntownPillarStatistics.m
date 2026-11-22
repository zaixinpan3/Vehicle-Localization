function [ground,offGround]=classifyDowntownPillarStatistics(ground,offGround,names,cfg)
% classifyDowntownPillarStatistics: Select pillars using distribution summaries.
% Only branch rasters and whole-pillar statistics are read. Fine geometry,
% source point labels and reference data are unavailable to this classifier.
    assert(abs(cfg.spacing-.3)<1e-12,'perception:DowntownRuleLattice', ...
        'Rebuild the Downtown configuration on the 0.3 m lattice.');
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
    if any(ismember(names,["pole","facade"]))
        occupied=offGround.columnMaps.occupiedMask;
        gate=[];
        if any(names=="pole")
            gate=filterDowntownPoleDistribution(offGround.columnMaps,cfg.poleDistributionGate);
            offGround.poleCellMask=offGround.poleCellMask & gate;
        end
        for name=["pole","facade"]
            halo=cfg.structuralHaloMeters;
            if name=="facade",halo=cfg.facadeHaloMeters;end
            radius=ceil(halo/cfg.spacing);
            mask=imdilate(offGround.(name+"CellMask"),ones(2*radius+1)) & occupied;
            if name=="pole" && ~isempty(gate)
                % A neighbor may propose a member, but cannot supply its
                % missing distribution evidence after seed filtering.
                mask=mask & gate;
            end
            offGround.(name+"CellMask")=mask;
            offGround.(name+"Probability")=single(mask.*max(.5,double(offGround.(name+"Probability"))));
        end
    end
    if any(names=="trafficSign") && isfield(cfg,'signMinimumReflectiveMeanClearance')
        radiation=offGround.columnMaps.radiometry;
        known=radiation.ground.r4_count>0;
        clearance=radiation.reflective.meanXYZ(:,3)-radiation.ground.r4_meanZ;
        keep=~known | clearance>=cfg.signMinimumReflectiveMeanClearance;
        mask=false(size(offGround.trafficSignCellMask));mask(double(radiation.pillarIndices))=keep;
        offGround.trafficSignCellMask=offGround.trafficSignCellMask & mask;
        offGround.trafficSignProbability(~offGround.trafficSignCellMask)=0;
    end
    [ground,offGround]=applyDowntownPrecisionRules(ground,offGround,names,cfg.precisionFilter.rules);
    if any(names=="curb")
        if ~isfield(cfg,'curbRoadPatch')
            cfg.curbRoadPatch=downtownCandidateConfig(cfg.spacing).curbRoadPatch;
        end
        ground.curbCellMask=rejectCurbRoadPatches(ground.curbCellMask,candidates,ground.stats,cfg.spacing,cfg.curbRoadPatch);
        ground.curbProbability(~ground.curbCellMask)=0;
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
