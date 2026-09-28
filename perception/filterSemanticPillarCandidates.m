function [ground,offGround]=filterSemanticPillarCandidates(ground,offGround,names,cfg,groundContext,offGroundPillars)
% filterSemanticPillarCandidates: Require class-specific distribution evidence.
% Filter masks and recover validated raw curb proposals before publication.
% Ground segmentation, reference labels and complete-cell moments are unchanged.
    if ~cfg.enabled,return;end
    recovery=any(string(names)=="curb") && any(cfg.classes=="curb") ...
        && isfield(cfg,'modelFiles') && isfield(cfg.modelFiles,'curbRecovery');
    if recovery,originalCurbMask=ground.curbCellMask;end
    for name=string(names(:)).'
        if ~any(cfg.classes==name),continue;end
        if name=="curb"
            spacing=ground.cellSize;
            present=any(ground.curbCellMask,'all');
        else
            spacing=[offGround.columnMaps.dx,offGround.columnMaps.dy];
            present=any(offGround.(name+"CellMask"),'all');
        end
        if ~present,continue;end
        assert(all(abs(spacing-.6)<1e-12),'perception:SemanticModelLattice', ...
            'Semantic precision models require 0.6 m whole pillars.');
        [baseX,baseNames,ids]=measureSemanticPillarFeatures(ground,offGround,name);
        [support,supportNames]=measureSemanticPointDistributions(ground,offGround,name,groundContext,offGroundPillars);
        x=[baseX,support];features=[baseNames,supportNames];
        if isempty(ids),continue;end
        if name=="facade"
            [groupFeatures,groupNames]=measureFacadeGroupEvidence(offGround,offGroundPillars);
            x=[x,groupFeatures];features=[features,groupNames]; %#ok<AGROW>
        end
        modelFile=name+"PillarPrecisionModel.json";
        if isfield(cfg,'modelFiles') && isfield(cfg.modelFiles,name)
            modelFile=cfg.modelFiles.(name);
        end
        model=readModel(fullfile(cfg.modelDirectory,modelFile));
        assert(isequal(features(:),string(model.featureNames(:))), ...
            'perception:SemanticFeatureSchema','Semantic precision feature schema mismatch.');
        threshold=model.decisionThreshold;
        if isfield(model.decisionThresholds,cfg.profile),threshold=model.decisionThresholds.(cfg.profile);end
        % The flattened-tree evaluator is independent of semantic class.
        scores=scoreSemanticPillarModel(x,model,threshold);accepted=scores>=threshold;
        maskKey=name+"CellMask";probabilityKey=name+"Probability";
        diagnostic=struct('candidateIds',ids,'scores',scores,'accepted',accepted,'threshold',threshold);
        if name=="curb"
            ground.(maskKey)(ids(~accepted))=false;
            ground.(probabilityKey)(ids(~accepted))=0;
            ground.precision=diagnostic;
        else
            offGround.(maskKey)(ids(~accepted))=false;
            offGround.(probabilityKey)(ids(~accepted))=0;
            offGround.precision.(name)=diagnostic;
        end
    end
    if recovery
        model=readModel(fullfile(cfg.modelDirectory,cfg.modelFiles.curbRecovery));
        ground=recoverCurbRawProposals(ground,originalCurbMask,model);
    end
end

function model=readModel(path)
    persistent paths models
    if isempty(paths),paths=strings(0,1);models=cell(0,1);end
    at=find(paths==path,1);
    if isempty(at)
        model=jsondecode(fileread(path));model.leaf=logical(model.leaf);
        paths(end+1,1)=path;models{end+1,1}=model;
    else
        model=models{at};
    end
end
