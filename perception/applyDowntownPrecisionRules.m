function [ground,offGround]=applyDowntownPrecisionRules(ground,offGround,names,rules)
% applyDowntownPrecisionRules: Reject uncertain whole-pillar candidates.
% Rules consume aggregate distribution descriptors and statistical raster
% context only. Every predicate must pass. Point labels and point validators
% are unavailable here; all surviving Gaussian moments remain whole-pillar.
    structuralFeatures=[];structuralNames=strings(1,0);structuralIds=[];
    for name=string(names(:)).'
        if ~isfield(rules,name),continue;end
        if name=="curb"
            [x,columns,ids]=measureDowntownPillarStatistics(ground,offGround,name);
            branch=ground;
        else
            if isempty(structuralNames)
                [structuralFeatures,structuralNames,structuralIds]=measureDowntownPillarStatistics(ground,offGround,name);
            end
            x=structuralFeatures;columns=structuralNames;ids=structuralIds;branch=offGround;
        end
        accepted=true(numel(ids),1);predicates=rules.(name);
        for k=1:numel(predicates)
            predicate=predicates(k);column=find(columns==string(predicate.feature));
            assert(isscalar(column),'perception:PrecisionFeatureSchema','Unknown precision descriptor.');
            if strcmp(predicate.operator,'>=')
                accepted=accepted & x(:,column)>=predicate.threshold;
            else
                assert(strcmp(predicate.operator,'<='),'perception:PrecisionOperator','Unknown precision operator.');
                accepted=accepted & x(:,column)<=predicate.threshold;
            end
        end
        % Preserve statistical support for bounded fine neighborhoods. This
        % context cannot restore candidates or acquire point labels itself.
        branch.precisionContext.(name)=branch.(name+'CellMask');
        mask=false(size(branch.(name+'CellMask')));mask(ids(accepted))=true;
        branch.(name+'CellMask')=branch.(name+'CellMask') & mask;
        branch.(name+'Probability')(~branch.(name+'CellMask'))=0;
        if name=="curb",ground=branch;else,offGround=branch;end
    end
end
