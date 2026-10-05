function accepted=evaluatePillarDistributionRules(features,names,rules)
% evaluatePillarDistributionRules: Union of aggregate-distribution conditions.
% Each alternative describes one supported statistical morphology. All its
% predicates must pass; any alternative can retain the complete pillar.
% Historical conjunction arrays remain supported for strictPrecision.
    if isstruct(rules) && isscalar(rules) && isfield(rules,'alternatives')
        accepted=false(size(features,1),1);
        for alternative=reshape(rules.alternatives,1,[])
            accepted=accepted | conjunction(features,names,alternative.predicates);
        end
    else
        accepted=conjunction(features,names,rules);
    end
end

function accepted=conjunction(features,names,predicates)
    accepted=true(size(features,1),1);
    for predicate=reshape(predicates,1,[])
        column=find(names==string(predicate.feature));
        assert(isscalar(column),'perception:PrecisionFeatureSchema','Unknown distribution descriptor.');
        value=features(:,column);
        switch string(predicate.operator)
            case ">=",pass=value>=predicate.threshold;
            case "<=",pass=value<=predicate.threshold;
            case ">",pass=value>predicate.threshold;
            otherwise,error('perception:PrecisionOperator','Unknown distribution predicate.');
        end
        accepted=accepted & isfinite(value) & pass;
    end
end
