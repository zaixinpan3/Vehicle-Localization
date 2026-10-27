function [result,audit]=solveCommonObserverLmis(base,cost,blocks,counts,witness,z)
% Finite constraint generation: every candidate is checked against ALL
% controls. A subset solve is never accepted as a common certificate.
    offsets=[0,cumsum(counts)];active=[];
    for family=1:3
        if counts(family)>0
            active=[active,offsets(family)+unique(round(linspace(1,counts(family),min(16,counts(family)))))]; %#ok<AGROW>
        end
    end
    iterationLog=zeros(40,3);converged=false;
    % Preserve 90% of the explicit 1% homogeneous lambda-I margin even
    % when a previously unchecked control is only weakly negative.
    target=-.009*witness.domain.lambda;
    for iteration=1:size(iterationLog,1)
        cuts=cell(1,numel(active));
        for j=1:numel(active),cuts{j}=blocks{active(j)}<=-z.margin*eye(8);end
        result=optimize([base,cuts{:}],cost,sdpsettings('solver',char(z.solver),'verbose',0));
        assert(ismember(result.problem,[0,4]),'VehicleLocalization:InfeasibleInvariantDesign', ...
            'Invariant synthesis failed: %s',result.info);
        candidate=witness;candidate.P=1.01*value(witness.P);
        for name=["kappaL","kappaG","speedSquaredGain","bilinearGain","directionGain"]
            candidate.(name)=1.01*value(witness.(name));
        end
        [L,G,M]=sensorRedundancySupport.blocks(candidate,true);allBlocks=[L,G,M];
        residual=zeros(1,numel(allBlocks));
        for j=1:numel(allBlocks)
            A=allBlocks{j};residual(j)=max(eig((A+A.')/2));
        end
        worst=max(residual);iterationLog(iteration,:)=[numel(active),worst,value(cost)];
        fprintf('COMMON CUT %d: %d active, all %d checked, worst %.9g\n',iteration,numel(active),sum(counts),worst);
        if isfinite(worst) && worst<target,converged=true;break;end
        add=[];
        for family=1:3
            ids=(offsets(family)+1):offsets(family+1);
            ids=setdiff(ids(residual(ids)>=target),active,'stable');
            [~,order]=sort(residual(ids),'descend');
            add=[add,ids(order(1:min(24,numel(order))))]; %#ok<AGROW>
        end
        assert(~isempty(add),'VehicleLocalization:CommonLmiNumericalFailure', ...
            'The active solve fails full numeric verification; no unchecked violating control remains.');
        active=unique([active,add]);
    end
    assert(converged,'VehicleLocalization:CommonLmiIterationLimit', ...
        'Finite constraint generation did not verify the full common family.');
    audit=struct('iterations',iteration,'activeControls',numel(active), ...
        'verifiedControls',sum(counts),'fullFamilyVerified',true, ...
        'requiredWorstResidual',target,'trace',iterationLog(1:iteration,:));
end
