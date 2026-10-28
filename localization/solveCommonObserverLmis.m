function [result,audit,multipliers]=solveCommonObserverLmis(base,cost,counts,witness,z)
% Finite constraint generation: every candidate is checked against ALL
% controls. A subset solve is never accepted as a common certificate.
    offsets=[0,cumsum(counts)];active=[];
    for family=1:3
        if counts(family)>0
            active=[active,offsets(family)+unique(round(linspace(1,counts(family),min(16,counts(family)))))]; %#ok<AGROW>
        end
    end
    % Every local multiplier must be constrained by a motion control from
    % its own box in the first solve, including interior-domain controls.
    [group,groups]=coupledMotionSupport.multiplierGroups(witness.domain);
    if groups>1
        motion=14:27:numel(group);
        for family=1:3
            sensors=counts(family)/numel(group);
            active=[active,offsets(family)+(motion-1)*sensors+1]; %#ok<AGROW>
        end
        active=unique(active);
    end
    iterationLog=zeros(40,3);converged=false;solverAttempts=zeros(0,5);
    % Preserve 90% of the explicit 1% homogeneous lambda-I margin even
    % when a previously unchecked control is only weakly negative.
    target=-.009*witness.domain.lambda;
    for iteration=1:size(iterationLog,1)
        [L,G,M]=sensorRedundancySupport.blocks(witness,true,active);blocks=[L,G,M];
        cuts=cell(1,numel(active));
        for j=1:numel(active),cuts{j}=blocks{j}<=-z.margin*eye(8);end
        % Adaptive step differentiation avoids observed false-infeasibility
        % reports for these SDPs. A retry changes numerics, never the LMIs.
        strategies=[2,1];
        if z.solver=="sedumi",strategies=[2,1;0,1;2,10;2,.1];end
        for attempt=1:size(strategies,1)
            step=strategies(attempt,1);costScale=strategies(attempt,2);
            options=sdpsettings('solver',char(z.solver),'verbose',0,'savesolveroutput',1);
            if z.solver=="sedumi",options.sedumi.stepdif=step;end
            % A positive scalar changes conditioning, not the minimizers.
            result=optimize([base,cuts{:}],costScale*cost,options);
            baseResidual=min(check(base));
            solverAttempts(end+1,:)=[iteration,step,costScale,result.problem,baseResidual]; %#ok<AGROW>
            if ismember(result.problem,[0,4]) && isfinite(baseResidual) && baseResidual>=-1e-6,break;end
            fprintf('COMMON NUMERICAL RETRY: cut %d, stepdif %d, objective scale %g, status %d\n', ...
                iteration,step,costScale,result.problem);
        end
        assert(ismember(result.problem,[0,4]),'VehicleLocalization:InfeasibleInvariantDesign', ...
            'Invariant synthesis failed: %s',result.info);
        assert(isfinite(baseResidual) && baseResidual>=-1e-6,'VehicleLocalization:CommonLmiNumericalFailure', ...
            'The SDP candidate violates a base constraint by %.9g.',-baseResidual);
        candidate=witness;candidate.P=1.01*value(witness.P);
        for name=["kappaL","kappaG","speedSquaredGain","bilinearGain","directionGain"]
            candidate.(name)=1.01*value(witness.(name));
        end
        for family=1:numel(witness.accelerationMultipliers)
            T=1.01*value(witness.accelerationMultipliers{family});
            candidate.accelerationMultipliers{family}=(T+T.')/2;
        end
        [L,G,M]=sensorRedundancySupport.blocks(candidate,true);allBlocks=[L,G,M];
        [candidate,allBlocks]=removeUnneededMultipliers(candidate,allBlocks,counts,target);
        [penalty,valid]=sensorRedundancySupport.multiplierResidual(candidate.accelerationMultipliers);
        residual=zeros(1,numel(allBlocks));
        for j=1:numel(allBlocks)
            A=allBlocks{j};residual(j)=max(eig((A+A.')/2));
        end
        for family=1:3
            ids=(offsets(family)+1):offsets(family+1);residual(ids)=residual(ids)+max(penalty(family,:));
        end
        worst=max(residual);iterationLog(iteration,:)=[numel(active),worst,value(cost)];
        fprintf('COMMON CUT %d: %d active, all %d checked, worst %.9g, objective %.9g\n',iteration,numel(active),sum(counts),worst,value(cost));
        if valid && isfinite(worst) && worst<target,converged=true;break;end
        add=[];
        for family=1:3
            ids=(offsets(family)+1):offsets(family+1);
            ids=setdiff(ids(residual(ids)>=target),active,'stable');
            [~,order]=sort(residual(ids),'descend');
            ranked=ids(order);
            if groups>1 && ~isempty(ranked)
                sensors=counts(family)/numel(group);
                regions=group(ceil((ranked-offsets(family))/sensors));
                [~,first]=unique(regions,'stable');
                first=first(:).';
                % Cover violating boxes before adding near-duplicate sensor
                % corners from one box. This changes only cut ordering.
                ranked=ranked([first,setdiff(1:numel(ranked),first,'stable')]);
            end
            add=[add,ranked(1:min(max(24,groups),numel(ranked)))]; %#ok<AGROW>
        end
        assert(~isempty(add),'VehicleLocalization:CommonLmiNumericalFailure', ...
            'The active solve fails full numeric verification; no unchecked violating control remains.');
        active=unique([active,add]);
    end
    assert(converged,'VehicleLocalization:CommonLmiIterationLimit', ...
        'Finite constraint generation did not verify the full common family.');
    audit=struct('iterations',iteration,'activeControls',numel(active), ...
        'verifiedControls',sum(counts),'fullFamilyVerified',true, ...
        'requiredWorstResidual',target,'trace',iterationLog(1:iteration,:), ...
        'solverAttempts',solverAttempts,'solverAttemptColumns',{{'iteration','stepdif','objectiveScale','status','baseResidual'}});
    % Residuals describe the unscaled final SDP solve. The full common
    % verification above describes the returned homogeneously scaled gains.
    audit.minimumBaseConstraintResidual=min(check(base));
    multipliers=candidate.accelerationMultipliers;
    audit.zeroMultipliers=nnz(cellfun(@(T)all(T==0,'all'),multipliers));
    if isfield(result.solveroutput,'info')
        audit.solverDiagnostics=result.solveroutput.info;
    end
end

function [w,blocks]=removeUnneededMultipliers(w,blocks,counts,target)
% Multipliers are certificate variables with no effect on the observer or
% objective. Free inactive multipliers can create artificial violated cuts.
% Set one to zero ONLY after its entire unaugmented region verifies. This
% preserves every accepted inequality without adding a regularization cost.
    controls=coupledMotionSupport.controls(w.inputs,w.domain);
    g=coupledMotionSupport.accelerationConstraint(w.inputs,controls);
    [group,groups]=coupledMotionSupport.multiplierGroups(w.domain);
    offsets=[0,cumsum(counts)];raw=cell(size(blocks));worst=-Inf(3,groups);
    for family=1:3
        sensors=counts(family)/numel(group);
        for j=1:counts(family)
            motion=ceil(j/sensors);region=group(motion);id=offsets(family)+j;
            T=w.accelerationMultipliers{family,region};F=blocks{id}-g(motion)*T;
            raw{id}=F;worst(family,region)=max(worst(family,region),max(eig((F+F.')/2)));
        end
    end
    useZero=worst<target;
    for j=find(useZero(:)).',w.accelerationMultipliers{j}=zeros(8);end
    for family=1:3
        sensors=counts(family)/numel(group);
        for j=1:counts(family)
            if useZero(family,group(ceil(j/sensors)))
                id=offsets(family)+j;blocks{id}=raw{id};
            end
        end
    end
end
