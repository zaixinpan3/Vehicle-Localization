function region=globalObserverHighGainRegion(cfg)
% Estimate the open lower endpoint of the continuous global LMI gain region.
% All theta > thetaInfimum are feasible with free block metrics. Inf denotes
% an empty region. maxScaledYawRate is the supremum of maximumBodyYawRate/theta.
    arguments
        cfg (1,1) struct=mncavFullObserverConfig()
    end
    setupVehicleLocalization();
    inputs=localizationSupport.invariantInputs(cfg);
    floors=min(localizationSupport.poseSourceBounds(cfg),[],2);
    region=struct('thetaInfimum',Inf,'maxScaledYawRate',0);
    if inputs.lambda>=2*min(floors)
        return
    end
    options=observerInternal.globalSolverOptions(inputs.solver);
    yalmip('clear');
    Q=sdpvar(3,3,'symmetric');
    u=sdpvar(1);w=sdpvar(1);
    % u is squared scaled rate. The 2-by-2 cone imposes w >= u^2.
    penalty=diag([2*floors(1),0,0])-inputs.lambda*diag([1,u,w]);
    constraints=[Q>=0,u>=0,[w,u;u,1]>=0, ...
        observerInternal.translationDriftForm(Q,0)<=penalty, ...
        observerInternal.translationDriftForm(Q,1)<=penalty];
    result=optimize(constraints,-u,options);
    assert(result.problem==0 && isfinite(value(u)) && value(u)>0, ...
        'VehicleLocalization:HighGainRegionSolve','Cannot resolve the high-gain boundary: %s.',result.info);
    region.maxScaledYawRate=sqrt(value(u));
    region.thetaInfimum=cfg.maximumBodyYawRate/region.maxScaledYawRate;
end
