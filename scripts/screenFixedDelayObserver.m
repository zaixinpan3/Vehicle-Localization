function results = screenFixedDelayObserver(outputFolder)
% screenFixedDelayObserver Compare nominal periodic delay-feedback multipliers.
% This finite linear diagnostic disables auxiliary injection and GPS. It is
% neither a nonlinear certificate nor a sweep of the full uncertainty box.
% The direct alternative freezes the acquisition-time innovation at delivery;
% the implemented alternative transports residual and gain by the nominal flow.
    arguments
        outputFolder (1,1) string = ""
    end
    setupVehicleLocalization;
    cfg=improvedObserverConfig;
    design=improvedObserverReferenceDesign(cfg);
    L=diag(cfg.observer.theta.^cfg.observer.scalingExponents)*design.K;
    C=zeros(3,7);C(:,[1,4,7])=eye(3);
    delay=cfg.measurement.fixedLidarDelay;
    onTime=cfg.measurement.lidarMaximumAge;
    rows=zeros(24,6);row=0;
    for rate=[0,-.6,.6]
        sample=struct('longitudinalSpeed',0,'longitudinalAcceleration',0, ...
            'lateralAcceleration',0,'lateralVelocity',0,'yawRate',rate, ...
            'sideSlipAngle',0,'sideSlipAngleRate',0);
        channels=evaluateImprovedObserverChannels(zeros(7,1),sample,cfg.operating);
        A=channels.modelMatrix;
        for weight=[.8,1]
            B=L*(weight*eye(3))*C;
            for interval=[.05,.08,.1,.11]
                direct=heldInnovationMap(A,B,interval,delay,onTime);
                % Constant A: the transport period map is similar to this
                % matrix for any fixed delay with ordered nonoverlapping pulses.
                transported=expm(A*interval)*expm(-B*onTime);
                row=row+1;
                rows(row,:)=[rate,weight,interval,delay, ...
                    max(abs(eig(direct))),max(abs(eig(transported)))];
            end
        end
    end
    results=array2table(rows,'VariableNames',{'courseRateRadPerS','weight', ...
        'intervalSeconds','delaySeconds','heldInnovationRadius','transportRadius'});
    if strlength(outputFolder)>0
        if ~isfolder(outputFolder),mkdir(outputFolder);end
        writetable(results,fullfile(outputFolder,'nominal_periodic_screen.csv'));
    end
end

function transition=heldInnovationMap(A,B,interval,delay,onTime)
% heldInnovationMap Exact one-period map with frozen sampled-error snapshots.
    n=size(A,1);depth=ceil(delay/interval)+1;
    columns=n*(depth+1);stateMap=[eye(n),zeros(n,columns-n)];
    arrivals=(-depth:0)*interval+delay;
    edges=sort(unique([0,interval,arrivals(arrivals>0 & arrivals<interval), ...
        arrivals(arrivals+onTime>0 & arrivals+onTime<interval)+onTime]));
    for part=1:numel(edges)-1
        left=edges(part);right=edges(part+1);mid=(left+right)/2;
        acquisitionIndex=floor((mid-delay)/interval+1e-12);
        age=mid-(acquisitionIndex*interval+delay);
        frozen=zeros(n,columns);
        if age<onTime
            first=-acquisitionIndex*n+1;
            frozen(:,first:first+n-1)=eye(n);
        end
        flow=expm([A,eye(n);zeros(n,2*n)]*(right-left));
        stateMap=flow(1:n,1:n)*stateMap-flow(1:n,n+1:end)*B*frozen;
    end
    transition=[stateMap;eye(n*depth),zeros(n*depth,n)];
end
