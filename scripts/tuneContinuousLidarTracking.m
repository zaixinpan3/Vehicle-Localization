function report=tuneContinuousLidarTracking(outputFolder)
% tuneContinuousLidarTracking Screen certified fixed-delay gains on identical inputs.
% Independent analytic truth, synthetic continuous LiDAR and exact lateral
% inputs isolate the global observer. Original bounds and delay are retained.
    arguments
        outputFolder (1,1) string="output/lidar_tracking_tuning_20260914"
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    cfg=improvedObserverConfig('lidar');reference=improvedObserverReferenceDesign(cfg);
    candidates=[3,3,1;3,3,1.5;3,4,2;3,5,3;3,6,4; ...
        4,4,2;4,5,3;4,6,4;4,8,6;4,10,8; ...
        5,5,3;5,6,4;5,8,6;5,10,8;5,12,10; ...
        6,6,4;6,8,6;6,10,8;2.5,4,2;3.5,6,4];
    certificates=cell(size(candidates,1),1);screen=cell(size(certificates));
    for k=1:numel(certificates)
        candidate=reference;candidate.K(1:3,1)=candidates(k,:).';candidate.K(4:6,2)=candidates(k,:).';
        solverInfo="stored reference";
        if k>1,[candidate,solverInfo]=synthesizeCandidate(candidate,cfg);end
        valid=~isempty(candidate);
        margin=NaN;if valid,margin=candidate.verification.uniformMargin;end
        screen{k}=struct('candidate',k,'k1',candidates(k,1),'k2',candidates(k,2), ...
            'k3',candidates(k,3),'certificatePassed',valid,'margin',margin,'solverInfo',solverInfo);
        certificates{k}=candidate;
        fprintf('Certificate %d/%d: [%g %g %g], pass=%d\n',k,numel(certificates),candidates(k,:),valid);
    end
    screen=struct2table(vertcat(screen{:}));writetable(screen,fullfile(outputFolder,'certificate_screen.csv'));
    scenarios=cell(2,1);for j=1:2,scenarios{j}=makeScenario(cfg,j-1,0);end
    rows={};runs={};
    for k=find(screen.certificatePassed).'
        for j=1:2
            scenario=scenarios{j};runCfg=cfg;runCfg.observer.initialState=scenario.initialState;
            estimate=runImprovedVehicleObserver(scenario.data,struct(),certificates{k},runCfg, ...
                LateralInputs=scenario.lateral,InitialHistory=scenario.history);
            rows{end+1}=metrics(estimate,scenario,k); %#ok<AGROW>
            runs{end+1}=estimate; %#ok<AGROW>
        end
    end
    results=struct2table(vertcat(rows{:}));writetable(results,fullfile(outputFolder,'metrics.csv'));
    base=results(results.candidate==1 & results.noisy,:);
    eligible=results.noisy & results.positionRmseM<=1.05*base.positionRmseM ...
        & results.velocityRmseMps<=base.velocityRmseMps ...
        & results.peakAccelerationErrorMps2<=1.5*base.peakAccelerationErrorMps2 ...
        & results.peakVelocityErrorMps<=1.5*base.peakVelocityErrorMps ...
        & ~results.outsideRateEnvelope;
    indices=find(eligible);[~,whichBest]=min(results.accelerationRmseMps2(indices));
    selectedIndex=indices(whichBest);selected=results.candidate(selectedIndex);
    report=struct('selectedCandidate',selected,'cfg',cfg,'selectedDesign',certificates{selected}, ...
        'screen',table2struct(screen),'metrics',table2struct(results), ...
        'selectionRule',"Minimize noisy acceleration RMSE with position <=1.05 baseline, velocity <=baseline, both startup derivative peaks <=1.5 baseline; same admitted coefficient box", ...
        'scope',"Finite candidate comparison on variable-speed gentle turns; unchanged continuous delayed observer, exact lateral inputs, no upstream modules");
    % Independent noise phases not used for gain selection.
    validationRows={};validationRuns={};
    for phase=[1,2,3]
        s=makeScenario(cfg,1,phase);
        for k=[1,selected]
            runCfg=cfg;runCfg.observer.initialState=s.initialState;
            e=runImprovedVehicleObserver(s.data,struct(),certificates{k},runCfg, ...
                LateralInputs=s.lateral,InitialHistory=s.history);
            row=metrics(e,s,k);row.phase=phase;validationRows{end+1}=row; %#ok<AGROW>
            validationRuns{end+1}=e; %#ok<AGROW>
        end
    end
    validation=struct2table(vertcat(validationRows{:}));
    writetable(validation,fullfile(outputFolder,'validation_phases.csv'));report.validation=table2struct(validation);
    refinedCfg=cfg;refinedCfg.observer.initialState=scenarios{2}.initialState;
    refinedCfg.measurement.maximumIntegrationStep=.0025;
    refined=runImprovedVehicleObserver(scenarios{2}.data,struct(),certificates{selected},refinedCfg, ...
        LateralInputs=scenarios{2}.lateral,InitialHistory=scenarios{2}.history);
    report.maximumStateRefinementDifference=max(abs(refined.z-runs{selectedIndex}.z),[],1);
    report.matlabVersion=version;
    save(fullfile(outputFolder,'traces.mat'),'report','scenarios','runs','certificates','refined','validationRuns','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    plotComparison(scenarios{2},runs{find(results.candidate==1 & results.noisy,1)},runs{selectedIndex},outputFolder);
    disp(results);disp(validation);fprintf('Selected candidate %d\n',selected);
end

function [design,info]=synthesizeCandidate(design,cfg)
    data=buildImprovedObserverCertificateData(cfg);theta=cfg.observer.theta;delay=cfg.measurement.fixedLidarDelay;
    A0=theta*data.A;Ad=-theta*design.K*data.C;
    uncertainty=data.modelPerturbation+norm(design.N,2)*data.outputBound4 ...
        +theta*norm(design.K,2)*norm(data.C,2)*(1-cfg.lidar.minimumPoseWeight);
    yalmip('clear');P=sdpvar(7,7,'symmetric');Q=sdpvar(7,7,'symmetric');R=sdpvar(7,7,'symmetric');
    g=sdpvar(1);margin=sdpvar(1);pb=sdpvar(1);rb=sdpvar(1);
    block=continuousObserverDelayLmi(A0,Ad,P,Q,R,g,delay,design.rate);
    constraints=[P>=1e-5*eye(7),Q>=1e-5*eye(7),R>=1e-5*eye(7),P<=pb*eye(7), ...
        R<=rb*eye(7),trace(P)==7,trace(Q)+trace(R)<=1000,g>=1e-5,g<=1000,margin>=1e-6, ...
        block<=-(margin+2*uncertainty*(pb+delay*rb))*eye(28)];
    result=optimize(constraints,-margin,sdpsettings('solver','sedumi','verbose',0));
    info=string(result.info);
    if result.problem~=0,design=[];return;end
    design.P=double(P);design.Q=double(Q);design.R=double(R);design.g=double(g);
    for name=["P","Q","R"],design.(name)=(design.(name)+design.(name).')/2;end
    design.verification=verifyImprovedObserverDesign(design,cfg);design.certified=design.verification.certified;
    if ~design.certified,info="Independent verification failed";design=[];end
end

function s=makeScenario(cfg,noisy,phase)
    t=(0:.01:40).';z=truthAt(t);beta=.0002*sin(.3*t);betaDot=.00006*cos(.3*t);
    speed=8+1.2*sin(.35*t);q=.001+.0005*sin(.4*t);yaw=z(:,7);r=q-betaDot;
    vx=speed.*cos(beta);vy=speed.*sin(beta);
    ax=z(:,3).*cos(yaw)+z(:,6).*sin(yaw);ay=-z(:,3).*sin(yaw)+z(:,6).*cos(yaw);
    data.highRate=struct('time',t,'steeringAngle',zeros(size(t)), ...
        'longitudinalSpeed',vx+noisy*.02*sin(1.1*t+phase), ...
        'longitudinalAcceleration',ax+noisy*.02*sin(1.7*t+phase), ...
        'lateralAcceleration',ay+noisy*.02*cos(1.4*t+phase), ...
        'yawRate',r+noisy*.0002*sin(.6*t+phase));
    lateral=struct('time',t,'lateralVelocity',vy,'sideSlipAngle',beta,'sideSlipAngleRate',betaDot);
    data.lidar=struct('delay',cfg.measurement.fixedLidarDelay,'headingConvention',"unwrapped", ...
        'evaluate',@(time) poseAt(time,cfg.measurement.fixedLidarDelay,noisy,phase));
    error=[1;.3;.1;-1;-.2;-.1;deg2rad(10)];
    s=struct('time',t,'truth',z,'data',data,'lateral',lateral,'initialState',z(1,:).'+error, ...
        'history',@(time) truthAt(time).'+error,'noisy',logical(noisy));
end

function z=truthAt(t)
    t=t(:);v=8+1.2*sin(.35*t);vd=.42*cos(.35*t);
    course=.2+.001*t+.00125*(1-cos(.4*t));q=.001+.0005*sin(.4*t);beta=.0002*sin(.3*t);
    % Independent deterministic Gauss-Legendre quadrature at each query time.
    persistent nodes weights
    if isempty(nodes)
        k=(1:39).';off=k./sqrt(4*k.^2-1);[V,D]=eig(diag(off,1)+diag(off,-1));
        nodes=diag(D).';weights=2*(V(1,:).^2);
    end
    u=t/2.*(nodes+1);vu=8+1.2*sin(.35*u);cu=.2+.001*u+.00125*(1-cos(.4*u));
    x=t/2.*sum(vu.*cos(cu).*weights,2);y=t/2.*sum(vu.*sin(cu).*weights,2);
    z=[x,v.*cos(course),vd.*cos(course)-v.*q.*sin(course), ...
        y,v.*sin(course),vd.*sin(course)+v.*q.*cos(course),course-beta];
end

function y=poseAt(t,delay,noisy,phase)
    z=truthAt(t-delay).';
    y=struct('pose',z([1,4,7])+noisy*[.01*sin(1.3*t+phase);.01*cos(.9*t+phase); ...
        deg2rad(.1)*sin(.7*t+phase)],'information',1e6*eye(3));
end

function row=metrics(e,s,candidate)
    d=e.z-s.truth;d(:,7)=atan2(sin(d(:,7)),cos(d(:,7)));mask=s.time>=20;
    pe=vecnorm(d(:,[1,4]),2,2);ve=vecnorm(d(:,[2,5]),2,2);ae=vecnorm(d(:,[3,6]),2,2);
    row=struct('candidate',candidate,'noisy',s.noisy,'positionRmseM',rms(pe(mask)), ...
        'velocityRmseMps',rms(ve(mask)),'accelerationRmseMps2',rms(ae(mask)), ...
        'headingRmseDeg',rad2deg(rms(d(mask,7))),'peakVelocityErrorMps',max(ve), ...
        'peakAccelerationErrorMps2',max(ae),'certificateVerified',e.observer.certificateVerified, ...
        'outsideRateEnvelope',e.diagnostics.anyStageOutsideTrackRateEnvelope);
end

function plotComparison(s,baseline,tuned,folder)
    fig=figure('Visible','off','Color','w','Position',[100,100,1100,720]);cleanup=onCleanup(@() close(fig));
    tiledlayout(fig,2,2);columns={[1,4],[2,5],[3,6],7};labels=["Position (m)","Velocity (m/s)","Acceleration (m/s^2)","Yaw (deg)"];
    for k=1:4
        nexttile;hold on;
        for e={baseline,tuned}
            d=e{1}.z-s.truth;values=vecnorm(d(:,columns{k}),2,2);if k==4,values=rad2deg(values);end
            plot(s.time,values,'LineWidth',1.3);
        end
        grid on;xlabel('Time (s)');ylabel('Error: '+labels(k));legend('Reference','Tuned');
    end
    exportgraphics(fig,fullfile(folder,'comparison.png'),'Resolution',150);
    exportgraphics(fig,fullfile(folder,'comparison.pdf'),'ContentType','vector');
end
