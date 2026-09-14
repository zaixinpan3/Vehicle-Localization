function report=validateMncavObserverParameters(outputFolder,options)
% validateMncavObserverParameters Check LiDAR gains on source-qualified MnCAV plants.
% Global observer only: exact lateral interface isolates gain behavior. The
% physical nominal plant varies independently in inertia and axle stiffness;
% this is not a calibrated vehicle or a lateral-estimator robustness test.
    arguments
        outputFolder (1,1) string="output/mncav_parameter_validation_20260914"
        options.SteeringScale (1,1) double {mustBeFinite,mustBePositive}=1
        options.Profiles (1,:) string=["reference","tracking"]
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    parameters=mncavVehicleConfig();baseCfg=lateralObserverConfig("mncav");
    changes=[1,1,1;.7,1,1;1.3,1,1;1,.7,1;1,1.3,1;1,1,.7;1,1,1.3];
    names=["nominal","inertia_07","inertia_13","front_07","front_13","rear_07","rear_13"];
    scenarios=cell(size(changes,1),1);runs=cell(size(changes,1),numel(options.Profiles));rows={};
    for k=1:size(changes,1)
        vehicle=baseCfg.vehicle;vehicle.yawInertia=vehicle.yawInertia*changes(k,1);
        vehicle.frontCorneringStiffness=vehicle.frontCorneringStiffness*changes(k,2);
        vehicle.rearCorneringStiffness=vehicle.rearCorneringStiffness*changes(k,3);
        s=makeScenario(vehicle,parameters.steeringRatio,options.SteeringScale);scenarios{k}=s;
        for j=1:numel(options.Profiles)
            profiles=options.Profiles;cfg=improvedObserverConfig("lidar",profiles(j));
            cfg.observer.initialState=s.initialState;design=improvedObserverReferenceDesign(cfg);
            e=runImprovedVehicleObserver(s.data,struct(),design,cfg, ...
                LateralInputs=s.lateral,InitialHistory=s.history);runs{k,j}=e;
            d=e.z-s.truth;mask=s.time>=20;
            rows{end+1}=struct('plant',names(k),'profile',profiles(j), ...
                'positionRmseM',rms(vecnorm(d(mask,[1,4]),2,2)), ...
                'velocityRmseMps',rms(vecnorm(d(mask,[2,5]),2,2)), ...
                'accelerationRmseMps2',rms(vecnorm(d(mask,[3,6]),2,2)), ...
                'headingRmseDeg',rad2deg(rms(d(mask,7))), ...
                'peakVelocityErrorMps',max(vecnorm(d(:,[2,5]),2,2)), ...
                'peakAccelerationErrorMps2',max(vecnorm(d(:,[3,6]),2,2)), ...
                'maximumTrueCourseRate',max(abs(s.trueCourseRate)), ...
                'outsideRateEnvelope',e.diagnostics.anyStageOutsideTrackRateEnvelope, ...
                'matrixCertificatePassed',e.observer.certificateVerified); %#ok<AGROW>
        end
        fprintf('MnCAV nominal sensitivity %s complete.\n',names(k));
    end
    metrics=struct2table(vertcat(rows{:}));writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    report=struct('vehicleParameters',parameters,'plantFactors',changes,'experimentOptions',options, ...
        'metrics',table2struct(metrics),'scope',"MnCAV stock-based nominal bicycle plants; global observer with exact lateral inputs, not calibrated full-cascade validation", ...
        'matlabVersion',version);
    save(fullfile(outputFolder,'traces.mat'),'report','scenarios','runs','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(metrics);
end

function s=makeScenario(vehicle,steeringRatio,steeringScale)
    t=(0:.01:40).';model=lateralBicycleModel(vehicle);start=-.15;
    [vx,~,steering]=inputs(start,steeringRatio,steeringScale);
    A=evaluateLateralModel(model,[vx;1/vx]);body0=-A\(model.B*steering);
    sol=ode45(@(time,x) plant(time,x,model,steeringRatio,steeringScale),[start,40], ...
        [body0;.2;0;0],odeset('RelTol',1e-10,'AbsTol',1e-12));
    [z,high,lateral,q]=sampleTruth(t,sol,model,steeringRatio,steeringScale);
    % Identical bounded synthetic sensor errors for all profiles and plants.
    high.longitudinalSpeed=high.longitudinalSpeed+.02*sin(1.1*t);
    high.longitudinalAcceleration=high.longitudinalAcceleration+.02*sin(1.7*t);
    high.lateralAcceleration=high.lateralAcceleration+.02*cos(1.4*t);
    high.yawRate=high.yawRate+.0002*sin(.6*t);
    error=[1;.3;.1;-1;-.2;-.1;deg2rad(10)];
    data=struct('highRate',high,'lidar',struct('delay',.15,'headingConvention',"unwrapped", ...
        'evaluate',@(time) pose(time,sol,model,steeringRatio,steeringScale)));
    s=struct('time',t,'truth',z,'vehicle',vehicle,'steeringRatio',steeringRatio, ...
        'trueCourseRate',q,'data',data,'lateral',lateral,'initialState',z(1,:).'+error, ...
        'history',@(time) history(time,sol,model,steeringRatio,steeringScale,error));
end

function dx=plant(t,x,model,ratio,steeringScale)
    [vx,~,delta]=inputs(t,ratio,steeringScale);A=evaluateLateralModel(model,[vx;1/vx]);
    body=A*x(1:2)+model.B*delta;psi=x(3);
    dx=[body;x(2);vx*cos(psi)-x(1)*sin(psi);vx*sin(psi)+x(1)*cos(psi)];
end

function [z,high,lateral,q]=sampleTruth(t,sol,model,ratio,steeringScale)
    t=t(:);x=deval(sol,t).';[vx,vxdot,delta]=inputs(t,ratio,steeringScale);vy=x(:,1);r=x(:,2);psi=x(:,3);
    bodyDerivative=zeros(numel(t),2);
    for k=1:numel(t)
        A=evaluateLateralModel(model,[vx(k);1/vx(k)]);
        bodyDerivative(k,:)=(A*x(k,1:2).'+model.B*delta(k)).';
    end
    ax=vxdot-vy.*r;ay=bodyDerivative(:,1)+vx.*r;
    beta=atan2(vy,vx);betaDot=(vx.*bodyDerivative(:,1)-vy.*vxdot)./(vx.^2+vy.^2);
    Vx=vx.*cos(psi)-vy.*sin(psi);Vy=vx.*sin(psi)+vy.*cos(psi);
    Ax=ax.*cos(psi)-ay.*sin(psi);Ay=ax.*sin(psi)+ay.*cos(psi);
    z=[x(:,4),Vx,Ax,x(:,5),Vy,Ay,psi];q=r+betaDot;
    high=struct('time',t,'steeringAngle',delta,'longitudinalSpeed',vx, ...
        'longitudinalAcceleration',ax,'lateralAcceleration',ay,'yawRate',r);
    lateral=struct('time',t,'lateralVelocity',vy,'sideSlipAngle',beta,'sideSlipAngleRate',betaDot);
end

function [vx,vxdot,delta]=inputs(t,ratio,steeringScale)
    vx=8+1.2*sin(.35*t);vxdot=.42*cos(.35*t);
    % Steering-wheel radians converted to road-wheel radians using stock ratio.
    delta=steeringScale*(.006+.002*sin(.4*t))/ratio;
end

function y=pose(t,sol,model,ratio,steeringScale)
    z=sampleTruth(t-.15,sol,model,ratio,steeringScale).';
    y=struct('pose',z([1,4,7])+[.01*sin(1.3*t);.01*cos(.9*t);deg2rad(.1)*sin(.7*t)], ...
        'information',1e6*eye(3));
end

function z=history(t,sol,model,ratio,steeringScale,error)
    z=sampleTruth(t,sol,model,ratio,steeringScale).'+error;
end
