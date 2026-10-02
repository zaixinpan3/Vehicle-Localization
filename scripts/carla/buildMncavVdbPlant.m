function [mdl,report]=buildMncavVdbPlant(outputFolder,routeFile)
% buildMncavVdbPlant Instantiate the installed 14DOF template, never vendor it.
% Generated SLX files and copied MathWorks assets belong only in outputFolder.
    root=setupVehicleLocalization();cfg=mncavVdbConfig();
    outputFolder=char(outputFolder);if ~isfolder(outputFolder),mkdir(outputFolder);end
    outputFolder=char(java.io.File(outputFolder).getCanonicalPath());
    vendor=fullfile(outputFolder,'vendor');if ~isfolder(vendor),mkdir(vendor);end
    for name=["PassVeh14DOF","VehicleConfig"]
        unzip(fullfile(matlabroot,'toolbox','vdynblks','vdynsolution',name+".zip"),vendor);
    end
    addpath(genpath(vendor));addpath(fullfile(root,'scripts','carla'));
    plant='Mncav14DOF';if bdIsLoaded(plant),close_system(plant,0);end
    load_system('PassVeh14DOF');save_system('PassVeh14DOF',fullfile(vendor,'Plant',[plant '.slx']));load_system(plant);
    mw=get_param(plant,'ModelWorkspace');veh=getVariable(mw,'VEH');
    v=cfg.vehicle;total=v.mass;unsprung=cfg.wheel.unsprungMassKg;
    % The 6DOF block uses the full vehicle mass; wheel masses set tire vertical inertia.
    veh.Mass=total;veh.SprungMass=total-sum(unsprung);
    veh.FrontAxlePositionfromCG=v.lf;veh.RearAxlePositionfromCG=v.lr;veh.WheelBase=v.lf+v.lr;
    veh.HeightCG=cfg.body.cgHeightM;veh.TrackWidth=mean([cfg.stock.frontTrackM,cfg.stock.rearTrackM]);
    veh.FrontalArea=cfg.body.frontalAreaM2;veh.DragCoefficient=cfg.body.dragCoefficient;
    veh.YawMomentInertia=v.yawInertia;veh.RollMomentInertia=cfg.body.rollInertiaKgM2;veh.PitchMomentInertia=cfg.body.pitchInertiaKgM2;
    veh.SteeringRatio=cfg.steeringRatio;
    route=readtable(routeFile);xy=[route.x_carla,route.y_carla];s=[0;cumsum(vecnorm(diff(xy),2,2))];
    curvature=abs(route.curvature_per_m);speed=min(cfg.simulation.maximumSpeedMps,sqrt(cfg.simulation.lateralAccelerationMps2./max(curvature,.001)));
    speed(end)=0;
    for k=numel(speed)-1:-1:1,speed(k)=min(speed(k),sqrt(speed(k+1)^2+2*cfg.simulation.maximumBrakeMps2*(s(k+1)-s(k))));end
    path=[xy,s,speed];
    veh.InitialLongPosition=xy(1,1);veh.InitialLatPosition=xy(1,2);veh.InitialVertPosition=0;
    veh.InitialYawAngle=deg2rad(route.yaw_carla_deg(1));veh.InitialLongVel=0;
    assignin(mw,'VEH',veh);
    set_param([plant '/Suspension'],'LabelModeActiveChoice','6');
    susp=[plant '/Suspension/Kinematics and Compliance Independent Suspension/Independent K and C Suspension'];
    set_param(susp,'IdealSuspEn','off');
    normal=total*9.81*[v.lr v.lr v.lf v.lf]/(2*(v.lf+v.lr));
    sprungNormal=normal;
    assignin(mw,'NrmlWhlFrcOff',sprungNormal);assignin(mw,'NrmlWhlRates',cfg.suspension.springRateNpm/1000);
    assignin(mw,'ShckFrcVsCompRate',cfg.suspension.damperRateNspm/1000);
    assignin(mw,'StatLdWhlR',ones(1,4)*(cfg.wheel.unloadedRadiusM-.025));
    % Default template K&C tables are retained as explicit unmeasured priors.
    tireRoot=[plant '/Wheels and Tires/VDBS/Tires'];set_param(tireRoot,'LabelModeActiveChoice','0');
    set_param([plant '/Wheels and Tires/VDBS/Pressure'],'const',num2str(cfg.wheel.pressurePa));
    tireBlocks=cell(4,1);
    for k=1:4
        suffix='';if k>1,suffix=num2str(k-1);end
        tireBlocks{k}=[tireRoot '/MF Tires/Combined Slip Wheel 2DOF' suffix];
    end
    stiffness=[v.frontCorneringStiffness v.frontCorneringStiffness v.rearCorneringStiffness v.rearCorneringStiffness]/2;
    kReported=zeros(1,4);
    for k=1:4
        b=tireBlocks{k};f0=normal(k);p2=str2double(get_param(b,'PKY2'));p4=str2double(get_param(b,'PKY4'));
        % MF6.2 zero-camber, nominal-pressure/load small-slip derivative.
        p1=-stiffness(k)/(f0*sin(p4*atan(1/p2)));kReported(k)=abs(p1*f0*sin(p4*atan(1/p2)));
        set_param(b,'FNOMIN',num2str(f0,17),'PKY1',num2str(p1,17), ...
            'UNLOADED_RADIUS',num2str(cfg.wheel.unloadedRadiusM,17),'WIDTH',num2str(cfg.wheel.widthM), ...
            'RIM_RADIUS',num2str(cfg.wheel.rimRadiusM),'ASPECT_RATIO',num2str(cfg.wheel.aspectRatio), ...
            'MASS',num2str(unsprung(k)),'IYY',num2str(cfg.wheel.spinInertiaKgM2), ...
            'VERTICAL_STIFFNESS',num2str(cfg.wheel.verticalStiffnessNpm), ...
            'VERTICAL_DAMPING',num2str(cfg.wheel.verticalDampingNspm), ...
            'NOMPRES',num2str(cfg.wheel.pressurePa),'zo','0','zdoto','0','omegao','0');
    end
    body=[plant '/Vehicle/Vehicle Body 6DOF'];
    set_param(body,'w',mat2str([cfg.stock.frontTrackM cfg.stock.rearTrackM],17), ...
        'Cd',num2str(cfg.body.dragCoefficient),'Tair','293.15','Cl','0','Cpm','0');
    save_system(plant);
    mdl='MncavTown10Plant';if bdIsLoaded(mdl),close_system(mdl,0);end
    new_system(mdl);workspace=get_param(mdl,'ModelWorkspace');assignin(workspace,'vdbConfig',cfg);assignin(workspace,'vdbRoute',path);
    add_block('simulink/Ports & Subsystems/Model',[mdl '/Plant'],'ModelName',plant,'SimulationMode','Normal');
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Driver'], ...
        'FunctionName','mncavVdbDriver','Parameters','vdbRoute,vdbConfig');
    add_block('simulink/Signal Routing/Demux',[mdl '/Commands'],'Outputs','[4 4 4]');
    add_line(mdl,'Driver/1','Commands/1');for k=1:3,add_line(mdl,['Commands/' num2str(k)],['Plant/' num2str(k)]);end
    vals={'zeros(3,1)','zeros(4,1)','ones(4,1)','repmat(eye(3),1,1,4)'};
    for k=1:4
        n=['Environment' num2str(k)];add_block('simulink/Sources/Constant',[mdl '/' n],'Value',vals{k});
        add_line(mdl,[n '/1'],['Plant/' num2str(k+3)]);
    end
    add_block('simulink/Signal Routing/Bus Selector',[mdl '/DriverPose'], ...
        'OutputSignals','InertFrm.Cg.Disp.X,InertFrm.Cg.Disp.Y,InertFrm.Cg.Ang.psi,BdyFrm.Cg.Vel.xdot');
    add_line(mdl,'Plant/1','DriverPose/1');add_block('simulink/Signal Routing/Mux',[mdl '/State'],'Inputs','4');
    for k=1:4,add_line(mdl,['DriverPose/' num2str(k)],['State/' num2str(k)]);end
    add_line(mdl,'State/1','Driver/1');
    for k=1:4
        names={'vehicle','wheels','commands','driver'};sources={'Plant/1','Plant/2','Driver/1','Driver/2'};
        add_block('simulink/Sinks/To Workspace',[mdl '/' names{k}],'VariableName',names{k}, ...
            'SaveFormat','Timeseries','SampleTime',num2str(cfg.simulation.sampleTimeSeconds));add_line(mdl,sources{k},[names{k} '/1']);
    end
    set_param(mdl,'Solver','ode23t','MaxStep',num2str(cfg.simulation.maximumStepSeconds), ...
        'RelTol','1e-5','AbsTol','1e-6','StopTime','180','ReturnWorkspaceOutputs','on');
    save_system(mdl,fullfile(outputFolder,[mdl '.slx']));
    report=struct('config',cfg,'body',veh,'staticWheelLoadsN',normal,'sprungWheelLoadsN',sprungNormal, ...
        'nominalCorneringStiffnessPerWheel',kReported,'routeFile',string(routeFile),'routeLengthM',s(end),'routePointCount',size(path,1), ...
        'matlabVersion',version,'plantModel',plant,'model',mdl);
    fid=fopen(fullfile(outputFolder,'plant_parameters.json'),'w');cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
