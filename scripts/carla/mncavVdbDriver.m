function mncavVdbDriver(block)
% mncavVdbDriver Sampled path-following test driver for the external VDB plant.
% Driver uses plant pose for scenario generation only; localization sees sensors.
% Route columns 5 and 6 (optional) carry the no-stop cruise profile and the
% dwell time of timed stops: the driver brakes to the next unserved stop on
% the with-stop profile, holds the brake for its dwell time once stopped
% within a waypoint of it, then follows the cruise profile again.
    block.NumDialogPrms=2;block.NumInputPorts=1;block.NumOutputPorts=2;
    block.SetPreCompInpPortInfoToDynamic;block.SetPreCompOutPortInfoToDynamic;
    block.InputPort(1).Dimensions=4;block.InputPort(1).DirectFeedthrough=false;
    block.OutputPort(1).Dimensions=12;block.OutputPort(2).Dimensions=3;
    cfg=block.DialogPrm(2).Data;block.SampleTimes=[cfg.simulation.sampleTimeSeconds 0];
    block.SimStateCompliance='DefaultSimState';
    block.RegBlockMethod('PostPropagationSetup',@setupWork);
    block.RegBlockMethod('InitializeConditions',@initialize);
    block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function setupWork(b)
    b.NumDworks=4;dims=[12 3 1 2];names={'command','diagnostics','steer','dwell'};
    for k=1:4,b.Dwork(k).Name=names{k};b.Dwork(k).Dimensions=dims(k);b.Dwork(k).DatatypeID=0;b.Dwork(k).Complexity='Real';b.Dwork(k).UsedAsDiscState=true;end
end
function initialize(b)
    b.Dwork(1).Data=zeros(12,1);b.Dwork(2).Data=[1;0;0];b.Dwork(3).Data=0;b.Dwork(4).Data=[0;-1];
end
function outputs(b)
    b.OutputPort(1).Data=b.Dwork(1).Data;b.OutputPort(2).Data=b.Dwork(2).Data;
end
function update(b)
    cfg=b.DialogPrm(2).Data;route=b.DialogPrm(1).Data;u=b.InputPort(1).Data;
    last=max(1,round(b.Dwork(2).Data(1)));ids=last:min(size(route,1),last+50);
    [err,j]=min(sum((route(ids,1:2)-u(1:2).').^2,2));idx=ids(j);
    look=cfg.driver.lookaheadBaseM+cfg.driver.lookaheadTimeSeconds*max(0,u(4));
    target=find(route(:,3)>=route(idx,3)+look,1);if isempty(target),target=size(route,1);end
    d=route(target,1:2)-u(1:2).';alpha=atan2(d(2),d(1))-u(3);
    L=cfg.vehicle.lf+cfg.vehicle.lr;
    desired=atan2(2*L*sin(alpha),max(norm(d),1));
    desired=max(-cfg.driver.maximumRoadWheelAngleRad,min(cfg.driver.maximumRoadWheelAngleRad,desired));
    dt=cfg.simulation.sampleTimeSeconds;old=b.Dwork(3).Data;
    change=(desired-old)*(-expm1(-dt/cfg.driver.steeringTimeConstantSeconds));
    lim=cfg.driver.maximumSteeringRateRadps*dt;delta=old+max(-lim,min(lim,change));
    stops=[];if size(route,2)>=6,stops=find(route(:,6)>0);end
    served=b.Dwork(4).Data(1);dwellStart=b.Dwork(4).Data(2);
    nextStop=Inf;if served<numel(stops),nextStop=stops(served+1);end
    lastServed=0;if served>0,lastServed=stops(served);end
    % Served stops are behind the vehicle on the cruise profile; the next
    % unserved stop keeps its zero on the with-stop profile.
    lo=max(idx,lastServed+1);vref=min(route(lo:target,4));
    if lo>idx,vref=min(vref,min(route(idx:lo-1,5)));end
    arrived=isfinite(nextStop) && route(nextStop,3)-route(idx,3)<=cfg.driver.lookaheadBaseM+2 && abs(u(4))<.1;
    if arrived && b.CurrentTime>=cfg.simulation.settleSeconds
        if dwellStart<0,dwellStart=b.CurrentTime;end
        if b.CurrentTime-dwellStart>=route(nextStop,6)
            served=served+1;dwellStart=-1;lo=max(idx,nextStop+1);
            vref=min(route(lo:max(lo,target),4));
        else
            vref=0;
        end
    end
    b.Dwork(4).Data=[served;dwellStart];
    % Tracking corrections can turn more sharply than the nominal path. Slow
    % for both the requested and lagged steering curvature before accelerating.
    steeringCurvature=max(abs(tan([desired,delta])))/L;
    vref=min(vref,sqrt(cfg.simulation.lateralAccelerationMps2/max(steeringCurvature,.001)));
    if b.CurrentTime<cfg.simulation.settleSeconds,vref=0;delta=0;end
    if route(end,3)-route(idx,3)<3,vref=0;end
    acc=max(-cfg.simulation.maximumBrakeMps2,min(cfg.simulation.longitudinalAccelerationMps2,cfg.driver.speedGain*(vref-u(4))));
    force=cfg.vehicle.mass*acc+cfg.vehicle.mass*9.81*.012+.5*1.225*cfg.body.dragCoefficient*cfg.body.frontalAreaM2*u(4)^2;
    if vref==0 && abs(u(4))<.1,force=-1000;end
    % SAE positive steering turns right. The FL wheel has negative body y.
    half=cfg.stock.frontTrackM/2;
    angles=[atan2(L*tan(delta),L+half*tan(delta));atan2(L*tan(delta),L-half*tan(delta));0;0];
    radius=cfg.wheel.unloadedRadiusM*.975;
    torque=[max(force,0)*radius/2;max(force,0)*radius/2;0;0];
    % Disc block parameters: mu=.2, bore=.05 m, mean radius=.177 m, two pads.
    pressure=max(-force,0)*radius/4/(.2*pi*.05^2*.177*2/4);
    b.Dwork(1).Data=[angles;torque;ones(4,1)*pressure];
    b.Dwork(2).Data=[idx;sqrt(err);vref];b.Dwork(3).Data=delta;
end
