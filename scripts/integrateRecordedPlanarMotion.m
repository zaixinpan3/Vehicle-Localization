function [poses,segment] = integrateRecordedPlanarMotion(time, bodyTwist, queryTime, motionValid)
% integrateRecordedPlanarMotion Integrate recorded body [vx,vy,yawRate].
% Inputs use seconds, meters/second and radians/second. Previous samples are
% held causally over each interval. Exact constant-twist SE(2) increments are
% evaluated at every sensor/query boundary. Output is relative to query 1.
% Optional motionValid withdraws the entire twist. Each uninterrupted valid
% segment has its own origin and positive ID; unavailable poses are NaN with
% ID zero. Consumers must never transport scans across different segment IDs.
    time=double(time(:)); queryTime=double(queryTime(:)); bodyTwist=double(bodyTwist);
    assert(numel(time)>=2 && all(isfinite(time)) && all(diff(time)>0), ...
        'VehicleLocalization:InvalidMotionTime','Motion times must increase.');
    assert(isequal(size(bodyTwist),[numel(time),3]) && all(isfinite(bodyTwist),'all'), ...
        'VehicleLocalization:InvalidMotionInput','Expected finite [vx,vy,yawRate].');
    if nargin<4,motionValid=true(numel(time),1);end
    assert(numel(motionValid)==numel(time) && all(ismember(motionValid(:),[0,1])), ...
        'VehicleLocalization:InvalidMotionValidity','Supply one logical validity per motion sample.');
    motionValid=logical(motionValid(:));
    nativeSegment=cumsum(motionValid & ~[false;motionValid(1:end-1)]).*motionValid;
    assert(~isempty(queryTime) && all(isfinite(queryTime)) && all(diff(queryTime)>0) && ...
        queryTime(1)>=time(1) && queryTime(end)<=time(end), ...
        'VehicleLocalization:MotionCoverage','Query times must lie inside the motion stream.');
    origin=time(1); time=time-origin; queryTime=queryTime-origin;
    grid=unique([time(time>=queryTime(1)&time<=queryTime(end));queryTime]);
    index=interp1(time,(1:numel(time)).',grid,'previous');
    gridSegment=nativeSegment(index);
    path=zeros(numel(grid),3);
    for k=1:numel(grid)-1
        if gridSegment(k)==0 || gridSegment(k+1)~=gridSegment(k)
            path(k+1,:)=0;
            continue;
        end
        dt=grid(k+1)-grid(k); u=bodyTwist(index(k),:); angle=u(3)*dt;
        half=angle/2; gain=1;
        if abs(half)>1e-8, gain=sin(half)/half; end
        yaw=path(k,3)+half;
        r=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        path(k+1,:)=[path(k,1:2)+dt*gain*u(1:2)*r.',path(k,3)+angle];
    end
    [~,index]=ismember(queryTime,grid);
    poses=path(index,:);
    segment=gridSegment(index);
    poses(segment==0,:)=NaN;
end
