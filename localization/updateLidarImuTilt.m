function [rotation,state,diagnostic]=updateLidarImuTilt(sample,speed,state,cfg)
% updateLidarImuTilt One causal six-axis IMU update; no pose/reference input.
% specificForce and angularVelocity are columns in stored point axes. time
% is native IMU time. speed is a currently available wheel measurement.
% Only stationary measurements initialize/update gyro bias. The gravity
% direction is propagated on the unit sphere and corrected by specific force
% minus the planar wheel-acceleration estimate. No yaw is observed/injected.
    if nargin<4,cfg=lidarImuTiltConfig();end
    f=sample.specificForce(:);w=sample.angularVelocity(:);t=sample.time;
    assert(numel(f)==3 && numel(w)==3 && all(isfinite([t;f;w;speed])), ...
        'VehicleLocalization:InvalidTiltImu','Require finite six-axis IMU and wheel speed.');
    stationary=abs(speed)<cfg.stationarySpeedMps && norm(w)<cfg.stationaryGyroRadps;
    if isempty(state)
        assert(stationary && norm(f)>cfg.gravityMps2/2, ...
            'VehicleLocalization:TiltAlignmentRequired','Start with stationary six-axis IMU and measured wheel speed.');
        state=struct('time',t,'up',f/norm(f),'gyroBias',w,'biasCount',1, ...
            'stationarySeconds',0,'speed',speed,'initialized',false);
        acceleration=0;
    else
        dt=t-state.time;
        assert(dt>0 && dt<=cfg.maximumImuStepSeconds, ...
            'VehicleLocalization:TiltImuGap','IMU timestamps must increase without an excessive gap.');
        filtered=state.speed+(1-exp(-dt/cfg.speedTimeSeconds))*(speed-state.speed);
        acceleration=(filtered-state.speed)/dt;state.speed=filtered;
        if stationary
            state.biasCount=state.biasCount+1;
            state.gyroBias=state.gyroBias+(w-state.gyroBias)/state.biasCount;
            state.stationarySeconds=state.stationarySeconds+dt;
        end
        angular=w-state.gyroBias;angle=norm(angular)*dt;
        if angle>0
            axis=-angular/norm(angular);u=state.up;
            state.up=u*cos(angle)+cross(axis,u)*sin(angle)+axis*dot(axis,u)*(1-cos(angle));
        end
        state.time=t;
        state.initialized=state.stationarySeconds>=cfg.minimumAlignmentSeconds;
    end
    angular=w-state.gyroBias;
    gravity=f-[acceleration;state.speed*angular(3);0];
    accepted=abs(norm(gravity)-cfg.gravityMps2)<=cfg.maximumGravityNormErrorMps2;
    if accepted
        measured=gravity/norm(gravity);
        if stationary && ~state.initialized
            alpha=1/state.biasCount;
        elseif exist('dt','var')
            alpha=1-exp(-dt/cfg.correctionTimeSeconds);
        else
            alpha=0;
        end
        state.up=state.up+alpha*(measured-state.up);state.up=state.up/norm(state.up);
    end
    roll=atan2(state.up(2),state.up(3));pitch=atan2(-state.up(1),hypot(state.up(2),state.up(3)));
    cr=cos(roll);sr=sin(roll);cp=cos(pitch);sp=sin(pitch);
    rotation=[cp,sp*sr,sp*cr;0,cr,-sr;-sp,cp*sr,cp*cr];
    diagnostic=struct('roll',roll,'pitch',pitch,'aligned',state.initialized, ...
        'gravityAccepted',accepted,'gravityNormMps2',norm(gravity),'gyroBias',state.gyroBias);
end
