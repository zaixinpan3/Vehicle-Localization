function cfg=mncavFullObserverConfig(options)
% Current MnCAV intrinsic predictor; IMU is confined to invariant innovations.
% Autonomous translational prediction uses a C2 extension below its speed cutoff.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-autonomous-motion-v17",'maximumTrackAngleRate',.4,'initialState',[]);
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('gainInformationScale',16,'outputPoint', ...
        jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json'))));
    cfg.lidar=struct('poseScales',[1;1;.1],'gainInformationScale',64);
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'maximumPositionHeadingCoupling',.06);
    z=struct('sigma',4,'headingLengthM',10, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',.23, ...
            'velocityMps',.256,'accelerationMps2',.233,'jerkMps3',.05), ...
        'nominalGnssWeight',.5, ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    z.provenance=struct( ...
        'noise',"Inherited declared sensor priors: conditional registration moments on the first 60 s of the 2026-10-02 study; BESTPOS reported sigma on 12-09-31; adopted lateral identification and independently corrected IMU on 12-11-24; these are design priors, not an overlap covariance certificate", ...
        'headingLengthM',"Heading error weighted by a 10 m landmark range", ...
        'normalization',"lambda = 1 and P >= I");
    z.theta=options.Theta;z.yawAccelerationRadps2=cfg.maximumTrackAngleRate/2;
    z.balancedPoseMetric=false;
    z.predictionMinimumSpeedMps=.5;
    z.noiseObjective="normalizedFullState";
    % One simultaneous SDP in the fixed-shear metric; no outer tuning loop.
    z.motionResponse=struct('enabled',true,'timeConstantS',.5);
    z.velocityAvailable=true;z.accelerationAvailable=true;
    z.fusion=struct('minimumCourseSpeed',10,'lidarOperatorMismatch',.001);
    z.provenance.speedOutput="Physical squared-speed output with bounded gain factor Vmax^2/(smin^2+sMeasured^2); coefficients optimized directly in the single nominal fusion SDP";
    z.provenance.motionResponse="Half-second nominal longitudinal contraction plus direct LiDAR pose and midpoint-speed direction-heading response bounds, all imposed in the same solve as fusion; no full-pose or nonlinear settling-time claim";
    z.provenance.operatingSet="Bessafa et al. (2026), Remark 3: compact invariant operating set assumed; invariance and numerical attraction radius are not computed or certified";
    z.headingGainLimits=[8,8]; % kappaL*(P^-1)77 and directionGain*(P^-1)77.
    z.matchedGainLimits=[8,8,2]; % Position L/G and velocity-direction self-feedback.
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]; predict uses course rate from its internal global velocity and acceleration; no upstream rate input; Four additional outputs: squared speed, velocity-acceleration dot/cross and body-frame lateral velocity; yaw-rate correction uses coupled heading gains from motion and LiDAR innovations; no body-acceleration projections or cross-minus-rate constraint; no direct gyro input to the global observer";
    z.provenance.angularModel="Yaw-rate process scale maximumTrackAngleRate / 2 s; heading and yaw rate are corrected by matched output innovations";
    z.provenance.lowSpeed="C2 denominator extension equals squared speed above 0.5 m/s; the positive-speed nominal certificate does not certify this low-speed extension";
    z.provenance.courseAcceleration="Unmeasured course acceleration is a process disturbance, never a differentiated predictor input. jerkMps3 scales the aggregate translational residual R(phi)*[sDoubleDot;s*qDot] in the noise objective; it is a design weight, not a verified physical upper bound";
    z.provenance.operatingPoints="One nominal simultaneous-source noise objective; motion cases cover minimum/midpoint/maximum speed and zero, tangential and normal acceleration";
    z.provenance.metric="Rotation-invariant translational blocks and a full heading/rate metric; off-diagonal coupling routes heading innovations to the yaw-rate state without a separate rate measurement correction";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="All eight normalized errors at the nominal simultaneous-source pose point; shared primitive velocity/acceleration columns including the measured lateral-velocity row, this energy proxy is not an empirical covariance guarantee";
    z.provenance.fusion="One nominal simultaneous LiDAR/GNSS/auxiliary certificate; eight analytic coefficient intervals enclose the nominal error bound, omitting the PSD body-lateral Gram uniformly over sideslip. No sensor-outage or finite nonlinear-error certificate";
    z.provenance.feedback="Shared normalized pose gain kappaL=kappaG. Fixed shear Pc=U'*diag(p,R)*U, U=[1 -1 1;0 1 0;0 0 1], makes the motion Schur metric R affine and retains pose-to-motion correction. Self-feedback caps stay inside the solve";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
