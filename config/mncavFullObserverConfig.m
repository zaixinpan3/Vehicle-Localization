function cfg=mncavFullObserverConfig(options)
% Current MnCAV predictor with explicit course-angular-acceleration coupling.
% The eighth, internal yaw-rate state removes speed-denominator singularities.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-course-acceleration-v13",'maximumTrackAngleRate',.4,'maximumTrackAngleAcceleration',.4,'initialState',[]);
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
            'velocityMps',.256,'accelerationMps2',.233,'gyroRadps',.0018,'jerkMps3',.05,'courseAccelerationRadps2',.05), ...
        'nominalGnssWeight',.5, ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    z.provenance=struct( ...
        'noise',"Inherited declared sensor priors: conditional registration moments on the first 60 s of the 2026-10-02 study; BESTPOS reported sigma on 12-09-31; adopted lateral identification and independently corrected IMU on 12-11-24; these are design priors, not an overlap covariance certificate", ...
        'headingLengthM',"Heading error weighted by a 10 m landmark range", ...
        'normalization',"lambda = 1 and P >= I");
    z.theta=options.Theta;z.yawAccelerationRadps2=cfg.maximumTrackAngleRate/2;
    z.balancedPoseMetric=false;
    z.angularMetricDiagonal=true;
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
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]; predict uses internal r and estimated lateral sideslip rate; Four additional outputs: squared speed, velocity-acceleration dot/cross and body-frame lateral velocity; gyro is a separate rate measurement; no body-acceleration projections or cross-minus-rate constraint; course acceleration is supplied or estimated from consecutive valid gyro-plus-sideslip-rate samples";
    z.provenance.angularModel="Yaw-rate process scale maximumTrackAngleRate / 2 s; upstream sideslip-rate error remains a predictor disturbance, not a separate auxiliary measurement";
    z.provenance.courseAcceleration="Declared design bound 0.4 rad/s^2 (course-rate change over 1 s), not a measured envelope; error scale 0.05 rad/s^2 is a design prior. jerkMps3 now scales only the second speed derivative. No finite-difference noise bound follows without a minimum sample interval and signal regularity";
    z.provenance.operatingPoints="One nominal simultaneous-source noise objective; motion cases cover minimum/midpoint/maximum speed and zero, tangential and normal acceleration";
    z.provenance.metric="Rotation-invariant translational blocks and the existing diagonal angular metric. A full angular block is supported by exact caps but is not the default: its lower noise objective regressed heading/recovery tests. Angular diagonality is a design restriction, not a consequence of rotational symmetry";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="All eight normalized errors at the nominal simultaneous-source pose point; shared primitive velocity/acceleration columns including the measured lateral-velocity row, plus a separate gyro column; this energy proxy is not an empirical covariance guarantee";
    z.provenance.fusion="One nominal simultaneous LiDAR/GNSS/auxiliary certificate; seven analytic coefficient intervals enclose the nominal error bound, omitting the PSD body-lateral Gram uniformly over sideslip. No sensor-outage or finite nonlinear-error certificate";
    z.provenance.feedback="Shared normalized pose gain kappaL=kappaG. Fixed shear Pc=U'*diag(p,R)*U, U=[1 -1 1;0 1 0;0 0 1], makes the motion Schur metric R affine and retains pose-to-motion correction. Self-feedback caps stay inside the solve";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
