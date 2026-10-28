function cfg=mncavFullObserverConfig(options)
% Current MnCAV intrinsic predictor; IMU is confined to invariant innovations.
% The eighth, internal yaw-rate state removes speed-denominator singularities.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-intrinsic-vector-iss-v10-physical-domain",'maximumTrackAngleRate',.4,'initialState',[]);
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
            'velocityMps',.256,'accelerationMps2',.233,'gyroRadps',.0018,'jerkMps3',.05), ...
        'weakPositionStrength',.05,'minimumGnssWeight',.5,'nominalGnssWeight',.5, ...
        'gnssCouplingSplit',.5,'operatingPoints',["lidar","lidarGnss","weakLidarGnss","gnssOnly","weakLidar"], ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    z.provenance=struct( ...
        'noise',"Inherited declared sensor priors: conditional registration moments on the first 60 s of the 2026-10-02 study; BESTPOS reported sigma on 12-09-31; adopted lateral identification and independently corrected IMU on 12-11-24; these are design priors, not an overlap covariance certificate", ...
        'headingLengthM',"Heading error weighted by a 10 m landmark range", ...
        'normalization',"lambda = 1 and P >= I");
    z.theta=options.Theta;z.yawAccelerationRadps2=cfg.maximumTrackAngleRate/2;
    z.balancedPoseMetric=false;
    z.angularMetricDiagonal=true;
    z.noiseObjective="normalizedFullState";
    z.velocityAvailable=true;z.accelerationAvailable=true;
    z.redundancy=struct('minimumCourseSpeed',10,'informativeLidarMismatch',.001, ...
        'degradedLidarMismatch',.001,'speedPartitions',10,'coursePartitions',2, ...
        'accelerationPartitions',2,'gnssSplit',.5,'lambda',1, ...
        'exactGnssLeverEnclosure',true,'accelerationMultiplierPartition',"zero");
    % The physical-domain global candidate lowers the noise objective but
    % regresses an existing recovery test and the development replay.
    % Retain the zero-multiplier member as default; global/sign/box are
    % implemented, verified choices in this same synthesis framework.
    z.provenance.speedOutput="Physical output is v'*v. Fixed numerical normalization is separate from the output and matched weights. No prescribed direct response or nominal decay floor; gains follow the noise objective and common stability conditions. No instantaneous-speed division or replacement output";
    z.regionalRadiusCap=1; % Outer normalized error ball used for Hessian bounds.
    z.headingGainLimits=[8,8]; % kappaL*(P^-1)77 and directionGain*(P^-1)77.
    z.matchedGainLimits=[8,8,2]; % Position L/G and velocity-direction self-feedback.
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]; predict uses internal r and estimated lateral sideslip rate; Five outputs: squared speed, velocity-acceleration dot/cross, course-direction constraint and yaw rate; no body-acceleration projections or cross-minus-rate constraint; IMU only in invariant innovations";
    z.provenance.angularModel="Yaw-rate process scale maximumTrackAngleRate / 2 s; upstream sideslip-rate error remains a predictor disturbance, not a separate auxiliary measurement";
    z.provenance.operatingPoints="Noise objective at five pose-information cases including weak LiDAR without GNSS; motion cases cover minimum/midpoint/maximum speed and zero, tangential and normal acceleration";
    z.provenance.metric="Rotation-invariant translational blocks and the existing diagonal angular metric. A full angular block is supported by exact caps but is not the default: its lower noise objective regressed heading/recovery tests. Angular diagonality is a design restriction, not a consequence of rotational symmetry";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="All eight normalized errors at pose-anchored nominal points; shared primitive velocity/acceleration/gyro columns and one upstream direction-constraint error budget; this energy proxy is not an empirical covariance guarantee";
    z.provenance.redundancy="All five auxiliary outputs are required throughout every L/G/M certificate. Joint Bernstein controls retain the exact dot/cross Gram and a_perp=s*q dynamics. Physical-domain modes optimize PSD multipliers for b^2+s^2*q^2<=Amax^2; the regression-qualified default fixes them to zero. Full Mixed translation-heading coupling remains. Output withdrawal and zero speed have defined runtime but no current ISS guarantee";
    z.provenance.feedback="Normalized matched self-feedback limits L-position 8, G-position 8, velocity-direction 2 and heading L/direction 8; constrained inside the solve, without runtime clipping; exact common course kinematics and GNSS lever coupling reduce sufficient-condition conservatism";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
