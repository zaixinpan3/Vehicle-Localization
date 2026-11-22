function cfg=mncavFullObserverConfig(options)
% MnCAV intrinsic prediction with direct linear upstream yaw-rate feedback.
% Body-rate translation has polynomial drift, including at zero speed.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite,mustBePositive}=4
    end
    assert(~isfield(options.DesignOverride,'yawRateGainPerSecond'),'VehicleLocalization:PrescribedRateGain', ...
        'The direct yaw-rate gain is a design output. Configure the physical noise scales and solve again.');
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-pose-four-vertex-v26",'maximumBodyYawRate',.4,'initialState',[]);
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('outputPoint', ...
        jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json'))));
    cfg.lidar=struct('poseScales',[1;1;.1]);
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'maximumPositionHeadingCoupling',.06);
    receiver=jsondecode(fileread(fullfile(here,'mncavGnssHeading.json')));
    z=struct('sigma',4,'headingLengthM',10, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',receiver.positionErrorScaleM,'gnssHeadingRad',receiver.headingErrorScaleRad, ...
            'velocityMps',.256,'accelerationMps2',.233,'jerkMps3',.05), ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    z.provenance=struct( ...
        'noise',"Inherited declared sensor priors: conditional registration moments on the first 60 s of the 2026-10-02 study; independent BESTPOS and HEADING2 calibration on 12-11-24; adopted lateral identification and independently corrected IMU on 12-11-24; these are design priors, not an overlap covariance certificate", ...
        'headingLengthM',"Heading error weighted by a 10 m landmark range", ...
        'normalization',"lambda = 1 and P >= I");
    z.theta=options.Theta;z.sigma=z.theta;
    z.motionTimeConstantS=.5; % Nominal longitudinal response used by the matched motion design.
    z.gnssBandwidthRatio=1; % Normalized poles of each measured position chain.
    z.yawAccelerationRadps2=cfg.maximumBodyYawRate/2;
    z.noiseObjective="normalizedFullState";
    lateral=lateralObserverConfig();
    z.noise.upstreamRateRadps=lateral.synthesis.measurementErrorScale(2);
    z.provenance.upstreamRateNoise="Use the lateral design's bias-plus-noise rate scale as an engineering proxy for upstream estimation error. The joint covariance with lateral velocity is unknown.";
    z.fusion=struct('minimumCourseSpeed',10,'lidarOperatorMismatch',.001);
    z.provenance.operatingSet="Bessafa et al. (2026), Remark 3: compact invariant operating set assumed; invariance and numerical attraction radius are not computed or certified";
    z.provenance.feedback="Fixed linear gain, independent LiDAR strength and Jacobian-matched motion correction in the primary metric.";
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]. The GNSS output is antenna position and independent vehicle heading. The motion packet supplies body yaw rate. The packet uses the lateral estimate in its design range and the measured gyro rate below the minimum design speed. The four nonlinear motion outputs are squared speed, velocity-acceleration dot/cross and body-frame lateral velocity.";
    z.provenance.independentHeading="Position and vehicle heading form one GNSS measurement. The default heading scale is a cross-half error on the independent calibration drive. Synthetic receivers supply their generator prior. Every complete measurement uses the same constant gain.";
    z.provenance.angularModel="Heading integrates the internal body rate. A valid upstream rate drives a finite-gain linear innovation on that state. A missing rate leaves its state constant.";
    z.provenance.lowSpeed="The body-rate drift is smooth at zero speed. The motion response design uses its declared speed range.";
    z.provenance.bodyJerk="The jerkMps3 design weight scales the aggregate translational process disturbance R(psi)*uDoubleDot + rDot*J*v. A physical upper bound must be established separately.";
    z.provenance.metric="The SDP uses a diagonal angular metric so only the direct linear rate channel injects into body rate.";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="LiDAR, process and rate injection costs select the common metric. The GNSS heading bandwidth minimizes a scalar noise proxy subject to packet dissipation.";
    z.provenance.fusion="Four drift inequalities use one rotation-invariant planar metric. Each complete GNSS pose adds a positive semidefinite output Gram term. The nominal certificate assumes the declared motion and LiDAR bounds.";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
