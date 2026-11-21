function cfg=mncavFullObserverConfig(options)
% MnCAV intrinsic prediction with direct linear upstream yaw-rate feedback.
% Autonomous translational prediction uses a C2 extension below its speed cutoff.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-linear-yaw-rate-v20",'maximumTrackAngleRate',.4,'initialState',[]);
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('outputPoint', ...
        jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json'))));
    cfg.lidar=struct('poseScales',[1;1;.1],'gainInformationScale',64);
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'maximumPositionHeadingCoupling',.06);
    z=struct('sigma',4,'headingLengthM',10, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',.23,'gnssHeadingRad',.01, ...
            'velocityMps',.256,'accelerationMps2',.233,'jerkMps3',.05), ...
        'gnssPositionAvailable',true,'gnssHeadingAvailable',true, ...
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
    z.yawRateGainPerSecond=1/z.motionResponse.timeConstantS;
    z.velocityAvailable=true;z.accelerationAvailable=true;
    z.fusion=struct('minimumCourseSpeed',10,'lidarOperatorMismatch',.001);
    z.provenance.speedOutput="Physical squared-speed output with bounded gain factor Vmax^2/(smin^2+sMeasured^2); coefficients optimized directly in the single nominal fusion SDP";
    z.provenance.motionResponse="Half-second nominal longitudinal contraction plus direct LiDAR pose and midpoint-speed direction-heading response bounds, all imposed in the same solve as fusion; no full-pose or nonlinear settling-time claim";
    z.provenance.operatingSet="Bessafa et al. (2026), Remark 3: compact invariant operating set assumed; invariance and numerical attraction radius are not computed or certified";
    z.headingGainLimits=[8,8]; % kappaL*(P^-1)77 and directionGain*(P^-1)77.
    z.matchedGainLimits=[8,8,2]; % Position L/G and velocity-direction self-feedback.
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]. The linear outputs are position, independent heading and estimated body yaw rate from the lateral observer. The four nonlinear motion outputs are squared speed, velocity-acceleration dot/cross and body-frame lateral velocity.";
    z.provenance.independentHeading="Independent heading input, e.g. dual-antenna GNSS; The 0.01 rad offline noise scale is a configurable design assumption, not a measured receiver specification. One fixed offline gain acts on valid linear outputs; no receiver-information reliability weight is used.";
    z.provenance.angularModel="Heading integrates the internal body rate. A valid upstream rate drives a finite-gain linear innovation on that state. A missing rate leaves its state constant.";
    z.provenance.lowSpeed="C2 denominator extension equals squared speed above 0.5 m/s; the positive-speed nominal certificate does not certify this low-speed extension";
    z.provenance.courseAcceleration="Unmeasured course acceleration is a process disturbance, never a differentiated predictor input. jerkMps3 scales the aggregate translational residual R(phi)*[sDoubleDot;s*qDot] in the noise objective; it is a design weight, not a verified physical upper bound";
    z.provenance.operatingPoints="One nominal simultaneous-source noise objective; motion cases cover minimum/midpoint/maximum speed and zero, tangential and normal acceleration";
    z.provenance.metric="The Schur complement preserves the seven previously designed physical gain rows. A separate scalar rate metric removes all motion and pose injection into body rate.";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="The SDP noise objective selects the original matched-gain template. It excludes upstream yaw-rate estimation error and its correlation with lateral velocity.";
    z.provenance.fusion="Positive-speed nominal vertex certificate with valid upstream body rate, available motion outputs and the declared pose geometry.";
    z.provenance.feedback="A single fixed-shear SDP sets the matched gains. The deployed metric separates the rate and is checked again at every fusion vertex with the direct rate row active.";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
