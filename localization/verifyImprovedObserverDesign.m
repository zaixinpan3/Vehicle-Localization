function verification = verifyImprovedObserverDesign(design, cfg)
% verifyImprovedObserverDesign Check gains, timing and every timer inequality.
% Legacy continuous designs remain auditable but are not accepted at runtime.
    arguments
        design (1,1) struct
        cfg (1,1) struct
    end
    if ~isfield(design,'kind') || string(design.kind)~="aperiodic-pose-v1"
        verification = verifyContinuousObserverDesign(design,cfg);
        return;
    end
    c = design.timer;
    assert(isequal(size(design.K),[7,3]) && isequal(size(design.N),[7,4]));
    assert(all(isfinite(design.K),'all') && all(isfinite(design.N),'all'));
    compatible = abs(cfg.observer.theta-c.theta)<1e-12 ...
        && abs(cfg.observer.sigma-c.theta)<1e-12 ...
        && max(abs(design.N-cfg.observer.invariantGain),[],'all')<1e-12 ...
        && abs(cfg.measurement.lidarMaximumAge-c.onTime)<1e-12 ...
        && abs(cfg.measurement.gpsMaximumAge-c.onTime)<1e-12 ...
        && abs(cfg.measurement.minimumPoseInterval-c.tMin)<1e-12 ...
        && abs(cfg.measurement.maximumPoseInterval-c.tMax)<1e-12 ...
        && c.wMin==1 && c.nFactor==1;
    fixture = cfg;
    fixture.K = design.K;
    fixture.N = design.N;
    verification = verifyLidarOnlyTimerCertificate(c,fixture);
    verification.certified = compatible && verification.passed;
    verification.productionImplemented = true;
    % Bound every mode in the causal prediction tail, including GPS-only
    % continuation. The LiDAR-only tail estimate is not silently reused for
    % this additional mode or for more than one active correction interval.
    b=buildImprovedObserverCertificateData(fixture);
    maximumLogNorm=max(verification.predictionEuclideanLogNormPerSecond, ...
        verification.onEuclideanLogNormPerSecond);
    gpsFeedback=design.K*diag([1,1,0])*b.Cb;
    for h=1:b.outputVertexCount
        for f=1:b.fVertexCount
            A=c.theta*(b.A+b.fVertices(:,:,f)+design.N*b.outputVertices(:,:,h)-gpsFeedback);
            maximumLogNorm=max(maximumLogNorm,max(eig((A+A.')/2)));
        end
    end
    verification.maximumContinuationLogNormPerSecond=maximumLogNorm;
    verification.continuation150msHomogeneousBound=exp(.15*maximumLogNorm);
    verification.continuation300msHomogeneousBound=exp(.30*maximumLogNorm);
    verification.verifiedModel = struct('K',design.K,'N',design.N,'P',c.P, ...
        'operating',cfg.operating,'scalingExponents',cfg.observer.scalingExponents, ...
        'minimumHeadingWeight',cfg.lidar.minimumHeadingWeight, ...
        'theta',c.theta,'knots',c.knots,'rate',c.rate,'nFactor',c.nFactor,'wMin',c.wMin);
    verification.checkedVertexCount = verification.flowInequalityCount;
    verification.scope = "homogeneous aperiodic full-pose pulses; conditional on bounded errors and invariant operating region";
end
