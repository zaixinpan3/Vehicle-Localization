function report=verifySensorRedundancyTheory(options)
% verifySensorRedundancyTheory Verify the unified continuous-theory witness.
% This checks three complementary sensor-adequacy sets with one
% constant metric and gain set. It never installs gains in the runtime.
% With no Witness, solve temporary theoretical gains for the requested domain.
% A supplied in-memory Witness is verified without solving the main observer.
    arguments
        options.Witness (1,1) struct=struct()
        options.Config (1,1) struct=mncavFullObserverConfig()
        options.MinimumCourseSpeed (1,1) double {mustBeReal}=NaN
        options.OperatorMismatch (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}=.001
    end
    root=setupVehicleLocalization();cfg=options.Config;
    if ~isfield(cfg,'invariantDesign')
        external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
        addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
        gyro=invariantObserverSupport.scalarGyroDesign(cfg);
        inputs=sensorRedundancySupport.inputs(cfg,gyro.physicalGain/cfg.invariantObserver.sigma);
    else
        inputs=theoryInputs(cfg);
    end
    if isempty(fieldnames(options.Witness))
        domain=cfg.invariantObserver.redundancy;
        if ~isnan(options.MinimumCourseSpeed),domain.minimumCourseSpeed=options.MinimumCourseSpeed;end
        domain.informativeLidarMismatch=options.OperatorMismatch;
        domain.degradedLidarMismatch=options.OperatorMismatch;
        validateDomain(domain,inputs);
        external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
        addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
        yalmip('clear');Pc=sdpvar(3,3);Ph=sdpvar(2,2);P=blkdiag(Pc,Pc,Ph);
        gains=sdpvar(5,1);witness=struct('inputs',inputs,'domain',domain,'P',P, ...
            'kappaL',gains(1),'kappaG',gains(2),'bilinearGain',gains(3),'directionGain',gains(4),'speedSquaredGain',gains(5));
        [L,G,M]=sensorRedundancySupport.blocks(witness);constraints=[P>=eye(8),P<=1e5*eye(8), ...
            Ph(1,2)==0,gains>=1e-7];
        if cfg.invariantObserver.balancedPoseMetric
            constraints=[constraints,Ph<=Pc(1:2,1:2)];
        end
        [result,solveAudit]=solveCommonObserverLmis(constraints,sum(gains)+.01*trace(P), ...
            [L,G,M],[numel(L),numel(G),numel(M)],witness,cfg.invariantObserver);
        assert(ismember(result.problem,[0,4]),'VehicleLocalization:InfeasibleRedundancyTheory', ...
            'The requested common theory domain is infeasible in this enclosure: %s',result.info);
        % Homogeneous rescaling preserves all matched feedback ratios and
        % moves the lambda-normalized inequalities away from solver tolerance.
        witness.P=1.01*value(P);witness.P=(witness.P+witness.P.')/2;
        for name=["kappaL","kappaG","bilinearGain","directionGain","speedSquaredGain"]
            witness.(name)=1.01*value(witness.(name));
        end
        witness.solverStatus=result.problem;witness.constraintGeneration=solveAudit;
    else
        witness=options.Witness;
    end
    assert(isequaln(witness.inputs,inputs),'VehicleLocalization:StaleRedundancyWitness', ...
        'The theoretical witness uses different declared physical/scalar-gain inputs.');
    validateDomain(witness.domain,inputs);
    P=witness.P;validateattributes(P,{'double'},{'real','finite','size',[8,8]});
    assert(norm(P-P.','fro')<1e-9*max(1,norm(P,'fro')) && min(eig(P))>0, ...
        'VehicleLocalization:InvalidRedundancyMetric','Require a constant symmetric positive definite metric.');
    rotationStructure=norm(P(1:3,1:3)-P(4:6,4:6),'fro') ...
        +norm(P(1:6,7:8),'fro')+norm(P(1:3,4:6),'fro');
    assert(rotationStructure<1e-9*max(1,norm(P,'fro')), ...
        'VehicleLocalization:InvalidRedundancyMetric','The proof requires the declared rotation-invariant metric.');
    for name=["kappaL","kappaG","bilinearGain","directionGain","speedSquaredGain"]
        validateattributes(witness.(name),{'double'},{'real','finite','positive','scalar'});
    end
    [L,G,M]=sensorRedundancySupport.blocks(witness);
    worstL=worstEigenvalue(L);worstG=worstEigenvalue(G);worstM=worstEigenvalue(M);
    verified=worstL<0 && worstG<0 && worstM<0;
    assert(verified,'VehicleLocalization:UnverifiedRedundancyTheory', ...
        'The common theory inequalities failed: L %.6g, G %.6g, M %.6g.',worstL,worstG,worstM);
    report=struct('witness',witness,'commonLmiVerified',verified, ...
        'lidarVertices',numel(L),'gnssBernsteinVertices',numel(G), ...
        'worstLidarEigenvalue',worstL,'worstGnssEigenvalue',worstG, ...
        'worstMixedEigenvalue',worstM,'mixedVertices',numel(M),'zeroSpeedCertified',false, ...
        'minimumMetricEigenvalue',min(eig(P)), ...
        'coversGnssAbsent',true,'coversZeroLidarInformation',true, ...
        'minimumCourseSpeed',witness.domain.minimumCourseSpeed, ...
        'speedInterval',[witness.domain.minimumCourseSpeed,inputs.maximumSpeed], ...
        'appliesToDefaultRuntimeGains',false,'runtimeModified',false, ...
        'scope',"One continuous observer with a common metric/gains; nonlinear local ISS follows only on the declared strictly positive speed interval and under the geometry, bias and auxiliary-input conditions; zero-speed runtime is not certified; temporary runtime gains are verified separately");
    if nargout==0,disp(rmfield(report,'witness'));end
end

function inputs=theoryInputs(cfg)
    inputs=sensorRedundancySupport.inputs(cfg,cfg.invariantDesign.gyroGainNormalized);
end

function validateDomain(d,p)
    assert(d.minimumCourseSpeed>0 && d.minimumCourseSpeed<=p.maximumSpeed, ...
        'VehicleLocalization:InvalidIssSpeedInterval','All sensor families require the declared strictly positive course-speed interval.');
    validateattributes(d.speedPartitions,{'double'},{'finite','integer','positive','scalar'});
    sensorRedundancySupport.courseControls(p.maximumCourseRate,d.coursePartitions);
    validateattributes(d.informativeLidarMismatch,{'double'},{'finite','nonnegative','scalar'});
    validateattributes(d.degradedLidarMismatch,{'double'},{'finite','nonnegative','scalar'});
    validateattributes(d.nominalDecayRatePerSecond,{'double'},{'finite','positive','scalar'});
    assert(d.bilinearSplit>0 && d.bilinearSplit<1 && d.gnssSplit>0 && d.gnssSplit<1 && d.lambda>0, ...
        'VehicleLocalization:InvalidRedundancyDomain','Declare positive decay and Young splits strictly between zero and one.');
end


function value=worstEigenvalue(matrices)
    value=-Inf;
    for k=1:numel(matrices)
        M=matrices{k};value=max(value,max(eig((M+M.')/2)));
    end
end
