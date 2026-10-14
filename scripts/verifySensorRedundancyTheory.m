function report=verifySensorRedundancyTheory(options)
% verifySensorRedundancyTheory Verify the unified continuous-theory witness.
% This checks the two sensor-adequacy sets of derivation Section 0.6 with one
% constant metric and gain set. It never installs gains in the runtime.
% The default witness is retained as a small table in the existing derivation.
% Synthesize=true solves the same theoretical LMIs for an explicit domain.
    arguments
        options.Witness (1,1) struct=struct()
        options.Synthesize (1,1) logical=false
        options.MinimumGnssSpeed (1,1) double {mustBeReal,mustBeFinite,mustBePositive}=10
        options.OperatorMismatch (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}=.001
    end
    root=setupVehicleLocalization();cfg=mncavFullObserverConfig();inputs=theoryInputs(cfg);
    if options.Synthesize
        domain=struct('minimumGnssSpeed',options.MinimumGnssSpeed, ...
            'informativeLidarMismatch',options.OperatorMismatch, ...
            'degradedLidarMismatch',options.OperatorMismatch, ...
            'speedPartitions',10,'accelerationSplit',.5,'gnssSplit',cfg.invariantObserver.gnssCouplingSplit,'lambda',1);
        validateDomain(domain,inputs);
        external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
        addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
        yalmip('clear');Pc=sdpvar(3,3);Ph=sdpvar(2,2);P=blkdiag(Pc,Pc,Ph);
        gains=sdpvar(4,1);witness=struct('inputs',inputs,'domain',domain,'P',P, ...
            'kappaL',gains(1),'kappaG',gains(2),'accelerationGain',gains(3),'directionGain',gains(4));
        [L,G]=sensorRedundancySupport.blocks(witness);constraints=[P>=eye(8),P<=1e5*eye(8), ...
            Ph(1,2)==0,Ph<=Pc(1:2,1:2),gains>=0];
        for j=1:numel(L),constraints=[constraints,L{j}<=-1e-7*eye(8)];end %#ok<AGROW>
        for j=1:numel(G),constraints=[constraints,G{j}<=-1e-7*eye(8)];end %#ok<AGROW>
        result=optimize(constraints,sum(gains)+.01*trace(P),sdpsettings('solver','sedumi','verbose',0));
        assert(ismember(result.problem,[0,4]),'VehicleLocalization:InfeasibleRedundancyTheory', ...
            'The requested common theory domain is infeasible in this enclosure: %s',result.info);
        % Homogeneous rescaling preserves all matched feedback ratios and
        % moves the lambda-normalized inequalities away from solver tolerance.
        witness.P=1.01*value(P);witness.P=(witness.P+witness.P.')/2;
        for name=["kappaL","kappaG","accelerationGain","directionGain"]
            witness.(name)=1.01*value(witness.(name));
        end
        witness.solverStatus=result.problem;
    elseif ~isempty(fieldnames(options.Witness))
        witness=options.Witness;
    else
        text=fileread(fullfile(root,'improved_observer_derivation.md'));
        token=regexp(text,'(?s)<!-- SENSOR_REDUNDANCY_WITNESS_BEGIN -->\s*```json\s*(.*?)\s*```\s*<!-- SENSOR_REDUNDANCY_WITNESS_END -->','tokens','once');
        assert(~isempty(token),'VehicleLocalization:MissingRedundancyWitness','The documented theory witness is missing.');
        witness=jsondecode(token{1});
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
    for name=["kappaL","kappaG","accelerationGain","directionGain"]
        validateattributes(witness.(name),{'double'},{'real','finite','positive','scalar'});
    end
    [L,G]=sensorRedundancySupport.blocks(witness);worstL=worstEigenvalue(L);worstG=worstEigenvalue(G);
    verified=worstL<0 && worstG<0;
    assert(verified,'VehicleLocalization:UnverifiedRedundancyTheory','The common theory inequalities failed: L %.6g, G %.6g.',worstL,worstG);
    report=struct('witness',witness,'commonLmiVerified',verified, ...
        'lidarVertices',numel(L),'gnssBernsteinVertices',numel(G), ...
        'worstLidarEigenvalue',worstL,'worstGnssEigenvalue',worstG, ...
        'minimumMetricEigenvalue',min(eig(P)), ...
        'coversGnssAbsent',true,'coversZeroLidarInformation',true, ...
        'minimumGnssCourseSpeed',witness.domain.minimumGnssSpeed, ...
        'appliesToDefaultRuntimeGains',false,'runtimeModified',false, ...
        'scope',"One continuous observer with a common metric/gains; nonlinear local ISS follows under the explicit geometry, motion, bias and auxiliary-input conditions of Section 0.6; the default runtime gain file is not certified by this separate witness");
    if nargout==0,disp(rmfield(report,'witness'));end
end

function inputs=theoryInputs(cfg)
    inputs=sensorRedundancySupport.inputs(cfg,cfg.invariantDesign.gyroGainNormalized, ...
        cfg.invariantDesign.speedGainNormalized);
end

function validateDomain(d,p)
    assert(d.minimumGnssSpeed>0 && d.minimumGnssSpeed<=p.maximumSpeed, ...
        'VehicleLocalization:UnobservableGnssHeading','Position-only GNSS needs the declared positive-motion domain for full-state convergence.');
    validateattributes(d.speedPartitions,{'double'},{'finite','integer','positive','scalar'});
    validateattributes(d.informativeLidarMismatch,{'double'},{'finite','nonnegative','scalar'});
    validateattributes(d.degradedLidarMismatch,{'double'},{'finite','nonnegative','scalar'});
    assert(d.accelerationSplit>0 && d.accelerationSplit<1 && d.gnssSplit>0 && d.gnssSplit<1 && d.lambda>0, ...
        'VehicleLocalization:InvalidRedundancyDomain','Declare positive decay and Young splits strictly between zero and one.');
end


function value=worstEigenvalue(matrices)
    value=-Inf;
    for k=1:numel(matrices)
        M=matrices{k};value=max(value,max(eig((M+M.')/2)));
    end
end
