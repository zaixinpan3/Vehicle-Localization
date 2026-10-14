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
        [L,G]=blocks(witness);constraints=[P>=eye(8),P<=1e5*eye(8), ...
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
    [L,G]=blocks(witness);worstL=worstEigenvalue(L);worstG=worstEigenvalue(G);
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
    z=cfg.invariantObserver;s=cfg.iss;
    assert(z.theta==z.sigma,'VehicleLocalization:RedundancyScaling','Re-solve this witness when theta differs from sigma.');
    inputs=struct('theta',z.theta,'headingLengthM',z.headingLengthM, ...
        'maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
        'maximumCourseRate',cfg.maximumTrackAngleRate, ...
        'minimumLidarPositionStrength',s.minimumPositionStrength, ...
        'minimumLidarHeadingStrength',s.minimumHeadingStrength, ...
        'maximumLidarCoupling',s.maximumPositionHeadingCoupling, ...
        'minimumGnssWeight',z.minimumGnssWeight, ...
        'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).', ...
        'gyroGainNormalized',cfg.invariantDesign.gyroGainNormalized, ...
        'speedGainNormalized',cfg.invariantDesign.speedGainNormalized);
    inputs=jsondecode(jsonencode(inputs));
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

function [L,G]=blocks(w)
    p=w.inputs;d=w.domain;P=w.P;th=p.theta;ell=p.headingLengthM;
    E=eye(8);e2=E(:,2);e3=E(:,3);e5=E(:,5);e6=E(:,6);e7=E(:,7);e8=E(:,8);
    C=E([1,4,7],:);Cp=C(1:2,:);Dacc=e3*e3.'+e6*e6.';
    q=p.maximumCourseRate;b=2*q*p.maximumSpeed+2*p.maximumAcceleration;
    rho=norm(p.gnssBodyOffset)/ell;tau=d.gnssSplit;eta=d.accelerationSplit;
    A0=zeros(8);A0(1,2)=1;A0(2,3)=1;A0(4,5)=1;A0(5,6)=1;A0(7,8)=1;
    corners=[1,1;1,-1;-1,1;-1,-1];speeds=linspace(d.minimumGnssSpeed,p.maximumSpeed,d.speedPartitions+1);
    L={};G={};
    for u=[0,q^2]
        for rate=[-q,q]
            for j=1:4
                A=A0;A([3,6],[2,5])=u/th^2*eye(2);
                A([3,6],[3,6])=2*rate/th*[0,-1;1,0];
                A([3,6],8)=b/(th^2*ell)*corners(j,:).';
                for k=1:4
                    c=p.maximumLidarCoupling*corners(k,:).';
                    S=[p.minimumLidarPositionStrength*eye(2),c;c.',p.minimumLidarHeadingStrength];
                    for speedMode=[0,1]
                        M=P*(A-p.gyroGainNormalized*(e8*e8.')-speedMode*p.speedGainNormalized*(e2*e2.')) ...
                            -w.kappaL*C.'*S*C;
                        L{end+1}=M+M.'+w.kappaG*rho^2*(e7*e7.') ...
                            +2*w.kappaL*d.informativeLidarMismatch*(C.'*C)+d.lambda*E; %#ok<AGROW>
                    end
                end
                for k=1:d.speedPartitions
                    left=e5-speeds(k)/(th*ell)*e7;right=e5-speeds(k+1)/(th*ell)*e7;
                    B=cat(3,left*left.',(left*right.'+right*left.')/2,right*right.');
                    for jB=1:3
                        M=P*(A-p.gyroGainNormalized*(e8*e8.')-p.speedGainNormalized*(e2*e2.')) ...
                            -w.kappaG*(1-tau)*p.minimumGnssWeight*(Cp.'*Cp) ...
                            -w.directionGain*B(:,:,jB)-(1-eta)*w.accelerationGain*Dacc;
                        G{end+1}=M+M.'+(w.kappaG*rho^2/tau ...
                            +2*w.accelerationGain*(1/eta-1)*p.maximumAcceleration^2/(th^4*ell^2))*(e7*e7.') ...
                            +2*w.kappaL*d.degradedLidarMismatch*(C.'*C)+d.lambda*E; %#ok<AGROW>
                    end
                end
            end
        end
    end
end

function value=worstEigenvalue(matrices)
    value=-Inf;
    for k=1:numel(matrices)
        M=matrices{k};value=max(value,max(eig((M+M.')/2)));
    end
end
