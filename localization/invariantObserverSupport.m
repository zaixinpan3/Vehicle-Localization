classdef invariantObserverSupport
% Intrinsic eight-state predictor and rotation-invariant output correction.
% State [X,VX,AX,Y,VY,AY,psi,r] contains an estimated yaw rate. IMU samples
% occur only in output packets; predict has no measured-motion argument.
    methods (Static)
        function [next,q]=predict(state,dt,sideslipRate)
            q=state(8)+sideslipRate;
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=q^2*eye(2);A([3,6],[3,6])=2*q*[0,-1;1,0];
            next=(eye(8)-dt*A)\state;
        end

        function [value,H]=outputs(state,beta,betaRate,cfg)
            v=state([2,5]);a=state([3,6]);s=cfg.iss.maximumSpeed;ell=cfg.invariantObserver.headingLengthM;
            cross=v(1)*a(2)-v(2)*a(1);q=state(8)+betaRate;angle=state(7)+beta;
            direction=[-sin(angle);cos(angle)];
            value=[(v.'*v)/(2*s);(v.'*a)/s;cross/s;ell*state(8); ...
                (cross-(v.'*v)*q)/s;direction.'*v; ...
                [cos(state(7)),sin(state(7))]*a;[-sin(state(7)),cos(state(7))]*a];
            if nargout>1
                [~,H]=invariantIssSupport.secants(state,state,beta,betaRate,cfg);
            end
        end

        function packet=measurements(speed,vy,ax,ay,gyro,beta,betaRate,velocityValid,accelerationValid,gyroValid,cfg)
        % Consume the upstream angle/rate pair; never reconstruct a course angle.
            s=cfg.iss.maximumSpeed;ell=cfg.invariantObserver.headingLengthM;
            velocityValid=velocityValid && cfg.invariantObserver.velocityAvailable;
            accelerationValid=accelerationValid && cfg.invariantObserver.accelerationAvailable;
            value=zeros(8,1);valid=[velocityValid;velocityValid&&accelerationValid; ...
                velocityValid&&accelerationValid;gyroValid;velocityValid;velocityValid;accelerationValid;accelerationValid];
            if valid(1),value(1)=(speed^2+vy^2)/(2*s);end
            if valid(2),value(2)=(speed*ax+vy*ay)/s;value(3)=(speed*ay-vy*ax)/s;end
            if gyroValid,value(4)=ell*gyro;end
            if accelerationValid,value(7:8)=[ax;ay];end
            if ~velocityValid,beta=0;betaRate=0;end
            packet=struct('value',value,'valid',valid,'beta',beta,'betaRate',betaRate);
        end

        function [next,audit]=correct(state,packet,dt,design,cfg)
            [value,H]=invariantObserverSupport.outputs(state,packet.beta,packet.betaRate,cfg);
            D=diag(design.theta.^-[2,3,3,2,3,2,3,3]);
            C=D*H*design.toPhysical;
            k=design.auxiliaryWeights([1,2,2,3,4,5,2,2]);k=k(:).*double(packet.valid);
            k(4)=0; % The linear gyro innovation has its separate scalar gain.
            residual=D*(packet.value-value);weighted=C.'*diag(k);
            Q=design.theta*weighted*C;rhs=dt*design.theta*(design.P\(weighted*residual));
            delta=(eye(8)+dt*(design.P\Q))\rhs;next=state+design.toPhysical*delta;
            if packet.valid(4)
                % The gyro innovation acts on its estimated rate state only.
                % Its own scalar LMI supplies the gain; heading is untouched.
                measuredRate=packet.value(4)/cfg.invariantObserver.headingLengthM;
                gain=design.theta*design.gyroGainNormalized;
                next(8)=(next(8)+dt*gain*measuredRate)/(1+dt*gain);
            end
            if packet.valid(1)
                % Scalar, rotation-invariant course-speed output. At zero
                % speed the direction comes from heading and the upstream
                % continuous sideslip interface,
                % so no normalization by estimated speed is needed.
                direction=[cos(next(7)+packet.beta);sin(next(7)+packet.beta)];
                measuredSpeed=sqrt(max(0,2*cfg.iss.maximumSpeed*packet.value(1)));
                velocity=next([2,5]);gain=design.theta*design.speedGainNormalized;
                deltaSpeed=dt*gain*(measuredSpeed-direction.'*velocity)/(1+dt*gain);
                next([2,5])=velocity+direction*deltaSpeed;
            end
            audit=struct('innovation',packet.value-value,'valid',packet.valid, ...
                'predicted',value,'normalizedCorrection',delta, ...
                'frozenInvariantMetricNonexpansive',min(eig((Q+Q.')/2))>=-1e-10*max(1,norm(Q,2)));
        end

        function inputs=inputs(cfg)
            z=cfg.invariantObserver;if isfield(z,'provenance'),z=rmfield(z,'provenance');end
            inputs=jsondecode(jsonencode(struct('architecture',"intrinsic-vector-iss-v3", ...
                'outputDegrees',[2,3,3,2,3,2,3,3], ...
                'observer',z,'iss',cfg.iss,'courseRateBound',cfg.maximumTrackAngleRate, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).','poseScales',cfg.lidar.poseScales(:).', ...
                'informationScale',cfg.lidar.gainInformationScale)));
        end

        function design=synthesize(cfg)
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','Add YALMIP and SeDuMi before synthesis.');
            m=model(cfg);z=cfg.invariantObserver;yalmip('clear');Pc=sdpvar(3,3);Ph=sdpvar(2,2);
            P=blkdiag(Pc,Pc,Ph);kL=sdpvar(1);kG=sdpvar(1);kI=sdpvar(5,1);
            constraints=[P>=eye(8),kL>=0,kG>=0,kI>=0,kI(3)==0,kI(1)==0];
            if z.angularMetricDiagonal
                constraints=[constraints,Ph(1,2)==0];
            end
            if z.balancedPoseMetric
                constraints=[constraints,Ph<=Pc(1:2,1:2)];
            end
            if isfield(z,'headingGainLimits')
                constraints=[constraints,kL<=z.headingGainLimits(1)*Ph(1,1), ...
                    kI(5)<=z.headingGainLimits(2)*Ph(1,1)];
            end
            if isfield(z,'matchedGainLimits')
                % For P>0, k*e'*inv(P)*e<=g is equivalent to
                % P-(k/g)*e*e'>=0. These caps constrain feedback gains
                % within the solve, without clipping runtime innovations.
                limits=z.matchedGainLimits;
                constraints=[constraints,P-kL/limits(1)*(m.e1*m.e1.')>=0, ...
                    P-kG/limits(2)*(m.e1*m.e1.')>=0, ...
                    P-kI(5)/limits(3)*(m.e5*m.e5.')>=0];
            end
            witness=sensorRedundancySupport.witness(cfg,P,kL,kG,kI,m.gyroGainNormalized,m.speedGainNormalized);
            [jointL,jointG,jointM]=sensorRedundancySupport.blocks(witness,m.gnssAdequacyEnabled,z.velocityAvailable);
            for block=[jointL,jointG,jointM]
                constraints=[constraints,block{1}<=-z.margin*eye(8)]; %#ok<AGROW>
            end
            cost=0;weights=[kI(1);kI(2);kI(2);kI(3);kI(4);kI(5);kI(2);kI(2)];
            for j=1:numel(m.points)
                p=m.points(j);H=m.Hnominal;
                % Each noise-objective case has a global pose anchor. Invariant
                % outputs alone cannot remove global translation/rotation.
                speedFeedback=m.speedGainNormalized*z.velocityAvailable*(m.e2*m.e2.');
                A=P*(z.sigma*(m.A-m.gyroGainNormalized*m.e8*m.e8.'-speedFeedback)-p.leak*(m.C.'*m.C))-z.sigma*kL*m.C.'*p.S*m.C ...
                    -z.sigma*kG*p.w*m.Cp.'*m.Cp-z.sigma*H.'*diag(weights)*H;
                % A sensor sample driving several outputs remains ONE input.
                % In particular speed feedback shares the velocity column.
                auxiliary=-z.sigma*H.'*diag(weights)*m.DI;
                auxiliary(:,5)=auxiliary(:,5)-P*m.e8*(m.gyroGainNormalized*z.headingLengthM*z.noise.gyroRadps/z.sigma);
                auxiliary(:,1)=auxiliary(:,1)-P*m.e2*(m.speedGainNormalized*z.noise.velocityMps/z.sigma)*z.velocityAvailable;
                PG=[-kL*m.C.'*p.S*m.DL,-kG*p.w*m.Cp.'*m.DG,auxiliary,P*m.B*m.Dd];
                Q=sdpvar(size(PG,2));
                constraints=[constraints,A+A.'+z.sigma^2*eye(8)<=0,[Q,PG.';PG,P]>=0]; %#ok<AGROW>
                cost=cost+trace(Q);
            end
            result=optimize(constraints,cost,sdpsettings('solver',char(z.solver),'verbose',0));
            assert(ismember(result.problem,[0,4]),'VehicleLocalization:InfeasibleInvariantDesign','Invariant synthesis failed: %s',result.info);
            Pv=value(P);Pv=(Pv+Pv.')/2;
            weights=value(kI);weights([1,3])=0;
            % Homogeneous normalization preserves all matched feedback ratios
            % while creating a clear margin in the lambda-normalized LMIs.
            Pv=1.01*Pv;weights=1.01*weights;
            design=struct('architecture',"intrinsic-vector-iss-v3",'sigma',z.sigma,'P',Pv, ...
                'kappaL',1.01*value(kL),'kappaG',1.01*value(kG),'auxiliaryWeights',weights, ...
                'gyroGainNormalized',m.gyroGainNormalized,'gyroDesign',m.gyroDesign, ...
                'speedGainNormalized',m.speedGainNormalized,'speedDesign',m.speedDesign, ...
                'lambda',z.lambda,'meanSquareBound',1.01*value(cost),'solverStatus',result.problem,'solverInfo',string(result.info));
            design.inputs=invariantObserverSupport.inputs(cfg);
            design.verification=invariantObserverSupport.verify(design,cfg);
            assert(design.verification.certified,'VehicleLocalization:InfeasibleInvariantDesign','Solved intrinsic design failed verification.');
        end

        function [gyro,speed]=scalarDesigns(cfg)
        % Re-solve the two scalar auxiliary-channel LMIs for this run.
            z=cfg.invariantObserver;gyro=gyroDesign(z);
            z.yawAccelerationRadps2=2*cfg.iss.maximumAcceleration;
            z.noise.gyroRadps=z.noise.velocityMps;speed=gyroDesign(z);
        end

        function v=verify(design,cfg)
            m=model(cfg,design.gyroDesign,design.speedDesign);P=(design.P+design.P.')/2;
            assert(isequal(size(P),[8,8]) && all(isfinite(P),'all'),'VehicleLocalization:InvalidInvariantDesign','Require an eight-state metric.');
            symmetry=norm(P(1:3,1:3)-P(4:6,4:6),'fro')+norm(P(1:6,7:8),'fro')+norm(P(1:3,4:6),'fro');
            balanced=~cfg.invariantObserver.balancedPoseMetric || ...
                max(eig(P(7:8,7:8)-P(1:2,1:2)))<=1e-8*max(1,norm(P,'fro'));
            diagonal=~cfg.invariantObserver.angularMetricDiagonal || abs(P(7,8))<=1e-8*max(1,norm(P,'fro'));
            authority=feedbackAuthority(design,cfg);
            witness=sensorRedundancySupport.witness(cfg,P,design.kappaL,design.kappaG, ...
                design.auxiliaryWeights,design.gyroGainNormalized,design.speedGainNormalized);
            joint=sensorRedundancySupport.verify(witness,m.gnssAdequacyEnabled,cfg.invariantObserver.velocityAvailable);
            worst=max([joint.worstLidarEigenvalue,joint.worstGnssEigenvalue,joint.worstMixedEigenvalue]);
            regional=invariantIssSupport.regionalCertificate(design,cfg,joint);
            v=struct('certified',worst<0 && min(eig(P))>=1-1e-6 && design.kappaL>=0 && design.kappaG>=0 ...
                && joint.certified && regional.certified && authority.verified ...
                && design.sigma==cfg.invariantObserver.sigma && design.lambda>0 ...
                && design.lambda==cfg.invariantObserver.lambda ...
                && norm(design.P-design.P.','fro')<=1e-10*max(1,norm(P,'fro')) ...
                && balanced && diagonal ...
                && abs(design.gyroGainNormalized-m.gyroGainNormalized)<1e-8*max(1,m.gyroGainNormalized) ...
                && abs(design.speedGainNormalized-m.speedGainNormalized)<1e-8*max(1,m.speedGainNormalized) ...
                && all(design.auxiliaryWeights>=-1e-9) && symmetry<1e-8*max(1,norm(P,'fro')), ...
                'worstVertexEigenvalue',worst,'vertices',joint.vertices,'minimumEigenvalueP',min(eig(P)), ...
                'maximumEigenvalueP',max(eig(P)),'intrinsicRateState',true,'zeroSpeedDivision',false, ...
                'actualOverlapCertified',false,'finiteInvariantSecantsCertified',true, ...
                'conditionalInvariantLocalClosureEstablished',regional.certified, ...
                'numericalInvariantRadiusCertified',regional.certified,'regionalIss',regional, ...
                'jointSensorLmiVerified',joint.certified,'jointSensor',joint, ...
                'conditionalNonlinearGainConditionsVerified',joint.certified, ...
                'feedbackAuthority',authority, ...
                'gyroAvailabilityRequired',true, ...
                'scope',regional.scope);
            if ~v.certified
                v.regionalIss.certified=false;
                v.numericalInvariantRadiusCertified=false;
                v.conditionalInvariantLocalClosureEstablished=false;
            end
        end

        function runtime=runtime(cfg)
            h=cfg.invariantDesign;theta=cfg.invariantObserver.theta;ell=cfg.invariantObserver.headingLengthM;
            assert(isfield(h,'inputs') && isequaln(h.inputs,invariantObserverSupport.inputs(cfg)), ...
                'VehicleLocalization:StaleInvariantDesign', ...
                'Temporary gains use different inputs; solve after configuring this run.');
            assert(theta==h.sigma,'VehicleLocalization:InvalidInvariantDesign','The common certificate requires its fixed theta=sigma.');
            v=invariantObserverSupport.verify(h,cfg);
            assert(v.certified,'VehicleLocalization:InfeasibleInvariantDesign','Temporary invariant design fails the current LMI.');
            T=diag(theta.^[1,2,3,1,2,3,1,2]);E=diag([ones(1,6),ell,ell]);toPhysical=E\T;
            C=zeros(3,8);C(1,1)=1;C(2,4)=1;C(3,7)=1;Cp=C(1:2,:);
            matched=struct('P',h.P,'T',toPhysical,'theta',theta,'scalingExponents',[1;2;3;1;2;3;1;2], ...
                'kappa',h.kappaL/theta,'baselineFullStateCertified',false,'metricSource',"Intrinsic eight-state LMI");
            runtime=struct('framework',"invariant",'kind',cfg.kind,'P',h.P,'theta',theta,'toPhysical',toPhysical, ...
                'auxiliaryWeights',max(0,h.auxiliaryWeights(:)),'gyroGainNormalized',h.gyroGainNormalized, ...
                'speedGainNormalized',h.speedGainNormalized, ...
                'LG',toPhysical*h.kappaG*(h.P\Cp.'), ...
                'lidarMatched',matched,'continuousCertificate',struct('design',h,'verification',v), ...
                'fullContinuousStateCertified',false,'surrogateContinuousLmiVerified',v.certified, ...
                'jointSensorLmiVerified',v.jointSensorLmiVerified, ...
                'designRoute',"Common L/G/M vector-increment LMIs with a computed conditional ISS region", ...
                'baselineFullStateCertified',false,'actualOverlapCertified',false,'sampledSystemCertified',false,'scope',v.scope);
        end
    end
end

function m=model(cfg,storedGyro,storedSpeed)
    z=cfg.invariantObserver;s=cfg.iss;ell=z.headingLengthM;sigma=z.sigma;n=z.noise;
    r=z.redundancy;
    validateattributes(z.minimumGnssWeight,{'double'},{'real','finite','positive','scalar','<=',1});
    validateattributes(r.minimumGnssSpeed,{'double'},{'real','finite','positive','scalar','<=',s.maximumSpeed});
    validateattributes(r.speedPartitions,{'double'},{'finite','integer','positive','scalar'});
    sensorRedundancySupport.courseControls(cfg.maximumTrackAngleRate,r.coursePartitions);
    validateattributes(z.regionalRadiusCap,{'double'},{'real','finite','positive','scalar'});
    validateattributes([r.informativeLidarMismatch,r.degradedLidarMismatch],{'double'},{'finite','nonnegative'});
    assert(r.accelerationSplit>0 && r.accelerationSplit<1 && r.gnssSplit>0 && r.gnssSplit<1 ...
        && r.lambda==z.lambda,'VehicleLocalization:InvalidRedundancyDomain','Require valid common theory bounds and Young splits.');
    m.gnssAdequacyEnabled=z.velocityAvailable && z.accelerationAvailable;
    if isfield(z,'headingGainLimits')
        validateattributes(z.headingGainLimits,{'double'},{'real','finite','positive','numel',2});
        assert(z.angularMetricDiagonal,'VehicleLocalization:FeedbackAuthorityMetric','Heading caps require the declared diagonal angular metric.');
    end
    if isfield(z,'matchedGainLimits')
        validateattributes(z.matchedGainLimits,{'double'},{'real','finite','positive','numel',3});
    end
    assert(z.noiseObjective=="normalizedFullState" && isequal(cfg.lidar.poseScales(:),[1;1;1/ell]), ...
        'VehicleLocalization:InvalidInvariantDesign','Require the declared full-state objective and consistent pose scaling.');
    if nargin<2,storedGyro=gyroDesign(z);end
    ratio=z.yawAccelerationRadps2/z.noise.gyroRadps;g=storedGyro.physicalGain;q=storedGyro.normalizedVarianceProxy;
    check=[q,ratio,g;ratio,2*g,0;g,0,2*g];
    assert(g>0 && min(eig(check))>=-1e-6*max(1,norm(check,2)), ...
        'VehicleLocalization:InvalidInvariantDesign','Stored scalar gyro LMI does not match current noise assumptions.');
    m.gyroDesign=storedGyro;m.gyroGainNormalized=g/z.sigma;
    speedParameters=z;speedParameters.yawAccelerationRadps2=2*s.maximumAcceleration;speedParameters.noise.gyroRadps=n.velocityMps;
    if nargin<3,storedSpeed=gyroDesign(speedParameters);end
    ratio=2*s.maximumAcceleration/n.velocityMps;g=storedSpeed.physicalGain;q=storedSpeed.normalizedVarianceProxy;
    check=[q,ratio,g;ratio,2*g,0;g,0,2*g];
    assert(g>0 && min(eig(check))>=-1e-6*max(1,norm(check,2)), ...
        'VehicleLocalization:InvalidInvariantDesign','Stored scalar speed LMI does not match current assumptions.');
    m.speedDesign=storedSpeed;m.speedGainNormalized=g/z.sigma;
    m.A=zeros(8);m.A(1,2)=1;m.A(2,3)=1;m.A(4,5)=1;m.A(5,6)=1;m.A(7,8)=1;
    m.C=zeros(3,8);m.C(1,1)=1;m.C(2,4)=1;m.C(3,7)=1;m.Cp=m.C(1:2,:);
    m.B=zeros(8,3);m.B(3,1)=1;m.B(6,2)=1;m.B(8,3)=1;m.e7=zeros(8,1);m.e7(7)=1;m.e8=zeros(8,1);m.e8(8)=1;
    m.e1=zeros(8,1);m.e1(1)=1;m.e2=zeros(8,1);m.e2(2)=1;m.e5=zeros(8,1);m.e5(5)=1;
    m.DL=diag([n.lidarPositionM,n.lidarPositionM,ell*n.lidarHeadingRad]);m.DG=n.gnssPositionM*eye(2);
    N=invariantIssSupport.measurementSecant([s.maximumSpeed;0],zeros(2,1), ...
        [s.maximumSpeed;0],zeros(2,1),cfg);
    N=N*diag([n.velocityMps,n.velocityMps,n.accelerationMps2,n.accelerationMps2,n.gyroRadps]);
    % Separate budgets for upstream/model constraint residuals, not invented
    % independent measurements of the zero-valued invariant outputs.
    N(:,6:7)=0;N(5,6)=s.maximumSpeed*z.sideslipRateNoiseRadps;N(6,7)=n.velocityMps;
    m.DI=diag(sigma.^-[2,3,3,2,3,2,3,3])*N;
    m.Dd=diag([n.jerkMps3/sigma^3,n.jerkMps3/sigma^3,ell*z.yawAccelerationRadps2/sigma^2]);
    nominal=zeros(8,1);nominal(2)=s.maximumSpeed;
    [~,H]=invariantObserverSupport.outputs(nominal,0,0,cfg);
    E=diag([ones(1,6),ell,ell]);T=diag(sigma.^[1,2,3,1,2,3,1,2]);
    m.Hnominal=diag(sigma.^-[2,3,3,2,3,2,3,3])*H*(E\T);
    available=[z.velocityAvailable,z.velocityAvailable&&z.accelerationAvailable, ...
        z.velocityAvailable&&z.accelerationAvailable,true,z.velocityAvailable,z.velocityAvailable,z.accelerationAvailable,z.accelerationAvailable];
    m.Hnominal(~available,:)=0;
    sw=z.weakPositionStrength;m.points=struct('S',{eye(3),eye(3),diag([sw,sw,1]),zeros(3),diag([sw,sw,1])}, ...
        'w',{0,z.nominalGnssWeight,z.nominalGnssWeight,z.nominalGnssWeight,0},'leak',{0,0,0,0,0});
    [found,index]=ismember(string(z.operatingPoints),["lidar","lidarGnss","weakLidarGnss","gnssOnly","weakLidar"]);
    assert(all(found) && ~isempty(index),'VehicleLocalization:InvalidInvariantDesign','Unknown invariant-noise operating point.');
    m.points=m.points(index);
end

function d=gyroDesign(z)
% Scalar output-feedback LMI: minimize the declared rate-error variance proxy.
    ratio=z.yawAccelerationRadps2/z.noise.gyroRadps;
    assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','Add YALMIP and SeDuMi for scalar gyro synthesis.');
    g=sdpvar(1);q=sdpvar(1);B=[q,ratio,g;ratio,2*g,0;g,0,2*g];
    result=optimize([g>=z.margin,B>=0],q,sdpsettings('solver',char(z.solver),'verbose',0));
    gv=value(g);qv=value(q);check=[qv,ratio,gv;ratio,2*gv,0;gv,0,2*gv];
    assert(gv>0 && min(eig(check))>=-1e-6*max(1,norm(check,2)), ...
        'VehicleLocalization:InfeasibleInvariantDesign','Gyro output LMI failed numerical verification.');
    d=struct('physicalGain',gv,'normalizedVarianceProxy',qv,'minimumLmiEigenvalue',min(eig(check)), ...
        'analyticOptimum',ratio,'solverStatus',result.problem);
end

function v=feedbackAuthority(d,cfg)
    inverse=d.P\eye(8);z=cfg.invariantObserver;
    v=struct('verified',true,'positionL',d.kappaL*inverse(1,1), ...
        'positionG',d.kappaG*inverse(1,1), ...
        'velocityDirection',d.auxiliaryWeights(5)*inverse(5,5), ...
        'headingL',d.kappaL*inverse(7,7),'headingDirection',d.auxiliaryWeights(5)*inverse(7,7));
    tolerance=1e-6;
    if isfield(z,'matchedGainLimits')
        v.verified=v.verified && all([v.positionL,v.positionG,v.velocityDirection]<=z.matchedGainLimits*(1+tolerance));
    end
    if isfield(z,'headingGainLimits')
        v.verified=v.verified && all([v.headingL,v.headingDirection]<=z.headingGainLimits*(1+tolerance));
    end
end
