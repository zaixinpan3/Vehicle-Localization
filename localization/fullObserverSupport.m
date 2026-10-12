classdef fullObserverSupport
% fullObserverSupport Continuous gain certificate and inputs of the full observer.
% Static methods design and verify the continuous seven-state ISS gains, align
% measurement sources on the LiDAR frame clock and move GNSS positions to the
% observer output point. runFullLocalizationObserver is the runtime entry.
% Example: design = fullObserverSupport.designFullObserverGains(cfg).

    methods (Static)
        function design=designFullObserverGains(cfg)
        % designFullObserverGains Certify continuous gains before discretization.
        % The continuous seven-state ISS LMIs include position/heading coupling and
        % motion-output feedback. Numerical integration uses these gains unchanged.
            arguments
                cfg (1,1) struct=fullObserverConfig()
            end
            if isfield(cfg,'hgo')
                design=fullObserverSupport.thetaFrameworkRuntime(cfg);
                return;
            end
            obsolete=intersect(string(fieldnames(cfg.gnss)).',["headingGain","courseWindow", ...
                "maximumCourseGap","minimumCourseDisplacement","minimumSpeed","headingErrorLimit"]);
            assert(isempty(obsolete),'VehicleLocalization:ObsoleteFullConfig', ...
                'GNSS does not correct the heading; remove cfg.gnss fields: %s.',strjoin(obsolete,', '));
            certificate=fullObserverSupport.solveFullObserverIssLmi(cfg);
            assert(certificate.certified,'VehicleLocalization:InfeasibleFullCertificate', ...
                'Candidate gains failed the continuous seven-state ISS LMIs: %s',certificate.solverInfo);
            g=cfg.gains(:);
            matched=struct('P',blkdiag(eye(6),g(1)/g(4)),'T',eye(7),'theta',1, ...
                'scalingExponents',[1;2;3;1;2;3;1],'kappa',g(1), ...
                'baselineFullStateCertified',false, ...
                'metricSource',"Algebraic LiDAR injection metric; the full continuous proof uses continuousCertificate.verification.P");
            source="declared";if isfield(cfg,'gainSource'),source=string(cfg.gainSource);end
            design=struct('kind',cfg.kind,'gains',g.','gnssPositionGain',cfg.gnss.positionGain,'gainSource',source, ...
                'unconditionalIssClaimed',false, ...
                'baselineFullStateCertified',false,'lidarMatched',matched, ...
                'continuousCertificate',certificate,'fullContinuousStateCertified',true, ...
                'designRoute',certificate.designRoute, ...
                'scope',certificate.verification.scope);
        end

        function design=thetaFrameworkRuntime(cfg)
        % thetaFrameworkRuntime Physical gains of a theta-framework design.
        % cfg.hgo holds the normalized solution of the paper's LMI (55) for the
        % error e = [px vx ax py vy ay ell*psi]: P, kappaL, kappaG, Z, theta, ell.
        % K = kappaL P^-1 C', KG = kappaG P^-1 Cp', N = P^-1 Z' and
        %   L = T(theta) K, LG = T(theta) KG, M = T(theta) N / theta^3.
        % The LiDAR pose output uses the matched injection of
        % lidarInjectionSupport with the metric P, T(theta) and kappaL/theta
        % in pose units [m m ell^-1 rad] (cfg.lidar.poseScales = [1;1;1/ell]).
            h=cfg.hgo;theta=h.theta;ell=h.ell;
            assert(isnumeric(theta) && isscalar(theta) && isreal(theta) && isfinite(theta) && theta>=1, ...
                'VehicleLocalization:InvalidThetaDesign','The runtime theta must be a finite scalar >= 1 (paper Eq. 42).');
            validateattributes(ell,{'numeric'},{'real','finite','positive','scalar'});
            P=(h.P+h.P.')/2;assert(isequal(size(P),[7,7]) && min(eig(P))>0,'VehicleLocalization:InvalidThetaDesign', ...
                'The theta-framework metric P must be 7-by-7 positive definite.');
            assert(isequal(cfg.lidar.poseScales(:),[1;1;1/ell]),'VehicleLocalization:InvalidThetaDesign', ...
                'The theta-framework LiDAR channel requires poseScales [1;1;1/ell].');
            Cp=zeros(2,7);Cp(1,1)=1;Cp(2,4)=1;C=[Cp;[zeros(1,6),1]];
            K=h.kappaL*(P\C.');KG=h.kappaG*(P\Cp.');N=P\h.Z.';
            T=diag(theta.^[1 2 3 1 2 3 1]);E=diag([ones(1,6),1/ell]);Ey=diag([1 1 1/ell]);
            physical=struct('theta',theta,'T',T,'L',E*T*K/Ey,'LG',E*T*KG,'M',E*T*N/theta^3);
            assert(all(abs(physical.LG(7,:))<1e-12) && all(abs(physical.M(7,:))<1e-12),'VehicleLocalization:InvalidThetaDesign', ...
                'GNSS and the extra outputs must not correct the heading.');
            matched=struct('P',E\P/E,'T',T,'theta',theta,'scalingExponents',[1;2;3;1;2;3;1],'kappa',h.kappaL/theta, ...
                'baselineFullStateCertified',false,'metricSource',"Lyapunov matrix of the theta-framework LMI (55)");
            persistent cache
            if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
            certified=false;verification=struct();
            if isfield(cfg,'thetaFramework') && all(isfield(h,{'sigma','lambda'}))
                assert(theta>=h.sigma,'VehicleLocalization:InvalidThetaDesign', ...
                    'The runtime theta %g is below the design sigma %g of the vertex set.',theta,h.sigma);
                key=jsonencode(struct('h',rmfield(h,'theta'),'f',cfg.thetaFramework,'iss',cfg.iss,'q',cfg.maximumTrackAngleRate, ...
                    'o',cfg.gnss.outputPoint.bodyOffset,'l',cfg.lidar.gainInformationScale));
                if ~isKey(cache,key),cache(key)=fullObserverSupport.verifyThetaFrameworkDesign(h,cfg);end
                verification=cache(key);certified=verification.certified;
                assert(certified,'VehicleLocalization:InfeasibleThetaDesign', ...
                    'The theta-framework design failed LMI (55) on the declared vertex set (worst eigenvalue %.3g).', ...
                    verification.worstVertexEigenvalue);
            end
            source="cfg.hgo";if isfield(cfg,'gainSource'),source=string(cfg.gainSource);end
            design=struct('kind',cfg.kind,'framework',"theta",'theta',theta,'hgo',physical, ...
                'gains',[physical.L(1,1),physical.L(2,1),physical.L(3,1),physical.L(7,3)], ...
                'gnssPositionGain',physical.LG(1,1),'gainSource',source,'unconditionalIssClaimed',false, ...
                'baselineFullStateCertified',false,'lidarMatched',matched, ...
                'continuousCertificate',struct('design',h,'verification',verification),'fullContinuousStateCertified',certified, ...
                'designRoute',"Theorem 5 of Bessafa et al. (2026): K and N from LMI (55), L = T(theta)K, M = T(theta)N/theta^3", ...
                'scope',"Continuous ISS of the theta-framework observer on the declared vertex set; no sampled-system or outage guarantee");
        end

        function design=synthesizeThetaFrameworkDesign(cfg)
        % synthesizeThetaFrameworkDesign Solve K and N from the source paper's LMI (55).
        % Bessafa et al. (Automatica 188, 2026), Theorem 5, for the normalized
        % error e = [px vx ax py vy ay ell*psi] of the seven-state observer
        % (ell = cfg.thetaFramework.headingLengthM) and eps = T(theta)^-1 e,
        % T = diag(theta^[1 2 3 1 2 3 1]):
        %   L  = T K,  K  = P^-1 Y',  Y = kappaL*C   LiDAR pose output [p; ell*psi]
        %   LG = T KG, KG = kappaG P^-1 Cp'          GNSS position output
        %   M  = T N / theta^3, N = P^-1 Z'          extra outputs: body velocity and
        %                                            acceleration rotated by the heading
        % P = blkdiag(Pc,Pc,ppsi) is rotation invariant. The matched main outputs
        % dissipate for every LiDAR information weight S and GNSS weight Wg. LMI
        % (55) holds on the vertex set V^sigma of the paper: the extra-output unit
        % Jacobians at {0,1} (the main outputs alone stabilize the error, Remark
        % 6), their heading couplings (|phi_v| <= vmax, |phi_a| <= amax), the
        % course-rate term of the jerk model (linear in the error for the measured
        % rate |q| <= qmax) through its scaled coefficients (q^2/theta^2, 2q/theta),
        % the LiDAR information bounds of cfg.iss and two GNSS scenarios (lever arm
        % by Young's inequality). Because the course-rate term enters exactly, the
        % Lipschitz part of condition (56) vanishes and theta >= sigma suffices.
        % The normalization is that of the paper's reproduction (lambda = 1,
        % P >= I). The paper fixes no objective: K and N minimize the summed
        % mean-square position and heading error bound trace(G'PG) at the
        % declared operating points for the declared noise scales: LiDAR, LiDAR
        % with GNSS, weak LiDAR translation with GNSS, and dead reckoning
        % (neither LiDAR nor GNSS, position over a declared horizon). The last
        % point is what makes the extra outputs carry velocity and acceleration
        % through outages. No trajectory or reference data enters. Requires
        % YALMIP and the configured solver.
            arguments
                cfg (1,1) struct
            end
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver', ...
                'Add YALMIP and the SDP solver to the path before the theta-framework synthesis.');
            m=thetaFrameworkModel(cfg);z=cfg.thetaFramework;sigma=m.sigma;
            yalmip('clear');
            Pc=sdpvar(3,3);ppsi=sdpvar(1);P=blkdiag(Pc,Pc,ppsi);
            kL=sdpvar(1);kG=sdpvar(1);zv=sdpvar(1,3);za=sdpvar(1,3);
            Z=[zv,zeros(1,3),0;zeros(1,3),zv,0;za,zeros(1,3),0;zeros(1,3),za,0];
            constraints=[P>=eye(7),kL>=0,kG>=0];
            for v=1:numel(m.vertices)
                constraints=[constraints,thetaFrameworkBlock(P,kL,kG,Z,m.vertices(v),m)+z.lambda*eye(7)<=-z.margin*eye(7)]; %#ok<AGROW>
            end
            cost=0;
            for k=1:numel(m.points)
                s=m.points(k);x=s.states;
                A=P*(m.A-s.leak*(m.Cp.'*m.Cp))-kL*m.CL.'*s.S*m.CL-kG*s.w*(m.Cp.'*m.Cp)-Z.'*m.Hnominal;A=A(x,x);Cz=s.Cz(:,x);
                PG=[-kL*m.CL.'*s.S*s.DL,-kG*s.w*m.Cp.'*m.DG,-Z.'*m.Dz/sigma^3,P*m.B*m.Dd/sigma^3,-P*m.e7*m.Dr/sigma];
                PG=PG(x,s.noise);Q=sdpvar(numel(s.noise),numel(s.noise));
                constraints=[constraints,A+A.'+sigma*(Cz.'*Cz)<=0,[Q,PG.';PG,P(x,x)]>=0]; %#ok<AGROW>
                cost=cost+trace(Q);
            end
            result=optimize(constraints,cost,sdpsettings('solver',char(z.solver),'verbose',0));
            assert(result.problem==0 || result.problem==4,'VehicleLocalization:InfeasibleThetaDesign', ...
                'Theta-framework synthesis failed: %s',result.info);
            Pv=value(P);Pv=(Pv+Pv.')/2;
            design=struct('sigma',sigma,'P',Pv,'kappaL',value(kL),'kappaG',value(kG),'Z',value(Z), ...
                'lambda',z.lambda,'meanSquareBound',value(cost),'solverStatus',result.problem,'solverInfo',string(result.info));
            design.verification=fullObserverSupport.verifyThetaFrameworkDesign(design,cfg);
            assert(design.verification.certified,'VehicleLocalization:InfeasibleThetaDesign', ...
                'The solved theta-framework design failed its independent vertex check.');
        end

        function inputs=thetaFrameworkInputs(cfg)
        % thetaFrameworkInputs Every declared input the theta-framework design depends on.
            s=cfg.iss;offset=zeros(2,1);
            if isfield(cfg.gnss,'outputPoint'),offset=cfg.gnss.outputPoint.bodyOffset(:);end
            design=cfg.thetaFramework;if isfield(design,'provenance'),design=rmfield(design,'provenance');end
            inputs=jsondecode(jsonencode(struct('thetaFramework',design, ...
                'domain',struct('maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
                    'minimumPositionStrength',s.minimumPositionStrength,'minimumHeadingStrength',s.minimumHeadingStrength, ...
                    'maximumPositionHeadingCoupling',s.maximumPositionHeadingCoupling,'maximumCourseRate',cfg.maximumTrackAngleRate), ...
                'gnssBodyOffset',offset.','lidarInformationScale',cfg.lidar.gainInformationScale)));
        end

        function verification=verifyThetaFrameworkDesign(design,cfg)
        % verifyThetaFrameworkDesign Recheck LMI (55) of a theta-framework design without a solver.
        % Every vertex of V^sigma is evaluated by eigenvalues; the certificate then
        % holds for every runtime theta >= sigma (inclusion 54 of the paper) with
        % V' <= -theta*lambda*|eps|^2 + noise terms (lambda = 1, P >= I).
        % Physical bounds, information availability and the local heading chart
        % remain assumptions; the sampled system is not certified.
            arguments
                design (1,1) struct
                cfg (1,1) struct
            end
            m=thetaFrameworkModel(cfg);P=(design.P+design.P.')/2;
            assert(isequal(size(P),[7,7]) && all(isfinite(P),'all') && isequal(size(design.Z),[4,7]), ...
                'VehicleLocalization:InvalidThetaDesign','Malformed theta-framework design.');
            worst=-Inf;
            for v=1:numel(m.vertices)
                B=thetaFrameworkBlock(P,design.kappaL,design.kappaG,design.Z,m.vertices(v),m)+design.lambda*eye(7);
                worst=max(worst,max(eig((B+B.')/2)));
            end
            Z=design.Z;
            structure=norm(P([1:3,7],4:6),'fro')+norm(P(1:3,1:3)-P(4:6,4:6),'fro')+norm(P(7,1:3))+ ...
                norm(Z(:,7))+norm(Z([1,3],4:6),'fro')+norm(Z([2,4],1:3),'fro')+norm(Z(1,1:3)-Z(2,4:6))+norm(Z(3,1:3)-Z(4,4:6));
            verification=struct('certified',worst<0 && min(eig(P))>=1-1e-6 && design.kappaL>=0 && design.kappaG>=0 ...
                && structure<=1e-9*max(1,norm(P,'fro')) && design.sigma==m.sigma, ...
                'worstVertexEigenvalue',worst,'vertices',numel(m.vertices),'minimumEigenvalueP',min(eig(P)), ...
                'maximumEigenvalueP',max(eig(P)),'lipschitzConstant',0,'sigma',m.sigma, ...
                'decayInMetricPerTheta',design.lambda/max(eig(P)), ...
                'scope',"LMI (55) on the vertex set V^sigma; valid for every theta >= sigma. Continuous time; the sampled observer is not certified.");
        end

        function certificate=solveFullObserverIssLmi(cfg)
        % solveFullObserverIssLmi Test candidate gains in the continuous ISS LMIs.
        % Solve before discretization. Outer parameter selection fixes the gains;
        % this inner SDP finds a common Lyapunov certificate. An infeasible result is
        % not a proof of instability.
        % Requires YALMIP and the configured SDP solver on the MATLAB path.
            arguments
                cfg (1,1) struct
            end
            problem=fullObserverSupport.fullObserverIssProblem(cfg);
            persistent cache
            if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
            key=jsonencode(struct('problem',problem,'solver',cfg.iss.solver));
            if isKey(cache,key),certificate=cache(key);return;end
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver', ...
                'Add YALMIP and the SDP solver to the path before continuous gain design.');
            % Homogeneous weights with fixed trace avoid the poorly conditioned
            % large-yaw-weight solutions produced by fixing the velocity weight to 1.
            x=sdpvar(4,1);slack=sdpvar(1);cp=x(1);cy=x(2);cv=x(3);mu=x(4);g=problem.gains;
            W=diag(x);constraints=[x>=1e-9,sum(x)==1];
            for q2=[0,problem.maximumCourseRate^2]
                a=cp*g(1);b=cy*g(4);cross=-(a+b)*problem.maximumPositionHeadingCoupling;
                M=[2*a*problem.minimumPositionStrength,cross,-cp,0; ...
                    cross,2*b*problem.minimumHeadingStrength-cp*problem.gnssLeverPenalty, ...
                    -cv*g(2)*problem.maximumSpeed,-mu*g(3)*problem.maximumAcceleration; ...
                    -cp,-cv*g(2)*problem.maximumSpeed,2*cv*g(2),-(cv+mu*q2); ...
                    0,-mu*g(3)*problem.maximumAcceleration,-(cv+mu*q2),2*mu*g(3)];
                constraints=[constraints,M-problem.decayRate*W>=slack*eye(4)]; %#ok<AGROW>
            end
            result=optimize(constraints,-slack,sdpsettings('solver',char(cfg.iss.solver),'verbose',0));
            recovered=value(x);
            certificate=struct('problem',problem,'weights',recovered([1,2,4])/recovered(3),'solverStatus',result.problem, ...
                'solverInfo',string(result.info),'solverSlack',value(slack),'certified',false, ...
                'designRoute',"Continuous ISS LMI, then discretization with unchanged gains");
            % A solver warning is not a mathematical infeasibility result. Accept a
            % returned witness only through the independent, strictly positive LMI
            % margin, and retain the solver status even when that witness is valid.
            if all(isfinite(certificate.weights)) && all(certificate.weights>0)
                certificate.verification=fullObserverSupport.verifyFullObserverIssCertificate(certificate,cfg);
                certificate.certified=certificate.verification.certified;
            end
            if certificate.certified,cache(key)=certificate;end
        end

        function verification=verifyFullObserverIssCertificate(certificate,cfg)
        % verifyFullObserverIssCertificate Recheck the continuous LMIs without a solver.
        % P certifies the complete continuous observer. It is distinct from the
        % algebraic metric used to implement the LiDAR injection. Physical bounds,
        % information availability and the local heading chart remain assumptions.
            arguments
                certificate (1,1) struct
                cfg (1,1) struct
            end
            p=fullObserverSupport.fullObserverIssProblem(cfg);
            assert(isfield(certificate,'problem') && isequaln(certificate.problem,p), ...
                'VehicleLocalization:StaleFullIssCertificate', ...
                'Re-solve the continuous LMIs after changing gains or design assumptions.');
            w=certificate.weights(:);
            validateattributes(w,{'numeric'},{'real','finite','positive','numel',3});
            cp=w(1);cy=w(2);mu=w(3);g=p.gains;
            W=diag([cp,cy,1,mu]);root=sqrt(diag(W));
            matrices=zeros(4,4,2);margins=zeros(2,1);rates=zeros(2,1);
            for vertex=1:2
                q2=(vertex-1)*p.maximumCourseRate^2;
                a=cp*g(1);b=cy*g(4);cross=-(a+b)*p.maximumPositionHeadingCoupling;
                M=[2*a*p.minimumPositionStrength,cross,-cp,0; ...
                    cross,2*b*p.minimumHeadingStrength-cp*p.gnssLeverPenalty, ...
                    -g(2)*p.maximumSpeed,-mu*g(3)*p.maximumAcceleration; ...
                    -cp,-g(2)*p.maximumSpeed,2*g(2),-(1+mu*q2); ...
                    0,-mu*g(3)*p.maximumAcceleration,-(1+mu*q2),2*mu*g(3)];
                matrices(:,:,vertex)=M;
                margins(vertex)=min(eig((M-p.decayRate*W)./(root*root.')));
                rates(vertex)=min(eig(M./(root*root.')));
            end
            P=diag([cp,1,mu,cp,1,mu,cy]);
            verification=struct('certified',all(margins>p.tolerance), ...
                'weights',w,'P',P,'comparisonMatrices',matrices,'comparisonWeight',W, ...
                'normalizedMargins',margins,'verifiedDecayRate',min(rates), ...
                'requestedDecayRate',p.decayRate,'continuousTime',true, ...
                'sampledSystemCertified',false, ...
                'scope',"Seven-state continuous ISS under the declared maneuver and uniformly sufficient LiDAR information bounds; estimated-heading GNSS lever arm included. No arbitrary-outage or numerical-integration guarantee.");
        end

        function problem=fullObserverIssProblem(cfg)
        % fullObserverIssProblem Declare the continuous seven-state ISS assumptions.
        % A fixed candidate gain makes the two comparison inequalities LMIs in the
        % Lyapunov weights. Timing and integration method do not enter this problem.
            arguments
                cfg (1,1) struct
            end
            assert(~isfield(cfg,'timing') || isequal(string(cfg.timing),"synchronous"), ...
                'VehicleLocalization:ObsoleteFullConfig', ...
                'Only synchronized localization is supported; the historical transported runtime was removed.');
            assert(isfield(cfg,'iss'),'VehicleLocalization:MissingFullIssDomain', ...
                'Declare the continuous ISS domain in cfg.iss before selecting gains.');
            s=cfg.iss;g=cfg.gains(:).';
            validateattributes(g,{'numeric'},{'real','finite','positive','numel',4});
            validateattributes(cfg.maximumTrackAngleRate,{'numeric'},{'real','finite','nonnegative','scalar'});
            validateattributes(cfg.gnss.positionGain,{'numeric'},{'real','finite','nonnegative','scalar'});
            scale=cfg.gnss.gainInformationScale;
            assert(isnumeric(scale) && isscalar(scale) && isreal(scale) && isfinite(scale) && scale>0, ...
                'VehicleLocalization:InvalidGnssInformationScale', ...
                'A positive GNSS regularizer is required for the certified weight bound.');
            validateattributes([s.maximumSpeed,s.maximumAcceleration,s.maximumPositionHeadingCoupling], ...
                {'numeric'},{'real','finite','nonnegative','numel',3});
            validateattributes([s.minimumPositionStrength,s.minimumHeadingStrength], ...
                {'numeric'},{'real','finite','positive','<',1,'numel',2});
            validateattributes([s.decayRate,s.tolerance],{'numeric'},{'real','finite','positive','numel',2});
            assert(isequal(cfg.lidar.poseScales(:),ones(3,1)), ...
                'VehicleLocalization:UnsupportedFullIssScaling','The continuous comparison uses metres and radians without pose rescaling.');
            lidarInjectionSupport.filterLidarPoseInformation(zeros(3),cfg);
            offset=zeros(2,1);
            if isfield(cfg.gnss,'outputPoint'),offset=cfg.gnss.outputPoint.bodyOffset(:);end
            validateattributes(offset,{'numeric'},{'real','finite','numel',2});
            problem=struct('gains',g,'maximumCourseRate',cfg.maximumTrackAngleRate, ...
                'maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
                'minimumPositionStrength',s.minimumPositionStrength, ...
                'minimumHeadingStrength',s.minimumHeadingStrength, ...
                'maximumPositionHeadingCoupling',s.maximumPositionHeadingCoupling, ...
                'decayRate',s.decayRate,'gnssLeverPenalty',.5*cfg.gnss.positionGain*sum(offset.^2), ...
                'gnssPositionGain',cfg.gnss.positionGain,'gnssBodyOffset',offset, ...
                'gnssInformationScale',cfg.gnss.gainInformationScale, ...
                'tolerance',s.tolerance);
            if isfield(cfg.lidar,'errorCalibration')
                problem.lidarErrorCalibration=cfg.lidar.errorCalibration;
            else
                problem.lidarInformationScale=cfg.lidar.gainInformationScale;
            end
        end

        function [aligned,lateral,metadata]=synchronizeLocalizationInputs(data,lateralInput,cfg)
        % synchronizeLocalizationInputs Align existing observations on LiDAR frames.
        % Offline bracket interpolation uses real endpoints only, without motion
        % extrapolation. Invalid endpoints or excessive gaps withdraw that channel.
        % The future-endpoint wait is reported separately from perception delay.
            arguments
                data (1,1) struct
                lateralInput (1,1) struct
                cfg (1,1) struct=fullObserverConfig()
            end
            h=data.highRate;overlapChannel=isfield(data,'lidarOverlap');
            if overlapChannel
                assert(~isfield(data,'lidar'),'VehicleLocalization:AmbiguousLidarInput', ...
                    'Supply one LiDAR channel when synchronizing inputs.');
                t=data.lidarOverlap.time(:);
            else
                t=data.lidar.time(:);
            end
            assert(all(diff(t)>0) && all(isfinite(t)),'VehicleLocalization:InvalidSyncClock','Invalid LiDAR clock.');
            selected=t>=h.time(1) & t<=h.time(end);t=t(selected);
            assert(numel(t)>=2,'VehicleLocalization:SyncCoverage','Need two covered localization frames.');
            [left,right,fraction,covered]=brackets(h.time,t,cfg.synchronization.motionMaximumBracket);
            assert(all(covered),'VehicleLocalization:SyncCoverage','Motion does not cover the frame clock.');
            aligned=struct('highRate',struct('time',t));
            for name=string(fieldnames(h)).'
                if name=="time",continue;end
                aligned.highRate.(name)=blend(h.(name),left,right,fraction);
            end
            assert(isequal(lateralInput.time,h.time),'VehicleLocalization:SyncCoverage','Lateral and motion clocks differ.');
            lateral=struct('time',t);
            for name=["lateralVelocity","sideSlipAngleRate"]
                lateral.(name)=blend(lateralInput.(name),left,right,fraction);
            end
            if overlapChannel
                aligned.lidarOverlap=data.lidarOverlap;aligned.lidarOverlap.time=t;
                aligned.lidarOverlap.sources=data.lidarOverlap.sources(selected);
                if isfield(data.lidarOverlap,'valid'),aligned.lidarOverlap.valid=data.lidarOverlap.valid(selected);end
            else
                aligned.lidar=data.lidar;aligned.lidar.time=t;
                aligned.lidar.pose=data.lidar.pose(selected,:);
                aligned.lidar.valid=data.lidar.valid(selected);
                aligned.lidar.information=data.lidar.information(:,:,selected);
                for name=["frameReliability","directionReliability"]
                    if isfield(data.lidar,name),aligned.lidar.(name)=data.lidar.(name)(selected,:);end
                end
                if isfield(data.lidar,'evaluateResidual')
                    originalIndex=find(selected);provider=data.lidar.evaluateResidual;
                    aligned.lidar.evaluateResidual=@(k,pose)provider(originalIndex(k),pose);
                end
                if isfield(data.lidar,'residualModels'),aligned.lidar.residualModels=data.lidar.residualModels(selected);end
            end
            motionWait=h.time(right)-t;gnssWait=zeros(size(t));gnssBracket=zeros(size(t));
            if isfield(data,'gnss')
                g=data.gnss;
                [a,b,w,ok]=brackets(g.time,t,cfg.synchronization.gnssMaximumBracket);
                ok=ok & g.valid(a) & g.valid(b);
                position=nan(numel(t),2);information=nan(2,2,numel(t));
                for k=find(ok).'
                    position(k,:)=(1-w(k))*g.position(a(k),:)+w(k)*g.position(b(k),:);
                    information(:,:,k)=(1-w(k))*g.information(:,:,a(k))+w(k)*g.information(:,:,b(k));
                end
                gnssWait(ok)=g.time(b(ok))-t(ok);gnssBracket(ok)=g.time(b(ok))-g.time(a(ok));
                aligned.gnss=struct('time',t,'position',position,'information',information,'valid',ok,'delay',0, ...
                    'sourceLeftTime',g.time(a),'sourceRightTime',g.time(b));
            end
            metadata=struct('clock',"Native LiDAR frames; no synthetic inter-frame localization outputs", ...
                'nominalRateHz',1/median(diff(t)),'frames',numel(t), ...
                'motionModelMeasurementExtrapolation',false,'offlineBracketInterpolation',true, ...
                'maximumGnssBracketSeconds',max(gnssBracket),'maximumGnssWaitSeconds',max(gnssWait), ...
                'maximumMotionWaitSeconds',max(motionWait),'frameWaitSeconds',max(motionWait,gnssWait), ...
                'inputScope',"Wheel/lateral estimates retain their native preprocessing; only aligned frame values enter localization");
        end

        function [position,information]=correctGnssOutputPoint(position,information,yaw,alignment)
        % correctGnssOutputPoint Express a receiver position at the observer point.
        % bodyOffset is receiver minus observer point, in forward/left coordinates.
        % An empirical output-point calibration is not a surveyed installation lever.
        % Its covariance and declared heading uncertainty are conservative additions;
        % unreported correlations with the receiver solution remain unknown.
            offset=alignment.bodyOffset(:);C=alignment.bodyCovariance;
            assert(numel(offset)==2 && all(isfinite(offset)) && isequal(size(C),[2,2]) && ...
                all(isfinite(C),'all') && norm(C-C.','fro')<1e-12 && min(eig(C))>=0 && ...
                isscalar(alignment.headingStdRad) && isfinite(alignment.headingStdRad) && alignment.headingStdRad>=0, ...
                'VehicleLocalization:InvalidGnssAlignment','Invalid output-point calibration.');
            if all(offset==0) && all(C==0,'all'),return;end
            R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
            position=position-(R*offset).';
            covariance=information\eye(2)+R*C*R.'+alignment.headingStdRad^2*(R*J*offset)*(R*J*offset).';
            information=covariance\eye(2);information=(information+information.')/2;
        end
    end
end

function [a,b,w,ok]=brackets(source,target,maximum)
    source=source(:);assert(numel(source)>=2 && all(isfinite(source)) && all(diff(source)>0), ...
        'VehicleLocalization:InvalidSyncClock','Need an increasing finite source clock.');
    a=ones(size(target));b=a;w=zeros(size(target));ok=false(size(target));j=1;
    for k=1:numel(target)
        while j<numel(source) && source(j+1)<=target(k),j=j+1;end
        a(k)=j;b(k)=min(j+1,numel(source));
        if source(j)==target(k),b(k)=j;ok(k)=true;
        elseif source(j)<target(k) && b(k)>j && source(b(k))>=target(k) && source(b(k))-source(j)<=maximum
            w(k)=(target(k)-source(j))/(source(b(k))-source(j));ok(k)=true;
        end
    end
end

function result=blend(values,a,b,w)
    result=(1-w).*values(a,:)+w.*values(b,:);
end

function m=thetaFrameworkModel(cfg)
% thetaFrameworkModel Matrices, vertex set and operating points of the
% theta-framework LMI for the declared inputs of cfg.thetaFramework.
    z=cfg.thetaFramework;s=cfg.iss;n=z.noise;ell=z.headingLengthM;sigma=z.sigma;
    validateattributes(sigma,{'numeric'},{'real','finite','scalar','>=',1});
    validateattributes(ell,{'numeric'},{'real','finite','positive','scalar'});
    validateattributes([n.lidarPositionM,n.lidarHeadingRad,n.gnssPositionM,n.velocityMps,n.accelerationMps2,n.gyroRadps,n.jerkMps3], ...
        {'numeric'},{'real','finite','positive','numel',7});
    m.sigma=sigma;m.A=zeros(7);m.A(1,2)=1;m.A(2,3)=1;m.A(4,5)=1;m.A(5,6)=1;
    m.Cp=zeros(2,7);m.Cp(1,1)=1;m.Cp(2,4)=1;m.CL=[m.Cp;[zeros(1,6),1]];
    m.Hv=zeros(2,7);m.Hv(1,2)=1;m.Hv(2,5)=1;m.Ha=zeros(2,7);m.Ha(1,3)=1;m.Ha(2,6)=1;
    m.B=zeros(7,2);m.B(3,1)=1;m.B(6,2)=1;m.e7=[zeros(6,1);1];m.ell=ell;
    m.Hnominal=[sigma^-2*m.Hv;sigma^-1*m.Ha];
    m.DG=n.gnssPositionM*eye(2);m.Dz=blkdiag(n.velocityMps*eye(2),n.accelerationMps2*eye(2));
    m.Dd=n.jerkMps3*eye(2);m.Dr=ell*n.gyroRadps;
    q=cfg.maximumTrackAngleRate;offset=zeros(2,1);
    if isfield(cfg.gnss,'outputPoint'),offset=cfg.gnss.outputPoint.bodyOffset(:);end
    rho=norm(offset)/ell;tau=z.gnssCouplingSplit;
    % Operating points of the mean-square objective. A weak LiDAR direction
    % injects the information-consistent sqrt(s(1-s)/lambda*) of the filter.
    % Without LiDAR the heading is unobservable and decoupled from the position
    % output, so that point is posed on the chain states without the gyro noise.
    sw=z.weakPositionStrength;weak=sqrt(sw*(1-sw)/cfg.lidar.gainInformationScale);
    DL=diag([n.lidarPositionM,n.lidarPositionM,ell*n.lidarHeadingRad]);
    % Dead reckoning (neither LiDAR nor GNSS): the position is an integrator,
    % so its mean-square error is taken over the declared horizon through a
    % leak 1/horizon on the position of that operating point only.
    leak=1/z.deadReckoningHorizonS;
    library=struct('name',{"lidar","lidarGnss","weakLidarGnss","gnssOnly","deadReckoning"}, ...
        'S',{eye(3),eye(3),diag([sw sw 1]),zeros(3),zeros(3)}, ...
        'w',{0,z.nominalGnssWeight,z.nominalGnssWeight,z.nominalGnssWeight,0}, ...
        'DL',{DL,DL,diag([weak,weak,ell*n.lidarHeadingRad]),DL,DL},'Cz',{m.CL,m.CL,m.CL,[m.Cp;zeros(1,7)],[m.Cp;zeros(1,7)]}, ...
        'states',{1:7,1:7,1:7,1:6,1:6},'noise',{1:12,1:12,1:12,1:11,6:11},'leak',{0,0,0,0,leak});
    [found,index]=ismember(string(z.operatingPoints),[library.name]);
    assert(all(found) && ~isempty(index),'VehicleLocalization:InvalidThetaDesign','Unknown operating point.');
    m.points=library(index);
    % Vertex set V^sigma. Rotation invariance puts the velocity coupling on the
    % first axis; reflection invariance restricts the course rate to q >= 0,
    % covered by the triangle (q^2,q) in {(0,0),(0,qmax),(qmax^2,qmax)}.
    corners=[1 1;1 -1;-1 1;-1 -1];
    scenarios=struct('sp',{s.minimumPositionStrength,sw},'dissipation',{0,(1-tau)*z.minimumGnssWeight}, ...
        'penalty',{rho^2,rho^2/tau});
    course=[0 0;0 q;q^2 q];
    m.vertices=struct('sV',{},'sA',{},'u',{},'w',{},'phiV',{},'phiA',{},'sp',{},'spsi',{},'sh',{}, ...
        'gnssDissipation',{},'gnssHeadingPenalty',{});
    for k=1:numel(scenarios)
        for sv=[0 1]
            for sa=[0 1]
                for f=1:size(course,1)
                    for pv=[0,s.maximumSpeed]
                        for ia=1:4
                            for ic=1:4
                                m.vertices(end+1)=struct('sV',sv,'sA',sa,'u',course(f,1),'w',course(f,2), ...
                                    'phiV',[pv;0],'phiA',s.maximumAcceleration*corners(ia,:).','sp',scenarios(k).sp, ...
                                    'spsi',s.maximumPositionHeadingCoupling*corners(ic,:).','sh',s.minimumHeadingStrength, ...
                                    'gnssDissipation',scenarios(k).dissipation,'gnssHeadingPenalty',scenarios(k).penalty); %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
    end
end

function block=thetaFrameworkBlock(P,kL,kG,Z,v,m)
% thetaFrameworkBlock He{P(A+B F) - Z' H} with the matched main outputs at one vertex.
    sigma=m.sigma;ell=m.ell;
    H=[v.sV*sigma^-2*m.Hv-sigma^-3*(v.phiV/ell)*m.e7.';v.sA*sigma^-1*m.Ha-sigma^-3*(v.phiA/ell)*m.e7.'];
    S=[v.sp*eye(2),v.spsi(:);v.spsi(:).',v.sh];
    F=zeros(2,7);F(1,2)=v.u/sigma^2;F(2,5)=v.u/sigma^2;F(:,[3,6])=2*v.w/sigma*[0,-1;1,0];
    M=P*(m.A+m.B*F)-Z.'*H;
    block=M+M.'-kL*m.CL.'*S*m.CL-kG*v.gnssDissipation*(m.Cp.'*m.Cp)+kG*v.gnssHeadingPenalty*(m.e7*m.e7.');
end
