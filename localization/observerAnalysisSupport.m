classdef observerAnalysisSupport
% observerAnalysisSupport Design, certificates and inputs of the analysis observers.
% Static methods serve the continuous MO-HGO analysis runner
% runImprovedVehicleObserver and the historical motion-aided runner
% runMotionAidedVehicleObserver: gain synthesis, constant-matrix certificate
% checks, channel evaluation, scenario simulation and continuous signal
% reconstruction. Example: design = observerAnalysisSupport.improvedObserverReferenceDesign(cfg).

    methods (Static)
        function design = designImprovedObserverGains(cfg)
        % designImprovedObserverGains Construct gains and a constant ISS certificate.
        % GNSS uses observable chains and the triangular yaw injection. LiDAR retains
        % the reference K/N and synthesizes P/Q/R for the declared delay and norm box.
        % Simultaneous gain/functional optimization is not claimed to be convex.
            arguments
                cfg (1,1) struct = improvedObserverConfig()
            end
            data=observerAnalysisSupport.buildImprovedObserverCertificateData(cfg);
            root=fileparts(fileparts(mfilename('fullpath')));
            stored=jsondecode(fileread(fullfile(root,'config','continuousObserverCertificate.json')));
            source=stored.(cfg.mode);
            if cfg.mode=="lidar" && isfield(cfg.observer,'lidarGainProfile')
                assert(any(string(cfg.observer.lidarGainProfile)==["tracking","mncav"]), ...
                    'VehicleLocalization:UnsupportedObserverProfile','Unknown LiDAR gain profile.');
                filename="lidarTrackingCertificate.json";
                if string(cfg.observer.lidarGainProfile)=="mncav",filename="mncavLidarCertificate.json";end
                source=jsondecode(fileread(fullfile(root,'config',filename)));
            end
            design=struct('kind',"continuous-mo-hgo-v1",'mode',cfg.mode,'theta',cfg.observer.theta);
            if isfield(cfg.synthesis,'certificateMethod')
                design.certificateMethod=cfg.synthesis.certificateMethod;
            end
            if cfg.mode=="gnss"
                if isfield(cfg.observer,'gnssChainGain')
                    gain=cfg.observer.gnssChainGain;
                    assert(isnumeric(gain) && isreal(gain) && numel(gain)==3 ...
                        && all(isfinite(gain)) && all(gain>0), ...
                        'VehicleLocalization:InvalidConfiguration','gnssChainGain must contain three positive finite coefficients.');
                    gain=double(gain(:));
                    source.K=blkdiag(gain,gain);
                end
                design.K=[source.K;zeros(1,2)];design.N=zeros(7,4);
                design.N(1:6,1:3)=source.N;design.N(7,4)=-cfg.observer.yawGain*design.theta^2;
                F=data.A(1:6,1:6)-source.K*data.Cg;identity=eye(6);
                operator=kron(eye(6),F.')+kron(F.',eye(6));
                design.P=reshape(operator\(-identity(:)),6,6);design.P=(design.P+design.P.')/2;
            else
                assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','YALMIP must be on the path.');
                design.K=source.K;design.N=source.N;design.rate=cfg.synthesis.rate;
                delay=cfg.measurement.fixedLidarDelay;theta=cfg.observer.theta;
                [vertices,uncertainty]=observerAnalysisSupport.continuousLidarCertificateVertices(design,cfg);
                Ad=-theta*design.K*data.C;
                yalmip('clear');
                P=sdpvar(7,7,'symmetric');Q=sdpvar(7,7,'symmetric');R=sdpvar(7,7,'symmetric');
                g=sdpvar(1);margin=sdpvar(1);pBound=sdpvar(1);rBound=sdpvar(1);
                constraints=[P>=1e-5*eye(7),Q>=1e-5*eye(7),R>=1e-5*eye(7), ...
                    P<=pBound*eye(7),R<=rBound*eye(7),trace(P)==7, ...
                    trace(Q)+trace(R)<=1000,g>=1e-5,g<=1000,margin>=1e-6];
                for vertex=1:size(vertices,3)
                    block=observerAnalysisSupport.continuousObserverDelayLmi(vertices(:,:,vertex),Ad,P,Q,R,g,delay,design.rate);
                    constraints=[constraints, ...
                        block<=-(margin+2*uncertainty*(pBound+delay*rBound))*eye(28)]; %#ok<AGROW>
                end
                result=optimize(constraints,-margin,sdpsettings('solver',char(cfg.synthesis.solver),'verbose',0));
                assert(result.problem==0,'VehicleLocalization:InfeasibleCertificate', ...
                    'Constant-matrix LiDAR synthesis failed: %s',result.info);
                design.P=double(P);design.Q=double(Q);design.R=double(R);design.g=double(g);
                for name=["P","Q","R"],design.(name)=(design.(name)+design.(name).')/2;end
            end
            design.verification=observerAnalysisSupport.verifyImprovedObserverDesign(design,cfg);design.certified=design.verification.certified;
            assert(design.certified,'VehicleLocalization:InfeasibleCertificate','Recovered matrices failed independent verification.');
            if strlength(cfg.synthesis.outputFolder)>0
                if ~isfolder(cfg.synthesis.outputFolder),mkdir(cfg.synthesis.outputFolder);end
                save(fullfile(cfg.synthesis.outputFolder,cfg.synthesis.saveFileName),'design');
            end
        end

        function verification = verifyImprovedObserverDesign(design,cfg)
        % verifyImprovedObserverDesign Recompute a constant-matrix ISS certificate.
        % Stored certified flags are ignored. This verifies continuous equations,
        % not numerical integration error, true-state bounds or unknown sensor errors.
            arguments
                design (1,1) struct
                cfg (1,1) struct
            end
            data=observerAnalysisSupport.buildImprovedObserverCertificateData(cfg);
            assert(isfield(design,'kind') && string(design.kind)=="continuous-mo-hgo-v1" ...
                && string(design.mode)==cfg.mode && design.theta==cfg.observer.theta, ...
                'VehicleLocalization:CertificateMismatch','Design must match the continuous mode and theta.');
            n=7; outputs=3;
            if cfg.mode=="gnss",n=6;outputs=2;end
            requireMatrix(design.K,[7,outputs]);requireMatrix(design.N,[7,4]);requireMatrix(design.P,[n,n]);
            P=design.P;assert(norm(P-P.','fro')<1e-10 && min(eig(P))>0, ...
                'VehicleLocalization:CertificateMismatch','P must be symmetric positive definite.');
            theta=cfg.observer.theta;
            if cfg.mode=="gnss"
                assert(all(design.K(7,:)==0) && all(design.N(1:6,4)==0) ...
                    && all(design.N(7,1:3)==0) && design.N(7,4)==-cfg.observer.yawGain*theta^2 ...
                    && isfinite(cfg.observer.yawGain) && cfg.observer.yawGain>0, ...
                    'VehicleLocalization:CertificateMismatch','GNSS requires the triangular gain and positive yaw gain.');
                assert(cfg.gnss.minimumSpeed>0 && isfinite(cfg.gnss.minimumSpeed) ...
                    && cfg.gnss.headingErrorLimit>0 && cfg.gnss.headingErrorLimit<pi/2, ...
                    'VehicleLocalization:InvalidConfiguration','GNSS needs positive speed and a local yaw sector.');
                A0=theta*(data.A(1:6,1:6)-design.K(1:6,:)*data.Cg);
                nominal=-max(eig(P*A0+A0.'*P));
                margin=nominal-2*norm(P,2)*(data.modelPerturbation+ ...
                    norm(design.N(1:6,1:3),2)*data.outputBound3);
                yawRate=cfg.observer.yawGain*cfg.gnss.minimumSpeed* ...
                    sin(cfg.gnss.headingErrorLimit)/cfg.gnss.headingErrorLimit;
                c=max(margin,0)*yawRate/(8*(cfg.observer.yawGain*theta^2)^2);
                rate=min(margin/(2*norm(P,2)),yawRate)/2;
                extra=struct('yawMetricWeight',c,'headingAdmissionRequired',true);
            else
                requireMatrix(design.Q,[7,7]);requireMatrix(design.R,[7,7]);
                Q=design.Q;R=design.R;
                assert(norm(Q-Q.','fro')<1e-10 && norm(R-R.','fro')<1e-10 ...
                    && min([eig(Q);eig(R)])>0 && isfinite(design.g) && design.g>0 ...
                    && isfinite(design.rate) && design.rate>0, ...
                    'VehicleLocalization:CertificateMismatch','Invalid constant delay-functional matrices.');
                delay=cfg.measurement.fixedLidarDelay;
                assert(isreal(delay) && isscalar(delay) && isfinite(delay) && delay>=0, ...
                    'VehicleLocalization:InvalidConfiguration','The LiDAR delay must be finite and nonnegative.');
                [vertices,uncertainty]=observerAnalysisSupport.continuousLidarCertificateVertices(design,cfg);
                Ad=-theta*design.K*data.C;vertexMargins=zeros(size(vertices,3),1);
                for vertex=1:size(vertices,3)
                    block=observerAnalysisSupport.continuousObserverDelayLmi(vertices(:,:,vertex),Ad,P,Q,R,design.g,delay,design.rate);
                    vertexMargins(vertex)=-max(eig((block+block.')/2));
                end
                nominal=min(vertexMargins);
                margin=nominal-2*(norm(P,2)+delay*norm(R,2))*uncertainty;
                rate=design.rate;extra=struct('delaySeconds',delay,'headingAdmissionRequired',false, ...
                    'vertexMargins',vertexMargins,'remainingPerturbationBound',uncertainty);

            end
            verification=struct('certified',margin>cfg.synthesis.tolerance, ...
                'nominalMargin',nominal,'uniformMargin',margin,'rate',rate, ...
                'constantMetric',true,'details',extra, ...
                'scope',"Conditional continuous-time ISS matrix certificate; numerical and physical assumptions remain separate.");
        end

        function design = improvedObserverReferenceDesign(cfg)
        % improvedObserverReferenceDesign Load and recheck one continuous-mode design.
            arguments
                cfg (1,1) struct = improvedObserverConfig()
            end
            if cfg.mode=="gnss" && isfield(cfg.observer,'gnssChainGain')
                % A changed chain needs its own metric, recomputed and verified with
                % the same operating bounds. Never attach the reference P blindly.
                design=observerAnalysisSupport.designImprovedObserverGains(cfg);
                return;
            end
            root=fileparts(fileparts(mfilename('fullpath')));
            stored=jsondecode(fileread(fullfile(root,'config','continuousObserverCertificate.json')));
            source=stored.(cfg.mode);
            if cfg.mode=="lidar" && isfield(cfg.observer,'lidarGainProfile')
                assert(any(string(cfg.observer.lidarGainProfile)==["tracking","mncav"]), ...
                    'VehicleLocalization:UnsupportedObserverProfile','Unknown LiDAR gain profile.');
                filename="lidarTrackingCertificate.json";
                if string(cfg.observer.lidarGainProfile)=="mncav",filename="mncavLidarCertificate.json";end
                source=jsondecode(fileread(fullfile(root,'config',filename)));
            end
            design=struct('kind',"continuous-mo-hgo-v1",'mode',cfg.mode,'theta',cfg.observer.theta);
            if isfield(cfg.synthesis,'certificateMethod')
                design.certificateMethod=cfg.synthesis.certificateMethod;
            end
            if cfg.mode=="gnss"
                design.P=source.P6;design.K=[source.K;zeros(1,2)];
                design.N=zeros(7,4);design.N(1:6,1:3)=source.N;
                design.N(7,4)=-cfg.observer.yawGain*design.theta^2;
            else
                for name=["P","Q","R","g","rate","K","N"],design.(name)=source.(name);end
            end
            design.verification=observerAnalysisSupport.verifyImprovedObserverDesign(design,cfg);
            design.certified=design.verification.certified;
            assert(design.certified,'VehicleLocalization:CertificateMismatch', ...
                'The reference matrices do not certify the requested continuous configuration.');
        end

        function data = buildImprovedObserverCertificateData(cfg)
        % buildImprovedObserverCertificateData Matrices for the two continuous modes.
            arguments
                cfg (1,1) struct
            end
            assert(isfield(cfg,'mode') && any(string(cfg.mode)==["gnss","lidar"]), ...
                'VehicleLocalization:InvalidContinuousMode','Select gnss or lidar mode.');
            theta=cfg.observer.theta;
            assert(isscalar(theta) && isreal(theta) && isfinite(theta) && theta>=1, ...
                'VehicleLocalization:InvalidConfiguration','theta must be finite and >=1.');
            assert(isequal(cfg.observer.scalingExponents(:),[1;2;3;1;2;3;1]), ...
                'VehicleLocalization:InvalidConfiguration','Unexpected state scaling exponents.');
            limits=[cfg.operating.maximumSpeed,cfg.operating.maximumAcceleration,cfg.operating.maximumTrackAngleRate];
            assert(isreal(limits) && all(isfinite(limits)) && all(limits>=0) && all(limits(1:2)>0), ...
                'VehicleLocalization:InvalidConfiguration','Operating bounds must be finite and nonnegative.');
            scales=cfg.lidar.poseScales(:);
            assert(isreal(scales) && numel(scales)==3 && all(isfinite(scales) & scales>0), ...
                'VehicleLocalization:InvalidConfiguration','Pose scales must be positive.');
            assert(isfinite(cfg.lidar.minimumPoseWeight) && cfg.lidar.minimumPoseWeight>0 ...
                && cfg.lidar.minimumPoseWeight<=1 && isfinite(cfg.lidar.gainInformationScale) ...
                && cfg.lidar.gainInformationScale>0, ...
                'VehicleLocalization:InvalidConfiguration','Invalid information sector or scale.');
            chain=[0,1,0;0,0,1;0,0,0];
            data.A=blkdiag(chain,chain,0); data.T=diag(theta.^cfg.observer.scalingExponents);
            data.C=zeros(3,7);
            data.C(1,1)=1/scales(1);data.C(2,4)=1/scales(2);data.C(3,7)=1/scales(3);
            data.Cg=zeros(2,6);data.Cg(1,1)=1;data.Cg(2,4)=1;
            data.outputBound3=sqrt(12*limits(1)^2+4*limits(2)^2);
            data.outputBound4=sqrt(16*limits(1)^2+4*limits(2)^2+2);
            data.modelPerturbation=limits(3)*sqrt(limits(3)^2/theta^2+4);
        end

        function [vertices,uncertainty]=continuousLidarCertificateVertices(design,cfg)
        % continuousLidarCertificateVertices Enclose the scaled drift for all rates.
        % The structured method embeds (q,q^2) in [-b,b] x [0,b^2]. The Schur LMI
        % is affine in both coordinates, so four vertices cover the entire curve,
        % including arbitrarily time-varying q. P/Q/R are common to all vertices.
        % Auxiliary-output and information uncertainty retain their norm enclosure.
            data=observerAnalysisSupport.buildImprovedObserverCertificateData(cfg);theta=cfg.observer.theta;
            method="norm-ball";
            if isfield(design,'certificateMethod'),method=string(design.certificateMethod);end
            assert(isscalar(method) && any(method==["norm-ball","course-rate-polytope"]), ...
                'VehicleLocalization:CertificateMismatch','Unknown certificate method.');
            uncertainty=norm(design.N,2)*data.outputBound4+ ...
                theta*norm(design.K,2)*norm(data.C,2)*(1-cfg.lidar.minimumPoseWeight);
            if method=="norm-ball"
                vertices=theta*data.A;uncertainty=uncertainty+data.modelPerturbation;
                return;
            end
            bound=cfg.operating.maximumTrackAngleRate;vertices=zeros(7,7,4);k=0;
            for q=[-bound,bound]
                for squaredRate=[0,bound^2]
                    k=k+1;A0=theta*data.A;
                    A0(3,2)=squaredRate/theta;A0(6,5)=squaredRate/theta;
                    A0(3,6)=-2*q;A0(6,3)=2*q;vertices(:,:,k)=A0;
                end
            end
        end

        function block = continuousObserverDelayLmi(A0,Ad,P,Q,R,g,delay,rate)
        % continuousObserverDelayLmi Affine Schur form of the constant-matrix LKF.
        % Accepts numeric matrices or symbolic SDP variables during synthesis.
            n=size(P,1);r=exp(-2*rate*delay);
            Pi=[P*A0+A0.'*P+2*rate*P+Q-r*R,P*Ad+r*R,P; ...
                Ad.'*P+r*R,-r*(Q+R),zeros(n);P,zeros(n),-g*eye(n)];
            flow=[A0,Ad,eye(n)];
            block=[Pi,delay*flow.'*R;delay*R*flow,-R];
        end

        function channels = evaluateImprovedObserverChannels(state, sample, operating)
        % evaluateImprovedObserverChannels Evaluate the seven-state model and h map.
        % SAMPLE contains longitudinalSpeed, lateralVelocity, longitudinalAcceleration,
        % lateralAcceleration, yawRate, sideSlipAngle, and sideSlipAngleRate.
        % Body acceleration must be compensated inertial acceleration at the vehicle
        % reference point. Prediction uses q^2*v+2*q*J*a with q=r_m+betaDot_m;
        % omitted speed jerk, course angular acceleration and q error are additive
        % model disturbances, not zero-motion assumptions (see the assimilation note).

            arguments
                state (7, 1) double {mustBeFinite}
                sample (1, 1) struct
                operating struct = struct()
            end

            requiredFields = ["longitudinalSpeed", "lateralVelocity", ...
                "longitudinalAcceleration", "lateralAcceleration", "yawRate", ...
                "sideSlipAngle", "sideSlipAngleRate"];
            for fieldName = requiredFields
                assert(isfield(sample, fieldName) && isscalar(sample.(fieldName)) && ...
                    isfinite(sample.(fieldName)), "sample.%s must be a finite scalar.", fieldName);
            end

            trackAngleRate = sample.yawRate + sample.sideSlipAngleRate;
            trackAngleRateSquared = trackAngleRate.^2;
            chain=[0,1,0;0,0,1;0,0,0];
            modelMatrix=blkdiag(chain,chain,0);
            modelMatrix(3,2)=trackAngleRateSquared;modelMatrix(6,5)=trackAngleRateSquared;
            modelMatrix(3,6)=-2*trackAngleRate;modelMatrix(6,3)=2*trackAngleRate;
            modelInput=zeros(7,1);modelInput(7)=sample.yawRate;
            modelDerivative=modelMatrix*state+modelInput;

            % Extend only h outside the physical operating box. Clipping its
            % arguments gives a globally bounded mean-value Jacobian already covered
            % by the certificate's interval vertices. The estimated state and linear
            % prediction are not clipped or reset.
            invariantState = state;
            if ~isempty(fieldnames(operating))
                invariantState([2,5]) = min(max(state([2,5]),-operating.maximumSpeed),operating.maximumSpeed);
                invariantState([3,6]) = min(max(state([3,6]),-operating.maximumAcceleration),operating.maximumAcceleration);
            end
            extensionActive = any(invariantState~=state);
            state = invariantState;
            velocitySquared = state(2).^2 + state(5).^2;
            velocityAccelerationDot = state(2) .* state(3) + state(5) .* state(6);
            velocityAccelerationCross = state(2) .* state(6) - state(5) .* state(3);
            coupledAngle = state(7) + sample.sideSlipAngle;
            coupling = state(5) .* cos(coupledAngle) - state(2) .* sin(coupledAngle);
            invariantPrediction = [velocitySquared; velocityAccelerationDot; ...
                velocityAccelerationCross; coupling];
            invariantMeasurement = [sample.longitudinalSpeed.^2 + sample.lateralVelocity.^2; ...
                sample.longitudinalSpeed .* sample.longitudinalAcceleration + ...
                    sample.lateralVelocity .* sample.lateralAcceleration; ...
                sample.longitudinalSpeed .* sample.lateralAcceleration - ...
                    sample.lateralVelocity .* sample.longitudinalAcceleration; ...
                0.0];

            channels = struct();
            channels.trackAngleRate = trackAngleRate;
            channels.modelDerivative = modelDerivative;
            channels.modelMatrix = modelMatrix;
            channels.modelInput = modelInput;
            channels.invariantPrediction = invariantPrediction;
            channels.invariantExtensionActive = extensionActive;
            channels.invariantMeasurement = invariantMeasurement;
            channels.invariantInnovation = invariantMeasurement - invariantPrediction;
            % Local sensitivity of h4 to yaw, not an absolute-heading observation.
            % It vanishes at zero velocity even if geometric heading is available.
            channels.motionHeadingSensitivity = -state(2)*cos(coupledAngle) ...
                -state(5)*sin(coupledAngle);
        end

        function result = simulateImprovedObserverScenario(observerDesign,lateralDesign,cfg)
        % simulateImprovedObserverScenario Validate one continuous measurement mode.
        % Constant-speed circular/straight Cartesian truth is analytic, including the
        % prehistory. Smooth bounded sinusoidal measurement errors remain continuous.
        % A nonempty lateralDesign runs the real upstream observer; struct() supplies
        % exact lateral interfaces to isolate the seven-state global observer.
            arguments
                observerDesign (1,1) struct
                lateralDesign (1,1) struct = struct()
                cfg (1,1) struct = improvedObserverConfig()
            end
            t=(0:cfg.simulation.sampleTime:cfg.simulation.finalTime).';
            n=numel(t);v=cfg.simulation.speed;q=cfg.simulation.courseRate;
            z=zeros(n,7);
            for k=1:n,z(k,:)=trajectory(t(k),v,q,cfg.simulation.initialHeading).';end
            high=struct('time',t,'steeringAngle',zeros(n,1),'longitudinalSpeed',v*ones(n,1), ...
                'longitudinalAcceleration',zeros(n,1),'lateralAcceleration',v*q*ones(n,1), ...
                'yawRate',q*ones(n,1));
            if ~isempty(fieldnames(lateralDesign))
                wheelbase=lateralDesign.cfg.vehicle.lf+lateralDesign.cfg.vehicle.lr;
                high.steeringAngle(:)=atan2(q*wheelbase,max(v,eps));
            end
            data=struct('highRate',high);delay=cfg.measurement.fixedLidarDelay;
            if cfg.mode=="gnss"
                data.gnss=struct('evaluate',@(time) positionOutput(time,cfg));
            else
                data.lidar=struct('delay',delay,'headingConvention',"unwrapped", ...
                    'evaluate',@(time) lidarOutput(time,cfg));
            end
            error=cfg.simulation.initialError;
            runCfg=cfg;runCfg.observer.initialState=z(1,:).'+error;
            initial=@(time) trajectory(time,v,q,cfg.simulation.initialHeading)+error;
            if isempty(fieldnames(lateralDesign))
                lateral=struct('time',t,'lateralVelocity',zeros(n,1), ...
                    'sideSlipAngle',zeros(n,1),'sideSlipAngleRate',zeros(n,1));
                estimate=runImprovedVehicleObserver(data,struct(),observerDesign,runCfg, ...
                    LateralInputs=lateral,InitialHistory=initial);
            else
                estimate=runImprovedVehicleObserver(data,lateralDesign,observerDesign,runCfg,InitialHistory=initial);
            end
            errorState=estimate.z-z;settled=t>=.5*cfg.simulation.finalTime;
            metrics=struct('positionRmse',sqrt(mean(sum(errorState(settled,[1,4]).^2,2))), ...
                'velocityRmse',sqrt(mean(sum(errorState(settled,[2,5]).^2,2))), ...
                'accelerationRmse',sqrt(mean(sum(errorState(settled,[3,6]).^2,2))), ...
                'headingRmse',sqrt(mean(errorState(settled,7).^2)), ...
                'maximumPositionError',max(vecnorm(errorState(:,[1,4]),2,2)), ...
                'maximumVelocityError',max(vecnorm(errorState(:,[2,5]),2,2)), ...
                'maximumAccelerationError',max(vecnorm(errorState(:,[3,6]),2,2)), ...
                'maximumHeadingError',max(abs(errorState(:,7))), ...
                'finalErrorNorm',norm(errorState(end,:)), ...
                'integrationStepCount',estimate.diagnostics.integrationStepCount);
            result=struct('truth',struct('time',t,'z',z,'position',z(:,[1,4]), ...
                'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]),'heading',z(:,7)), ...
                'sensorData',data,'estimate',estimate,'metrics',metrics,'cfg',runCfg);
        end

        function [translationWeight,headingWeight,diagnostics,poseWeight] = computeLidarInformationWeights(informationMatrix,cfg)
        % computeLidarInformationWeights Normalize full pose information and shape gain.
        % W=J/(lambda*I+J), J=S'*information*S. All returned weights use normalized
        % pose coordinates; the observer multiplies them by a normalized residual.
        % No eigenvalue floor or source fusion is applied. The continuous LiDAR mode
        % requires the configured uniformly positive weight sector.
            arguments
                informationMatrix double
                cfg (1,1) struct
            end
            poseWeight=zeros(3);translationWeight=zeros(2);headingWeight=0;
            diagnostics=struct('qualified',false,'reason',"missingInformation", ...
                'rank',0,'minimumNormalizedEigenvalue',0,'normalizedWeight',zeros(3), ...
                'normalizedInformation',zeros(3),'weightEigenvalues',zeros(3,1));
            if isempty(informationMatrix),return;end
            if ~isreal(informationMatrix) || ~isequal(size(informationMatrix),[3,3]) ...
                    || any(~isfinite(informationMatrix),'all')
                diagnostics.reason="invalidInformation";return;
            end
            tolerance=1e-10*max(1,norm(informationMatrix,2));
            if norm(informationMatrix-informationMatrix.','fro')>tolerance
                diagnostics.reason="asymmetricInformation";return;
            end
            S=diag(cfg.lidar.poseScales(:));J=S*((informationMatrix+informationMatrix.')/2)*S;
            [U,D]=eig(J);values=diag(D);
            if min(values)<0
                diagnostics.reason="nonpositiveInformation";return;
            end
            weights=values./(values+cfg.lidar.gainInformationScale);
            poseWeight=U*diag(weights)*U.';poseWeight=(poseWeight+poseWeight.')/2;
            diagnostics.rank=nnz(values>1e-12*max(1,max(values)));
            diagnostics.minimumNormalizedEigenvalue=min(values);
            diagnostics.normalizedInformation=J;diagnostics.normalizedWeight=poseWeight;
            diagnostics.weightEigenvalues=sort(weights);
            diagnostics.qualified=min(weights)>=cfg.lidar.minimumPoseWeight-1e-12;
            diagnostics.reason="insufficientInformation";
            if diagnostics.qualified,diagnostics.reason="uniformlyInformative";end
            translationWeight=poseWeight(1:2,1:2);headingWeight=poseWeight(3,3);
        end

        function [data,metadata] = reconstructContinuousObserverSignals(high,poseTime,pose,information,cfg,options)
        % reconstructContinuousObserverSignals Declare an offline linear reconstruction.
        % poseTime is the physical measurement time, never a delivery clock. For LiDAR
        % the returned output at t reconstructs the pose at t-fixedLidarDelay. Samples
        % with missing directions, large gaps or nonfinite values are rejected. This
        % adapter does not establish that recorded sensors delivered continuous outputs.
            arguments
                high (1,1) struct
                poseTime (:,1) double {mustBeFinite}
                pose double {mustBeFinite}
                information double
                cfg (1,1) struct
                options.MaximumGap (1,1) double {mustBePositive,mustBeFinite} = .12
            end
            assert(numel(poseTime)>=2 && all(diff(poseTime)>0), ...
                'VehicleLocalization:InvalidReconstruction','Physical pose times must strictly increase.');
            gap=max(diff(poseTime));
            assert(gap<=options.MaximumGap+1e-12,'VehicleLocalization:ReconstructionGap', ...
                'The recorded pose gap %.6g s exceeds the declared reconstruction limit %.6g s.',gap,options.MaximumGap);
            width=2;delay=0;
            if cfg.mode=="lidar",width=3;delay=cfg.measurement.fixedLidarDelay;end
            assert(isreal(pose) && isequal(size(pose),[numel(poseTime),width]), ...
                'VehicleLocalization:InvalidReconstruction','Unexpected recorded pose shape.');
            time=high.time(:);keep=time>=poseTime(1)+delay & time<=poseTime(end)+delay;
            assert(nnz(keep)>=2,'VehicleLocalization:InvalidReconstruction','No covered reconstruction interval.');
            for name=string(fieldnames(high)).'
                assert(numel(high.(name))==numel(time),'VehicleLocalization:InvalidReconstruction','High-rate inputs must be aligned.');
                high.(name)=high.(name)(keep);high.(name)=high.(name)(:);
            end
            query=high.time-delay;
            source=struct('representation',"piecewiseLinear",'time',high.time);
            if cfg.mode=="gnss"
                source.position=interp1(poseTime,pose,query,'linear');
            else
                assert(isequal(size(information),[3,3,numel(poseTime)]), ...
                    'VehicleLocalization:InvalidReconstruction','Each LiDAR pose needs physical information.');
                for k=1:numel(poseTime)
                    [~,~,info]=observerAnalysisSupport.computeLidarInformationWeights(information(:,:,k),cfg);
                    assert(info.qualified,'VehicleLocalization:InsufficientLidarInformation', ...
                        'Recorded LiDAR cannot be reconstructed through an insufficient direction.');
                end
                pose(:,3)=unwrap(pose(:,3));source.pose=interp1(poseTime,pose,query,'linear');
                source.information=zeros(3,3,numel(query));
                for row=1:3
                    for column=1:3
                        source.information(row,column,:)=reshape(interp1(poseTime,reshape(information(row,column,:),[],1),query,'linear'),1,1,[]);
                    end
                end
                source.delay=delay;source.headingConvention="unwrapped";
            end
            data=struct('highRate',high);data.(cfg.mode)=source;
            metadata=struct('representation',"offline piecewise-linear reconstruction", ...
                'maximumOriginalPoseGapSeconds',gap,'declaredMaximumGapSeconds',options.MaximumGap, ...
                'fixedDelaySeconds',delay,'trimmedInputSamples',nnz(~keep), ...
                'physicalContinuityVerified',false,'onlineCausalityClaimed',false, ...
                'headingUnwrapAssumption',"Adjacent recorded yaw changes are less than pi; cycle slips are not verified.");
        end

        function [data,lateralInput,metadata]=reconstructFrameAlignedLidarSignals(high,lateral, ...
                frameTime,poseTime,pose,information,cfg,options)
        % observerAnalysisSupport.reconstructFrameAlignedLidarSignals Align precomputed poses with zero delay.
        % Every frame timestamp becomes an integration knot. Only accepted poses
        % supply measurement knots; rejected frame times remain evaluation times.
        % Between accepted poses, the existing continuous observer uses an explicitly
        % offline linear reconstruction. Precomputation removes processing latency,
        % not the distinction between a measurement and an interpolated value.
            arguments
                high (1,1) struct
                lateral (1,1) struct
                frameTime (:,1) double {mustBeFinite}
                poseTime (:,1) double {mustBeFinite}
                pose double {mustBeFinite}
                information double
                cfg (1,1) struct
                options.MaximumOfflineGap (1,1) double {mustBePositive,mustBeFinite}=1
                options.MaximumMotionEdgeHold (1,1) double {mustBeNonnegative,mustBeFinite}=.02
            end
            assert(cfg.mode=="lidar" && cfg.measurement.fixedLidarDelay==0, ...
                'VehicleLocalization:ZeroDelayRequired','Frame-aligned precomputed replay requires zero LiDAR delay.');
            assert(numel(frameTime)>=2 && all(diff(frameTime)>0) && numel(high.time)>=2 ...
                && all(diff(high.time)>0),'VehicleLocalization:InvalidFrameTimes','Input and frame clocks must increase.');
            assert(isequal(lateral.time(:),high.time(:)), ...
                'VehicleLocalization:InvalidLateralInputs','Lateral outputs must cover the original motion grid.');
            assert(all(ismember(poseTime,frameTime)), ...
                'VehicleLocalization:InvalidFrameTimes','Every accepted pose must identify a processed frame.');
            before=max(0,high.time(1)-frameTime(1));after=max(0,frameTime(end)-high.time(end));
            assert(max(before,after)<=options.MaximumMotionEdgeHold+1e-12, ...
                'VehicleLocalization:MotionCoverage','The frame clock exceeds the declared motion edge-hold limit.');
            originalTime=high.time(:);
            time=unique([originalTime(originalTime>=frameTime(1) & originalTime<=frameTime(end));frameTime]);
            query=min(max(time,originalTime(1)),originalTime(end));
            for name=string(fieldnames(high)).'
                if name=="time",continue;end
                assert(isvector(high.(name)) && numel(high.(name))==numel(originalTime) ...
                    && all(isfinite(high.(name))),'VehicleLocalization:InvalidInputs','Motion fields must be finite and aligned.');
                high.(name)=interp1(originalTime,high.(name)(:),query,'linear');
            end
            high.time=time;lateralInput=struct('time',time);
            for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
                assert(isfield(lateral,name) && numel(lateral.(name))==numel(originalTime) ...
                    && all(isfinite(lateral.(name))),'VehicleLocalization:InvalidLateralInputs','Lateral fields must be finite and aligned.');
                lateralInput.(name)=interp1(originalTime,lateral.(name)(:),query,'linear');
            end
            [data,metadata]=observerAnalysisSupport.reconstructContinuousObserverSignals(high,poseTime,pose,information,cfg, ...
                MaximumGap=options.MaximumOfflineGap);
            assert(isequal(data.highRate.time,time),'VehicleLocalization:FramePoseCoverage', ...
                'Accepted measurements must bracket the requested replay; no endpoint pose is fabricated.');
            metadata.frameCount=numel(frameTime);metadata.acceptedPoseCount=numel(poseTime);
            metadata.motionStartHoldSeconds=before;metadata.motionEndHoldSeconds=after;
            metadata.frameTimestampMaximumMismatchSeconds=0;
            metadata.processingLatencyAppliedSeconds=0;
            metadata.frameIndicesInIntegrationGrid=arrayfun(@(u) find(time==u,1),frameTime);
            metadata.rejectedFramePolicy="Retain frame evaluation; no new measurement knot. Reconstruct from adjacent accepted poses offline.";
            metadata.measurementPolicy="Exact accepted pose/information at its capture time; linear reconstruction between accepted frames.";
        end

        function design=designMotionAidedObserverGains(cfg)
        % designMotionAidedObserverGains Certify physical pose and motion gains.
        % For the error coordinates [ep,ev,ea], P=I gives a dissipation lower bound
        % M=[2*kp*w,-1,0;-1,2*kv,-(1+q^2);0,-(1+q^2),2*ka].
        % M is affine in q^2; checking 0 and qMax^2 covers arbitrary time variation.
        % The skew 2*q*J term cancels. Heading is a preceding scalar stable system.
        % This certificate is specific to the new output structure, not the old LMI.
            arguments
                cfg (1,1) struct=motionAidedObserverConfig()
            end
            g=cfg.gains;
            assert(isnumeric(g) && isreal(g) && numel(g)==4 && all(isfinite(g)) ...
                && all(g>0),'VehicleLocalization:InvalidMotionGains','Four positive physical gains are required.');
            assert(isscalar(cfg.maximumTrackAngleRate) && isfinite(cfg.maximumTrackAngleRate) ...
                && cfg.maximumTrackAngleRate>=0 && isequal(cfg.lidar.poseScales(:),ones(3,1)) ...
                && isscalar(cfg.lidar.minimumPoseWeight) && isfinite(cfg.lidar.minimumPoseWeight) ...
                && cfg.lidar.minimumPoseWeight>0 && cfg.lidar.minimumPoseWeight<=1, ...
                'VehicleLocalization:InvalidMotionConfiguration','Invalid rate bound or physical pose weighting.');
            w=cfg.lidar.minimumPoseWeight;q2=[0,cfg.maximumTrackAngleRate^2];
            matrices=zeros(3,3,2);margins=zeros(2,1);
            for k=1:2
                matrices(:,:,k)=[2*g(1)*w,-1,0;-1,2*g(2),-(1+q2(k));0,-(1+q2(k)),2*g(3)];
                margins(k)=min(eig(matrices(:,:,k)));
            end
            margin=min(margins);
            assert(margin>1e-9,'VehicleLocalization:InfeasibleMotionCertificate', ...
                'The structured common quadratic certificate is not positive definite.');
            design=struct('kind',cfg.kind,'gains',g(:).','certificateVerified',true, ...
                'translationP',eye(6),'comparisonMatrices',matrices, ...
                'translationDissipationMargin',margin,'translationDecayRate',margin/2, ...
                'headingDecayRate',g(4)*w,'maximumTrackAngleRate',cfg.maximumTrackAngleRate, ...
                'minimumPoseWeight',w,'scope', ...
                "Conditional ISS for a zero-delay lifted-heading cascade with bounded sensor/model disturbances; no RMSE dominance theorem.");
        end

        function [data,metadata]=reconstructMotionAidedLidarGaps(data,lateral,poseTime,options)
        % reconstructMotionAidedLidarGaps Reconstruct long offline gaps using motion.
        % Integrate measured yaw rate, then distribute the two LiDAR endpoint yaw
        % discrepancy over the interval. Integrate measured longitudinal speed and
        % estimated lateral velocity in that orientation, and distribute the endpoint
        % position discrepancy likewise. Original accepted poses remain exact.
        % These generated knots are motion-informed interpolation, not new LiDAR
        % measurements. The right endpoint is required: this is explicitly offline.
        % Geometric information is unchanged; it is not a new calibrated covariance.
            arguments
                data (1,1) struct
                lateral (1,1) struct
                poseTime (:,1) double {mustBeFinite}
                options.MinimumGap (1,1) double {mustBePositive,mustBeFinite}=.25
            end
            h=data.highRate;s=data.lidar;t=h.time(:);n=numel(t);
            assert(n>=2 && all(isfinite(t)) && all(diff(t)>0) && isequal(s.delay,0) ...
                && string(s.representation)=="piecewiseLinear" && string(s.headingConvention)=="unwrapped" ...
                && isequal(s.time(:),t) && isequal(lateral.time(:),t), ...
                'VehicleLocalization:InvalidMotionGapInput','Require aligned, lifted, zero-delay continuous inputs.');
            assert(isequal(size(s.pose),[n,3]) && all(isfinite(s.pose),'all'), ...
                'VehicleLocalization:InvalidMotionGapInput','Require a finite aligned pose reconstruction.');
            for name=["longitudinalSpeed","yawRate"]
                assert(isvector(h.(name)) && numel(h.(name))==n && all(isfinite(h.(name))), ...
                    'VehicleLocalization:InvalidMotionGapInput','Motion samples must be finite and aligned.');
            end
            assert(isvector(lateral.lateralVelocity) && numel(lateral.lateralVelocity)==n ...
                && all(isfinite(lateral.lateralVelocity)), ...
                'VehicleLocalization:InvalidMotionGapInput','Lateral speed must be finite and aligned.');
            [present,anchors]=ismember(poseTime,t);
            assert(numel(anchors)>=2 && all(present) && all(diff(anchors)>0) ...
                && anchors(1)==1 && anchors(end)==n, ...
                'VehicleLocalization:InvalidMotionGapAnchors','Accepted timestamps must bracket and belong to the integration grid.');
            original=s.pose;selected=find(diff(poseTime)>options.MinimumGap);corrected=0;
            speed=h.longitudinalSpeed(:);gyro=h.yawRate(:);lateralSpeed=lateral.lateralVelocity(:);
            for j=selected(:).'
                ix=(anchors(j):anchors(j+1)).';tau=t(ix)-t(ix(1));fraction=tau/tau(end);
                yaw=original(ix(1),3)+cumtrapz(tau,gyro(ix));
                yaw=yaw+fraction*(original(ix(end),3)-yaw(end));
                vx=speed(ix);vy=lateralSpeed(ix);
                velocity=[cos(yaw).*vx-sin(yaw).*vy,sin(yaw).*vx+cos(yaw).*vy];
                displacement=cumtrapz(tau,velocity);
                endpointResidual=original(ix(end),1:2)-original(ix(1),1:2)-displacement(end,:);
                position=original(ix(1),1:2)+displacement+fraction*endpointResidual;
                interior=ix(2:end-1);
                data.lidar.pose(interior,:)=[position(2:end-1,:),yaw(2:end-1)];
                corrected=corrected+numel(interior);
            end
            mismatch=max(abs(data.lidar.pose(anchors,:)-original(anchors,:)),[],'all');
            assert(mismatch==0,'VehicleLocalization:MotionGapAnchorMismatch','Accepted LiDAR poses must remain exact.');
            metadata=struct('method',"Endpoint-constrained integral of measured motion in long LiDAR gaps", ...
                'minimumGapSeconds',options.MinimumGap,'correctedIntervals',numel(selected), ...
                'correctedInteriorKnots',corrected,'maximumAcceptedPoseMismatch',mismatch, ...
                'futureEndpointUsed',true,'onlineCausalityClaimed',false,'processingDelaySeconds',0, ...
                'referenceUsed',false,'informationUnchanged',isequaln(data.lidar.information,s.information), ...
                'motionInputs',"Measured longitudinal speed and gyro; estimated lateral velocity", ...
                'interpretation',"Offline interpolation, not independent or additional LiDAR measurements; no covariance calibration claimed");
            data.lidar.offlineMotionGapReconstruction=metadata;
        end
    end
end

function requireMatrix(value,shape)
    assert(isnumeric(value) && isreal(value) && isequal(size(value),shape) && all(isfinite(value),'all'), ...
        'VehicleLocalization:CertificateMismatch','Invalid certificate matrix shape or entries.');
end

function z=trajectory(t,v,q,initialHeading)
    yaw=initialHeading+q*t;
    if q==0
        position=v*t*[cos(initialHeading);sin(initialHeading)];
    else
        position=(v/q)*[sin(yaw)-sin(initialHeading);cos(initialHeading)-cos(yaw)];
    end
    velocity=v*[cos(yaw);sin(yaw)];acceleration=v*q*[-sin(yaw);cos(yaw)];
    z=[position(1);velocity(1);acceleration(1);position(2);velocity(2);acceleration(2);yaw];
end

function y=positionOutput(t,cfg)
    z=trajectory(t,cfg.simulation.speed,cfg.simulation.courseRate,cfg.simulation.initialHeading);
    y=z([1,4])+cfg.simulation.positionNoiseAmplitude*[sin(1.3*t);cos(.9*t)];
end

function y=lidarOutput(t,cfg)
    z=trajectory(t-cfg.measurement.fixedLidarDelay,cfg.simulation.speed, ...
        cfg.simulation.courseRate,cfg.simulation.initialHeading);
    noise=[cfg.simulation.positionNoiseAmplitude*sin(1.3*t); ...
        cfg.simulation.positionNoiseAmplitude*cos(.9*t);cfg.simulation.headingNoiseAmplitude*sin(.7*t)];
    y=struct('pose',z([1,4,7])+noise,'information',1e6*eye(3));
end
