classdef motionResponseSupport
% LMI response design at a fresh common ISS metric, with bounded scheduling.
% At straight course let z=[dv/theta^2; da/theta^3], M=(P^-1)(2:3,2:3),
% R=M^-1 and A=[0,theta;0,0]. The nominal motion subsystem is
% zdot=(A-theta*M*K(s,b))*z. Its constant-metric rate condition is
% He(R*A)-2*theta*K+2*r*R<=0, without position feedback.
%
% Retain baseline weights k0. Solve nonnegative c for the additional weights
% dk_i(s_m)=c_i*V^2/(s_min^2+s_m^2), i=speed,dot/cross. Their denominator
% stays positive at zero speed. Only the existing squared-speed output
% schedules gains; no IMU signal enters prediction. A single P is retained.
% Put B=He(R*A)+2*r*R. The sufficient response LMI is
% (s_min^2+s^2)*B-2*theta*V^2*(c1*Gspeed+c2*Gdot)<=0.
% Degree-(2,2) Bernstein controls cover the whole speed/acceleration box.
% The baseline motion Gram, omitted in this inequality, adds dissipation.
%
% In the full common certificate every extra nominal term is an exact PSD
% Gram with nonnegative weight, irrespective of the scheduling signal. Thus
% the verified baseline L/G/M inequalities still apply. The nonlinear
% remainder uses MAXIMUM scheduled weights, not k0; it does not treat
% Hhat'*Hsec as PSD. No derivative of a time-varying P is introduced.
% The second SDP minimizes the unchanged primitive-noise proxy at this fixed
% metric and response target. It is not a joint optimum over P and gains,
% an empirical covariance, or a full nonlinear/sampled settling-time claim.
    methods (Static)
        function d=refine(seed,cfg)
            [R,controls,rate]=motionResponseSupport.model(seed.P,cfg);
            [Q,q]=motionResponseSupport.noiseQuadratic(seed,cfg);
            t=cfg.invariantObserver.theta;scale=max(diag(R))*rate/t;
            u=sdpvar(2,1);increment=scale*u;
            constraints=[u>=1e-10];
            matrices=motionResponseSupport.blocks(R,controls,increment,cfg,rate);
            for j=1:numel(matrices)
                constraints=[constraints,matrices{j}<=-1e-7*eye(2)]; %#ok<AGROW>
            end
            cost=scale^2*(u.'*Q*u)+2*scale*q.'*u;
            options=sdpsettings('solver',char(cfg.invariantObserver.solver),'verbose',0);
            result=optimize(constraints,cost/(norm(Q,2)*scale^2),options);
            assert(ismember(result.problem,[0,4]) && min(check(constraints))>=-1e-8, ...
                'VehicleLocalization:MotionResponseSynthesis', ...
                'Response LMI solve failed numerical verification: %s.',result.info);
            d=seed;
            d.motionGainSchedule=struct('kind',"bounded-invariant-speed-v1", ...
                'incrementWeights',value(increment), ...
                'minimumSpeedMps',cfg.invariantObserver.redundancy.minimumCourseSpeed, ...
                'maximumSpeedMps',cfg.iss.maximumSpeed);
            d.meanSquareBound=seed.meanSquareBound+value(cost);
            d.motionResponseDesign=struct('method',"fixed-ISS-metric-bounded-Gram-schedule", ...
                'seedNoiseObjective',seed.meanSquareBound,'finalNoiseObjective',d.meanSquareBound, ...
                'responseTimeConstantS',1/rate,'solverStatus',result.problem, ...
                'minimumSdpResidual',min(check(constraints)), ...
                'controls',numel(matrices),'gainsSelectedFromReference',false);
            fprintf('MOTION RESPONSE: tau %g s, %d controls, added coefficients %s, noise proxy %.8g -> %.8g\n', ...
                1/rate,numel(matrices),mat2str(d.motionGainSchedule.incrementWeights.',6),seed.meanSquareBound,d.meanSquareBound);
        end

        function v=verify(d,cfg)
            response=cfg.invariantObserver.motionResponse;
            validateattributes(response.enabled,{'logical'},{'scalar'});
            validateattributes(response.timeConstantS,{'double'},{'real','finite','positive','scalar'});
            enabled=response.enabled;present=isfield(d,'motionGainSchedule');
            v=struct('required',enabled,'verified',~enabled && ~present, ...
                'positiveGramAugmentationVerified',~present);
            if ~enabled || ~present,return;end
            g=d.motionGainSchedule;motionResponseSupport.validateSchedule(g);
            consistent=g.minimumSpeedMps==cfg.invariantObserver.redundancy.minimumCourseSpeed ...
                && g.maximumSpeedMps==cfg.iss.maximumSpeed;
            [R,controls,rate]=motionResponseSupport.model(d.P,cfg);
            blocks=motionResponseSupport.blocks(R,controls,g.incrementWeights,cfg,rate);
            worst=-Inf;
            for j=1:numel(blocks),worst=max(worst,max(eig((blocks{j}+blocks{j}.')/2)));end
            v=struct('required',true,'verified',consistent && isfinite(worst) && worst<=0, ...
                'positiveGramAugmentationVerified',consistent, ...
                'timeConstantS',1/rate,'decayRatePerSecond',rate, ...
                'worstNormalizedControlEigenvalue',worst,'controls',numel(blocks), ...
                'metric',R,'speedInterval',[g.minimumSpeedMps,g.maximumSpeedMps], ...
                'accelerationInterval',cfg.iss.maximumAcceleration*[-1,1], ...
                'maximumAuxiliaryWeights',motionResponseSupport.maximumWeights(d), ...
                'poseFeedbackRequired',false,'fullNonlinearResponseCertified',false, ...
                'scope',"Nominal straight-course longitudinal velocity/acceleration subsystem, with heading/interface errors held zero, exact motion packets and no pose correction. Constant metric covers arbitrary variation within the declared speed/acceleration intervals. Full nonlinear/cascade/sampled settling time is not certified.");
        end

        function k=weights(d,speedSquared)
        % Called only with the existing packet's nonnegative squared speed.
            k=d.auxiliaryWeights(:);
            if ~isfield(d,'motionGainSchedule'),return;end
            validateattributes(speedSquared,{'double'},{'real','finite','nonnegative','scalar'});
            g=d.motionGainSchedule;
            factor=g.maximumSpeedMps^2/(g.minimumSpeedMps^2+speedSquared);
            k(1:2)=k(1:2)+factor*g.incrementWeights;
        end

        function k=maximumWeights(d)
        % This upper bound also covers noisy scheduling and zero-speed packets.
            k=motionResponseSupport.weights(d,0);
        end

        function [R,controls,rate]=model(P,cfg)
            rate=1/cfg.invariantObserver.motionResponse.timeConstantS;
            % Use the Schur complement, not P(2:3,2:3). Position-motion
            % coupling remains in the original eight-state ISS metric.
            inverse=P\eye(8);R=inverse(2:3,2:3)\eye(2);R=(R+R.')/2;
            assert(min(eig(R))>0,'VehicleLocalization:MotionResponseMetric','Require an SPD motion metric.');
            z=cfg.invariantObserver;V=cfg.iss.maximumSpeed;
            edge=exp(linspace(log(z.redundancy.minimumCourseSpeed),log(V),z.redundancy.speedPartitions+1));
            edge([1,end])=[z.redundancy.minimumCourseSpeed,V];
            accel=linspace(-cfg.iss.maximumAcceleration,cfg.iss.maximumAcceleration,z.redundancy.accelerationPartitions+1);
            controls=zeros(9*z.redundancy.speedPartitions*z.redundancy.accelerationPartitions,3);index=0;
            for i=1:numel(edge)-1
                s=[edge(i),mean(edge(i:i+1)),edge(i+1)];s2=[edge(i)^2,prod(edge(i:i+1)),edge(i+1)^2];
                for j=1:numel(accel)-1
                    b=[accel(j),mean(accel(j:j+1)),accel(j+1)];b2=[accel(j)^2,prod(accel(j:j+1)),accel(j+1)^2];
                    for is=1:3
                        for ib=1:3
                            index=index+1;controls(index,:)=[s2(is),s(is)*b(ib),b2(ib)];
                        end
                    end
                end
            end
        end

        function blocks=blocks(R,controls,c,cfg,rate)
            t=cfg.invariantObserver.theta;s0=cfg.invariantObserver.redundancy.minimumCourseSpeed;
            A=[0,t;0,0];base=R*A+A.'*R+2*rate*R;
            blocks=cell(1,size(controls,1));scale=norm(R,2);
            for j=1:numel(blocks)
                s2=controls(j,1);sb=controls(j,2);b2=controls(j,3);
                speed=[s2,0;0,0];dot=[b2/t^2,sb/t;sb/t,s2];
                % V^2 cancels the fixed output normalization. Positive
                % per-control scaling improves conditioning without changing
                % the sign of the original polynomial Bernstein coefficient.
                blocks{j}=((s0^2+s2)*base-2*t*(c(1)*speed+c(2)*dot))/(scale*(s0^2+s2));
            end
        end

        function [Q,q]=noiseQuadratic(d,cfg)
        % Exact varying part of the existing normalized primitive-noise proxy:
        % Delta J=c'*Q*c+2*q'*c. Retain correlated speed/dot/cross effects.
            z=cfg.invariantObserver;t=z.theta;n=z.noise;V=cfg.iss.maximumSpeed;s0=z.redundancy.minimumCourseSpeed;
            speeds=unique([s0,(s0+V)/2,V]);accels=cfg.iss.maximumAcceleration*[0,1,-1,0,0;0,0,0,1,-1];
            T=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),z.headingLengthM,z.headingLengthM]);
            D=invariantObserverSupport.outputNormalization(cfg);L=chol(d.P,'lower');Q=zeros(2);q=zeros(2,1);
            for s=speeds
                for a=accels
                    x=[0;s;a(1);0;0;a(2);0;0];[~,H]=invariantObserverSupport.outputs(x,0,cfg);
                    N=invariantIssSupport.measurementSecant([s;0],a,[s;0],a)* ...
                        diag([n.velocityMps,n.velocityMps,n.accelerationMps2,n.accelerationMps2,n.gyroRadps]);
                    C=D*H*T;DI=D*N;
                    B1=L\(t*C(1,:).'*DI(1,:));B2=L\(t*C(2:3,:).'*DI(2:3,:));
                    J=[B1(:),B2(:)];W=J.'*J/(numel(speeds)*size(accels,2));factor=V^2/(s0^2+s^2);
                    Q=Q+factor^2*W;q=q+factor*W*d.auxiliaryWeights(1:2);
                end
            end
            Q=(Q+Q.')/2;
            assert(min(eig(Q))>0,'VehicleLocalization:MotionResponseNoise','Require a positive definite motion noise objective.');
        end

        function validateSchedule(g)
            assert(g.kind=="bounded-invariant-speed-v1",'VehicleLocalization:MotionResponseSchedule','Unknown motion gain schedule.');
            validateattributes(g.incrementWeights,{'double'},{'real','finite','nonnegative','column','numel',2});
            validateattributes(g.minimumSpeedMps,{'double'},{'real','finite','positive','scalar'});
            validateattributes(g.maximumSpeedMps,{'double'},{'real','finite','scalar','>=',g.minimumSpeedMps});
        end
    end
end
