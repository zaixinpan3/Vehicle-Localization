classdef sensorRedundancySupport
% Shared finite LMIs for one observer with complementary sensor adequacy.
% The same construction is used by theory verification and production synthesis.
    methods (Static)
        function p=inputs(cfg,gyroGain)
            z=cfg.invariantObserver;s=cfg.iss;
            assert(z.theta==z.sigma,'VehicleLocalization:RedundancyScaling', ...
                'The common design is certified at its fixed theta=sigma.');
            p=struct('theta',z.theta,'headingLengthM',z.headingLengthM, ...
                'maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
                'maximumCourseRate',cfg.maximumTrackAngleRate, ...
                'minimumLidarPositionStrength',s.minimumPositionStrength, ...
                'minimumLidarHeadingStrength',s.minimumHeadingStrength, ...
                'maximumLidarCoupling',s.maximumPositionHeadingCoupling, ...
                'mixedLidarEnclosure',"psd-heading-coupling-v1", ...
                'speedOutput',"raw-squared-speed-matched-rate-v2", ...
                'allAuxiliaryOutputsRequired',true, ...
                'auxiliaryOutputs',"squared-dot-cross-direction-rate-v1", ...
                'minimumGnssWeight',z.minimumGnssWeight, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).', ...
                'gyroGainNormalized',gyroGain);
            p=jsondecode(jsonencode(p));
        end

        function w=witness(cfg,P,kL,kG,kI,gyroGain)
            w=struct('inputs',sensorRedundancySupport.inputs(cfg,gyroGain), ...
                'domain',cfg.invariantObserver.redundancy,'P',P,'kappaL',kL,'kappaG',kG, ...
                'speedSquaredGain',kI(1),'bilinearGain',kI(2),'directionGain',kI(3));
        end

        function v=verify(w,includeGnss)
            [L,G,M]=sensorRedundancySupport.blocks(w,includeGnss);
            worstL=worstEigenvalue(L);worstG=worstEigenvalue(G);worstM=worstEigenvalue(M);
            v=struct('certified',worstL<0 && worstG<0 && worstM<0, ...
                'worstLidarEigenvalue',worstL,'worstGnssEigenvalue',worstG, ...
                'worstMixedEigenvalue',worstM,'mixedVertices',numel(M), ...
                'lidarVertices',numel(L),'gnssBernsteinVertices',numel(G), ...
                'vertices',numel(L)+numel(G)+numel(M),'gnssAdequacyEnabled',includeGnss, ...
                'mixedInformationEnabled',~isempty(M), ...
                'mixedLidarCouplingRetained',~isempty(M), ...
                'mixedLidarCouplingBound',w.inputs.maximumLidarCoupling, ...
                'minimumCourseSpeed',w.domain.minimumCourseSpeed, ...
                'speedInterval',[w.domain.minimumCourseSpeed,w.inputs.maximumSpeed], ...
                'zeroSpeedCertified',false,'allAuxiliaryOutputsRequired',true, ...
                'nominalDecayRatePerSecond',w.domain.nominalDecayRatePerSecond);
            if isempty(G),v.worstGnssEigenvalue=[];end
            if isempty(M),v.worstMixedEigenvalue=[];end
        end

        function [L,G,Mixed]=blocks(w,includeGnss)
            arguments
                w (1,1) struct
                includeGnss (1,1) logical=true
            end
            p=w.inputs;d=w.domain;P=w.P;th=p.theta;ell=p.headingLengthM;
            validateattributes(d.minimumCourseSpeed,{'double'}, ...
                {'real','finite','positive','scalar','<=',p.maximumSpeed});
            validateattributes(d.nominalDecayRatePerSecond,{'double'}, ...
                {'real','finite','positive','scalar'});
            E=eye(8);e2=E(:,2);e3=E(:,3);e5=E(:,5);e6=E(:,6);e7=E(:,7);e8=E(:,8);
            C=E([1,4,7],:);Cp=C(1:2,:);Dacc=e3*e3.'+e6*e6.';Dvel=e2*e2.'+e5*e5.';
            q=min(p.maximumCourseRate,p.maximumAcceleration/d.minimumCourseSpeed);
            rho=norm(p.gnssBodyOffset)/ell;tau=d.gnssSplit;eta=d.bilinearSplit;
            A0=zeros(8);A0(1,2)=1;A0(2,3)=1;A0(4,5)=1;A0(5,6)=1;A0(7,8)=1;
            corners=[1,1;1,-1;-1,1;-1,-1];speeds=linspace(d.minimumCourseSpeed,p.maximumSpeed,d.speedPartitions+1);
            L={};G={};Mixed={};
            exactLever=isfield(d,'exactGnssLeverEnclosure') && d.exactGnssLeverEnclosure;
            Wg={eye(2)};lever=zeros(2,1);
            if exactLever && rho>0
                w0=p.minimumGnssWeight;mid=(1+w0)/2;radius=(1-w0)/2;
                Wg={w0*eye(2),eye(2)};
                for k=1:4
                    Wg{end+1}=mid*eye(2)+radius*[corners(k,1),corners(k,2);corners(k,2),-corners(k,1)]; %#ok<AGROW>
                end
                lever=rho*corners.';
            elseif exactLever
                % With no lever arm, the G term is -kG*Cp'*W*Cp.
                % Every W>=w0*I only adds dissipation, so its lower corner
                % suffices. Do not send duplicated zero-lever vertices to SDP.
                Wg={p.minimumGnssWeight*eye(2)};
            end
            controls=sensorRedundancySupport.courseControls(q,d.coursePartitions);
            mixedLidar=sensorRedundancySupport.mixedLidarLowerControls(p);
            for iQ=1:size(controls,1)
                rate=controls(iQ,1);u=controls(iQ,2);
                for tangentialSign=[-1,1]
                    A=A0;A([3,6],[2,5])=u/th^2*eye(2);
                    A([3,6],[3,6])=2*rate/th*[0,-1;1,0];
                    % All three families now share the positive-speed domain.
                    % In true course coordinates a_perp=s*q, hence
                    % d(f_a)/d(r)=2*q*v+2*J*a=[0;2*a_parallel].
                    % This cancellation is kinematic, not sensor-specific.
                    % Upstream angle/rate discrepancies remain input errors.
                    A([3,6],8)=[0;2*p.maximumAcceleration/(th^2*ell)*tangentialSign];
                    % All five motion outputs are available in every family.
                    % Use the SAME speed-correlated auxiliary lower bound in
                    % L/G/M; gyro remains in the common intrinsic dynamics.
                    for k=1:d.speedPartitions
                        left=e5-speeds(k)/(th*ell)*e7;right=e5-speeds(k+1)/(th*ell)*e7;
                        B=cat(3,left*left.',(left*right.'+right*left.')/2,right*right.');
                        speedSquared=[speeds(k)^2,speeds(k)*speeds(k+1),speeds(k+1)^2]/p.maximumSpeed^2;
                        for jB=1:3
                            M=P*(A-p.gyroGainNormalized*(e8*e8.')) ...
                                -w.speedSquaredGain*speedSquared(jB)*(e2*e2.') ...
                                -w.directionGain*B(:,:,jB) ...
                                -(1-eta)*w.bilinearGain*speedSquared(jB)*Dacc;
                            common=M+M.' ...
                                +2*w.bilinearGain*(1/eta-1)*(p.maximumAcceleration/(th*p.maximumSpeed))^2*Dvel ...
                                +d.lambda*E+2*d.nominalDecayRatePerSecond/th*P;
                            for iS=1:4
                                c=p.maximumLidarCoupling*corners(iS,:).';
                                S=[p.minimumLidarPositionStrength*eye(2),c;c.',p.minimumLidarHeadingStrength];
                                L{end+1}=common-2*w.kappaL*C.'*S*C ...
                                    +w.kappaG*rho^2*(e7*e7.') ...
                                    +2*w.kappaL*d.informativeLidarMismatch*(C.'*C); %#ok<AGROW>
                            end
                            if ~includeGnss,continue;end
                            for iW=1:numel(Wg)
                                for iR=1:size(lever,2)
                                    if exactLever
                                        gnss=-w.kappaG*Cp.'*Wg{iW}*(Cp+lever(:,iR)*e7.');penalty=0;
                                    else
                                        gnss=-w.kappaG*(1-tau)*p.minimumGnssWeight*(Cp.'*Cp);penalty=w.kappaG*rho^2/tau;
                                    end
                                    paired=common+gnss+gnss.'+penalty*(e7*e7.');
                                    G{end+1}=paired+2*w.kappaL*d.degradedLidarMismatch*(C.'*C); %#ok<AGROW>
                                    for iS=1:numel(mixedLidar)
                                        % Keep actual translation-heading coupling;
                                        % PSD plus a heading diagonal is not a
                                        % diagonal Loewner lower bound.
                                        S=mixedLidar{iS};
                                        Mixed{end+1}=paired-2*w.kappaL*C.'*S*C ...
                                            +2*w.kappaL*d.informativeLidarMismatch*(C.'*C); %#ok<AGROW>
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end

        function controls=mixedLidarLowerControls(p)
        % For actual S=[T,c;c',h]>=0, h>=hmin and ||c||_2<=b:
        % S-[zeros(2),c;c',hmin]=blkdiag(T,h-hmin)>=0.
        % Each c_j lies in [-b,b], so four affine corner controls enclose
        % the retained cross block. No PSD claim is made for a lower control.
        % The combined GNSS/LiDAR LMI, not LiDAR alone, supplies dissipation.
            arguments
                p (1,1) struct
            end
            b=p.maximumLidarCoupling;h=p.minimumLidarHeadingStrength;
            validateattributes([b,h],{'double'},{'real','finite','nonnegative'});
            corners=[1,1;1,-1;-1,1;-1,-1];
            if b==0,corners=zeros(1,2);end
            controls=cell(1,size(corners,1));
            for j=1:numel(controls)
                c=b*corners(j,:).';controls{j}=[zeros(2),c;c.',h];
            end
        end

        function controls=courseControls(bound,partitions)
        % Degree-two Bernstein controls of [q,q^2] on each signed subinterval.
        % Convex combinations certify the entire curve, not sampled rates.
            validateattributes(bound,{'double'},{'real','finite','nonnegative','scalar'});
            validateattributes(partitions,{'double'},{'real','finite','integer','positive','scalar'});
            assert(mod(partitions,2)==0,'VehicleLocalization:InvalidRedundancyDomain', ...
                'Use an even number of course partitions so zero is a boundary.');
            edge=linspace(-bound,bound,partitions+1);controls=zeros(3*partitions,2);
            for j=1:partitions
                a=edge(j);b=edge(j+1);
                controls(3*j-2:3*j,:)=[a,a^2;(a+b)/2,a*b;b,b^2];
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
