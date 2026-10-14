classdef sensorRedundancySupport
% Shared finite LMIs for one observer with complementary sensor adequacy.
% The same construction is used by theory verification and production synthesis.
    methods (Static)
        function p=inputs(cfg,gyroGain,speedGain)
            z=cfg.invariantObserver;s=cfg.iss;
            assert(z.theta==z.sigma,'VehicleLocalization:RedundancyScaling', ...
                'The common design is certified at its fixed theta=sigma.');
            p=struct('theta',z.theta,'headingLengthM',z.headingLengthM, ...
                'maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
                'maximumCourseRate',cfg.maximumTrackAngleRate, ...
                'minimumLidarPositionStrength',s.minimumPositionStrength, ...
                'minimumLidarHeadingStrength',s.minimumHeadingStrength, ...
                'maximumLidarCoupling',s.maximumPositionHeadingCoupling, ...
                'minimumGnssWeight',z.minimumGnssWeight, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).', ...
                'gyroGainNormalized',gyroGain,'speedGainNormalized',speedGain);
            p=jsondecode(jsonencode(p));
        end

        function w=witness(cfg,P,kL,kG,kI,gyroGain,speedGain)
            w=struct('inputs',sensorRedundancySupport.inputs(cfg,gyroGain,speedGain), ...
                'domain',cfg.invariantObserver.redundancy,'P',P,'kappaL',kL,'kappaG',kG, ...
                'accelerationGain',kI(2),'directionGain',kI(5));
        end

        function v=verify(w,includeGnss,velocityAvailable)
            [L,G]=sensorRedundancySupport.blocks(w,includeGnss,velocityAvailable);
            worstL=worstEigenvalue(L);worstG=worstEigenvalue(G);
            v=struct('certified',worstL<0 && worstG<0, ...
                'worstLidarEigenvalue',worstL,'worstGnssEigenvalue',worstG, ...
                'lidarVertices',numel(L),'gnssBernsteinVertices',numel(G), ...
                'vertices',numel(L)+numel(G),'gnssAdequacyEnabled',includeGnss, ...
                'minimumGnssCourseSpeed',w.domain.minimumGnssSpeed);
            if isempty(G),v.worstGnssEigenvalue=[];end
        end

        function [L,G]=blocks(w,includeGnss,velocityAvailable)
            arguments
                w (1,1) struct
                includeGnss (1,1) logical=true
                velocityAvailable (1,1) logical=true
            end
            p=w.inputs;d=w.domain;P=w.P;th=p.theta;ell=p.headingLengthM;
            E=eye(8);e2=E(:,2);e3=E(:,3);e5=E(:,5);e6=E(:,6);e7=E(:,7);e8=E(:,8);
            C=E([1,4,7],:);Cp=C(1:2,:);Dacc=e3*e3.'+e6*e6.';
            q=p.maximumCourseRate;b=2*q*p.maximumSpeed+2*p.maximumAcceleration;
            rho=norm(p.gnssBodyOffset)/ell;tau=d.gnssSplit;eta=d.accelerationSplit;
            A0=zeros(8);A0(1,2)=1;A0(2,3)=1;A0(4,5)=1;A0(5,6)=1;A0(7,8)=1;
            corners=[1,1;1,-1;-1,1;-1,-1];speeds=linspace(d.minimumGnssSpeed,p.maximumSpeed,d.speedPartitions+1);
            L={};G={};speedModes=0;if velocityAvailable,speedModes=[0,1];end
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
            for u=[0,q^2]
                for rate=[-q,q]
                    for j=1:4
                        A=A0;A([3,6],[2,5])=u/th^2*eye(2);
                        A([3,6],[3,6])=2*rate/th*[0,-1;1,0];
                        A([3,6],8)=b/(th^2*ell)*corners(j,:).';
                        for k=1:4
                            c=p.maximumLidarCoupling*corners(k,:).';
                            S=[p.minimumLidarPositionStrength*eye(2),c;c.',p.minimumLidarHeadingStrength];
                            for speedMode=speedModes
                                M=P*(A-p.gyroGainNormalized*(e8*e8.')-speedMode*p.speedGainNormalized*(e2*e2.')) ...
                                    -w.kappaL*C.'*S*C;
                                L{end+1}=M+M.'+w.kappaG*rho^2*(e7*e7.') ...
                                    +2*w.kappaL*d.informativeLidarMismatch*(C.'*C)+d.lambda*E; %#ok<AGROW>
                            end
                        end
                        if ~includeGnss,continue;end
                        Ag=A;
                        if isfield(d,'kinematicGnssEnclosure') && d.kinematicGnssEnclosure
                            % In the true course frame a_perp=s*q, hence
                            % d(f_a)/d(r)=[0;2*a_parallel]. Also |q|<=a_max/s0.
                            % Upstream angle/rate errors are bounded inputs;
                            % the ideal true-point enclosure uses this identity.
                            if j>2,continue;end
                            qg=min(q,p.maximumAcceleration/d.minimumGnssSpeed);
                            Ag([3,6],[2,5])=double(u>0)*qg^2/th^2*eye(2);
                            Ag([3,6],[3,6])=2*sign(rate)*qg/th*[0,-1;1,0];
                            Ag([3,6],8)=[0;2*p.maximumAcceleration/(th^2*ell)*corners(j,2)];
                        end
                        for k=1:d.speedPartitions
                            left=e5-speeds(k)/(th*ell)*e7;right=e5-speeds(k+1)/(th*ell)*e7;
                            B=cat(3,left*left.',(left*right.'+right*left.')/2,right*right.');
                            for jB=1:3
                                for iW=1:numel(Wg)
                                    for iR=1:size(lever,2)
                                        if exactLever
                                            gnss=-w.kappaG*Cp.'*Wg{iW}*(Cp+lever(:,iR)*e7.');penalty=0;
                                        else
                                            gnss=-w.kappaG*(1-tau)*p.minimumGnssWeight*(Cp.'*Cp);penalty=w.kappaG*rho^2/tau;
                                        end
                                        M=P*(Ag-p.gyroGainNormalized*(e8*e8.')-p.speedGainNormalized*(e2*e2.')) ...
                                            +gnss-w.directionGain*B(:,:,jB)-(1-eta)*w.accelerationGain*Dacc;
                                        G{end+1}=M+M.'+(penalty ...
                                            +2*w.accelerationGain*(1/eta-1)*p.maximumAcceleration^2/(th^4*ell^2))*(e7*e7.') ...
                                            +2*w.kappaL*d.degradedLidarMismatch*(C.'*C)+d.lambda*E; %#ok<AGROW>
                                    end
                                end
                            end
                        end
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
