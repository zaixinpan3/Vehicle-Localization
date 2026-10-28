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
                'speedOutput',"raw-squared-speed-matched-v3", ...
                'allAuxiliaryOutputsRequired',true, ...
                'auxiliaryOutputs',"squared-dot-cross-direction-rate-v1", ...
                'motionEnclosure',"physical-acceleration-multiplier-v2", ...
                'minimumGnssWeight',z.minimumGnssWeight, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).', ...
                'gyroGainNormalized',gyroGain);
            p=jsondecode(jsonencode(p));
        end

        function w=witness(cfg,P,kL,kG,kI,gyroGain,multipliers)
            [~,groups]=coupledMotionSupport.multiplierGroups(cfg.invariantObserver.redundancy);
            if nargin<7,multipliers=repmat({zeros(8)},3,groups);end
            w=struct('inputs',sensorRedundancySupport.inputs(cfg,gyroGain), ...
                'domain',cfg.invariantObserver.redundancy,'P',P,'kappaL',kL,'kappaG',kG, ...
                'speedSquaredGain',kI(1),'bilinearGain',kI(2),'directionGain',kI(3), ...
                'accelerationMultipliers',{multipliers});
        end

        function v=verify(w,includeGnss)
            [L,G,M]=sensorRedundancySupport.blocks(w,includeGnss);
            [penalty,valid,minimum]=sensorRedundancySupport.multiplierResidual(w.accelerationMultipliers);
            raw=[worstEigenvalue(L),worstEigenvalue(G),worstEigenvalue(M)];
            worst=raw+max(penalty,[],2).';worstL=worst(1);worstG=worst(2);worstM=worst(3);
            v=struct('certified',valid && all(worst<0), ...
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
                'physicalAccelerationConstraintRetained',true, ...
                'accelerationDomainMultiplierActive',any(cellfun(@(T)any(T~=0,'all'),w.accelerationMultipliers),'all'), ...
                'multiplierPsdVerified',valid,'minimumMultiplierEigenvalues',minimum, ...
                'multiplierRoundoffPenalties',penalty,'worstAugmentedEigenvalues',raw);
            if isempty(G),v.worstGnssEigenvalue=[];end
            if isempty(M),v.worstMixedEigenvalue=[];end
        end

        function [penalty,valid,minimum]=multiplierResidual(multipliers)
        % A small negative eigenvalue is charged to the physical LMI bound:
        % F=F_aug-g*T <= F_aug+max(0,-lambda_min(T))*I, since 0<=g<=1.
        % Thus numerical PSD tolerance never silently weakens the certificate.
            assert(iscell(multipliers) && size(multipliers,1)==3 && size(multipliers,2)>=1, ...
                'VehicleLocalization:InvalidDomainMultiplier','Require three rows of domain multipliers.');
            minimum=zeros(size(multipliers));valid=true;
            for j=1:numel(multipliers)
                T=multipliers{j};validateattributes(T,{'double'},{'real','finite','size',[8,8]});
                scale=max(1,norm(T,2));minimum(j)=min(eig((T+T.')/2));
                valid=valid && norm(T-T.','fro')<=1e-10*scale && minimum(j)>=-1e-8*scale;
            end
            penalty=max(0,-minimum);
        end

        function counts=counts(w,includeGnss)
            if nargin<2,includeGnss=true;end
            sensor=sensorRedundancySupport.sensorBlocks(w,includeGnss);
            d=w.domain;n=27*d.speedPartitions*d.coursePartitions*d.accelerationPartitions;
            counts=n*cellfun(@numel,sensor);
        end

        function [L,G,Mixed]=blocks(w,includeGnss,selected)
        % Optional global control IDs instantiate only the active SDP cuts.
        % Numeric verification still evaluates the complete finite family.
            if nargin<2,includeGnss=true;end
            if nargin<3,selected=[];end
            p=w.inputs;d=w.domain;
            validateattributes(d.minimumCourseSpeed,{'double'}, ...
                {'real','finite','positive','scalar','<=',p.maximumSpeed});
            controls=coupledMotionSupport.controls(p,d);
            domainControls=coupledMotionSupport.accelerationConstraint(p,controls);
            [group,groups]=coupledMotionSupport.multiplierGroups(d);
            assert(isequal(size(w.accelerationMultipliers),[3,groups]), ...
                'VehicleLocalization:InvalidDomainMultiplier','Multiplier partition does not match this domain.');
            sensor=sensorRedundancySupport.sensorBlocks(w,includeGnss);
            sizes=cellfun(@numel,sensor);counts=size(controls,1)*sizes;
            offsets=[0,cumsum(counts)];requested=cell(1,3);needed=[];
            for family=1:3
                if isempty(selected)
                    requested{family}=1:counts(family);
                else
                    requested{family}=selected(selected>offsets(family) & selected<=offsets(family+1))-offsets(family);
                end
                if sizes(family)>0
                    needed=[needed,ceil(requested{family}/sizes(family))]; %#ok<AGROW>
                end
            end
            common=cell(1,size(controls,1));
            for j=unique(needed)
                [A,speed,pair,direction]=coupledMotionSupport.matrices(p,controls(j,:));
                M=w.P*A-w.speedSquaredGain*speed-w.bilinearGain*pair-w.directionGain*direction;
                common{j}=M+M.'+d.lambda*eye(8);
            end
            result=cell(1,3);
            for family=1:3
                ids=requested{family};result{family}=cell(1,numel(ids));
                for j=1:numel(ids)
                    motion=ceil(ids(j)/sizes(family));pose=mod(ids(j)-1,sizes(family))+1;
                    result{family}{j}=common{motion}+sensor{family}{pose} ...
                        +domainControls(motion)*w.accelerationMultipliers{family,group(motion)};
                end
            end
            L=result{1};G=result{2};Mixed=result{3};
        end

        function result=sensorBlocks(w,includeGnss)
        % Preserve the coupled GNSS/LiDAR information contracts independently
        % of the joint motion polynomial. The returned terms are affine gains.
            p=w.inputs;d=w.domain;ell=p.headingLengthM;
            E=eye(8);e7=E(:,7);C=E([1,4,7],:);Cp=C(1:2,:);
            rho=norm(p.gnssBodyOffset)/ell;tau=d.gnssSplit;
            corners=[1,1;1,-1;-1,1;-1,-1];
            L={};G={};Mixed={};
            for j=1:4
                c=p.maximumLidarCoupling*corners(j,:).';
                S=[p.minimumLidarPositionStrength*eye(2),c;c.',p.minimumLidarHeadingStrength];
                L{end+1}=-2*w.kappaL*C.'*S*C+w.kappaG*rho^2*(e7*e7.') ...
                    +2*w.kappaL*d.informativeLidarMismatch*(C.'*C); %#ok<AGROW>
            end
            if ~includeGnss,result={L,G,Mixed};return;end
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
                Wg={p.minimumGnssWeight*eye(2)};
            end
            mixedLidar=sensorRedundancySupport.mixedLidarLowerControls(p);
            for iW=1:numel(Wg)
                for iR=1:size(lever,2)
                    if exactLever
                        gnss=-w.kappaG*Cp.'*Wg{iW}*(Cp+lever(:,iR)*e7.');penalty=0;
                    else
                        gnss=-w.kappaG*(1-tau)*p.minimumGnssWeight*(Cp.'*Cp);penalty=w.kappaG*rho^2/tau;
                    end
                    paired=gnss+gnss.'+penalty*(e7*e7.');
                    G{end+1}=paired+2*w.kappaL*d.degradedLidarMismatch*(C.'*C); %#ok<AGROW>
                    for iS=1:numel(mixedLidar)
                        Mixed{end+1}=paired-2*w.kappaL*C.'*mixedLidar{iS}*C ...
                            +2*w.kappaL*d.informativeLidarMismatch*(C.'*C); %#ok<AGROW>
                    end
                end
            end
            result={L,G,Mixed};
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
