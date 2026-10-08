classdef overlapGradientChannelTest < matlab.unittest.TestCase
% overlapGradientChannelTest Correspondence-free semanticGaussianOverlap LiDAR channel.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function gradientIsTheDerivativeOfNegativeLogScore(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig();
            pose=truth+[0.15,-0.10,deg2rad(1.5)];
            m=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose(:),cfg);
            steps=[1e-6,1e-6,1e-7];expected=zeros(3,1);
            for j=1:3
                e=zeros(1,3);e(j)=steps(j);
                plus=registrationSupport.scoreSemanticProbabilityCloudAlignment(map,source,pose+e,cfg.registration);
                minus=registrationSupport.scoreSemanticProbabilityCloudAlignment(map,source,pose-e,cfg.registration);
                expected(j)=-(log(plus)-log(minus))/(2*steps(j));
            end
            testCase.verifyTrue(m.available);
            testCase.verifyEqual(m.similarity,registrationSupport.scoreSemanticProbabilityCloudAlignment( ...
                map,source,pose,cfg.registration),RelTol=1e-12);
            testCase.verifyEqual(m.gradient,expected,RelTol=1e-5,AbsTol=1e-8);
            testCase.verifyEqual(m.linearizationPose,pose(:));
        end
        function informationFollowsTheCurvaturePolicy(testCase)
            [map,source,truth]=scene();pose=(truth+[0.6,0.4,deg2rad(6)]).';
            m=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose);
            [V,E]=eig((m.curvature+m.curvature.')/2,'vector');
            testCase.verifyLessThan(min(E),0);
            testCase.verifyEqual(m.curvature,m.curvature.',AbsTol=1e-12);
            testCase.verifyEqual(m.information,V*diag(abs(E))*V.',AbsTol=1e-9*max(1,norm(m.curvature)));
            cfg=overlapGradientConfig();cfg.curvature="positivePart";
            p=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose,cfg);
            testCase.verifyEqual(p.information,V*diag(max(E,0))*V.',AbsTol=1e-9*max(1,norm(m.curvature)));
            testCase.verifyGreaterThanOrEqual(min(eig(m.information)),-1e-9*max(1,norm(m.information)));
            testCase.verifyEqual(m.minimumCurvature,min(E),AbsTol=1e-12);
        end
        function exactAlignmentIsStationary(testCase)
            [map,source,truth]=scene();
            m=lidarInjectionSupport.evaluateOverlapGradient(map,source,truth(:));
            testCase.verifyLessThan(norm(m.gradient),1e-9);
            testCase.verifyGreaterThan(min(eig(m.information)),0);
            testCase.verifyEqual(m.similarity,1,AbsTol=1e-12);
        end
        function smoothedGradientIsTheDerivativeOfItsNegativeLogSimilarity(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig();cfg.kernelBandwidth=0.5;
            pose=(truth+[0.15,-0.10,deg2rad(1.5)]).';
            m=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose,cfg);
            steps=[1e-6;1e-6;1e-7];expected=zeros(3,1);
            for j=1:3
                e=zeros(3,1);e(j)=steps(j);
                plus=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose+e,cfg).similarity;
                minus=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose-e,cfg).similarity;
                expected(j)=-(log(plus)-log(minus))/(2*steps(j));
            end
            testCase.verifyEqual(m.gradient,expected,RelTol=1e-5,AbsTol=1e-8);
            aligned=lidarInjectionSupport.evaluateOverlapGradient(map,source,truth(:),cfg);
            exact=lidarInjectionSupport.evaluateOverlapGradient(map,source,truth(:));
            testCase.verifyLessThan(norm(aligned.gradient),1e-9);
            testCase.verifyEqual(aligned.similarity,1,AbsTol=1e-12);
            testCase.verifyLessThan(trace(aligned.information),trace(exact.information));
        end
        function smoothingExtendsTheConvexBasin(testCase)
            [map,source,truth]=scene();pose=truth(:);heading=[cos(truth(3));sin(truth(3))];
            pose(1:2)=pose(1:2)+heading; % one metre along the curbs
            cfg=overlapGradientConfig();cfg.kernelBandwidth=1;
            exact=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose);
            smoothed=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose,cfg);
            testCase.verifyLessThan(exact.minimumCurvature,0);
            testCase.verifyGreaterThan(smoothed.minimumCurvature,0);
            testCase.verifyLessThan(-smoothed.gradient(1:2).'*heading,0);
            invalid=cfg;invalid.kernelBandwidth=-1;
            testCase.verifyError(@()lidarInjectionSupport.evaluateOverlapGradient(map,source,pose,invalid), ...
                'MATLAB:expectedNonnegative');
        end
        function mixtureMassScaleDoesNotChangeTheChannel(testCase)
            [map,source,truth]=scene();pose=(truth+[0.2,0.1,deg2rad(2)]).';
            a=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose);
            map.components.mixtureWeight=7*map.components.mixtureWeight;
            source.components.mixtureWeight=0.3*source.components.mixtureWeight;
            b=lidarInjectionSupport.evaluateOverlapGradient(map,source,pose);
            testCase.verifyEqual(b.gradient,a.gradient,RelTol=1e-10,AbsTol=1e-12);
            testCase.verifyEqual(b.information,a.information,RelTol=1e-9,AbsTol=1e-9);
        end
        function missingOverlapIsUnavailableWithoutCorrection(testCase)
            [map,source,truth]=scene();
            far=lidarInjectionSupport.evaluateOverlapGradient(map,source,(truth+[500,0,0]).');
            testCase.verifyFalse(far.available);
            testCase.verifyEqual(far.gradient,zeros(3,1));testCase.verifyEqual(far.information,zeros(3));
            empty=source;empty.components=selectComponents(source.components,[]);
            none=lidarInjectionSupport.evaluateOverlapGradient(map,empty,truth(:));
            testCase.verifyFalse(none.available);
        end
        function gradientModeMatchesResidualMode(testCase)
            rng(20261008);J=randn(9,3);r=randn(9,1);pose=[1;2;0.1];
            design=fullObserverDesign();cfg=fullObserverConfig();
            residual=struct('residual',r,'jacobian',J,'weights',ones(9,1),'linearizationPose',pose);
            gradient=struct('gradient',J.'*r,'information',J.'*J,'linearizationPose',pose);
            G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            [a,auditA]=lidarInjectionSupport.computeLidarMatchedCorrection(residual,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            [b,auditB]=lidarInjectionSupport.computeLidarMatchedCorrection(gradient,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(b,a,AbsTol=1e-12);
            testCase.verifyEqual(auditB.filter.S,auditA.filter.S,AbsTol=1e-12);
            testCase.verifyEqual(auditB.representation,"predictedPoseGradient");
            testCase.verifyTrue(auditB.jumpNonexpansive);
        end
        function unsupportedGradientDirectionsReceiveNoCorrection(testCase)
            design=fullObserverDesign();cfg=fullObserverConfig();G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            pose=[0;0;0];information=diag([5,0,40]);
            m=struct('gradient',[1;2;3],'information',information,'linearizationPose',pose);
            [delta,audit]=lidarInjectionSupport.computeLidarMatchedCorrection(m,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(delta(4),0,AbsTol=1e-15);
            testCase.verifyLessThan(delta(1),0);testCase.verifyLessThan(delta(7),0);
            testCase.verifyEqual(audit.unsupportedGradientNorm,2,AbsTol=1e-12);
            projected=m;projected.gradient=[1;0;3];
            [same,~]=lidarInjectionSupport.computeLidarMatchedCorrection(projected,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(delta,same,AbsTol=1e-15);
        end
        function gradientModeRejectsStaleOrAmbiguousInput(testCase)
            design=fullObserverDesign();cfg=fullObserverConfig();G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            stale=struct('gradient',[1;0;0],'information',eye(3),'linearizationPose',[0;0;0]);
            testCase.verifyError(@()lidarInjectionSupport.computeLidarMatchedCorrection(stale,[1;0;0],G,design,cfg), ...
                'VehicleLocalization:StaleLidarLinearization');
            ambiguous=stale;ambiguous.pose=[0;0;0];
            testCase.verifyError(@()lidarInjectionSupport.computeLidarMatchedCorrection(ambiguous,[0;0;0],G,design,cfg), ...
                'VehicleLocalization:AmbiguousLidarMeasurement');
            both=stale;both.residual=0;both.jacobian=zeros(1,3);both.weights=1;
            testCase.verifyError(@()lidarInjectionSupport.computeLidarMatchedCorrection(both,[0;0;0],G,design,cfg), ...
                'VehicleLocalization:AmbiguousLidarMeasurement');
        end
        function observerConvergesWithoutRegistration(testCase)
            f=movingFixture(5);r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
            error=r.pose-f.truth;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
            testCase.verifyEqual(r.diagnostics.lidarChannel,"overlapGradient");
            testCase.verifyTrue(all(r.diagnostics.lidarOverlap.available(2:end)));
            testCase.verifyGreaterThan(norm(error(1,1:2)),0.2);
            testCase.verifyLessThan(norm(error(end,1:2)),1e-4);
            testCase.verifyLessThan(abs(rad2deg(error(end,3))),1e-3);
            testCase.verifyEqual(r.diagnostics.lidarMatched{end}.representation,"predictedPoseGradient");
        end
        function unavailableFramesWithdrawOnlyTheLidarCorrection(testCase)
            f=movingFixture(2);f.data.lidarOverlap.valid=true(numel(f.data.highRate.time),1);
            f.data.lidarOverlap.valid(8:12)=false;
            r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
            testCase.verifyEqual(r.diagnostics.mode(8:12),zeros(5,1));
            testCase.verifyFalse(any(r.diagnostics.lidarOverlap.evaluated(8:12)));
            testCase.verifyEqual(r.diagnostics.positionCorrection(8:12,3:4),zeros(5,2));
        end
        function overlapChannelIsExclusiveAndRejectsCalibratedWeights(testCase)
            f=movingFixture(1);
            ambiguous=f.data;ambiguous.lidarMatcher=@(~,~,~)struct();
            testCase.verifyError(@()runFullLocalizationObserver(ambiguous,struct(),f.cfg,LateralInputs=f.lateral), ...
                'VehicleLocalization:AmbiguousLidarInput');
            calibrated=mncavFullObserverConfig(GainProfile="mississippi-20240607-calibrated");
            calibrated.initialState=f.cfg.initialState;
            testCase.verifyError(@()runFullLocalizationObserver(f.data,struct(),calibrated,LateralInputs=f.lateral), ...
                'VehicleLocalization:OverlapCalibrationUnavailable');
        end
    end
end

function [map,source,truth]=scene()
% Two curbs, two poles and one sign, expressed in world and body coordinates.
    means=[-6 -3;0 -3.2;6 -3;-2 4;3 4.5;7 2];
    covariance=cat(3,[2.5 0;0 0.03],[2.5 0;0 0.03],[2.5 0;0 0.03], ...
        [0.04 0;0 0.04],[0.05 0.01;0.01 0.04],[0.2 0.05;0.05 0.03]);
    c=struct('mean',means,'covariance',covariance, ...
        'semanticName',["curb";"curb";"curb";"pole";"pole";"trafficSign"], ...
        'mixtureWeight',[0.2;0.2;0.2;0.15;0.15;0.1],'numComponents',6);
    c.repeatability=ones(6,1);
    truth=[100,50,deg2rad(20)];
    source=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
    map=source;map.components=place(c,truth);
end

function c=place(c,pose)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    c.mean=c.mean*r.'+pose(1:2);
    for k=1:c.numComponents,c.covariance(:,:,k)=r*c.covariance(:,:,k)*r.';end
end

function c=selectComponents(c,keep)
    c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);c.semanticName=c.semanticName(keep);
    c.mixtureWeight=c.mixtureWeight(keep);c.repeatability=c.repeatability(keep);c.numComponents=numel(keep);
end

function f=movingFixture(duration)
% Constant-speed straight drive; every source is the map in current body axes.
    [map,source,start]=scene();
    t=(0:.1:duration).';n=numel(t);zero=zeros(n,1);speed=2;
    truth=[start(1)+speed*cos(start(3))*t,start(2)+speed*sin(start(3))*t,start(3)+zero];
    sources=cell(n,1);
    for k=1:n
        inverse=[-truth(k,1:2)*[cos(truth(k,3)) -sin(truth(k,3));sin(truth(k,3)) cos(truth(k,3))],-truth(k,3)];
        sources{k}=source;sources{k}.components=place(map.components,inverse);
    end
    high=struct('time',t,'longitudinalSpeed',speed+zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngleRate',zero);
    data=struct('highRate',high,'lidarOverlap',struct('map',map,'sources',{sources}));
    cfg=fullObserverConfig();
    initial=truth(1,:)+[0.25,-0.20,deg2rad(1)];
    cfg.initialState=[initial(1);speed*cos(start(3));0;initial(2);speed*sin(start(3));0;initial(3)];
    f=struct('data',data,'lateral',lateral,'cfg',cfg,'truth',truth);
end

function design=fullObserverDesign()
    design=struct('P',blkdiag(eye(6),1),'T',eye(7),'kappa',4);
end
