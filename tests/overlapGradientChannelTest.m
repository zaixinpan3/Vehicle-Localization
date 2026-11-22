classdef (SharedTestFixtures={freshObserverGainsFixture}) overlapGradientChannelTest < matlab.unittest.TestCase
% overlapGradientChannelTest Correspondence-free semanticGaussianOverlap LiDAR channel.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function singleChannelGradientMatchesIndependentAlignmentScore(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig();
            map.components=selectComponents(map.components,1:3);source.components=selectComponents(source.components,1:3);
            pose=truth+[0.15,-0.10,deg2rad(1.5)];
            m=localizationSupport.evaluateOverlapGradient(map,source,pose(:),cfg);
            steps=[1e-6,1e-6,1e-7];expected=zeros(3,1);
            for j=1:3
                e=zeros(1,3);e(j)=steps(j);
                plus=localizationSupport.scoreSemanticProbabilityCloudAlignment(map,source,pose+e,cfg.geometry);
                minus=localizationSupport.scoreSemanticProbabilityCloudAlignment(map,source,pose-e,cfg.geometry);
                expected(j)=-(log(plus)-log(minus))/(2*steps(j));
            end
            testCase.verifyTrue(m.available);
            testCase.verifyEqual(m.similarity,localizationSupport.scoreSemanticProbabilityCloudAlignment( ...
                map,source,pose,cfg.geometry),RelTol=1e-12);
            testCase.verifyEqual(m.gradient,expected,RelTol=1e-5,AbsTol=1e-8);
            testCase.verifyEqual(m.linearizationPose,pose(:));
        end
        function uniformBackgroundMatchesTheScoreAndAttenuatesASpuriousSign(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig(UniformBackground=true);
            source.components.mean(end,1)=source.components.mean(end,1)+6;
            p=truth(:)+[.08;-.02;.006];
            m=localizationSupport.evaluateOverlapGradient(map,source,p,cfg);
            sign=m.channels([m.channels.name]=="trafficSign");curb=m.channels([m.channels.name]=="curb");
            testCase.verifyLessThan(sign.supportFraction,.001);
            testCase.verifyGreaterThan(curb.supportFraction,.99);
            testCase.verifyGreaterThan(sign.similarity,sign.rawSimilarity);
            for j=1:3
                step=zeros(3,1);step(j)=1e-6;
                a=localizationSupport.evaluateOverlapGradient(map,source,p+step,cfg);
                b=localizationSupport.evaluateOverlapGradient(map,source,p-step,cfg);
                testCase.verifyEqual(m.gradient(j),-(log(a.similarity)-log(b.similarity))/2e-6,AbsTol=2e-6);
            end
            testCase.verifyGreaterThanOrEqual(min(eig(m.information)),-1e-12);
        end
        function featureChannelsMatchIndependentClassEvaluations(testCase)
            [map,source,truth]=scene();pose=truth(:)+[0.35;-0.2;0.04];
            cfg=overlapGradientConfig();
            result=localizationSupport.evaluateOverlapGradient(map,source,pose,cfg);
            names=unique(source.components.semanticName);
            testCase.verifyEqual([result.channels.name],names.');
            gradients=zeros(3,numel(names));metrics=zeros(3,3,numel(names));
            scores=zeros(size(names));
            for c=1:numel(names)
                indices=find(source.components.semanticName==names(c));
                fixed=map;moving=source;
                fixed.components=selectComponents(map.components,indices);
                moving.components=selectComponents(source.components,indices);
                one=localizationSupport.evaluateOverlapGradient(fixed,moving,pose,cfg);
                scores(c)=one.similarity;gradients(:,c)=one.gradient;metrics(:,:,c)=one.information;
                testCase.verifyEqual(result.channels(c).similarity,scores(c),AbsTol=1e-12);
                testCase.verifyEqual(result.channels(c).gradient,one.gradient,AbsTol=1e-10);
                testCase.verifyEqual(result.channels(c).information,one.information,AbsTol=1e-10);
            end
            testCase.verifyEqual(result.similarity,exp(mean(log(scores))),AbsTol=1e-12);
            testCase.verifyEqual(result.gradient,mean(gradients,2),AbsTol=1e-10);
            testCase.verifyEqual(result.information,mean(metrics,3),AbsTol=1e-10);
        end
        function unevaluableChannelIsOmitted(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig();
            % The sign lies far from its sole partner, but is inside the map crop.
            map.components.mean(6,1)=map.components.mean(6,1)+45;
            result=localizationSupport.evaluateOverlapGradient(map,source,truth(:),cfg);
            sign=result.channels([result.channels.name]=="trafficSign");
            testCase.verifyTrue(isnan(sign.similarity));
            costs=arrayfun(@(ch) -log(ch.similarity),result.channels);
            testCase.verifyEqual(result.similarity,exp(-mean(costs,'omitnan')),AbsTol=1e-12);
        end
        function gaussNewtonMetricIsExactForOnePairAtAnyDistance(testCase)
            % One sign in map and source: -log E is quadratic in translation
            % with the exact pair precision inv(Sigma + R*Lambda*R'), so the
            % translation Gauss-Newton step from any offset lands on the optimum.
            % With one pair the 3-by-3 metric has rank two.
            [map,source,truth]=scene();
            map.components=selectComponents(map.components,6);source.components=selectComponents(source.components,6);
            R=[cos(truth(3)) -sin(truth(3));sin(truth(3)) cos(truth(3))];
            pair=map.components.covariance(:,:,1)+R*source.components.covariance(:,:,1)*R.';
            precision=inv(pair);
            offsets=[0.1 0;1 0;0 2;-3 1.5];
            for k=1:size(offsets,1)
                pose=truth(:);pose(1:2)=pose(1:2)+offsets(k,:).';
                m=localizationSupport.evaluateOverlapGradient(map,source,pose);
                testCase.verifyTrue(m.available);
                testCase.verifyEqual(m.information(1:2,1:2),precision,RelTol=1e-8);
                step=-m.information(1:2,1:2)\m.gradient(1:2);
                testCase.verifyEqual(pose(1:2)+step,truth(1:2).',AbsTol=1e-8);
                testCase.verifyEqual(rank(m.information,1e-9*norm(m.information)),2);
            end
            full=localizationSupport.evaluateOverlapGradient(map,source,(truth+[0.3,-0.2,deg2rad(3)]).');
            testCase.verifyGreaterThanOrEqual(min(eig(full.information)),0);
            testCase.verifyEqual(full.information,full.information.',AbsTol=1e-12);
        end
        function exactAlignmentIsStationary(testCase)
            [map,source,truth]=scene();
            m=localizationSupport.evaluateOverlapGradient(map,source,truth(:));
            testCase.verifyLessThan(norm(m.gradient),1e-9);
            testCase.verifyGreaterThan(min(eig(m.information)),0);
            testCase.verifyEqual(m.similarity,1,AbsTol=1e-12);
        end
        function gradientMatchesNegativeLogSimilarityDerivative(testCase)
            [map,source,truth]=scene();cfg=overlapGradientConfig();
            pose=(truth+[0.15,-0.10,deg2rad(1.5)]).';
            m=localizationSupport.evaluateOverlapGradient(map,source,pose,cfg);
            steps=[1e-6;1e-6;1e-7];expected=zeros(3,1);
            for j=1:3
                e=zeros(3,1);e(j)=steps(j);
                plus=localizationSupport.evaluateOverlapGradient(map,source,pose+e,cfg).similarity;
                minus=localizationSupport.evaluateOverlapGradient(map,source,pose-e,cfg).similarity;
                expected(j)=-(log(plus)-log(minus))/(2*steps(j));
            end
            testCase.verifyEqual(m.gradient,expected,RelTol=1e-5,AbsTol=1e-8);
            testCase.verifyEqual(m.minimumInformation,min(eig(m.information)),AbsTol=1e-12);
        end
        function mixtureMassScaleDoesNotChangeTheChannel(testCase)
            [map,source,truth]=scene();pose=(truth+[0.2,0.1,deg2rad(2)]).';
            a=localizationSupport.evaluateOverlapGradient(map,source,pose);
            map.components.mixtureWeight=7*map.components.mixtureWeight;
            source.components.mixtureWeight=0.3*source.components.mixtureWeight;
            b=localizationSupport.evaluateOverlapGradient(map,source,pose);
            testCase.verifyEqual(b.gradient,a.gradient,RelTol=1e-10,AbsTol=1e-12);
            testCase.verifyEqual(b.information,a.information,RelTol=1e-9,AbsTol=1e-9);
        end
        function missingOverlapIsUnavailableWithoutCorrection(testCase)
            [map,source,truth]=scene();
            far=localizationSupport.evaluateOverlapGradient(map,source,(truth+[500,0,0]).');
            testCase.verifyFalse(far.available);
            testCase.verifyEqual(far.gradient,zeros(3,1));testCase.verifyEqual(far.information,zeros(3));
            empty=source;empty.components=selectComponents(source.components,[]);
            none=localizationSupport.evaluateOverlapGradient(map,empty,truth(:));
            testCase.verifyFalse(none.available);
        end
        function gradientModeMatchesResidualMode(testCase)
            rng(20261008);J=randn(9,3);r=randn(9,1);pose=[1;2;0.1];
            design=fullObserverDesign();cfg=filterConfig();
            residual=struct('residual',r,'jacobian',J,'weights',ones(9,1),'linearizationPose',pose);
            gradient=struct('gradient',J.'*r,'information',J.'*J,'linearizationPose',pose);
            G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            [a,auditA]=localizationSupport.computeLidarMatchedCorrection(residual,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            [b,auditB]=localizationSupport.computeLidarMatchedCorrection(gradient,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(b,a,AbsTol=1e-12);
            testCase.verifyEqual(auditB.filter.S,auditA.filter.S,AbsTol=1e-12);
            testCase.verifyEqual(auditB.representation,"predictedPoseGradient");
            testCase.verifyTrue(auditB.jumpNonexpansive);
        end
        function unsupportedGradientDirectionsReceiveNoCorrection(testCase)
            design=fullObserverDesign();cfg=filterConfig();G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            pose=[0;0;0];information=diag([5,0,40]);
            m=struct('gradient',[1;2;3],'information',information,'linearizationPose',pose);
            [delta,audit]=localizationSupport.computeLidarMatchedCorrection(m,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(delta(4),0,AbsTol=1e-15);
            testCase.verifyLessThan(delta(1),0);testCase.verifyLessThan(delta(7),0);
            testCase.verifyEqual(audit.unsupportedGradientNorm,2,AbsTol=1e-12);
            projected=m;projected.gradient=[1;0;3];
            [same,~]=localizationSupport.computeLidarMatchedCorrection(projected,pose,G,design,cfg, ...
                StepSize=0.1,Discretization="implicit");
            testCase.verifyEqual(delta,same,AbsTol=1e-15);
        end
        function gradientModeRejectsStaleOrAmbiguousInput(testCase)
            design=fullObserverDesign();cfg=filterConfig();G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            stale=struct('gradient',[1;0;0],'information',eye(3),'linearizationPose',[0;0;0]);
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(stale,[1;0;0],G,design,cfg), ...
                'VehicleLocalization:StaleLidarLinearization');
            ambiguous=stale;ambiguous.pose=[0;0;0];
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(ambiguous,[0;0;0],G,design,cfg), ...
                'VehicleLocalization:AmbiguousLidarMeasurement');
            both=stale;both.residual=0;both.jacobian=zeros(1,3);both.weights=1;
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(both,[0;0;0],G,design,cfg), ...
                'VehicleLocalization:AmbiguousLidarMeasurement');
        end
        function channelOverlapReducesErrorOutsideTheIssSpeedDomain(testCase)
            % This 2 m/s finite-landmark scene is outside the default ISS
            % interval. Check finite execution and finite-horizon reduction;
            % neither asymptotic convergence nor old settling rates follow.
            f=movingFixture(20);r=runFullLocalizationObserver(f.data,f.cfg,LateralInputs=f.lateral);
            error=r.pose-f.truth;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
            testCase.verifyEqual(r.diagnostics.lidarChannel,"overlapGradient");
            testCase.verifyTrue(all(r.diagnostics.lidarOverlap.available(2:end)));
            audit=r.diagnostics.lidarOverlap;
            testCase.verifyEqual([audit.channels{end}.name],["curb","pole","trafficSign"]);
            testCase.verifyGreaterThan(norm(error(1,1:2)),0.2);
            testCase.verifyTrue(all(isfinite(r.internalState),'all'));
            testCase.verifyLessThan(norm(error(end,1:2)),.1*norm(error(1,1:2)));
            testCase.verifyLessThan(abs(error(end,3)),.1*abs(error(1,3)));
            testCase.verifyFalse(any(r.diagnostics.observedSpeedWithinIssDomain));
            testCase.verifyFalse(r.diagnostics.allTheoremHypothesesVerified);
            testCase.verifyEqual(r.diagnostics.lidarMatched{end}.representation,"predictedPoseGradient");
        end
        function unavailableFramesWithdrawOnlyTheLidarCorrection(testCase)
            f=movingFixture(2);f.data.lidarOverlap.valid=true(numel(f.data.highRate.time),1);
            f.data.lidarOverlap.valid(8:12)=false;
            r=runFullLocalizationObserver(f.data,f.cfg,LateralInputs=f.lateral);
            testCase.verifyEqual(r.diagnostics.gnssPoseValid(8:12) | r.diagnostics.lidarValid(8:12),false(5,1));
            testCase.verifyFalse(any(r.diagnostics.lidarOverlap.evaluated(8:12)));
            testCase.verifyEqual(r.diagnostics.positionCorrection(8:12,3:4),zeros(5,2));
        end
        function evidenceWeightsScaleMassesBeforeBalancing(testCase)
            [map,source,truth]=scene();pose=(truth+[0.2,-0.1,deg2rad(1)]).';
            % Halving one pole's temporal stability changes the balanced gradient;
            % a uniform factor on a class does not (balancing removes it).
            plain=localizationSupport.evaluateOverlapGradient(map,source,pose);
            source.components.temporalStability=ones(6,1);source.components.temporalStability(4)=0.5;
            weighted=localizationSupport.evaluateOverlapGradient(map,source,pose);
            testCase.verifyGreaterThan(norm(weighted.gradient-plain.gradient),1e-6*norm(plain.gradient));
            uniform=source;uniform.components.temporalStability=0.5*ones(6,1);
            same=localizationSupport.evaluateOverlapGradient(map,uniform,pose);
            testCase.verifyEqual(same.gradient,plain.gradient,RelTol=1e-9,AbsTol=1e-12);
            zeroed=source;zeroed.components.temporalStability(4:5)=0;
            noPoles=localizationSupport.evaluateOverlapGradient(map,zeroed,pose);
            testCase.verifyFalse(ismember("pole",noPoles.sharedClasses));
        end
        function retiredOverlapSelectorsAreRejected(testCase)
            [map,source,pose]=scene();
            retired=struct('aggregation',"frame",'orientation',true,'outlierDensity',1e-4, ...
                'viewConditioning',false,'evidenceWeights',false,'scaleLadder',[0 0.5 1 2]);
            settings=overlapGradientConfig();
            for name=string(fieldnames(retired)).'
                testCase.verifyFalse(isfield(settings,name));
                stale=settings;stale.(name)=retired.(name);
                testCase.verifyError(@()localizationSupport.evaluateOverlapGradient(map,source,pose(:),stale), ...
                    'VehicleLocalization:ObsoleteOverlapConfig');
            end
            settings.geometry.heightMode="auto";
            testCase.verifyError(@()localizationSupport.evaluateOverlapGradient(map,source,pose(:),settings), ...
                'VehicleLocalization:PlanarOverlapRequired');
        end
        function retiredSolverConfigurationCannotReachOverlap(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=overlapGradientConfig();
            cfg.registration=cfg.geometry;cfg=rmfield(cfg,'geometry');
            testCase.verifyError(@()localizationSupport.evaluateOverlapGradient(cloud,cloud,zeros(3,1),cfg), ...
                'VehicleLocalization:ObsoleteOverlapConfig');
        end
        function frozenRegistrationPayloadsCannotBeReplayedOrAligned(testCase)
            f=movingFixture(1);t=f.lateral.time;n=numel(t);
            data=rmfield(f.data,'lidarOverlap');
            for name=["residualModels","poseErrorVariance"]
                data.lidar=struct('time',t,'pose',zeros(n,3),'valid',true(n,1), ...
                    'information',repmat(eye(3),1,1,n),'delay',0);
                data.lidar.(name)=zeros(n,2);
                testCase.verifyError(@()runFullLocalizationObserver(data,f.cfg,LateralInputs=f.lateral), ...
                    'VehicleLocalization:ArchivedRegistrationMeasurement');
                testCase.verifyError(@()localizationSupport.synchronizeLocalizationInputs(data,f.lateral,f.cfg), ...
                    'VehicleLocalization:ArchivedRegistrationMeasurement');
            end
        end
        function overlapChannelIsExclusiveAndRejectsCalibratedWeights(testCase)
            f=movingFixture(1);
            ambiguous=f.data;n=numel(f.lateral.time);
            ambiguous.lidar=struct('time',f.lateral.time,'pose',zeros(n,3), ...
                'information',repmat(eye(3),1,1,n),'valid',true(n,1),'delay',0);
            testCase.verifyError(@()runFullLocalizationObserver(ambiguous,f.cfg,LateralInputs=f.lateral), ...
                'VehicleLocalization:AmbiguousLidarInput');
            calibrated=f.cfg;
            calibrated.lidar.errorCalibration=struct('kind',"conditional-pose-second-moment-v1", ...
                'referenceVariance',[.01;.001],'coefficients',[.01,1;.001,1]);
            testCase.verifyError(@()runFullLocalizationObserver(f.data,calibrated,LateralInputs=f.lateral), ...
                'VehicleLocalization:ArchivedRegistrationMeasurement');
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

function f=movingFixture(duration,offset)
% Constant-speed straight drive; every source is the map in current body axes.
% OFFSET is the initial [x y psi] estimate error in world axes.
    arguments
        duration (1,1) double
        offset (1,3) double=[0.25,-0.20,deg2rad(1)]
    end
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
    lateral=struct('time',t,'lateralVelocity',zero,'yawRate',zero,'sideSlipAngleRate',zero);
    data=struct('highRate',high,'lidarOverlap',struct('map',map,'sources',{sources}));
    cfg=freshObserverTestConfig();
    initial=truth(1,:)+offset;
    cfg.initialState=[initial(1);speed*cos(start(3));0;initial(2);speed*sin(start(3));0;initial(3)];
    f=struct('data',data,'lateral',lateral,'cfg',cfg,'truth',truth);
end

function design=fullObserverDesign()
    design=struct('P',blkdiag(eye(6),1),'T',eye(7),'kappa',4);
end

function cfg=filterConfig()
% Standalone geometric-filter fixture; no observer implementation is selected.
    cfg=struct('lidar',struct('poseScales',ones(3,1)));
end
