classdef synchronousLocalizationTest < matlab.unittest.TestCase
% synchronousLocalizationTest Low-rate input and observer behavior.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function straightMotionIsPreserved(testCase)
            f=fixture();r=run(f);
            testCase.verifyEqual(r.position,[8*r.time,zeros(size(r.time))],AbsTol=1e-10);
            testCase.verifyEqual(r.diagnostics.localizationUpdates,numel(r.time)-1);
            testCase.verifyEqual(r.diagnostics.virtualPoseUpdates,0);
            testCase.verifyEqual(r.diagnostics.integrationSubsteps,0);
        end
        function onlineMatcherReceivesCurrentAlignedAid(testCase)
            f=fixture();f.data=rmfield(f.data,'lidar');
            f.data.lidarMatcher=@onlineResult;
            r=run(f);
            testCase.verifyEqual(r.position,[8*r.time,zeros(size(r.time))],AbsTol=1e-10);
            testCase.verifyTrue(r.diagnostics.matchingFeedback);
            testCase.verifyEqual(r.matchingResults{11}.aidTimestamp,1,AbsTol=1e-12);
            testCase.verifyEqual(r.matchingSeeds(:,1),8*r.time,AbsTol=1e-10);
        end
        function onlineMatcherDoesNotUseFutureGnss(testCase)
            f=fixture();f.data=rmfield(f.data,'lidar');f.data.lidarMatcher=@onlineResult;
            a=run(f);f.data.gnss.position(12:end,:)=100;b=run(f);
            testCase.verifyEqual(a.z(1:11,:),b.z(1:11,:),AbsTol=0);
        end
        function onlineAndRecordedLidarCannotBeMixed(testCase)
            f=fixture();f.data.lidarMatcher=@onlineResult;
            testCase.verifyError(@()run(f),'VehicleLocalization:AmbiguousLidarInput');
        end
        function asynchronousSourceIsRejected(testCase)
            f=fixture();f.data.gnss.time(2)=.11;
            testCase.verifyError(@()run(f),'VehicleLocalization:SynchronousInputsRequired');
        end
        function missingChannelDoesNotWaitForTheOther(testCase)
            f=fixture();f.data.gnss.valid(5:end)=false;f.data.gnss.position(5:end,:)=NaN;
            r=run(f);testCase.verifyEqual(r.diagnostics.mode(5:end),2*ones(17,1));
            testCase.verifyEqual(r.position(:,1),8*r.time,AbsTol=1e-10);
        end
        function invalidLidarDoesNotCreateAPredictedMeasurement(testCase)
            f=fixture();f.data.lidar.valid(5:end)=false;f.data.lidar.pose(5:end,:)=NaN;
            r=run(f);testCase.verifyEqual(r.diagnostics.mode(5:end),ones(17,1));
            testCase.verifyEqual(r.diagnostics.positionCorrection(5:end,3:4),zeros(17,2),AbsTol=0);
        end
        function bothMissingUseOnlyStateDynamics(testCase)
            f=fixture();f.data.gnss.valid(:)=false;f.data.lidar.valid(:)=false;r=run(f);
            testCase.verifyEqual(r.position(:,1),8*r.time,AbsTol=1e-10);
            testCase.verifyEqual(r.diagnostics.mode,zeros(21,1));
        end
        function noFutureFrameChangesThePast(testCase)
            f=fixture();a=run(f);f.data.gnss.position(12:end,:)=100;f.data.lidar.pose(12:end,:)=100;b=run(f);
            testCase.verifyEqual(a.z(1:11,:),b.z(1:11,:),AbsTol=0);
        end
        function largeGainHasImplicitContraction(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');f.cfg.gains(1)=100;
            f.cfg.initialState(4)=1;r=run(f);
            testCase.verifyGreaterThanOrEqual(min(r.position(:,2)),0);
            testCase.verifyLessThan(abs(r.position(end,2)),1e-12);
            testCase.verifyFalse(r.observer.sampledSystemCertified);
        end
        function curbOnlyCannotCorrectLongitudinalError(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');f.cfg.initialState(1)=2;
            f.cfg.initialState(4)=.2;
            f.data.lidar.information=repmat(diag([0,100,100]),1,1,numel(f.data.highRate.time));
            r=run(f);
            testCase.verifyEqual(r.position(:,1)-8*r.time,2*ones(size(r.time)),AbsTol=1e-11);
            testCase.verifyLessThan(abs(r.position(end,2)),.002);
            testCase.verifyEqual(r.diagnostics.lidarMatched{end}.filter.rank,2);
        end
        function translationYawCrossTermsReachTheRuntime(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');f.cfg.initialState(1)=.2;
            f.data.lidar.information=repmat([100,0,20;0,100,0;20,0,100],1,1,21);
            r=run(f);
            testCase.verifyLessThan(r.headingUnwrapped(2),0);
            testCase.verifyGreaterThan(abs(r.diagnostics.poseWeight(1,3,2)),0);
            testCase.verifyTrue(r.diagnostics.lidarMatched{2}.jumpNonexpansive);
        end
        function completeReliabilityRejectionEqualsMissingLidar(testCase)
            f=fixture();f.data.lidar.frameReliability=zeros(21,1);a=run(f);
            f.data=rmfield(f.data,'lidar');b=run(f);
            testCase.verifyEqual(a.z,b.z,AbsTol=0);
            testCase.verifyEqual(a.diagnostics.yawCorrection,zeros(21,1),AbsTol=0);
        end
        function zeroInformationIsAValidZeroCorrection(testCase)
            f=fixture();f.data.lidar.information=zeros(3,3,21);a=run(f);
            f.data=rmfield(f.data,'lidar');b=run(f);
            testCase.verifyEqual(a.z,b.z,AbsTol=0);
        end
        function validDirectionalMatcherFeedsItsFilteredInformation(testCase)
            f=fixture();f.data=rmfield(f.data,{'gnss','lidar'});
            f.cfg.initialState(1)=2;f.cfg.initialState(4)=.2;
            f.data.lidarMatcher=@directionalResult;r=run(f);
            testCase.verifyEqual(r.position(:,1)-8*r.time,2*ones(size(r.time)),AbsTol=1e-11);
            testCase.verifyLessThan(abs(r.position(end,2)),.002);
            testCase.verifyEqual(r.diagnostics.lidarMatched{2}.filter.rank,2);
        end
        function runtimeResidualProviderUsesPostBaselinePrediction(testCase)
            f=fixture();f.cfg.initialState(4)=.2;
            f.data.lidar.evaluateResidual=@lineResidual;
            r=run(f);
            testCase.verifyEqual(r.diagnostics.lidarMatched{2}.representation,"predictedPoseResidual");
            testCase.verifyLessThan(abs(r.position(end,2)),.002);
        end
        function residualProviderReliabilityIsNotOverwritten(testCase)
            f=fixture();f.cfg.initialState(4)=.2;f.data.lidar.evaluateResidual=@rejectedResidual;
            a=run(f);f.data=rmfield(f.data,'lidar');b=run(f);
            testCase.verifyEqual(a.z,b.z,AbsTol=0);
        end
        function sourceReliabilityIsAlignedWithItsOwnFrame(testCase)
            f=unaligned();f.data.lidar.frameReliability=(0:20).'/20;
            f.data.lidar.directionReliability=repmat([0,.5,1],21,1);
            [d,~,~]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(d.lidar.frameReliability,f.data.lidar.frameReliability,AbsTol=0);
            testCase.verifyEqual(d.lidar.directionReliability,f.data.lidar.directionReliability,AbsTol=0);
        end
        function splitUpdateSatisfiesReportedPositionEquation(testCase)
            f=fixture();f.cfg.initialState([1,4,7])=[.1;.2;.03];r=run(f);
            rates=r.velocity(2:end,:)+r.diagnostics.positionCorrection(2:end,1:2) ...
                +r.diagnostics.positionCorrection(2:end,3:4);
            testCase.verifyEqual(diff(r.position),diff(r.time).*rates,AbsTol=1e-12);
            testCase.verifyFalse(r.observer.lidarMatched.baselineFullStateCertified);
        end
        function stateUsesOnlyOneLowRateSolve(testCase)
            f=fixture();r=run(f);
            testCase.verifyFalse(isfield(f.cfg,'maximumIntegrationStep'));
            testCase.verifyEqual(r.diagnostics.integrationSubsteps,0);
            testCase.verifyEqual(r.diagnostics.localizationUpdates,numel(r.time)-1);
        end
        function delayedSourceRejected(testCase)
            f=fixture();f.data.lidar.delay=.1;
            testCase.verifyError(@()run(f),'VehicleLocalization:FullZeroDelayRequired');
        end
        function alignmentRetainsRealLidarAndReportsWaiting(testCase)
            f=unaligned();[d,~,m]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(d.lidar.pose,f.data.lidar.pose,AbsTol=0);
            testCase.verifyEqual(d.gnss.position(:,1),8*d.highRate.time,AbsTol=1e-12);
            testCase.verifyEqual(m.maximumGnssWaitSeconds,.01,AbsTol=1e-12);
            testCase.verifyFalse(m.motionModelMeasurementExtrapolation);
        end
        function alignmentCannotBridgeMissingMeasurements(testCase)
            f=unaligned();f.data.gnss.valid(:)=false;
            [d,~,~]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyFalse(any(d.gnss.valid));
            testCase.verifyTrue(all(isnan(d.gnss.position),'all'));
        end
        function alignmentDoesNotExtrapolatePastFinalGnss(testCase)
            f=unaligned();f.data.gnss.time(end)=f.data.gnss.time(end)-.011;
            [d,~,~]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyFalse(d.gnss.valid(end));
        end
        function alignmentRejectsWideGnssBrackets(testCase)
            f=unaligned();f.cfg.synchronization.gnssMaximumBracket=.005;
            [d,~,~]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyFalse(any(d.gnss.valid));
        end
        function malformedInformationRejected(testCase)
            f=fixture();f.data.lidar.information(3,3,3)=-1;
            testCase.verifyError(@()run(f),'VehicleLocalization:InvalidFullSource');
        end
        function turningDiscretizationConvergesWithSmallerSteps(testCase)
            coarse=turning(.1);fine=turning(.05);a=run(coarse);b=run(fine);
            testCase.verifyLessThan(norm(b.position(end,:)-fine.truth(end,:)), ...
                norm(a.position(end,:)-coarse.truth(end,:)));
            testCase.verifyEqual(a.heading,coarse.data.highRate.time*.2,AbsTol=1e-12);
        end
        function missingPreparedLateralInputIsRejected(testCase)
            f=fixture();f.lateral=struct();
            testCase.verifyError(@()run(f),'VehicleLocalization:AlignedLateralRequired');
        end
    end
end

function r=directionalResult(~,seed,~)
    r=struct('poseXYTheta',[seed(1),0,0],'information',100*eye(3),'accepted',false, ...
        'directionalAccepted',true,'supportedConverged',true,'observableRank',2, ...
        'directionalInformation',diag([0,100,100]),'physicalObservableProjector',diag([0,1,1]));
end

function m=lineResidual(~,pose)
    points=[-2,0;0,0;2,0];
    m=lidarInjectionSupport.buildLidarLineMeasurement(points,points,repmat([0,1],3,1),pose,100*ones(3,1));
end

function m=rejectedResidual(k,pose)
    m=lineResidual(k,pose);m.frameReliability=0;
end

function r=onlineResult(~,seed,aid)
    r=struct('poseXYTheta',seed,'information',100*eye(3),'accepted',false,'aidTimestamp',aid.timestamp);
    if aid.valid,r.poseXYTheta(1:2)=aid.position;r.accepted=true;end
end

function f=turning(step)
    f=fixture();t=(0:step:2).';z=zeros(size(t));n=numel(t);r=.2;v=8;
    f.truth=[v/r*sin(r*t),v/r*(1-cos(r*t))];
    f.data.highRate=struct('time',t,'longitudinalSpeed',v+z,'longitudinalAcceleration',z, ...
        'lateralAcceleration',v*r+z,'yawRate',r+z);
    f.lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
    f.data.gnss=struct('time',t,'position',f.truth,'valid',true(n,1),'information',repmat(1e6*eye(2),1,1,n),'delay',0);
    f.data.lidar=struct('time',t,'pose',[f.truth,r*t],'valid',true(n,1),'information',repmat(1e6*eye(3),1,1,n),'delay',0);
    f.cfg.initialState=[0;v;0;0;0;v*r;0];
end

function f=fixture()
    t=(0:.1:2).';z=zeros(size(t));n=numel(t);
    h=struct('time',t,'longitudinalSpeed',8+z,'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z);
    lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
    g=struct('time',t,'position',[8*t,z],'valid',true(n,1),'information',repmat(1e6*eye(2),1,1,n),'delay',0);
    l=struct('time',t,'pose',[8*t,z,z],'valid',true(n,1),'information',repmat(1e6*eye(3),1,1,n),'delay',0);
    cfg=fullObserverConfig;cfg.initialState=[0;8;0;0;0;0;0];
    f=struct('data',struct('highRate',h,'gnss',g,'lidar',l),'lateral',lateral,'cfg',cfg);
end

function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end

function f=unaligned()
    f=fixture();t=(0:.01:2).';z=zeros(size(t));
    f.data.highRate=struct('time',t,'longitudinalSpeed',8+z,'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z);
    f.lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
    g=(-.01:.02:2.01).';n=numel(g);
    f.data.gnss=struct('time',g,'position',[8*g,zeros(n,1)],'valid',true(n,1), ...
        'information',repmat(1e6*eye(2),1,1,n),'delay',0);
end
