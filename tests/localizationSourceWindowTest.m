classdef localizationSourceWindowTest < matlab.unittest.TestCase
% localizationSourceWindowTest Causal odometry transport and information scope.
    properties (TestParameter)
        nonmonotonicTime={0,-.1}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function unavailableMotionKeepsOnlyTheUnmodifiedCurrentGeometry(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,history]=updateLocalizationSourceWindow(cloud,0,[0,0,0],[]);
            [actual,history,details]=updateLocalizationSourceWindow(cloud,.1,[NaN,NaN,NaN],history,localizationSourceWindowConfig(),0);
            expected=registrationSupport.projectSemanticProbabilityCloud(cloud,2);
            testCase.verifyEqual(actual,expected);
            testCase.verifyEmpty(history.time);
            testCase.verifyFalse(details.historyTransportApplied);
            testCase.verifyFalse(isfield(actual.components,'temporalStability'));
        end
        function aValidityGapCannotBridgeTwoVisibleQueries(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=localizationSourceWindowConfig();
            [~,history]=updateLocalizationSourceWindow(cloud,0,[0,0,0],[],cfg,1);
            [after,history,details]=updateLocalizationSourceWindow(cloud,.1,[0,0,0],history,cfg,2);
            testCase.verifyEmpty(after.components.mean);
            testCase.verifyEqual(history.time,.1);
            testCase.verifyEqual(details.frameCount,1);
            [recovered,~]=updateLocalizationSourceWindow(cloud,.2,[0,0,0],history,cfg,2);
            testCase.verifyEqual(recovered.components.mean,cloud.components.mean,AbsTol=1e-12);
        end
        function motionTransportsMeansAndFullXyCovariance(testCase)
            old=distributionRegistrationTest.exampleCloud();
            current=distributionRegistrationTest.transform(old,[0 1 -pi/2]);
            [~,history]=updateLocalizationSourceWindow(old,0,[0 0 0],[]);
            [pooled,~,details]=updateLocalizationSourceWindow(current,.1,[1 0 pi/2],history);
            testCase.verifyEqual(pooled.components.mean,current.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(pooled.components.covariance,current.components.covariance,AbsTol=1e-12);
            testCase.verifyEqual(sum(pooled.components.mixtureWeight),1,AbsTol=1e-12);
            testCase.verifyEqual(details.spanSeconds,.1,AbsTol=0);
            testCase.verifyEqual(details.dimension,2);
        end
        function arbitraryGlobalOdometryOriginDoesNotMatter(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            current=distributionRegistrationTest.transform(cloud,[0 1 -pi/2]);
            [~,a]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [expected,~]=updateLocalizationSourceWindow(current,.1,[1 0 pi/2],a);
            [~,b]=updateLocalizationSourceWindow(cloud,0,[700000 4300000 .7],[]);
            [actual,~]=updateLocalizationSourceWindow(current,.1,[700000+cos(.7) 4300000+sin(.7) .7+pi/2],b);
            testCase.verifyEqual(actual.components.mean,expected.components.mean,AbsTol=1e-8);
            testCase.verifyEqual(actual.components.covariance,expected.components.covariance,AbsTol=1e-12);
        end
        function retainsFiveCausalScansByDefault(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [~,h]=updateLocalizationSourceWindow(cloud,.2,[0 0 0],h);
            [~,h]=updateLocalizationSourceWindow(cloud,.3,[0 0 0],h);
            [before,h]=updateLocalizationSourceWindow(cloud,.4,[0 0 0],h);
            [after,h,d]=updateLocalizationSourceWindow(cloud,.5,[0 0 0],h);
            testCase.verifyEqual(h.time,[.1;.2;.3;.4;.5],AbsTol=0);
            testCase.verifyEqual(d.frameCount,5);
            testCase.verifyEqual(d.oldestTimestamp,.1,AbsTol=0);
            testCase.verifyEqual(before.components.mean,after.components.mean,AbsTol=0);
        end
        function oldEvidenceExpiresAcrossAnAcquisitionGap(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [after,h,d]=updateLocalizationSourceWindow(cloud,.5,[100 0 0],h);
            testCase.verifyEqual(h.time,.5,AbsTol=0);
            testCase.verifyEmpty(after.components.mean);
            testCase.verifyEqual(d.rejectedSingletons,cloud.components.numComponents);
            testCase.verifyEqual(d.frameCount,1);
        end
        function rejectsRepeatedOrReversedTime(testCase,nonmonotonicTime)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            testCase.verifyError(@()updateLocalizationSourceWindow(cloud,nonmonotonicTime,[0 0 0],h), ...
                'VehicleLocalization:NonmonotonicWindowTime');
        end
        function historyMustUseTheSameCalibration(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            cloud.frameCalibration.translation(1)=cloud.frameCalibration.translation(1)+.1;
            testCase.verifyError(@()updateLocalizationSourceWindow(cloud,.1,[0 0 0],h), ...
                'VehicleLocalization:CalibrationMismatch');
        end
        function rejectsOneFrameHorizon(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=localizationSourceWindowConfig();cfg.maximumFrames=1;
            testCase.verifyError(@()updateLocalizationSourceWindow(cloud,0,[0 0 0],[],cfg), ...
                'VehicleLocalization:InvalidWindowConfiguration');
        end
        function singleScanFlashIsAbsentFromMatchingCloud(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            flashed=distributionRegistrationTest.withZeroMassComponents(cloud);
            flashed.components.mixtureWeight(end)=.1;
            [actual,~,d]=updateLocalizationSourceWindow(flashed,.2,[0 0 0],h);
            testCase.verifyEqual(actual.components.mean,cloud.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(actual.components.detectionFrameCount,3*ones(6,1),AbsTol=0);
            testCase.verifyGreaterThanOrEqual(d.rejectedSingletons,1);
        end
        function duplicatePillarsInOneScanCannotCountAsTwoFrames(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cloud=testCase.duplicate(cloud);
            [actual,~,d]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            testCase.verifyEmpty(actual.components.mean);
            testCase.verifyEqual(d.rejectedSingletons,12);
        end
        function oneObservationCannotConfirmTwoNearbyTracks(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(testCase.duplicate(cloud),0,[0 0 0],[]);
            [actual,~,d]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            testCase.verifyEqual(actual.components.numComponents,6);
            testCase.verifyEqual(actual.components.detectionFrameCount,2*ones(6,1),AbsTol=0);
            testCase.verifyEqual(d.rejectedSingletons,6);
        end
        function classChangeDoesNotConfirmTheOldClass(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            changed=cloud;changed.components.semanticName(:)="trafficSign";
            [actual,~,d]=updateLocalizationSourceWindow(changed,.1,[0 0 0],h);
            testCase.verifyEmpty(actual.components.mean);
            testCase.verifyEqual(d.rejectedSingletons,12);
        end
        function singletonTrackHandlesMultipleNewCandidates(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            c=cloud.components;
            cloud.components=struct('mean',c.mean(2,:),'covariance',c.covariance(:,:,2), ...
                'semanticName',"pole",'mixtureWeight',1,'numComponents',1);
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [actual,~,d]=updateLocalizationSourceWindow(testCase.duplicate(cloud),.1,[0 0 0],h);
            testCase.verifyEqual(actual.components.numComponents,1);
            testCase.verifyEqual(actual.components.detectionFrameCount,2);
            testCase.verifyEqual(d.rejectedSingletons,1);
        end
        function previousProductCannotBeRecountedAsANewScan(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [pooled,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            testCase.verifyError(@()updateLocalizationSourceWindow(pooled,.2,[0 0 0],h), ...
                'VehicleLocalization:AlreadyStackedSource');
        end
        function repeatedDistributionsRetainBetweenScanScatter(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            shifted=distributionRegistrationTest.transform(cloud,[.2 0 0]);
            [actual,~]=updateLocalizationSourceWindow(shifted,.1,[0 0 0],h);
            testCase.verifyEqual(actual.components.mean,cloud.components.mean+[.1 0],AbsTol=1e-12);
            testCase.verifyEqual(actual.components.covariance,cloud.components.covariance+[.01 0;0 0],AbsTol=1e-12);
        end
        function stableFeatureSurvivesMissesUntilItsSupportExpires(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [~,h]=updateLocalizationSourceWindow(testCase.empty(cloud),.2,[0 0 0],h);
            [~,h]=updateLocalizationSourceWindow(testCase.empty(cloud),.3,[0 0 0],h);
            [actual,h]=updateLocalizationSourceWindow(testCase.empty(cloud),.4,[0 0 0],h);
            [expired,~]=updateLocalizationSourceWindow(testCase.empty(cloud),.5,[0 0 0],h);
            testCase.verifyEqual(actual.components.detectionFrameCount,2*ones(6,1),AbsTol=0);
            testCase.verifyEmpty(expired.components.mean);
        end

    end
    methods (Static)
        function cloud=duplicate(cloud)
            c=cloud.components;
            cloud.components=struct('mean',[c.mean;c.mean],'covariance',cat(3,c.covariance,c.covariance), ...
                'semanticName',[c.semanticName;c.semanticName],'mixtureWeight',[c.mixtureWeight;c.mixtureWeight]/2, ...
                'numComponents',2*c.numComponents);
        end
        function cloud=empty(cloud)
            cloud.components=struct('mean',zeros(0,2),'covariance',zeros(2,2,0), ...
                'semanticName',strings(0,1),'mixtureWeight',zeros(0,1),'numComponents',0);
        end
    end
end
