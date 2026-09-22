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
        function retainsOnlyThreeCausalScans(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [before,h]=updateLocalizationSourceWindow(cloud,.2,[0 0 0],h);
            [after,h,d]=updateLocalizationSourceWindow(cloud,.3,[0 0 0],h);
            testCase.verifyEqual(h.time,[.1;.2;.3],AbsTol=0);
            testCase.verifyEqual(d.frameCount,3);
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
        function repeatedScansDoNotMultiplyPoseInformation(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            single=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [pooled,~]=updateLocalizationSourceWindow(cloud,.2,[0 0 0],h);
            result=registerSemanticProbabilityCloud(cloud,pooled,[0 0 0]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,single.poseXYTheta,AbsTol=1e-12);
            testCase.verifyEqual(result.information,single.information,AbsTol=1e-10);
            testCase.verifyFalse(result.informationCalibrated);
        end
        function preservesRealNullDirectionsWithRepeatedScans(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [pooled,~]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            result=registerSemanticProbabilityCloud(cloud,pooled,[.8 .3 .02]);
            testCase.verifyEqual(result.observableRank,2);
            testCase.verifyTrue(result.directionalAccepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta(1),.8,AbsTol=1e-12);
            testCase.verifyEqual(result.directionalInformation*[1;0;0],zeros(3,1),AbsTol=1e-10);
        end
        function rejectsOneFrameHorizon(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=localizationSourceWindowConfig();cfg.maximumFrames=1;
            testCase.verifyError(@()updateLocalizationSourceWindow(cloud,0,[0 0 0],[],cfg), ...
                'VehicleLocalization:InvalidWindowConfiguration');
        end
        function firstFrameCannotProvideTemporalEvidence(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [pooled,~,details]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            result=registerSemanticProbabilityCloud(cloud,pooled,[0 0 0]);
            testCase.verifyEqual(details.componentCount,0);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEmpty(registrationSupport.registrationPoseMeasurement(result,0));
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
        function stableFeatureCanSurviveOneMissInsideHorizon(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [actual,h]=updateLocalizationSourceWindow(testCase.empty(cloud),.2,[0 0 0],h);
            [expired,~]=updateLocalizationSourceWindow(testCase.empty(cloud),.3,[0 0 0],h);
            testCase.verifyEqual(actual.components.detectionFrameCount,2*ones(6,1),AbsTol=0);
            testCase.verifyEmpty(expired.components.mean);
        end
        function repeatedSupportIncreasesWeightWithoutInflatingScatter(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [twice,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [thrice,~]=updateLocalizationSourceWindow(cloud,.2,[0 0 0],h);
            a=registerSemanticProbabilityCloud(cloud,twice,[0 0 0]);
            b=registerSemanticProbabilityCloud(cloud,thrice,[0 0 0]);
            testCase.verifyEqual(twice.components.temporalStability,2/3*ones(6,1),AbsTol=1e-12);
            testCase.verifyEqual(thrice.components.temporalStability,ones(6,1),AbsTol=1e-12);
            testCase.verifyEqual(a.information,b.information*2/3,AbsTol=1e-10);
            testCase.verifyEqual(twice.components.covariance,thrice.components.covariance,AbsTol=1e-12);
        end
        function stabilitySurvivesClassNormalization(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            weighted=cloud;weighted.components.temporalStability=ones(6,1);
            weighted.components.temporalStability(weighted.components.semanticName=="pole")=2/3;
            result=registerSemanticProbabilityCloud(cloud,weighted,[0 0 0]);
            pairs=result.correspondences;
            testCase.verifyEqual(sum(pairs.weight(pairs.semanticName=="pole")),2/9,AbsTol=1e-12);
            testCase.verifyEqual(sum(pairs.weight(pairs.semanticName=="curb")),1/3,AbsTol=1e-12);
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
