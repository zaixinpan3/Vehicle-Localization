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
            testCase.verifyEqual(pooled.components.mean,repmat(current.components.mean,2,1),AbsTol=1e-12);
            testCase.verifyEqual(pooled.components.covariance,repmat(current.components.covariance,1,1,2),AbsTol=1e-12);
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
            testCase.verifyEqual(after.components.mean,cloud.components.mean,AbsTol=0);
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
        function oneFrameConfigurationPreservesTheCurrentProduct(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=localizationSourceWindowConfig();cfg.maximumFrames=1;
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[],cfg);
            [actual,~,d]=updateLocalizationSourceWindow(cloud,.1,[1 0 0],h,cfg);
            testCase.verifyEqual(actual.components.mean,cloud.components.mean,AbsTol=0);
            testCase.verifyEqual(d.frameCount,1);
        end
    end
end
