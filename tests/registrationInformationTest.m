classdef registrationInformationTest < matlab.unittest.TestCase
% registrationInformationTest Physical units, null spaces, and event contract.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function matchesAnalyticIsotropicLandmarkInformation(testCase)
            cloud=poles();
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            testCase.verifyTrue(result.accepted,result.reason);
            expected=diag([1 1 13])/(2*.2+.1^2);
            testCase.verifyEqual(result.information,expected,'AbsTol',1e-12);
            event=registrationSupport.registrationPoseMeasurement(result,4.2);
            testCase.verifyEqual(event.information,expected,'AbsTol',1e-12);
            testCase.verifyEqual(event.timestamp,4.2);
            testCase.verifyEqual(event.pose,[0 0 0]);
        end
        function numericalYawScalingDoesNotChangePhysicalInformation(testCase)
            cloud=poles(); cfg=distributionRegistrationConfig();
            first=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg);
            cfg.yawLeverArm=2;
            second=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg);
            testCase.verifyEqual(second.information,first.information,'AbsTol',1e-12);
        end
        function globalRotationTransformsInformationAndRetainsCrossTerms(testCase)
            cloud=poles(); cloud.components.mean=cloud.components.mean+[3 -1];
            first=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            angle=.7; pose=[700000 4300000 angle];
            fixed=distributionRegistrationTest.transform(cloud,pose);
            second=registerSemanticProbabilityCloud(fixed,cloud,pose);
            r=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            transform=blkdiag(r,1);
            testCase.verifyEqual(second.information,transform*first.information*transform.','AbsTol',1e-7);
            testCase.verifyGreaterThan(norm(first.information(1:2,3)),1);
        end
        function uniformMapRepeatabilityReducesInformation(testCase)
            cloud=poles(); fixed=cloud;
            fixed.components.repeatability=.25*ones(4,1);
            first=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            second=registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]);
            testCase.verifyEqual(second.information,.25*first.information,'AbsTol',1e-12);
        end
        function duplicatedSourceComponentsDoNotInventIndependentEvidence(testCase)
            cloud=poles(); moving=cloud; c=cloud.components;
            moving.components=struct('mean',repmat(c.mean,4,1), ...
                'covariance',repmat(c.covariance,1,1,4), ...
                'semanticName',repmat(c.semanticName,4,1), ...
                'mixtureWeight',repmat(c.mixtureWeight,4,1),'numComponents',16);
            first=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            second=registerSemanticProbabilityCloud(cloud,moving,[0 0 0]);
            testCase.verifyEqual(second.information,first.information,'AbsTol',1e-12);
        end
        function roadTangentRemainsUnobservableInDirectionalEvent(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            testCase.verifyEqual(result.information*[1;0;0],zeros(3,1),'AbsTol',1e-12);
            event=registrationSupport.registrationPoseMeasurement(result,0);
            testCase.verifyFalse(result.accepted);
            testCase.verifyTrue(result.directionalAccepted);
            testCase.verifyEqual(event.measurementType,"directionalPose");
            testCase.verifyEqual(event.observableRank,2);
            testCase.verifyEqual(event.information*[1;0;0],zeros(3,1),'AbsTol',1e-12);
        end
        function unvalidatedPartialFlagCannotAuthorizeAnEvent(testCase)
            result=struct('accepted',false,'partialPoseAvailable',true);
            testCase.verifyEmpty(registrationSupport.registrationPoseMeasurement(result,0));
        end
        function nonconvergedDirectionalGeometryEmitsNoEvent(testCase)
            cloud=geometricRegistrationTest.parallelRoad();cfg=distributionRegistrationConfig();
            cfg.maximumIterationsPerScale=1;
            result=registerSemanticProbabilityCloud(cloud,cloud,[.8 .3 .02],cfg);
            testCase.verifyTrue(result.partialPoseAvailable);
            testCase.verifyFalse(result.directionalAccepted);
            testCase.verifyEqual(result.reason,"notConverged");
            testCase.verifyEmpty(registrationSupport.registrationPoseMeasurement(result,0));
        end
        function inconsistentPartialClassesEmitsNoEvent(testCase)
            moving=conflictingRoad();fixed=moving;
            fixed.components.mean(11:20,2)=fixed.components.mean(11:20,2)+.2;
            cfg=distributionRegistrationConfig();cfg.geometric.maximumClassCorrection=.05;
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyEqual(result.observableRank,2);
            testCase.verifyEqual(result.reason,"inconsistentClasses");
            testCase.verifyFalse(result.directionalAccepted);
            testCase.verifyEmpty(registrationSupport.registrationPoseMeasurement(result,0));
        end
        function directionalInformationAndProjectorUsePhysicalCoordinates(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            cloud.components.mean(:,:)=repmat([3 1],10,1);
            cfg=distributionRegistrationConfig();
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg);
            event=registrationSupport.registrationPoseMeasurement(result,0,.15);
            scale=diag([1 1 1/cfg.yawLeverArm]);
            testCase.verifyEqual(event.observableRank,1);
            testCase.verifyEqual(event.observableProjector,scale*result.observableProjector/scale,'AbsTol',1e-12);
            testCase.verifyEqual(event.information*[0;-3;1],zeros(3,1),'AbsTol',1e-10);
            testCase.verifyGreaterThan(norm(event.observableProjector-event.observableProjector.'),.1);
            testCase.verifyFalse(event.arrivalTimeIsPlaceholder);
            testCase.verifyEqual(event.arrivalTime,.15,'AbsTol',0);
        end
        function filteredWeakDirectionsDoNotLeakIntoExport(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            angle=.001;r=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            cloud.components.covariance(:,:,1)=r*cloud.components.covariance(:,:,1)*r.';
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            event=registrationSupport.registrationPoseMeasurement(result,0);
            nullProjector=eye(3)-event.observableProjector;
            testCase.verifyEqual(event.observableRank,2);
            testCase.verifyGreaterThan(norm(result.information*nullProjector,'fro'),1e-8);
            testCase.verifyLessThan(norm(event.information*nullProjector,'fro'),1e-10);
        end
        function acceptedPoseCannotOmitOrFakeInformation(testCase)
            result=struct('accepted',true,'poseXYTheta',[0 0 0]);
            testCase.verifyError(@() registrationSupport.registrationPoseMeasurement(result,0), ...
                'VehicleLocalization:RegistrationInformationUnavailable');
            result.information=diag([1 1 0]);
            testCase.verifyError(@() registrationSupport.registrationPoseMeasurement(result,0), ...
                'VehicleLocalization:InvalidRegistrationInformation');
        end
    end
end

function cloud=conflictingRoad()
    cloud=geometricRegistrationTest.parallelRoad();c=cloud.components;
    cloud.components=struct('mean',repmat(c.mean,2,1), ...
        'covariance',repmat(c.covariance,1,1,2), ...
        'semanticName',[c.semanticName;repmat("roadMarking",10,1)], ...
        'mixtureWeight',ones(20,1)/20,'numComponents',20);
end

function cloud=poles()
    c=struct('mean',[-2 -3;-2 3;2 -3;2 3], ...
        'covariance',repmat(.2*eye(2),1,1,4), ...
        'semanticName',repmat("pole",4,1), ...
        'mixtureWeight',ones(4,1)/4,'numComponents',4);
    cloud=struct('components',c);
end
