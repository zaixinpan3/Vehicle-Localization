classdef positionAidedRegistrationTest < matlab.unittest.TestCase
% positionAidedRegistrationTest Ambiguity, missing aid and conditional information.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function missingAidPreservesGeometry(testCase)
            [fixed,moving,cfg,aid]=fixture();aid.valid=false;
            a=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg);
            b=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg,aid);
            testCase.verifyEqual(b.poseXYTheta,a.poseXYTheta,AbsTol=0);
            testCase.verifyEqual(b.information,a.information,AbsTol=0);
            testCase.verifyFalse(b.positionAiding.used);
        end
        function positionSelectsCorrectRepeatedStructure(testCase)
            [fixed,moving,cfg,aid]=fixture();
            before=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg);
            after=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg,aid);
            testCase.verifyGreaterThan(before.poseXYTheta(1),.4);
            testCase.verifyEqual(after.poseXYTheta,[0 0 0],AbsTol=1e-6);
            testCase.verifyTrue(after.accepted);
            testCase.verifyEqual(after.positionAiding.selected,2);
            testCase.verifyFalse(after.positionAiding.gnssInformationAdded);
            event=registrationSupport.registrationPoseMeasurement(after,1);
            testCase.verifyTrue(event.conditionedOnPositionAid);
            testCase.verifyFalse(event.positionAidInformationAdded);
        end
        function uncertainAidDoesNotRedirectMatching(testCase)
            [fixed,moving,cfg,aid]=fixture();aid.covariance=100*eye(2);
            r=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg,aid);
            testCase.verifyGreaterThan(r.poseXYTheta(1),.4);
            testCase.verifyEqual(r.positionAiding.reason,"uncertainPosition");
        end
        function disagreementCannotIncreaseInformation(testCase)
            [fixed,moving,cfg,aid]=fixture();aid.covariance=.25*eye(2);
            r=registerSemanticProbabilityCloud(fixed,moving,[.48 0 0],cfg,aid);
            conditional=registerSemanticProbabilityCloud(fixed,moving,r.poseXYTheta,cfg);
            testCase.verifyGreaterThan(r.positionAiding.betweenHypothesisSecondMoment(1,1),.01);
            testCase.verifyLessThanOrEqual(max(eig(r.information-conditional.information)),1e-8);
            testCase.verifyGreaterThan(min(eig(r.information)),0);
        end
        function strongDisagreementWithAllOptimaWithholdsMeasurement(testCase)
            [~,moving,cfg,aid]=fixture();aid.position=[0 1];aid.covariance=.001*eye(2);
            r=registerSemanticProbabilityCloud(moving,moving,[0 0 0],cfg,aid);
            testCase.verifyFalse(r.accepted || r.directionalAccepted);
            testCase.verifyEqual(r.reason,"positionAidConflict");
        end
        function aidDoesNotInventAnUnobservableDirection(testCase)
            cloud=geometricRegistrationTest.parallelRoad();cfg=distributionRegistrationConfig();
            aid=struct('position',[0 0],'covariance',.04*eye(2),'valid',true);
            r=registerSemanticProbabilityCloud(cloud,cloud,[.8 .3 .02],cfg,aid);
            testCase.verifyFalse(r.accepted);
            testCase.verifyTrue(r.directionalAccepted);
            testCase.verifyEqual(r.observableRank,2);
            testCase.verifyLessThan(abs(min(eig(r.information))),1e-8);
        end
        function malformedAidIsRejected(testCase)
            [fixed,moving,cfg,aid]=fixture();aid.covariance(1,1)=-1;
            testCase.verifyError(@()registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg,aid), ...
                'VehicleLocalization:InvalidPositionAid');
        end
        function truncatedWeakDirectionStaysOutOfDirectionalInformation(testCase)
            cloud=geometricRegistrationTest.parallelRoad();R=[cos(.05) -sin(.05);sin(.05) cos(.05)];
            cloud.components.covariance(:,:,1)=R*cloud.components.covariance(:,:,1)*R.';
            aid=struct('position',[0 0],'covariance',.04*eye(2));
            r=registerSemanticProbabilityCloud(cloud,cloud,[.8 .3 .02],distributionRegistrationConfig(),aid);
            testCase.verifyTrue(r.directionalAccepted);
            testCase.verifyGreaterThan(min(eig(r.information)),1e-5);
            testCase.verifyLessThan(abs(min(eig(r.directionalInformation))),1e-10);
        end
        function mismatchedAcquisitionTimeIsRejected(testCase)
            cfg=struct('registration',distributionRegistrationConfig());
            testCase.verifyError(@()localizeLidarFrame([],[],[0 0 0],1,cfg,[],[],struct('timestamp',0)), ...
                'VehicleLocalization:PositionAidTimeMismatch');
        end
    end
end

function [fixed,moving,cfg,aid]=fixture()
    moving=distributionRegistrationTest.exampleCloud();fixed=moving;c=moving.components;
    fixed.components.mean=[c.mean;c.mean+[.5 0]];
    fixed.components.covariance=cat(3,c.covariance,c.covariance);
    fixed.components.semanticName=[c.semanticName;c.semanticName];
    fixed.components.mixtureWeight=[c.mixtureWeight;c.mixtureWeight]/2;
    fixed.components.repeatability=[c.repeatability;c.repeatability];
    fixed.components.numComponents=2*c.numComponents;
    cfg=distributionRegistrationConfig();
    aid=struct('position',[0 0],'covariance',.01*eye(2),'valid',true);
end
