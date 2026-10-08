classdef partialSignMatchingTest < matlab.unittest.TestCase
% partialSignMatchingTest Partial panels retain normal force and honest information.
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function intrinsicShapeExcludesBetweenViewCenterDrift(t)
            cloud=landmarkViewMapTest.example();
            conditioned=registrationSupport.conditionSemanticMapOnView(cloud,[0 0 0]);
            t.verifyEqual(conditioned.components.intrinsicCovariance(:,:,1),diag([.01 .02]),AbsTol=1e-14);
            t.verifyEqual(conditioned.components.covariance(:,:,1),diag([.05 .02]),AbsTol=1e-14);
        end
        function coherentCenterAndDisplacedPatchUseDifferentConstraints(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyEqual(s.pairs.partialSignSurface,[false(12,1);true]);
            t.verifyEqual(s.J(2,:,13),zeros(1,3),AbsTol=1e-14);
            t.verifyGreaterThan(abs(s.J(1,1,13)),1);
            t.verifyEqual(s.residual(:,13),zeros(size(s.residual,1),1),AbsTol=1e-14);
        end
        function noMajorityPreservesOriginalPointInformation(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            moving.components.mean(12,2)=1.3;
            model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyFalse(any(s.pairs.partialSignSurface));
            t.verifyGreaterThan(abs(s.J(2,2,13)),1);
        end
        function inconsistentNormalDoesNotBecomeAPartialPanel(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            moving.components.mean(13,1)=3.5;
            model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyFalse(any(s.pairs.partialSignSurface));
        end
        function partialObservationReducesCenterBias(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            actual=registerSemanticProbabilityCloud(fixed,moving,[.1 .1 .01],cfg);
            cfg.partialSign.enabled=false;
            baseline=registerSemanticProbabilityCloud(fixed,moving,[.1 .1 .01],cfg);
            t.verifyTrue(actual.accepted,actual.reason);
            t.verifyLessThan(norm(actual.poseXYTheta(1:2)),norm(baseline.poseXYTheta(1:2))/2);
        end
        function frozenObjectiveGradientMatchesFiniteDifferences(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            pose=[.05 .03 .01];s=model.linearize(pose,ones(3,1));h=1e-6;
            numerical=[model.frozenCost(s,pose+[h 0 0])-model.frozenCost(s,pose-[h 0 0]); ...
                model.frozenCost(s,pose+[0 h 0])-model.frozenCost(s,pose-[0 h 0]); ...
                model.frozenCost(s,pose+[0 0 h])-model.frozenCost(s,pose-[0 0 h])]/(2*h);
            t.verifyEqual(numerical,2*s.gradient,AbsTol=1e-7);
        end
        function cropAndProjectionPreserveIntrinsicShape(t)
            [fixed,~,~]=partialSignMatchingTest.fixture();
            [cropped,ids]=registrationSupport.selectLocalProbabilityCloud(fixed,[3 1 0],.1);
            projected=registrationSupport.projectSemanticProbabilityCloud(cropped,2);
            t.verifyEqual(ids,11);
            t.verifyEqual(projected.components.intrinsicCovariance,diag([.001 .1]),AbsTol=1e-14);
        end
        function rejectsInvalidShape(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            fixed.components.intrinsicCovariance(1,1,11)=-1;
            t.verifyError(@()prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg), ...
                'VehicleLocalization:InvalidIntrinsicShape');
        end
        function emptyMapCropRejectsCleanly(t)
            [fixed,moving,cfg]=partialSignMatchingTest.fixture();
            fixed=registrationSupport.selectLocalProbabilityCloud(fixed,[1000 1000 0],.1);
            actual=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            t.verifyFalse(actual.accepted||actual.directionalAccepted);
            t.verifyEqual(actual.reason,"insufficientComponents");
            t.verifyEqual(actual.information,zeros(3),AbsTol=0);
        end
    end
    methods (Static)
        function [fixed,moving,cfg]=fixture()
            fixed=geometricRegistrationTest.parallelRoad();c=fixed.components;
            c.mean(11,:)=[3 1];c.covariance(:,:,11)=diag([.01 .1]);c.semanticName(11,1)="trafficSign";
            c.repeatability(11,1)=1;c.numComponents=11;c.mixtureWeight=ones(11,1)/11;
            c.intrinsicCovariance=zeros(2,2,11);c.intrinsicCovariance(:,:,11)=diag([.001 .1]);fixed.components=c;
            moving=fixed;moving.components=rmfield(c,'intrinsicCovariance');
            moving.components.mean(12:13,:)=[3 1.04;3 1.5];
            moving.components.covariance(:,:,12:13)=repmat(diag([.01 .02]),1,1,2);
            moving.components.semanticName(12:13,1)="trafficSign";moving.components.repeatability(12:13,1)=1;
            moving.components.numComponents=13;moving.components.mixtureWeight=ones(13,1)/13;
            cfg=distributionRegistrationConfig();cfg.method="geometricD2D";cfg.pyramid.sourceMergeRadius=0;
            cfg.partialSign=struct('enabled',true,'minimumAnisotropy',5,'minimumVariance',.01, ...
                'consensusRadius',.15,'maximumNormalDifference',.2);
        end
    end
end
