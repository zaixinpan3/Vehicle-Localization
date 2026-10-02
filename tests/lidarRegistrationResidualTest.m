classdef lidarRegistrationResidualTest < matlab.unittest.TestCase
% lidarRegistrationResidualTest Matcher-to-observer residual/metric consistency.
    properties (TestParameter)
        method={"supportD2D","geometricD2D","anisotropicD2D"}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function exportedNormalMatrixMatchesRegistration(testCase,method)
            [result,~]=matchedCloud(method);
            m=evaluateLidarRegistrationResidual(result.lidarResidualModel,result.poseXYTheta.');
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(m.jacobian.'*m.jacobian,result.information,AbsTol=1e-8);
            testCase.verifyEqual(m.jacobian.'*m.residual,zeros(3,1),AbsTol=1e-8);
        end
        function predictionResidualDoesNotReuseOptimizerGradient(testCase,method)
            [result,~]=matchedCloud(method);prediction=result.poseXYTheta.'+[.2;-.1;.01];
            m=evaluateLidarRegistrationResidual(result.lidarResidualModel,prediction);
            cfg=fullObserverConfig();d=designFullObserverGains(cfg);
            G=zeros(3,7);G(1,1)=1;G(2,4)=1;G(3,7)=1;
            correction=computeLidarMatchedCorrection(m,prediction,G,d.lidarMatched,cfg);
            testCase.verifyGreaterThan(norm(m.jacobian.'*m.residual),1e-3);
            testCase.verifyLessThan(correction(1),0);
            testCase.verifyGreaterThan(correction(4),0);
        end
        function projectedCurbResidualCannotReintroduceTangent(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            result=registerSemanticProbabilityCloud(cloud,cloud,[0,0,0]);
            first=evaluateLidarRegistrationResidual(result.lidarResidualModel,[0;.1;0]);
            second=evaluateLidarRegistrationResidual(result.lidarResidualModel,[4;.1;0]);
            anchor=evaluateLidarRegistrationResidual(result.lidarResidualModel,result.poseXYTheta.');
            testCase.verifyTrue(result.directionalAccepted);
            testCase.verifyEqual(first.residual,second.residual,AbsTol=1e-12);
            testCase.verifyEqual(first.jacobian(:,1),zeros(size(first.residual)),AbsTol=1e-12);
            testCase.verifyEqual(anchor.jacobian.'*anchor.jacobian,result.directionalInformation,AbsTol=1e-7);
        end
        function positionAidUncertaintyReducesResidualInformationConsistently(testCase)
            [fixed,moving,cfg,aid]=ambiguous();
            result=registerSemanticProbabilityCloud(fixed,moving,[1.9,0,0],cfg,aid);
            frozen=result.lidarResidualModel;
            m=evaluateLidarRegistrationResidual(frozen,result.poseXYTheta.');
            frozen.poseSpread=zeros(3);raw=evaluateLidarRegistrationResidual(frozen,result.poseXYTheta.');
            testCase.verifyTrue(result.positionAiding.used);
            testCase.verifyGreaterThan(norm(result.positionAiding.betweenHypothesisSecondMoment),.01);
            testCase.verifyEqual(m.jacobian.'*m.jacobian,result.information,AbsTol=1e-8);
            testCase.verifyLessThanOrEqual(max(eig(m.jacobian.'*m.jacobian-raw.jacobian.'*raw.jacobian)),1e-8);
            testCase.verifyTrue(m.conditionedOnPositionAid);
        end
        function frozenExportSurvivesSerialization(testCase)
            [result,~]=matchedCloud("supportD2D");frozen=result.lidarResidualModel;
            file=[tempname,'.mat'];testCase.addTeardown(@()delete(file));save(file,'frozen');
            restored=load(file);pose=result.poseXYTheta.'+[.1;0;0];
            a=evaluateLidarRegistrationResidual(frozen,pose);
            b=evaluateLidarRegistrationResidual(restored.frozen,pose);
            testCase.verifyEqual(a,b);
        end
        function directionalUncertaintyKeepsRejectedGeometryOut(testCase)
            cloud=geometricRegistrationTest.parallelRoad();R=[cos(.05),-sin(.05);sin(.05),cos(.05)];
            cloud.components.covariance(:,:,1)=R*cloud.components.covariance(:,:,1)*R.';
            aid=struct('position',[0,0],'covariance',.04*eye(2));
            result=registerSemanticProbabilityCloud(cloud,cloud,[.8,.3,.02],distributionRegistrationConfig(),aid);
            model=result.lidarResidualModel;
            m=evaluateLidarRegistrationResidual(model,result.poseXYTheta.');
            testCase.verifyTrue(result.directionalAccepted);
            testCase.verifyEqual(m.jacobian.'*m.jacobian,result.directionalInformation,AbsTol=1e-8);
            testCase.verifyEqual(m.jacobian*(eye(3)-model.poseProjector),zeros(size(m.jacobian)),AbsTol=1e-10);
        end
        function currentRuntimeConsumesExportedGeometry(testCase)
            [result,~]=matchedCloud("supportD2D");t=[0;.1;.2];z=zeros(3,1);
            high=struct('time',t,'longitudinalSpeed',z,'longitudinalAcceleration',z, ...
                'lateralAcceleration',z,'yawRate',z);
            lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
            data=struct('highRate',high,'lidarMatcher',@(~,~,~)result);
            cfg=fullObserverConfig();cfg.initialState=[.2;0;0;-.1;0;0;0];
            r=runSynchronousLocalizationObserver(data,cfg,lateral);
            testCase.verifyEqual(r.diagnostics.lidarMatched{2}.representation,"predictedPoseResidual");
            testCase.verifyLessThan(norm(r.position(end,:)),norm(r.position(1,:)));
        end
        function rejectedMatchCannotExportAnAdmittedResidual(testCase)
            cloud=distributionRegistrationTest.exampleCloud();cfg=distributionRegistrationConfig();
            cfg.maximumIterationsPerScale=1;
            result=registerSemanticProbabilityCloud(cloud,cloud,[1,.5,.1],cfg);
            testCase.verifyFalse(result.accepted || result.directionalAccepted);
            testCase.verifyEmpty(registrationSupport.registrationPoseMeasurement(result,0));
        end
    end
end

function [result,cloud]=matchedCloud(method)
    cloud=distributionRegistrationTest.exampleCloud();cfg=distributionRegistrationConfig();cfg.method=method;
    cfg.pyramid.mapMergeRadius=0;cfg.pyramid.sourceMergeRadius=0;
    result=registerSemanticProbabilityCloud(cloud,cloud,[0,0,0],cfg);
end

function [fixed,moving,cfg,aid]=ambiguous()
    moving=distributionRegistrationTest.exampleCloud();fixed=moving;c=moving.components;
    fixed.components.mean=[c.mean;c.mean+[2,0]];
    fixed.components.covariance=cat(3,c.covariance,c.covariance);
    fixed.components.semanticName=[c.semanticName;c.semanticName];
    fixed.components.mixtureWeight=[c.mixtureWeight;c.mixtureWeight]/2;
    fixed.components.repeatability=[c.repeatability;c.repeatability];
    fixed.components.numComponents=2*c.numComponents;
    cfg=distributionRegistrationConfig();aid=struct('position',[0,0],'covariance',4*eye(2),'valid',true);
end
