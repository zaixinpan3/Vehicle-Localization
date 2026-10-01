classdef anisotropicRegistrationTest < matlab.unittest.TestCase
% anisotropicRegistrationTest Continuous geometry and exact objective gradients.
    properties (TestParameter)
        semanticName={"curb","pole","trafficSign","facade","roadMarking"}
    end
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function recoversPoseWithMixedFeatures(t)
            moving=distributionRegistrationTest.exampleCloud();expected=[700001.2 4300000.7 .16];
            fixed=distributionRegistrationTest.transform(moving,expected);
            actual=registerSemanticProbabilityCloud(fixed,moving,expected+[-.6 .3 -.09],anisotropicRegistrationConfig());
            t.verifyTrue(actual.accepted,actual.reason);
            t.verifyEqual(actual.poseXYTheta,expected,AbsTol=3e-4);
        end
        function coincidentRoundCloudsCannotInventYaw(t)
            c=distributionRegistrationTest.exampleCloud();c.components.mean(:)=0;
            c.components.covariance=repmat(eye(2),1,1,6);
            actual=registerSemanticProbabilityCloud(c,c,[0 0 .2],anisotropicRegistrationConfig());
            t.verifyFalse(actual.accepted);t.verifyTrue(actual.directionalAccepted,actual.reason);
            t.verifyEqual(actual.observableRank,2);
            t.verifyEqual(actual.poseXYTheta,[0 0 .2],AbsTol=1e-12);
            t.verifyEqual(actual.directionalInformation*[0;0;1],zeros(3,1),AbsTol=1e-12);
        end
        function graphUsesTheSameContinuousResidualModel(t)
            c=distributionRegistrationTest.exampleCloud();c.observationScope="confirmedCurrentAcquisition";
            c.acquisitionTime=0;cfg=robustPoseGraphConfig();cfg.registration=anisotropicRegistrationConfig();
            packet=struct('time',0,'seed',[0 0 0],'source',c,'relativeMotion',[0 0 0], ...
                'motionCovariance',diag([.04 .04 .003].^2),'positionAid',struct('valid',false));
            [first,state]=updateRobustPoseGraph([],packet,c,cfg);
            t.verifyTrue(any(first.correspondences.shapeRotationUsed));
            packet.time=.1;packet.source.acquisitionTime=.1;packet.seed=[.1 -.1 .02];
            actual=updateRobustPoseGraph(state,packet,c,cfg);
            t.verifyLessThan(norm(actual.poseXYTheta),1e-4);
            t.verifyTrue(all(isfinite(actual.information),'all'));
        end
        function everyClassUsesIdenticalDistributionGeometry(t,semanticName)
            c=anisotropicRegistrationTest.cloud(diag([2 .01]),semanticName);
            model=prepareSemanticRegistrationGeometry(c,c,[0 0 0],anisotropicRegistrationConfig());
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyEqual(s.H(2,2)/s.H(1,1),4.01/.03,AbsTol=1e-10);
            t.verifyGreaterThan(s.H(1,1),0);
            t.verifyGreaterThan(s.H(3,3),0);
            t.verifyEqual(s.cost,0,AbsTol=1e-14);
            t.verifyEqual(s.similarity,1,AbsTol=1e-14);
        end
        function axisWeightsChangeContinuously(t)
            ratios=[1 1+1e-7 1.1 2 10 100];weights=zeros(size(ratios));yaw=weights;
            for k=1:numel(ratios)
                c=anisotropicRegistrationTest.cloud(diag([.02*ratios(k) .02]),"curb");
                model=prepareSemanticRegistrationGeometry(c,c,[0 0 0],anisotropicRegistrationConfig());
                s=model.linearize([0 0 0],ones(3,1));
                weights(k)=s.H(1,1)/s.H(2,2);yaw(k)=s.H(3,3);
            end
            t.verifyEqual(weights(1),1,AbsTol=1e-14);
            t.verifyLessThan(max(diff(weights)),0);
            t.verifyGreaterThan(min(diff(yaw)),0);
            t.verifyEqual(yaw(1),0,AbsTol=1e-14);
            t.verifyLessThan(yaw(2),1e-12);
        end
        function roundCloudsConstrainOnlyTheirCenters(t)
            c=anisotropicRegistrationTest.cloud(.02*eye(2),"curb");
            model=prepareSemanticRegistrationGeometry(c,c,[0 0 0],anisotropicRegistrationConfig());
            s=model.linearize([.2 -.1 .3],ones(3,1));
            t.verifyFalse(any(s.pairs.shapeRotationUsed));
            t.verifyEqual(s.J(:,:,1),[diag([sqrt(20) sqrt(20) 0]);zeros(1,3)],AbsTol=1e-12);
            t.verifyEqual(model.frozenCost(s,[.2 -.1 -.4]),s.cost,AbsTol=1e-12);
        end
        function rotatingAnElongatedCloudAtItsCenterCostsMore(t)
            c=anisotropicRegistrationTest.cloud(diag([2 .01]),"pole");
            model=prepareSemanticRegistrationGeometry(c,c,[0 0 0],anisotropicRegistrationConfig());
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyGreaterThan(model.frozenCost(s,[0 0 .1]),s.cost);
            t.verifyGreaterThan(model.frozenCost(s,[0 .1 0]),model.frozenCost(s,[.1 0 0]));
        end
        function roundTargetSuppliesNoIntrinsicRotationEvidence(t)
            source=anisotropicRegistrationTest.cloud(diag([2 .01]),"trafficSign");
            target=anisotropicRegistrationTest.cloud(.1*eye(2),"trafficSign");
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],anisotropicRegistrationConfig());
            s=model.linearize([0 0 .2],ones(3,1));
            t.verifyEqual(s.gradient(3),0,AbsTol=1e-12);
            t.verifyEqual(model.frozenCost(s,[0 0 -.3]),s.cost,AbsTol=1e-12);
            t.verifyEqual(s.J(3,3,1),0,AbsTol=1e-12);
        end
        function exactGradientIncludesRotationOfSourceScatter(t)
            source=anisotropicRegistrationTest.cloud([.8 .12;.12 .03],"facade");
            target=anisotropicRegistrationTest.cloud([.4 -.04;-.04 .06],"facade");
            cfg=anisotropicRegistrationConfig();model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],cfg);
            pose=[.2 -.13 .12];scale=[1;1;.1];s=model.linearize(pose,scale);numerical=zeros(3,1);
            for j=1:3
                step=zeros(1,3);step(j)=1e-6*scale(j);
                numerical(j)=(model.frozenCost(s,pose+step)-model.frozenCost(s,pose-step))/(2e-6);
            end
            t.verifyEqual(numerical,2*s.gradient,AbsTol=1e-8);
        end
        function matchCostIsSymmetricAndFrameInvariant(t)
            source=anisotropicRegistrationTest.cloud([.8 .12;.12 .03],"pole");
            target=anisotropicRegistrationTest.cloud([.4 -.04;-.04 .06],"pole");
            pose=[.2 -.13 .12];cfg=anisotropicRegistrationConfig();
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],cfg);s=model.linearize(pose,ones(3,1));
            r=rotation(pose(3));inverse=[-pose(1:2)*r,-pose(3)];
            reversed=prepareSemanticRegistrationGeometry(source,target,[0 0 0],cfg);
            other=reversed.linearize(inverse,ones(3,1));
            t.verifyEqual(other.cost,s.cost,AbsTol=1e-12);
            a=.47;source=distributionRegistrationTest.transform(source,[0 0 a]);
            target=distributionRegistrationTest.transform(target,[0 0 a]);
            rotated=prepareSemanticRegistrationGeometry(target,source,[0 0 0],cfg);
            other=rotated.linearize([pose(1:2)*rotation(a).',pose(3)],ones(3,1));
            t.verifyEqual(other.cost,s.cost,AbsTol=1e-12);
        end
    end
    methods (Static)
        function c=cloud(covariance,name)
            components=struct('mean',[0 0],'covariance',covariance, ...
                'semanticName',name,'mixtureWeight',1,'numComponents',1);
            c=struct('components',components,'frameCalibration',lidarFrameCalibrationConfig());
        end
    end
end
function r=rotation(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
