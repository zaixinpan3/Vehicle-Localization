classdef supportRegistrationTest < matlab.unittest.TestCase
% supportRegistrationTest Partial observations and continuous shape constraints.
    properties (TestParameter)
        semanticName={"curb","pole","trafficSign","facade","roadMarking"}
    end
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function recoversMixedFeaturePose(t)
            source=distributionRegistrationTest.exampleCloud();pose=[700001.2 4300000.7 .16];
            target=distributionRegistrationTest.transform(source,pose);
            actual=registerSemanticProbabilityCloud(target,source,pose+[-.6 .3 -.09],supportRegistrationConfig());
            t.verifyTrue(actual.accepted,actual.reason);
            t.verifyEqual(actual.poseXYTheta,pose,AbsTol=3e-4);
        end
        function everyClassAllowsPartialSupport(t,semanticName)
            source=anisotropicRegistrationTest.cloud(diag([.04 .001]),semanticName);
            target=anisotropicRegistrationTest.cloud(diag([3 .001]),semanticName);
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],supportRegistrationConfig());
            s=model.linearize([1 0 0],ones(3,1));
            t.verifyLessThan(s.H(1,1)/s.H(2,2),1e-5);
            t.verifyLessThan(abs(s.gradient(1)),1e-3);
            t.verifyGreaterThan(s.H(2,2),50);
            t.verifyGreaterThan(s.H(3,3),1);
        end
        function roundCloudsRetainCenterConstraints(t)
            cloud=anisotropicRegistrationTest.cloud(.02*eye(2),"curb");
            model=prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],supportRegistrationConfig());
            s=model.linearize([.2 -.1 .3],ones(3,1));
            t.verifyEqual(s.J(:,:,1),diag([sqrt(20) sqrt(20) 0]),AbsTol=1e-12);
            t.verifyEqual(model.frozenCost(s,[.2 -.1 -.4]),s.cost,AbsTol=1e-12);
        end
        function partialConsensusIsIndependentOfClass(t,semanticName)
            target=anisotropicRegistrationTest.cloud(diag([.04 .004]),semanticName);
            source=target;source.components.mean=[0 0;.04 0;.45 0];
            source.components.covariance=repmat(diag([.05 .01]),1,1,3);
            source.components.semanticName=repmat(semanticName,3,1);
            source.components.mixtureWeight=ones(3,1)/3;source.components.numComponents=3;
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],supportRegistrationConfig());
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyEqual(s.pairs.partialSupport,[false;false;true]);
            t.verifyLessThan(s.pairs.slidingFraction(1:2),[.5;.5]);
            t.verifyEqual(s.pairs.slidingFraction(3),1,AbsTol=1e-12);
            t.verifyLessThan(s.J(1,1,3)^2,.5*s.J(1,1,1)^2);
        end
        function frozenGradientIncludesRotatingScatter(t)
            source=anisotropicRegistrationTest.cloud([.8 .12;.12 .03],"facade");
            target=anisotropicRegistrationTest.cloud([.4 -.04;-.04 .06],"facade");
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],supportRegistrationConfig());
            pose=[.2 -.13 .12];scale=[1;1;.1];s=model.linearize(pose,scale);
            numerical=supportRegistrationTest.gradient(model,s,pose,scale);
            t.verifyEqual(numerical,2*s.gradient,AbsTol=1e-8);
        end
        function unequalScalesRetainAlignedAngularCurvature(t)
            source=diag([.16 .01]);target=diag([3 .03]);
            [residual,J]=gaussianRegistrationResiduals([0 0],source,[0 0],target,[0 0 0],.1);
            plus=gaussianRegistrationResiduals([0 0],source,[0 0],target,[0 0 1e-4],.1);
            minus=gaussianRegistrationResiduals([0 0],source,[0 0],target,[0 0 -1e-4],.1);
            curvature=(sum(plus.^2)-2*sum(residual.^2)+sum(minus.^2))/(2e-8);
            t.verifyEqual(J(3,3)^2,curvature,AbsTol=2e-6);
            t.verifyGreaterThan(J(3,3)^2,2.8);
            t.verifyEqual(J(4,:),zeros(1,3),AbsTol=1e-12);
            t.verifyGreaterThan(residual(4),0);
        end
        function aRoundTargetCannotInventShapeDirection(t)
            source=anisotropicRegistrationTest.cloud(diag([2 .01]),"pole");
            target=anisotropicRegistrationTest.cloud(.1*eye(2),"pole");
            model=prepareSemanticRegistrationGeometry(target,source,[0 0 0],supportRegistrationConfig());
            s=model.linearize([0 0 .2],ones(3,1));
            t.verifyEqual(s.J(3,:,1),zeros(1,3),AbsTol=1e-12);
            t.verifyEqual(model.frozenCost(s,[0 0 -.3]),s.cost,AbsTol=1e-12);
        end
        function identicalNeighborhoodsHaveZeroResidual(t)
            cloud=anisotropicRegistrationTest.cloud([.05 .015;.015 .01],"curb");
            cloud.components.mean=[-2 0;0 0;2 0];
            cloud.components.covariance=repmat(cloud.components.covariance,1,1,3);
            cloud.components.semanticName=repmat("curb",3,1);
            cloud.components.mixtureWeight=ones(3,1)/3;cloud.components.numComponents=3;
            model=prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],supportRegistrationConfig());
            s=model.linearize([0 0 0],ones(3,1));
            t.verifyEqual(s.cost,0,AbsTol=1e-12);
            t.verifyEqual(s.gradient,zeros(3,1),AbsTol=1e-12);
        end
        function continuousAnisotropyReachesTheRoundLimit(t)
            [ratio,angular]=supportRegistrationTest.axisSweep();
            t.verifyEqual(ratio(1),1,AbsTol=1e-12);
            t.verifyEqual(angular(1),0,AbsTol=1e-12);
            t.verifyLessThan(max(diff(ratio)),0);
            t.verifyGreaterThan(min(diff(angular)),0);
            t.verifyLessThan(abs(ratio(2)-ratio(1)),1e-6);
        end
        function defaultPoseGraphUsesSupportGeometry(t)
            cloud=distributionRegistrationTest.exampleCloud();cloud.observationScope="confirmedCurrentAcquisition";
            cloud.acquisitionTime=0;cfg=robustPoseGraphConfig();
            packet=struct('time',0,'seed',[0 0 0],'source',cloud,'relativeMotion',[0 0 0], ...
                'motionCovariance',diag([.04 .04 .003].^2),'positionAid',struct('valid',false));
            [first,state]=updateRobustPoseGraph([],packet,cloud,cfg);
            packet.time=.1;packet.source.acquisitionTime=.1;packet.seed=[.1 -.1 .02];
            actual=updateRobustPoseGraph(state,packet,cloud,cfg);
            t.verifyEqual(cfg.registration.method,"supportD2D");
            t.verifyTrue(any(first.correspondences.shapeRotationUsed));
            t.verifyLessThan(norm(actual.poseXYTheta),1e-4);
        end
        function rejectsInvalidIntrinsicScatter(t)
            cloud=anisotropicRegistrationTest.cloud(.02*eye(2),"pole");
            cloud.components.intrinsicCovariance=diag([-.01 .1]);
            t.verifyError(@()prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],supportRegistrationConfig()), ...
                'VehicleLocalization:InvalidIntrinsicShape');
        end
        function rejectsInvalidSupportScale(t)
            cloud=anisotropicRegistrationTest.cloud(.02*eye(2),"pole");cfg=supportRegistrationConfig();
            cfg.support.maximumGap=0;
            t.verifyError(@()prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],cfg), ...
                'VehicleLocalization:InvalidSupportGeometry');
        end

    end
    methods (Static)
        function [ratios,angular]=axisSweep()
            shapes=[1 1+1e-7 1.1 2 10 100];ratios=zeros(size(shapes));angular=ratios;
            for k=1:numel(shapes)
                cloud=anisotropicRegistrationTest.cloud(diag([.02*shapes(k) .02]),"pole");
                model=prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],supportRegistrationConfig());
                s=model.linearize([0 0 0],ones(3,1));
                ratios(k)=s.H(1,1)/s.H(2,2);angular(k)=s.H(3,3);
            end
        end
        function g=gradient(model,s,pose,scale)
            g=zeros(3,1);
            for k=1:3
                step=zeros(1,3);step(k)=1e-6*scale(k);
                g(k)=(model.frozenCost(s,pose+step)-model.frozenCost(s,pose-step))/(2e-6);
            end
        end
    end
end
