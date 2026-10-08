classdef lineDirectionRegistrationTest < matlab.unittest.TestCase
% lineDirectionRegistrationTest Geometry, null-space and derivative checks.
    methods (TestClassSetup)
        function paths(~)
            root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function ignoresSparseOrNonlinearNeighborhoods(testCase)
            c=road();cfg=settings();c.components.mean=c.components.mean(1:2,:);c.components.semanticName=c.components.semanticName(1:2);c.components.numComponents=2;c.components.mixtureWeight=ones(2,1);
            [~,valid]=registrationSupport.sourceLineDirections(c.components,cfg.lineDirection);testCase.verifyFalse(any(valid));
            c.components.mean=[0 0;1 0;0 1;1 1];c.components.semanticName=repmat("curb",4,1);c.components.numComponents=4;c.components.mixtureWeight=ones(4,1);
            [~,valid]=registrationSupport.sourceLineDirections(c.components,cfg.lineDirection);testCase.verifyFalse(any(valid));
        end
        function duplicatedMeansCannotCreateDirectionSupport(testCase)
            c=road();cfg=settings();c.components.mean=repmat([0 0;3 0],4,1);
            [~,valid]=registrationSupport.sourceLineDirections(c.components,cfg.lineDirection);
            testCase.verifyFalse(any(valid));
        end
        function disabledFactorPreservesLegacyConfiguration(testCase)
            c=withPole();cfg=settings();legacy=rmfield(cfg,'lineDirection');cfg.lineDirection.enabled=false;
            a=registerSemanticProbabilityCloud(c,c,[.2 .1 .03],legacy);
            b=registerSemanticProbabilityCloud(c,c,[.2 .1 .03],cfg);
            testCase.verifyEqual(a.poseXYTheta,b.poseXYTheta);
            testCase.verifyEqual(a.information,b.information);
        end
        function rejectsInvalidDirectionScale(testCase)
            c=road();cfg=settings();cfg.lineDirection.standardDeviation=0;
            testCase.verifyError(@() registrationSupport.sourceLineDirections(c.components,cfg.lineDirection), ...
                'VehicleLocalization:InvalidLineDirectionConfiguration');
        end
        function roadDirectionCannotInventAlongRoadInformation(testCase)
            c=road();cfg=settings();result=registerSemanticProbabilityCloud(c,c,[.4 .2 .03],cfg);
            testCase.verifyTrue(result.directionalAccepted,result.reason);testCase.verifyEqual(result.observableRank,2);
            testCase.verifyEqual(result.poseXYTheta,[.4 0 0],'AbsTol',1e-4);
            testCase.verifyEqual(result.information*[1;0;0],zeros(3,1),'AbsTol',1e-10);
        end
        function recoversRigidTransform(testCase)
            c=withPole();fixed=transform(c,[700000 4300000 .21]);cfg=settings();expected=[700000 4300000 .21];
            result=registerSemanticProbabilityCloud(fixed,c,expected+[-.3 .2 -.05],cfg);
            testCase.verifyTrue(result.accepted,result.reason);testCase.verifyEqual(result.poseXYTheta,expected,'AbsTol',1e-4);
        end
        function directionPreventsBiasedPoleFromRotatingRoad(testCase)
            c=withPole();fixed=c;fixed.components.mean(end,:)=fixed.components.mean(end,:)+[.1 .35];cfg=settings();
            disabled=cfg;disabled.lineDirection.enabled=false;
            before=registerSemanticProbabilityCloud(fixed,c,[0 0 0],disabled);after=registerSemanticProbabilityCloud(fixed,c,[0 0 0],cfg);
            testCase.verifyTrue(after.accepted,after.reason);
            testCase.verifyLessThan(abs(after.poseXYTheta(3)),abs(before.poseXYTheta(3))*.4);
            % Biased point position can still bias translation; yaw is the tested contract.
            testCase.verifyLessThan(norm(after.poseXYTheta(1:2)),.35);
        end
        function associationUsesTheSameDirectionEvidence(testCase)
            moving=road();fixed=moving;c=fixed.components;n=c.numComponents;
            angle=deg2rad(25);r=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            fixed.components.mean=[c.mean+[0 .02];c.mean+[0 .2]];
            fixed.components.covariance=cat(3,pagemtimes(pagemtimes(r,c.covariance),r.'),c.covariance);
            fixed.components.semanticName=repmat("curb",2*n,1);fixed.components.mixtureWeight=ones(2*n,1)/(2*n);fixed.components.numComponents=2*n;
            cfg=settings();model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            system=model.linearize([0 0 0],[1;1;.1]);
            testCase.verifyGreaterThan(system.pairs.target,n*ones(n,1));
            cfg.lineDirection.enabled=false;model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);
            system=model.linearize([0 0 0],[1;1;.1]);testCase.verifyTrue(any(system.pairs.target<=n));
        end
        function frozenCostGradientMatchesFiniteDifferences(testCase)
            c=withPole();cfg=settings();model=prepareSemanticRegistrationGeometry(c,c,[0 0 0],cfg);scale=[1;1;.1];pose=[.04 -.03 .015];
            sys=model.linearize(pose,scale);gradient=zeros(3,1);
            for j=1:3
                delta=zeros(1,3);delta(j)=1e-6*scale(j);
                gradient(j)=(model.frozenCost(sys,pose+delta)-model.frozenCost(sys,pose-delta))/(2e-6);
            end
            testCase.verifyEqual(gradient,2*sys.gradient,'AbsTol',1e-6);
        end
        function directionIsInvariantToAxisSign(testCase)
            c=road();cfg=settings();[t,valid]=registrationSupport.sourceLineDirections(c.components,cfg.lineDirection);
            c.components.mean=-c.components.mean;
            [u,other]=registrationSupport.sourceLineDirections(c.components,cfg.lineDirection);
            testCase.verifyEqual(other,valid);testCase.verifyEqual(abs(sum(t(valid,:).*u(valid,:),2)),ones(nnz(valid),1),'AbsTol',1e-12);
        end
    end
end
function cfg=settings()
    cfg=distributionRegistrationConfig();cfg.method="geometricD2D";cfg.lineDirection=struct('enabled',true,'radius',4,'minimumComponents',3,'minimumAnisotropy',9,'minimumSpan',2.4,'standardDeviation',deg2rad(.5));
end
function c=road()
    p=[(0:1.2:8.4).' zeros(8,1)];n=size(p,1);
    x=struct('mean',p,'covariance',repmat(diag([.3 .03]),1,1,n),'semanticName',repmat("curb",n,1),'mixtureWeight',ones(n,1)/n,'numComponents',n);
    c=struct('components',x,'frameCalibration',lidarFrameCalibrationConfig());
end
function c=withPole()
    c=road();n=c.components.numComponents+1;c.components.mean(end+1,:)=[11 -1];c.components.covariance(:,:,end+1)=.03*eye(2);c.components.semanticName(end+1)="pole";c.components.mixtureWeight=ones(n,1)/n;c.components.numComponents=n;
end
function c=transform(c,pose)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];c.components.mean=c.components.mean*r.'+pose(1:2);
    c.components.covariance=pagemtimes(pagemtimes(r,c.components.covariance),r.');
end
