classdef accelerationDomainTest < matlab.unittest.TestCase
% Physical-set certificate tests independent of optimizer success.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function domainPolynomialReconstructsTheAccelerationDisk(testCase)
            testCase.verifyLessThan(reconstructionError(),2e-14);
        end
        function physicalSetCanBeCertifiedDespiteAnUnstableOuterCorner(testCase)
            p=struct('maximumAcceleration',2);box=[1,3,-.4,0,-2,0];
            controls=coupledMotionSupport.boxControls(box);
            g=coupledMotionSupport.accelerationConstraint(p,controls);
            % F=(||a||^2-Amax^2-.1)*I is negative on the whole disk.
            % T=Amax^2*I makes every augmented coefficient exactly -.1*I.
            original=controls(:,6)-4-.1;
            testCase.verifyGreaterThan(max(original),0);
            testCase.verifyEqual(original+4*g,-.1*ones(27,1),AbsTol=2e-15);
        end
        function formerLimitingCornersAreOutsideThePhysicalSet(testCase)
            p=struct('maximumAcceleration',2);
            f=[coupledMotionSupport.features(1,-.4,2);coupledMotionSupport.features(10,-.2,2)];
            testCase.verifyEqual(coupledMotionSupport.accelerationConstraint(p,f),[-.04;-1],AbsTol=1e-14);
        end
        function zeroAccelerationNeedsNoDivisionOrMultiplier(testCase)
            p=struct('maximumAcceleration',0);
            testCase.verifyEqual(coupledMotionSupport.accelerationConstraint(p,zeros(5,8)),zeros(5,1));
        end
        function indefiniteMultiplierIsRejected(testCase)
            [penalty,valid]=sensorRedundancySupport.multiplierResidual({eye(8);-eye(8);zeros(8)});
            testCase.verifyFalse(valid);testCase.verifyEqual(penalty,[0;1;0]);
        end
        function psdRoundoffIsChargedRatherThanIgnored(testCase)
            [penalty,valid]=sensorRedundancySupport.multiplierResidual({-1e-10*eye(8);zeros(8);eye(8)});
            testCase.verifyTrue(valid);testCase.verifyEqual(penalty,[1e-10;0;0]);
        end
        function zeroMultiplierExactlyRecoversThePreviousConditions(testCase)
            c=mncavFullObserverConfig();c.invariantObserver.redundancy.speedPartitions=1;
            w=sensorRedundancySupport.witness(c,eye(8),2,3,[4;5;6],1);
            [L,~,~]=sensorRedundancySupport.blocks(w,true,1);
            features=coupledMotionSupport.controls(w.inputs,w.domain);
            [A,sp,pa,di]=coupledMotionSupport.matrices(w.inputs,features(1,:));
            M=A-4*sp-5*pa-6*di;sensor=sensorRedundancySupport.sensorBlocks(w,true);
            testCase.verifyEqual(L{1},M+M.'+w.domain.lambda*eye(8)+sensor{1}{1},AbsTol=1e-14);
        end
        function sharedMultiplierEmbedsExactlyInEveryBox(testCase)
            testCase.verifyLessThan(embeddingError(),1e-12);
        end
    end
end

function error=embeddingError()
    c=mncavFullObserverConfig();c.invariantObserver.redundancy.speedPartitions=2;
    c.invariantObserver.redundancy.accelerationMultiplierPartition="global";
    T={diag(1:8);ones(8);eye(8)};
    shared=sensorRedundancySupport.witness(c,eye(8),2,3,[4;5;6],1,T);
    c.invariantObserver.redundancy.accelerationMultiplierPartition="box";
    [~,groups]=coupledMotionSupport.multiplierGroups(c.invariantObserver.redundancy);
    local=sensorRedundancySupport.witness(c,eye(8),2,3,[4;5;6],1,repmat(T,1,groups));
    counts=sensorRedundancySupport.counts(shared);ids=unique(round(linspace(1,sum(counts),61)));
    [L,G,M]=sensorRedundancySupport.blocks(shared,true,ids);a=[L,G,M];
    [L,G,M]=sensorRedundancySupport.blocks(local,true,ids);b=[L,G,M];error=0;
    for j=1:numel(a),error=max(error,norm(a{j}-b{j},'fro'));end
end

function error=reconstructionError()
    c=mncavSmallOffsetObserverConfig();p=sensorRedundancySupport.inputs(c,1);
    box=[1,3,-.4,0,-2,0];controls=coupledMotionSupport.boxControls(box);
    g=coupledMotionSupport.accelerationConstraint(p,controls);
    stream=RandStream('mt19937ar','Seed',2810);error=0;
    for j=1:30
        u=rand(stream,1,3);x=box([1,3,5])+u.*(box([2,4,6])-box([1,3,5]));
        basis=@(t)[(1-t)^2,2*t*(1-t),t^2];
        weights=kron(kron(basis(u(1)),basis(u(2))),basis(u(3)));
        expected=1-(x(3)^2+(x(1)*x(2))^2)/p.maximumAcceleration^2;
        error=max(error,abs(weights*g-expected));
    end
end
