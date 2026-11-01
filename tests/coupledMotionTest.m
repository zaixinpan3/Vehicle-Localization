classdef coupledMotionTest < matlab.unittest.TestCase
% Analytic interval enclosure, affine convexity and independent Jacobians.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function physicalJacobiansDominateTheEnclosedMatrices(testCase)
            [errors,minimum]=jacobianErrors();
            testCase.verifyLessThan(max(errors),2e-11);
            testCase.verifyGreaterThanOrEqual(minimum,-2e-11);
        end
        function everyPhysicalCoefficientHasConvexVertexWeights(testCase)
            [error,minimumWeight]=containmentErrors();
            testCase.verifyLessThan(error,2e-11);
            testCase.verifyGreaterThanOrEqual(minimumWeight,-1e-14);
        end
        function degenerateDomainsHaveNoDuplicateVertices(testCase)
            c=mncavFullObserverConfig();c.iss.maximumAcceleration=0;
            c.iss.maximumSpeed=c.invariantObserver.fusion.minimumCourseSpeed;
            [v,b]=localizationSupport.fusionVertices(c);
            testCase.verifySize(v,[1,8]);testCase.verifyEqual(v,b(:,1).');
        end
        function affineVertexInterpolationPreservesTheWholeLmi(testCase)
            testCase.verifyLessThan(affineError(),1e-10);
        end
    end
end

function [errors,minimum]=jacobianErrors()
    stream=RandStream('mt19937ar','Seed',20261031);errors=zeros(1,3);minimum=Inf;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;s0=c.invariantObserver.fusion.minimumCourseSpeed;
        B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);D=localizationSupport.outputNormalization(c);
        for j=1:200
            s=s0+rand(stream)*(c.iss.maximumSpeed-s0);q=min(c.maximumTrackAngleRate,c.iss.maximumAcceleration/s)*(2*rand(stream)-1);
            a=[sqrt(max(0,c.iss.maximumAcceleration^2-(s*q)^2))*(2*rand(stream)-1);s*q];
            x=[0;s;a(1);0;0;a(2);0;q];[F,H]=localizationSupport.secants(x,x,0,0,c);H=D*H*B;
            [A,speed,pair,direction]=localizationSupport.fusionMatrices(c,localizationSupport.fusionFeatures(s,q,a(1),c),1);
            expected=B\F*B/t;expected(8,8)=-1;factor=c.iss.maximumSpeed^2/(s0^2+s^2);
            errors=max(errors,[norm(A-expected,'fro'),norm(speed-factor*H(1,:).'*H(1,:),'fro'), ...
                norm(direction-H(4,:).'*H(4,:),'fro')]);
            difference=factor*H(2:3,:).'*H(2:3,:)-pair;
            minimum=min(minimum,min(eig((difference+difference.')/2)));
        end
    end
end

function [error,minimumWeight]=containmentErrors()
    stream=RandStream('mt19937ar','Seed',3110);error=0;minimumWeight=1;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};[vertices,b]=localizationSupport.fusionVertices(c);s0=c.invariantObserver.fusion.minimumCourseSpeed;
        for j=1:500
            s=s0+(c.iss.maximumSpeed-s0)*rand(stream);q=min(c.maximumTrackAngleRate,c.iss.maximumAcceleration/s)*(2*rand(stream)-1);
            a=sqrt(max(0,c.iss.maximumAcceleration^2-(s*q)^2))*(2*rand(stream)-1);
            f=localizationSupport.fusionFeatures(s,q,a,c);w=vertexWeights(f,b);
            error=max(error,max([norm(w.'*vertices-f,Inf),abs(sum(w)-1)]));minimumWeight=min(minimumWeight,min(w));
        end
    end
end

function w=vertexWeights(f,b)
    u=(f(:)-b(:,1))./(b(:,2)-b(:,1));w=ones(2^numel(u),1);
    for j=1:numel(u)
        high=bitget((0:numel(w)-1).',j);w=w.*(high*u(j)+(1-high)*(1-u(j)));
    end
end

function error=affineError()
    c=mncavFullObserverConfig();[vertices,b]=localizationSupport.fusionVertices(c);
    f=(.37*b(:,1)+.63*b(:,2)).';w=vertexWeights(f,b);P=diag(1:8);combined=zeros(8);
    for j=1:size(vertices,1)
        [A,S,B,D]=localizationSupport.fusionMatrices(c,vertices(j,:),2);M=P*A-3*S-4*B-5*D;
        combined=combined+w(j)*(M+M.');
    end
    [A,S,B,D]=localizationSupport.fusionMatrices(c,f,2);M=P*A-3*S-4*B-5*D;
    error=norm(combined-M-M.','fro');
end
