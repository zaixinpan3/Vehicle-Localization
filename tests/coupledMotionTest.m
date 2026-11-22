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
            c=mncavFullObserverConfig();c.iss.maximumAcceleration=0;c.maximumBodyYawRate=0;
            c.iss.maximumSpeed=c.invariantObserver.fusion.minimumCourseSpeed;
            [v,b]=localizationSupport.fusionVertices(c);
            testCase.verifySize(v,[1,4]);testCase.verifyEqual(v,b(:,1).');
        end
        function affineVertexInterpolationPreservesTheWholeLmi(testCase)
            testCase.verifyLessThan(affineError(),1e-10);
        end
    end
end

function [errors,minimum]=jacobianErrors()
    stream=RandStream('mt19937ar','Seed',20261031);errors=zeros(1,2);minimum=Inf;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;s0=c.invariantObserver.fusion.minimumCourseSpeed;
        B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);D=localizationSupport.outputNormalization(c);
        for j=1:200
            [v,a,r]=sampleMotion(c,s0,stream);
            x=[0;v(1);a(1);0;v(2);a(2);-pi+2*pi*rand(stream);r];
            [F,H]=localizationSupport.secants(x,x,c);H=D*H*B;
            A=localizationSupport.fusionMatrices(c,localizationSupport.fusionFeatures(v,a,r));
            expected=B\F*B/t;
            errors=max(errors,[norm(A-expected,'fro'),0]);
            minimum=min(minimum,min(eig(H.'*H)));
        end
    end
end

function [error,minimumWeight]=containmentErrors()
    stream=RandStream('mt19937ar','Seed',3110);error=0;minimumWeight=1;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};[vertices,b]=localizationSupport.fusionVertices(c);s0=c.invariantObserver.fusion.minimumCourseSpeed;
        for j=1:500
            [v,a,r]=sampleMotion(c,s0,stream);
            f=localizationSupport.fusionFeatures(v,a,r);w=vertexWeights(f,b);
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
        A=localizationSupport.fusionMatrices(c,vertices(j,:));M=P*A;
        combined=combined+w(j)*(M+M.');
    end
    A=localizationSupport.fusionMatrices(c,f);M=P*A;
    error=norm(combined-M-M.','fro');
end

function [v,a,r]=sampleMotion(c,s0,stream)
    phi=2*pi*rand(stream);alpha=2*pi*rand(stream);
    v=(s0+(c.iss.maximumSpeed-s0)*rand(stream))*[cos(phi);sin(phi)];
    a=c.iss.maximumAcceleration*rand(stream)*[cos(alpha);sin(alpha)];
    r=c.maximumBodyYawRate*(2*rand(stream)-1);
end
