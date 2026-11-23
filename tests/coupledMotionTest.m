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
            v=localizationSupport.fusionVertices(c);
            testCase.verifySize(v,[1,4]);testCase.verifyEqual(v,zeros(1,4));
        end
        function affineVertexInterpolationPreservesTheWholeLmi(testCase)
            testCase.verifyLessThan(affineError(),1e-10);
        end
    end
end

function [errors,minimum]=jacobianErrors()
    stream=RandStream('mt19937ar','Seed',20261031);errors=zeros(1,2);minimum=Inf;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};t=c.invariantObserver.theta;s0=0;
        B=diag(t.^[1,2,3,1,2,3,1,2]);
        for j=1:200
            [v,a,r]=sampleMotion(c,s0,stream);
            x=[0;v(1);a(1);0;v(2);a(2);-pi+2*pi*rand(stream);r];
            [F,H]=localizationSupport.secants(x,x,c);H=H*B;
            A=localizationSupport.fusionMatrices(c,localizationSupport.fusionFeatures(v,a,r));
            expected=B\F*B/t;
            errors=max(errors,[norm(A-expected,'fro'),0]);
            minimum=min(minimum,min(eig(H.'*H))/max(1,norm(H)^2));
        end
    end
end

function [error,minimumWeight]=containmentErrors()
    stream=RandStream('mt19937ar','Seed',3110);error=0;minimumWeight=1;
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};vertices=localizationSupport.fusionVertices(c);s0=0;
        for j=1:500
            [v,a,r]=sampleMotion(c,s0,stream);f=localizationSupport.fusionFeatures(v,a,r);
            u=abs(r);B=2*(u*c.iss.maximumSpeed+c.iss.maximumAcceleration);
            % Independent barycentric recovery on the polygon's two triangles.
            lift=[u;u^2;1];w=zeros(4,1);
            for tri={[1,2,3],[1,3,4]}
                ix=tri{1};weights=[vertices(ix,1:2).';ones(1,3)]\lift;
                if min(weights)>=-1e-13,w(ix)=weights;break;end
            end
            minimumWeight=min([minimumWeight;w]);f0=w.'*vertices;
            error=max(error,norm(f0-[u,u^2,B,0],Inf));
            % Reflection changes the sign of r. Rotations put the rate column along x.
            d=f(3:4).';dNorm=norm(d);sgn=1;if r<0,sgn=-1;end
            angle=atan2(d(2),d(1));Q=[cos(angle),sin(angle);-sgn*sin(angle),sgn*cos(angle)]*sgn;
            S=blkdiag(kron(Q,eye(3)),sgn*eye(2));physical=localizationSupport.fusionMatrices(c,f);
            aligned=localizationSupport.fusionMatrices(c,[u,u^2,dNorm,0]);
            error=max([error,norm(aligned-S*physical*S.'),max(0,dNorm-B)]);
        end
    end
end

function error=affineError()
    c=mncavFullObserverConfig();vertices=localizationSupport.fusionVertices(c);
    w=[.11;.23;.29;.37];P=diag([1:3,1:3,7,8]);combined=zeros(8);
    for j=1:numel(w)
        A=localizationSupport.fusionMatrices(c,vertices(j,:));M=P*A;
        combined=combined+w(j)*(M+M.');
    end
    A=localizationSupport.fusionMatrices(c,w.'*vertices);M=P*A;error=norm(combined-M-M.','fro');
end

function [v,a,r]=sampleMotion(c,s0,stream)
    phi=2*pi*rand(stream);alpha=2*pi*rand(stream);
    v=(s0+(c.iss.maximumSpeed-s0)*rand(stream))*[cos(phi);sin(phi)];
    a=c.iss.maximumAcceleration*rand(stream)*[cos(alpha);sin(alpha)];
    r=c.maximumBodyYawRate*(2*rand(stream)-1);
end
