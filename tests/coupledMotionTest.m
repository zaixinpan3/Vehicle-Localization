classdef coupledMotionTest < matlab.unittest.TestCase
% Independent Jacobian, domain-coverage and continuous-enclosure checks.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function exactPolynomialMatchesTheOriginalObserverJacobians(testCase)
            errors=jacobianErrors();
            testCase.verifyLessThan(max(errors),2e-12);
        end
        function tensorControlsReconstructEveryInteriorPolynomialValue(testCase)
            [error,minimumWeight,weightError]=reconstructionErrors();
            testCase.verifyLessThan(error,2e-11);
            testCase.verifyGreaterThanOrEqual(minimumWeight,0);
            testCase.verifyLessThan(weightError,1e-14);
        end
        function outerBoxesCoverThePhysicalCoupledMotionDomain(testCase)
            [uncovered,relativeWidthSpread]=coverageErrors();
            testCase.verifyEqual(uncovered,0);
            testCase.verifyLessThan(relativeWidthSpread,1e-12);
        end
        function activeCutConstructionAgreesWithFullVerification(testCase)
            error=selectedErrors();
            testCase.verifyLessThan(error,1e-12);
        end
        function subdivisionCannotIncreaseThePolynomialMatrixUpperBound(testCase)
            difference=subdivisionDifference();
            testCase.verifyLessThanOrEqual(difference,1e-10);
        end
        function oldPairRelaxationIsAbsentFromCurrentInputs(testCase)
            c=mncavFullObserverConfig();p=sensorRedundancySupport.inputs(c,1);
            testCase.verifyFalse(isfield(c.invariantObserver.redundancy,'bilinearSplit'));
            testCase.verifyEqual(string(p.motionEnclosure),"joint-polynomial-gram-v1");
        end
    end
end

function errors=jacobianErrors()
    stream=RandStream('mt19937ar','Seed',20261028);errors=zeros(1,4);
    for cfg={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()}
        c=cfg{1};t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;
        B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);
        D=invariantObserverSupport.outputNormalization(c);p=sensorRedundancySupport.inputs(c,1);
        for j=1:120
            s=c.invariantObserver.redundancy.minimumCourseSpeed+rand(stream)*(c.iss.maximumSpeed-c.invariantObserver.redundancy.minimumCourseSpeed);
            a=c.iss.maximumAcceleration*sqrt(rand(stream))*[cos(j);sin(j)];q=a(2)/s;
            x=[0;s;a(1);0;0;a(2);0;q];
            [F,H]=invariantIssSupport.secants(x,x,0,0,c);H=D*H*B;
            [A,speed,pair,direction]=coupledMotionSupport.matrices(p,coupledMotionSupport.features(s,q,a(1)));
            expected=B\F*B/t;expected(8,8)=-1;
            errors=max(errors,[norm(A-expected,'fro'),norm(speed-H(1,:).'*H(1,:),'fro'), ...
                norm(pair-H(2:3,:).'*H(2:3,:),'fro'),norm(direction-H(4,:).'*H(4,:),'fro')]);
        end
    end
end

function [error,minimumWeight,weightError]=reconstructionErrors()
    c=mncavSmallOffsetObserverConfig();p=sensorRedundancySupport.inputs(c,1);
    [~,boxes]=coupledMotionSupport.controls(p,c.invariantObserver.redundancy);
    stream=RandStream('mt19937ar','Seed',2801);error=0;minimumWeight=1;weightError=0;
    for j=1:size(boxes,1)
        box=boxes(j,:);u=rand(stream,1,3);x=box([1,3,5])+u.*(box([2,4,6])-box([1,3,5]));
        weights=kron(kron(bernstein(u(1)),bernstein(u(2))),bernstein(u(3)));
        exact=coupledMotionSupport.features(x(1),x(2),x(3));
        error=max(error,norm(weights*coupledMotionSupport.boxControls(box)-exact,Inf));
        minimumWeight=min(minimumWeight,min(weights));weightError=max(weightError,abs(sum(weights)-1));
    end
end

function [uncovered,widthSpread]=coverageErrors()
    c=mncavSmallOffsetObserverConfig();p=sensorRedundancySupport.inputs(c,1);d=c.invariantObserver.redundancy;
    [~,boxes]=coupledMotionSupport.controls(p,d);stream=RandStream('mt19937ar','Seed',2802);uncovered=0;
    for j=1:1000
        s=d.minimumCourseSpeed+(p.maximumSpeed-d.minimumCourseSpeed)*rand(stream);
        a=p.maximumAcceleration*sqrt(rand(stream))*[cos(j);sin(j)];q=a(2)/s;
        if abs(q)>p.maximumCourseRate,continue;end
        x=[s,q,a(1)];inside=all(x>=boxes(:,[1,3,5])-1e-13 & x<=boxes(:,[2,4,6])+1e-13,2);
        uncovered=uncovered+~any(inside);
    end
    ratios=boxes(:,2)./boxes(:,1);widthSpread=max(ratios)-min(ratios);
end

function error=selectedErrors()
    c=mncavFullObserverConfig();c.invariantObserver.redundancy.speedPartitions=1;
    w=sensorRedundancySupport.witness(c,diag(1:8),2,3,[4;5;6],1);
    [L,G,M]=sensorRedundancySupport.blocks(w);all=[L,G,M];
    ids=unique(round(linspace(1,numel(all),31)));
    [L,G,M]=sensorRedundancySupport.blocks(w,true,ids);selected=[L,G,M];error=0;
    for j=1:numel(ids),error=max(error,norm(selected{j}-all{ids(j)},'fro'));end
end

function difference=subdivisionDifference()
    c=mncavSmallOffsetObserverConfig();p=sensorRedundancySupport.inputs(c,1);box=[1,3,-.4,0,-2,0];
    P=diag(1:8);parent=upperBound(p,P,box);children=-Inf;
    for i=1:3
        mid=mean(box(2*i-1:2*i));a=box;b=box;a(2*i)=mid;b(2*i-1)=mid;
        children=max(children,max(upperBound(p,P,a),upperBound(p,P,b)));
    end
    difference=children-parent;
end

function bound=upperBound(p,P,box)
    controls=coupledMotionSupport.boxControls(box);bound=-Inf;
    for j=1:27
        [A,speed,pair,direction]=coupledMotionSupport.matrices(p,controls(j,:));
        M=P*A-2*speed-3*pair-4*direction;bound=max(bound,max(eig(M+M.')));
    end
end

function b=bernstein(u)
    b=[(1-u)^2,2*u*(1-u),u^2];
end
