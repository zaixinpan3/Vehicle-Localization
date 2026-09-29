classdef curbSurfaceGeometryTest < matlab.unittest.TestCase
% curbSurfaceGeometryTest Continuous curb-face localization on the shared lattice.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function profileIsRestrictedToCoarseMississippi(testCase)
            coarse=perceptionConfig("Mississippi");offline=perceptionConfig("Mississippi","offline");downtown=perceptionConfig("Downtown");
            testCase.verifyTrue(coarse.curbBoundary.enabled);
            testCase.verifyFalse(offline.curbBoundary.enabled);
            testCase.verifyFalse(downtown.curbBoundary.enabled);
        end
        function boundaryScatterPreservesPositiveJointCovariance(testCase)
            c=scatterFixture();before=c.components;cfg=curbBoundaryGeometryConfig();
            after=applyCurbBoundaryScatter(c,cfg);a=after.components;
            testCase.verifyEqual(a.mean,before.mean);
            testCase.verifyEqual(a.covariance(1,1,1),before.covariance(1,1,1),AbsTol=1e-12);
            testCase.verifyEqual(a.covariance(2,2,1),cfg.minimumNormalVariance,AbsTol=1e-12);
            testCase.verifyGreaterThan(min(eig(a.covarianceXYZ(:,:,1))),0);
            testCase.verifyEqual(a.covarianceXYZ(1:2,1:2,:),a.covariance,AbsTol=1e-12);
            testCase.verifyEqual(a.invCovariance(:,:,1)*a.covariance(:,:,1),eye(2),AbsTol=1e-12);
            testCase.verifyEqual(a.covarianceXYZ(:,:,end),before.covarianceXYZ(:,:,end),AbsTol=0);
            testCase.verifyFalse(after.curbGeometryCovarianceCalibrated);
        end
        function unequalRoadAndShoulderSlopesLocateTheFace(testCase)
            [g,c,ids]=fixture(.16,.6);
            [m,d]=estimateCurbBoundaryGeometry(g,c,eye(3),[0 0 0]);
            testCase.verifyTrue(all(d.accepted));
            testCase.verifyEqual(m.mean(ids,2),.18*ones(numel(ids),1),AbsTol=.035);
            testCase.verifyEqual(m.count,g.moments.count);
            testCase.verifyEqual(m.covariance,g.moments.covariance,AbsTol=0);
        end
        function slopedPlaneDoesNotCreateAnEdge(testCase)
            [g,c]=fixture(0,0);
            [m,d]=estimateCurbBoundaryGeometry(g,c,eye(3),[0 0 0]);
            testCase.verifyFalse(any(d.accepted));
            testCase.verifyEqual(m.mean,g.moments.mean,AbsTol=0);
        end
        function edgeOutsideSelectedPillarDoesNotPullItsCenter(testCase)
            [g,c,ids]=fixture(.16,.6);
            g.curbCellMask(:)=false;g.curbCellMask(ids+1)=true;
            [m,d]=estimateCurbBoundaryGeometry(g,c,eye(3),[0 0 0]);
            testCase.verifyFalse(any(d.accepted));
            testCase.verifyEqual(m.mean,g.moments.mean,AbsTol=0);
        end
        function rigidProjectionPreservesTheRecoveredEdge(testCase)
            [g,c,ids]=fixture(.16,.6);angle=.7;
            R=[cos(angle) -sin(angle) 0;sin(angle) cos(angle) 0;0 0 1];t=[1 -2 .4];
            xyz=[g.moments.mean,g.moments.meanZ]*R.'+t;
            g.moments.mean=xyz(:,1:2);g.moments.meanZ=xyz(:,3);
            [m,d]=estimateCurbBoundaryGeometry(g,c,R,t);
            recovered=([m.mean,m.meanZ]-t)*R;
            testCase.verifyTrue(all(d.accepted));
            testCase.verifyEqual(recovered(ids,2),.18*ones(numel(ids),1),AbsTol=.035);
        end
        function insufficientLineSupportLeavesMomentsAlone(testCase)
            [g,c,ids]=fixture(.16,.6);g.curbCellMask(:)=false;g.curbCellMask(ids(1:2))=true;
            [m,d]=estimateCurbBoundaryGeometry(g,c,eye(3),[0 0 0]);
            testCase.verifyEmpty(d);
            testCase.verifyEqual(m.mean,g.moments.mean,AbsTol=0);
        end
    end
end

function cloud=scatterFixture()
    xy=[(-2:2).',ones(5,1);0 -3];n=size(xy,1);C=[.4 .02 .03;.02 .09 .02;.03 .02 .2];
    c=struct('mean',xy,'numComponents',n,'semanticName',[repmat("curb",5,1);"pole"], ...
        'mixtureWeight',ones(n,1)/n,'covariance',repmat(C(1:2,1:2),1,1,n), ...
        'covarianceXYZ',repmat(C,1,1,n),'invCovariance',repmat(inv(C(1:2,1:2)),1,1,n), ...
        'determinant',repmat(det(C(1:2,1:2)),n,1),'logNormalizationConstant',zeros(n,1));
    cloud=struct('components',c);
end

function [ground,context,ids]=fixture(height,slopeChange)
    [x,y]=ndgrid(-2.2:.08:2.2,-.95:.025:1.5);z=-2+.02*x-.10*y+height*(y>=.18)+slopeChange*max(y-.18,0);
    [fx,fz]=ndgrid(-2.2:.08:2.2,linspace(0,height,9));fy=.18*ones(size(fx));fz=fz-2+.02*fx-.1*fy;
    xyz=[x(:),y(:),z(:);fx(:),fy(:),fz(:)];origin=[-2.4 -1.2];spacing=[.6 .6];dims=[5 8];
    bins=floor((xyz(:,1:2)-origin)./spacing)+1;valid=all(bins>=1,2)&bins(:,1)<=dims(2)&bins(:,2)<=dims(1);xyz=xyz(valid,:);bins=bins(valid,:);
    cells=sub2ind(dims,bins(:,2),bins(:,1));moments=aggregatePlanarCellMoments(xyz,cells,prod(dims));ids=sub2ind(dims,3*ones(4,1),(3:6).');mask=false(dims);mask(ids)=true;
    ground=struct('moments',moments,'curbCellMask',mask,'cellMapSize',dims,'cellOrigin',origin,'cellSize',spacing);context=struct('groundPoints',xyz);
end
