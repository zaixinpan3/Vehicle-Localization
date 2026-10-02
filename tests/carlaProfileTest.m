classdef carlaProfileTest < matlab.unittest.TestCase
% carlaProfileTest: The CARLA perception profile and its support changes.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function profileSelectsChannelsCalibrationAndTunedValues(testCase)
            for mode=["coarseProbabilityCloud","offline"]
                cfg=perceptionConfig("Carla",mode);
                testCase.verifyEqual(cfg.featureNames,["curb","pole","facade"]);
                testCase.verifyFalse(cfg.semanticPrecision.enabled);
                testCase.verifyEqual(cfg.frameCalibration,lidarFrameCalibrationConfig("carla"));
                testCase.verifyEqual(cfg.voxel.executionMode,mode);
            end
            coarse=perceptionConfig("Carla");
            testCase.verifyEqual(coarse.groundFeatures.curb.minPointsPerCell,1);
            testCase.verifyEqual(coarse.offGroundFeatures.pole.distributionValidation.minimumScore,0.70);
            testCase.verifyEqual(coarse.offGroundFeatures.pole.distributionValidation.modelFile, ...
                pillarPoleDistributionConfig("mississippi").modelFile);
            testCase.verifyEqual(coarse.offGroundFeatures.facadeContinuousSupportMinimumSpan,3.0);
            testCase.verifyEqual(coarse.offGroundFeatures.facadeMaximumBaseHeight,0.6);
            testCase.verifyTrue(coarse.curbBoundary.enabled);
            offline=perceptionConfig("Carla","offline");
            testCase.verifyEqual(offline.fine.facadeMinimumHeight,2.48);
            testCase.verifyFalse(offline.fine.poleStructureRejectionEnabled);
            testCase.verifyEqual(offline.fine.poleCanopyMaximumSectors,5);
            testCase.verifyEqual(offline.fine.poleMaximumAdjacentReturns,20);
            testCase.verifyEqual(offline.groundFeatures,perceptionConfig("Mississippi","offline").groundFeatures);
        end
        function existingProfilesAreUnchangedByTheCarlaProfile(testCase)
            for dataset=["Mississippi","Downtown"]
                for mode=["coarseProbabilityCloud","offline"]
                    cfg=perceptionConfig(dataset,mode);
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeContinuousSupportMinimumSpan'));
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeContinuousSupportMinimumPoints'));
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeMaximumBaseHeight'));
                    testCase.verifyFalse(isfield(cfg.fine,'poleCanopyMaximumSectors'));
                end
            end
            testCase.verifyTrue(perceptionConfig("Mississippi").semanticPrecision.enabled);
            testCase.verifyEqual(perceptionConfig("Mississippi").featureNames,["curb","pole","trafficSign"]);
            testCase.verifyEqual(perceptionConfig("Mississippi").groundFeatures.curb.minPointsPerCell,3);
        end
        function carlaCalibrationIsTheRoofLidarAtTheReferencePoint(testCase)
            c=lidarFrameCalibrationConfig("carla");
            testCase.verifyEqual(c.rotation,eye(3));
            testCase.verifyEqual(c.translation,[1.38776 0 2],'AbsTol',1e-12);
            testCase.verifyEqual(c.identifier,"carla-town10hd-lincoln-mkz-roof-20261001-v1");
        end
        function facadeGroundContactGateRemovesElevatedSupport(testCase)
            % A wall reaching the ground and a canopy 3 m above it, both
            % with a continuous 3.5 m vertical run, in a flat synthetic scan.
            [gx,gy]=meshgrid(-29:0.3:29,-29:0.3:29);
            ground=[gx(:),gy(:),-2*ones(numel(gx),1)];
            [wx,wz]=meshgrid(-6:0.1:6,-1.9:0.1:1.6);
            wall=[wx(:),12*ones(numel(wx),1),wz(:)];
            canopy=[wx(:),-12*ones(numel(wx),1),wz(:)+3];
            xyz=[ground;wall;canopy];
            frame=struct('x',single(xyz(:,1)),'y',single(xyz(:,2)),'z',single(xyz(:,3)));
            cfg=perceptionConfig("Carla");cfg.featureNames="facade";
            gated=perceiveFrame(frame,cfg);
            cfg.offGroundFeatures=rmfield(cfg.offGroundFeatures,'facadeMaximumBaseHeight');
            open=perceiveFrame(frame,cfg);
            centres=@(r) facadeCentres(r);
            testCase.verifyTrue(any(abs(centres(gated)-12)<1));
            testCase.verifyFalse(any(abs(centres(gated)+12)<1));
            testCase.verifyTrue(any(abs(centres(open)+12)<1));
        end
        function canopyGateRejectsTrunkAndKeepsPole(testCase)
            % Ray-cast ground, a 7 m thin pole and a trunk below a porous
            % canopy; the offline pole gate must keep the pole only.
            [frame,xyz]=poleAndTreeScene();
            cfg=perceptionConfig("Carla","offline");cfg.featureNames="pole";
            open=perceiveFrame(frame,cfg).featureMasks.pole;
            cfg.fine.poleStructureRejectionEnabled=true;
            gated=perceiveFrame(frame,cfg).featureMasks.pole;
            cfg.fine=rmfield(cfg.fine,'poleStructureRejectionEnabled');
            legacy=perceiveFrame(frame,cfg).featureMasks.pole;
            testCase.verifyEqual(open,legacy);
            nearPole=hypot(xyz(:,1)-8,xyz(:,2)-4)<0.5;nearTrunk=hypot(xyz(:,1)-8,xyz(:,2)+4)<0.5;
            testCase.verifyGreaterThan(nnz(gated&nearPole),50);
            testCase.verifyEqual(nnz(gated&nearPole),nnz(open&nearPole));
            testCase.verifyGreaterThan(nnz(open&nearTrunk),50);
            testCase.verifyEqual(nnz(gated&nearTrunk),0);
        end
        function continuousSupportSpanDefaultsToOneMetre(testCase)
            % Two pillars: a 1.4 m and a 2.4 m dense vertical run.
            z=[(0:0.1:1.4).';(0:0.1:2.4).'];
            ids=[ones(15,1);2*ones(25,1)];
            pillars=struct('pointPillarLinIdx',int32(ids),'points',[zeros(numel(z),2),z], ...
                'pointAttributes',struct());
            three=continuousPillarHeightSupport(pillars,[2 1],4);
            four=continuousPillarHeightSupport(pillars,[2 1],4,1);
            tall=continuousPillarHeightSupport(pillars,[2 1],4,2);
            testCase.verifyEqual(three,four);
            testCase.verifyEqual(double(three(:).'),[1.4 2.4]/0.5,'AbsTol',1e-6);
            testCase.verifyEqual(double(tall(:).'),[0 2.4/0.5],'AbsTol',1e-6);
        end
    end
end

function y=facadeCentres(result)
% Y of the centres of the selected facade pillars.
    g=result.candidates.geometry;ids=double(result.candidates.pillarIndices{result.candidates.semanticNames=="facade"});
    [r,~]=ind2sub(double(g.mapSize),ids);y=double(g.origin(2))+(r-.5)*double(g.cellSize(2));
end

function [frame,p]=poleAndTreeScene()
% Sensor at the origin, ground at z=-2 m, a 0.09 m pole at (8,4) up to
% z=5 m, a 0.15 m trunk at (8,-4) up to z=1.2 m and 600 canopy returns in a
% 2 m sphere centred at z=3.6 m, hollow within 0.9 m of the trunk axis.
    stream=RandStream('mt19937ar','Seed',1);
    el=linspace(-17.7,14.2,64)*pi/180;az=(0:1023)/1024*2*pi;[A,E]=meshgrid(az,el);
    d=[cos(E(:)).*cos(A(:)),cos(E(:)).*sin(A(:)),sin(E(:))];
    r=(-2)./d(:,3);r(d(:,3)>=0)=inf;
    r=min(r,cylinderHit(d,[8 4],0.09,-2,5));r=min(r,cylinderHit(d,[8 -4],0.15,-2,1.2));
    keep=isfinite(r)&r<60;p=d(keep,:).*r(keep);p=p+0.01*randn(stream,size(p));
    u=randn(stream,4000,3);u=u./vecnorm(u,2,2).*(2*rand(stream,4000,1).^(1/3));
    u=u(hypot(u(:,1),u(:,2))>0.9,:);p=[p;u(1:600,:)+[8 -4 3.6]];
    frame=struct('x',single(p(:,1)),'y',single(p(:,2)),'z',single(p(:,3)));
end

function t=cylinderHit(d,c,radius,z0,z1)
    a=d(:,1).^2+d(:,2).^2;b=-2*(d(:,1)*c(1)+d(:,2)*c(2));cc=c(1)^2+c(2)^2-radius^2;
    disc=b.^2-4*a.*cc;t=(-b-sqrt(max(disc,0)))./(2*a);t(disc<0|t<=0)=inf;
    z=d(:,3).*t;t(z<z0|z>z1)=inf;
end
