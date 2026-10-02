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
            testCase.verifyEqual(offline.groundFeatures,perceptionConfig("Mississippi","offline").groundFeatures);
        end
        function existingProfilesAreUnchangedByTheCarlaProfile(testCase)
            for dataset=["Mississippi","Downtown"]
                for mode=["coarseProbabilityCloud","offline"]
                    cfg=perceptionConfig(dataset,mode);
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeContinuousSupportMinimumSpan'));
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeContinuousSupportMinimumPoints'));
                    testCase.verifyFalse(isfield(cfg.offGroundFeatures,'facadeMaximumBaseHeight'));
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
