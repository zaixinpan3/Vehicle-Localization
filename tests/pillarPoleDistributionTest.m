classdef pillarPoleDistributionTest < matlab.unittest.TestCase
% pillarPoleDistributionTest: Joint statistics, inference and physical controls.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function jointMomentsDistinguishEqualCovarianceShapes(t)
            z=[-1;-.5;.5;1];x=[.42;.18;.18;.42];a=[x,.3*ones(4,1),z];b=[.6-x,a(:,2:3)];
            t.verifyEqual(mean(a),mean(b),'AbsTol',1e-14);
            t.verifyEqual(cov(a),cov(b),'AbsTol',1e-14);
            h=t.axis();u=measurePillarPoleMoments(a,[0 0],[.6 .6],h);v=measurePillarPoleMoments(b,[0 0],[.6 .6],h);
            t.verifyGreaterThan(u.moment102,.1);t.verifyLessThan(v.moment102,-.1);
        end
        function momentStatisticsIgnoreInputOrderAndCoordinateOrigin(t)
            z=linspace(-1,1,31).';p=[.3+.04*cos(z*20),.3+.04*sin(z*20),z];h=t.axis();
            a=measurePillarPoleMoments(p,[0 0],[.6 .6],h);shift=[14 -8 .173];
            h.axisXY=h.axisXY+shift(1:2);h.axisZ=h.axisZ+shift(3);h.minimumZ=h.minimumZ+shift(3);h.maximumZ=h.maximumZ+shift(3);
            b=measurePillarPoleMoments(flipud(p)+shift,shift(1:2),[.6 .6],h);
            t.verifyEqual(struct2array(a),struct2array(b),'AbsTol',1e-12);
        end
        function treeThresholdsAndMissingValuesHaveExplicitSemantics(t)
            model=t.toyModel();x=[.5 0;.6 1;NaN Inf];expected=1./(1+exp(-[-1;1.25;-1]));
            t.verifyEqual(scorePillarPoleModel(x,model),expected,'AbsTol',1e-15);
            t.verifyEqual(scorePillarPoleModel(x(2,:),model),expected(2),'AbsTol',1e-15);
            t.verifySize(scorePillarPoleModel(zeros(0,2),model),[0 1]);
        end
        function roundoffCannotCreateDifferentTreePaths(t)
            model=t.toyModel();model.featureScale=1e8;x=[.5 0;.5 4e-12;.5 -4e-12];
            t.verifyEqual(scorePillarPoleModel(x,model),repmat(scorePillarPoleModel(x(1,:),model),3,1),'AbsTol',0);
            model.threshold(1)=.40816327;
            ratio=[.40816326530612201 0;.40816326530612246 0];p=scorePillarPoleModel(ratio,model);
            t.verifyEqual(p(1),p(2),'AbsTol',0);
        end
        function farMinorityShaftSurvivesOtherHeightClutter(t)
            z=linspace(-1,3,60).';a=(1:60).'*2.399;
            shaft=[10.12+.025*cos(a),2.16+.025*sin(a),z];rng(14);
            clutter=[10.04+.52*rand(800,1),1.9+.52*rand(800,1),3.6+.5*rand(800,1)];
            r=t.detect([shaft;clutter]);t.verifyTrue(any(r.poleCellMask,'all'));
            t.verifyEqual(sum(r.columnMaps.statistics.count),860);
        end
        function extendedWallIsNotACollectionOfPoles(t)
            [x,z]=meshgrid(8:.04:12,-1:.08:3);p=[x(:),ones(numel(x),1)*2,z(:)];
            r=t.detect(p);t.verifyFalse(any(r.poleCellMask,'all'));
        end
        function separateBlobsCannotSupplyAContinuousShaft(t)
            z=[(0:.05:.5)';(3:.05:3.5)'];p=[10.3+.01*cos(z*20),2.3+.01*sin(z*20),z];
            r=t.detect(p);t.verifyFalse(any(r.poleCellMask,'all'));
        end
        function widenedNearRoiUsesGeometricBoundarySupport(t)
            z=repelem(linspace(-1,3,12).',2);p=[repmat([1.29;1.31],12,1),ones(size(z))*1.05,z];
            r=t.detect(p);t.verifyEqual(nnz(r.poleCellMask),2);
        end
        function offlineConfigurationDoesNotLoadTheDistributionModel(t)
            cfg=perceptionConfig('Mississippi','offline');t.verifyEqual(cfg.offGroundFeatures.pole.detector,"pillar");
            t.verifyFalse(isfield(cfg.offGroundFeatures.pole,'distributionValidation'));
        end
        function defaultUsesOnlyDistributionInputs(t)
            cfg=perceptionConfig();v=cfg.offGroundFeatures.pole.distributionValidation;t.verifyTrue(v.enabled);
            model=loadPillarPoleModel(v.modelFile);names=string(model.featureNames);
            t.verifyFalse(any(ismember(names,["dataset","frame","pillar","hypothesis","finePointCount"])));
            t.verifyTrue(any(startsWith(names,"moment")));
            t.verifyEqual(v.minimumScore,model.decisionThreshold,'AbsTol',0);t.verifyEqual(model.featureScale,1e8);
        end
        function emptyMarginCroppingPreservesPoleContext(t)
            root=fileparts(fileparts(mfilename('fullpath')));file=fullfile(root,'data','raw','downTownPointClouds.mat');
            t.assumeTrue(isfile(file));frame=loadPointCloudFrame(file,375);cfg=perceptionConfig('Downtown');
            cfg.voxel.useNativeKernels=perceptionNativeAvailable;cfg.groundSegmentation.useNativeKernels=perceptionNativeAvailable;
            cfg.offGroundFeatures.useNativeKernels=perceptionNativeAvailable;g=pillarizePointCloud(frame,cfg.voxel);
            ground=segmentGround(g,cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
            p=struct('points',g.points(keep,:),'pointPillarLinIdx',g.pointPillarLinIdx(keep), ...
                'pillarGeometry',g.pillarGeometry,'pointAttributes',struct('intensity',g.pointAttributes.intensity(keep)));
            cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames="pole";
            full=analyzeStructuralPillars(p,cfg.offGroundFeatures,cloud);cropped=perceiveFrame(frame,cfg);
            expected=find(full.poleCellMask);actual=double(cropped.candidates.pillarIndices{cropped.candidates.semanticNames=="pole"});
            t.verifyEqual(actual(:),expected(:));
        end
    end
    methods (Static,Access=private)
        function h=axis()
            h=struct('axisXY',[.3 .3],'axisZ',0,'slopeXY',[0 0],'minimumZ',-1,'maximumZ',1);
        end
        function model=toyModel()
            model=struct('featureNames',{{'a','b'}},'initialLogit',-.25,'roots',[1;4], ...
                'feature',[1;1;1;2;1;1],'threshold',[.5;0;0;0;0;0], ...
                'left',[2;1;1;5;1;1],'right',[3;1;1;6;1;1], ...
                'leaf',logical([0;1;1;0;1;1]),'value',[0;-1;2;0;.25;-.5]);
        end
        function r=detect(p)
            cfg=perceptionConfig();cfg.voxel.exclusionHalfSize=0;g=pillarizePointCloud(p,cfg.voxel);
            cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames="pole";
            r=analyzeStructuralPillars(g,cfg.offGroundFeatures,cloud);
        end
    end
end
