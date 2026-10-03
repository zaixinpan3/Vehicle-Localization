classdef downtownRadiometryTest < matlab.unittest.TestCase
% downtownRadiometryTest: Conditional moments and pooled distribution context.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function conditionalMomentsPreserveAllPointGaussianGeometry(testCase)
            [pillars,cfg,cloud,ground]=scene();
            result=analyzeDowntownPillarStatistics(pillars,cfg,cloud,ground);
            maps=result.columnMaps;stats=maps.statistics;r=maps.radiometry;
            testCase.verifyEqual(r.reflective.count+r.nonreflective.count,stats.count);
            merged=(r.reflective.meanXYZ.*r.reflective.count+ ...
                r.nonreflective.meanXYZ.*r.nonreflective.count)./stats.count;
            testCase.verifyEqual(merged,stats.meanXYZ,'AbsTol',1e-12);
            for k=1:numel(stats.pillarIndices)
                points=pillars.points(pillars.pointPillarLinIdx==stats.pillarIndices(k),:);
                expected=cov(points,1);packed=expected([1 2 5 3 6 9]);
                testCase.verifyEqual(stats.covarianceXYZ(k,:),packed,'AbsTol',1e-12);
                testCase.verifyEqual(maps.moments.count(stats.pillarIndices(k)),size(points,1));
            end
            testCase.verifyFalse(isfield(r,'pointIndices'));
            testCase.verifyFalse(isfield(r.reflective,'pointIndices'));
        end
        function summariesAndFeaturesAreInvariantToReturnOrder(testCase)
            [pillars,cfg,cloud,ground]=scene();
            original=analyzeDowntownPillarStatistics(pillars,cfg,cloud,ground);
            order=size(pillars.points,1):-1:1;permuted=pillars;
            permuted.points=pillars.points(order,:);
            permuted.pointPillarLinIdx=pillars.pointPillarLinIdx(order);
            for name=string(fieldnames(pillars.pointAttributes)).'
                permuted.pointAttributes.(name)=pillars.pointAttributes.(name)(order);
            end
            changed=analyzeDowntownPillarStatistics(permuted,cfg,cloud,ground);
            ids=find(original.columnMaps.occupiedMask);
            [a,an]=measureDowntownDistributionContext(original.columnMaps,ids);
            [b,bn]=measureDowntownDistributionContext(changed.columnMaps,ids);
            testCase.verifyEqual(an,bn);
            testCase.verifyEqual(a,b,'AbsTol',1e-8);
            testCase.verifyTrue(all(isfinite(a),'all'));
        end
        function groundClearanceUsesMappedCellMoments(testCase)
            [pillars,cfg,cloud,ground]=scene();
            result=analyzeDowntownPillarStatistics(pillars,cfg,cloud,ground);
            maps=result.columnMaps;r=maps.radiometry;ids=find(maps.occupiedMask);
            [values,names]=measureDowntownDistributionContext(maps,ids);
            testCase.verifyEqual(r.ground.r1_meanZ,zeros(size(ids)),'AbsTol',1e-12);
            testCase.verifyGreaterThan(r.ground.r1_count,zeros(size(ids)));
            field=names=="reflective_ground_r1_minimumClearance";
            bright=r.reflective.count>0;
            testCase.verifyEqual(values(bright,field),r.reflective.minimumXYZ(bright,3),'AbsTol',1e-8);
        end
        function pooledCovarianceMatchesAnExplicitWholePillarUnion(testCase)
            [pillars,cfg,cloud,ground]=scene();
            result=analyzeDowntownPillarStatistics(pillars,cfg,cloud,ground);
            maps=result.columnMaps;ids=find(maps.occupiedMask);
            [values,names]=measureDowntownDistributionContext(maps,ids);
            [row,col]=ind2sub(maps.mapSize,ids);
            [pr,pc]=ind2sub(maps.mapSize,double(pillars.pointPillarLinIdx));
            for k=1:numel(ids)
                included=abs(pr-row(k))<=1 & abs(pc-col(k))<=1;
                expected=cov(pillars.points(included,:),1);
                fields=["varXX","covXY","varYY","covXZ","covYZ","varZZ"];
                [found,at]=ismember("whole_r1_"+fields,names);testCase.verifyTrue(all(found));
                testCase.verifyEqual(values(k,at),expected([1 2 5 3 6 9]),'AbsTol',1e-8);
            end
        end
    end
end

function [pillars,cfg,cloud,ground]=scene()
    gridCfg=pillarGridConfig();gridCfg.exclusionHalfSize=0;
    xyz=[10 5 .8;10.03 5.02 1.6;10.08 5.04 2.4; ...
        10.7 5.02 1;10.73 5.04 2;10.78 5.08 3; ...
        10.03 5.7 .7;10.09 5.73 1.4;10.12 5.78 2.1];
    frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3), ...
        'intensity',[100;1601;2000;100;1700;1800;NaN;200;300]);
    pillars=pillarizePointCloud(frame,gridCfg);cfg=structuralPillarConfig(.6);
    cfg.trafficSignIntensityThreshold=1600;cloud=coarseSemanticProbabilityCloudConfig(gridCfg);
    [x,y]=ndgrid(8.2:.3:12.4,3.4:.3:8.2);g=[x(:),y(:),zeros(numel(x),1)];
    gp=pillarizePointCloud(g,gridCfg);bins=double(gp.pointPillarSub);
    ground=struct('groundPoints',gp.points, ...
        'groundCellLinIdx',sub2ind(gridCfg.gridDims,bins(:,1),bins(:,2)), ...
        'groundXYView',struct('origin',gp.pillarGeometry.origin, ...
        'cellSize',gp.pillarGeometry.cellSize,'gridSize',gridCfg.gridDims));
end
