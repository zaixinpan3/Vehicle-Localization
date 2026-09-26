classdef pillarShaftExecutionTest < matlab.unittest.TestCase
% pillarShaftExecutionTest: Preserve shaft evidence across execution strategies.
    properties (TestParameter)
        nativeThreads=struct('serial',1,'twoWorkers',2,'fourWorkers',4);
        scene=struct('repeatedHeights','repeatedHeights', ...
            'radiusBoundaries','radiusBoundaries','tiedHypotheses','tiedHypotheses', ...
            'neighborOwners','neighborOwners','unsortedRadii','unsortedRadii', ...
            'permissiveThreshold','permissiveThreshold','tightThreshold','tightThreshold', ...
            'empty','empty','sparse','sparse');
        invalidThreads=struct('zero',0,'negative',-1,'fractional',1.5, ...
            'notFinite',NaN,'tooMany',65);
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function nativePreservesMatlabEvidence(testCase,nativeThreads,scene)
            testCase.assumeTrue(perceptionNativeAvailable);
            [points,ids,geometry,cfg]=pillarShaftExecutionTest.makeScene(scene);
            cfg.useNativeKernels=false;
            reference=findPillarShaftModes(points,ids,geometry,cfg);
            [referenceAssigned,referenceGroups]=assignPillarShaftSupport( ...
                points,ids,geometry,reference,cfg);
            cfg.useNativeKernels=true;cfg.nativeThreads=nativeThreads;
            actual=findPillarShaftModes(points,ids,geometry,cfg);
            [actualAssigned,actualGroups]=assignPillarShaftSupport( ...
                points,ids,geometry,actual,cfg);
            testCase.verifyEqual(actual,reference,'AbsTol',1e-9);
            testCase.verifyEqual(actualAssigned,referenceAssigned,'AbsTol',1e-9);
            testCase.verifyEqual(actualGroups,referenceGroups);
        end
        function manyOwnersAreInvariantToWorkerCount(testCase,nativeThreads)
            testCase.assumeTrue(perceptionNativeAvailable);
            [points,ids,geometry,cfg]=pillarShaftExecutionTest.manyOwners();
            cfg.useNativeKernels=true;cfg.nativeThreads=1;
            reference=findPillarShaftModes(points,ids,geometry,cfg);
            cfg.nativeThreads=nativeThreads;
            actual=findPillarShaftModes(points,ids,geometry,cfg);
            testCase.verifyEqual(numel(actual.pillarIndices),64);
            testCase.verifyGreaterThanOrEqual(nnz(reference.found),32);
            testCase.verifyEqual(actual,reference,'AbsTol',0);
        end
        function invalidWorkerCountsFailBeforeExecution(testCase,invalidThreads)
            testCase.assumeTrue(perceptionNativeAvailable);
            [points,ids,geometry,cfg]=pillarShaftExecutionTest.makeScene('repeatedHeights');
            cfg.useNativeKernels=true;cfg.nativeThreads=invalidThreads;
            testCase.verifyError(@() findPillarShaftModes(points,ids,geometry,cfg), ...
                'perception:native:InvalidInput');
        end
    end
    methods (Static,Access=private)
        function [points,ids,geometry,cfg]=makeScene(scene)
            cfg=pillarShaftConfig();mapSize=[1 1];
            points=pillarShaftExecutionTest.shaft([.3 .3]);
            switch scene
                case 'repeatedHeights'
                    points(:,3)=repelem(linspace(-1,3,18).',2);
                case 'radiusBoundaries'
                    % Cardinal offsets use exactly representable radii.
                    center=[.25 .25];radius=.0625;
                    points=pillarShaftExecutionTest.shaft(center);
                    offsets=[radius 0;-radius 0;0 radius;0 -radius];
                    ring=[repmat(center,4,1)+offsets,zeros(4,1)];
                    inner=ring;inner(:,1:2)=ring(:,1:2)-4*eps(center).*sign(offsets);
                    outer=ring;outer(:,1:2)=ring(:,1:2)+4*eps(center).*sign(offsets);
                    points=[points;ring;inner;outer];
                    cfg.radii=[radius .125 .15];
                case 'tiedHypotheses'
                    % Repeated radii and fitting scales give identical modes.
                    cfg.radii=[.10 .10 .06 .15];cfg.fitRadii=[.10 .10 .20];
                case 'neighborOwners'
                    points=pillarShaftExecutionTest.shaft([.6 .3]);
                    points=[points;.59 .3 -1.1;.59 .3 3.1;.61 .3 -1.1;.61 .3 3.1];
                    mapSize=[1 2];
                case 'unsortedRadii'
                    cfg.radii=[.15 .06 .10];cfg.fitRadii=[.20 .10];
                case 'permissiveThreshold'
                    cfg.columnMinimumScores=0;
                case 'tightThreshold'
                    cfg.columnMinimumScores=1;
                case 'empty'
                    points=zeros(0,3);
                case 'sparse'
                    points=points([1 18 36],:);
            end
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',mapSize);
            bins=floor(points(:,1:2)./.6)+1;
            ids=sub2ind(mapSize,bins(:,2),bins(:,1));
        end
        function [points,ids,geometry,cfg]=manyOwners()
            cfg=pillarShaftConfig();geometry=struct( ...
                'origin',[0 0],'cellSize',[.6 .6],'mapSize',[16 16]);
            [x,y]=meshgrid(.3+1.2*(0:7),.3+1.2*(0:7));
            pieces=arrayfun(@(a,b)pillarShaftExecutionTest.shaft([a b]), ...
                x(:),y(:),'UniformOutput',false);
            points=vertcat(pieces{:});bins=floor(points(:,1:2)./.6)+1;
            ids=sub2ind(geometry.mapSize,bins(:,2),bins(:,1));
        end
        function points=shaft(center)
            z=linspace(-1,3,36).';angle=(1:36).'*2.399;
            points=[center(1)+.025*cos(angle),center(2)+.025*sin(angle),z];
        end
    end
end
