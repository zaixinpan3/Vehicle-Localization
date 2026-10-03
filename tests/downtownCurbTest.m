classdef downtownCurbTest < matlab.unittest.TestCase
% downtownCurbTest: Boundary support, obstacle rejection, and bounded extension.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsShortFragmentsAndRetainsMeasuredLongBoundary(testCase)
            x=[(0:.3:1.5),6,6.3].';
            points=[x,zeros(size(x)),zeros(size(x))];
            [cells,mapSize]=pointCells(points);
            cfg=downtownCurbConfig(.3);
            [keep,metrics]=filterDowntownCurbPointSupport(points,cells,mapSize, ...
                true(size(x)),cfg.pointSupport);
            testCase.verifyEqual(keep,x<2);
            testCase.verifyEqual(nnz(metrics.accepted),1);
        end
        function denseElevatedObstacleRejectsOnlyOverlappingComponent(testCase)
            x=(0:.3:2.4).';
            points=[x,zeros(size(x)),zeros(size(x));x,4*ones(size(x)),zeros(size(x))];
            [ox,oy,oz]=ndgrid(0:.15:2.4,[-.1,.1],.4:.2:2);
            allPoints=[points;ox(:),oy(:),oz(:)];
            [cells,mapSize]=pointCells(points);
            cfg=downtownCurbConfig(.3);
            [keep,~,blocked]=filterDowntownCurbObstacles(points,allPoints,cells, ...
                mapSize,true(size(points,1),1),cfg.obstacleClearance);
            testCase.verifyEqual(keep,points(:,2)>0);
            testCase.verifyEqual(blocked,points(:,2)==0);
        end
        function flatNearbyGroundDoesNotCountAsElevatedObstacle(testCase)
            [x,y]=ndgrid(0:.1:3,-.5:.1:.5);
            allPoints=[x(:),y(:),zeros(numel(x),1)];
            points=[(0:.3:3).',zeros(11,2)];
            [cells,mapSize]=pointCells(points);
            cfg=downtownCurbConfig(.3);
            [keep,~,blocked]=filterDowntownCurbObstacles(points,allPoints,cells, ...
                mapSize,true(size(points,1),1),cfg.obstacleClearance);
            testCase.verifyTrue(all(keep));
            testCase.verifyFalse(any(blocked));
        end
        function extendsMeasuredSamplesAndStopsAtUnsupportedGap(testCase)
            x=[(0:.1:9),11].';
            points=[x,zeros(size(x)),zeros(size(x))];
            points=[points;7,.3,0;8,0,.3];
            anchors=points(:,1)<=6;
            [cells,mapSize]=pointCells(points);
            maps=curbMaps(mapSize);
            cfg=downtownCurbConfig(.3);
            [keep,~,added]=extendDowntownCurbEndpoints(points,points,cells,mapSize, ...
                maps,anchors,cfg.endpointExtension,cfg.obstacleClearance);
            expected=points(:,1)<=9 & points(:,2)==0 & points(:,3)==0;
            testCase.verifyEqual(keep,expected);
            testCase.verifyEqual(added,expected & ~anchors);
        end
        function unsupportedShortAnchorCannotBootstrapAnExtension(testCase)
            points=[(0:.1:5).',zeros(51,2)];
            anchors=points(:,1)<=1;
            [cells,mapSize]=pointCells(points);
            cfg=downtownCurbConfig(.3);
            [keep,~,added]=extendDowntownCurbEndpoints(points,points,cells,mapSize, ...
                curbMaps(mapSize),anchors,cfg.endpointExtension,cfg.obstacleClearance);
            testCase.verifyEqual(keep,anchors);
            testCase.verifyFalse(any(added));
        end
        function extensionCannotLeaveOrBridgeAnExcludedCandidateInterval(testCase)
            points=[(0:.1:10).',zeros(101,2)];
            anchors=points(:,1)<=6;
            eligible=points(:,1)<=7 | points(:,1)>=8.5;
            [cells,mapSize]=pointCells(points);
            cfg=downtownCurbConfig(.3);
            cfg.endpointExtension.minRunPoints=30;
            [keep,~,added]=extendDowntownCurbEndpoints(points,points,cells,mapSize, ...
                curbMaps(mapSize),anchors,cfg.endpointExtension,cfg.obstacleClearance,eligible);
            testCase.verifyEqual(keep,points(:,1)<=7);
            testCase.verifyFalse(any(added & ~eligible));
        end
        function fineRefinementPreservesCoarseEnvelopeAndWholePillarState(testCase)
            [x,y]=ndgrid(.05:.1:2.35,[.02,.10,.18]);
            points=[x(:),y(:),zeros(numel(x),1)];
            spacing=.6; dims=[8,4]; mapSize=fliplr(dims);
            bins=floor(points(:,1:2)/spacing)+1;
            cells=sub2ind(dims,bins(:,1),bins(:,2));
            [xm,ym]=meshgrid(((1:dims(1))-.5)*spacing,((1:dims(2))-.5)*spacing);
            xy=struct('countMap',zeros(mapSize),'yMap',ym,'xMap',xm, ...
                'yCenters',ym(:,1));
            context=struct('groundPoints',points,'groundCellLinIdx',cells, ...
                'groundOriginalPointIdx',(1:size(points,1)).','groundXYView',xy);
            ground=struct('energyMaps',curbMaps(mapSize), ...
                'stats',struct('heightMap',zeros(mapSize),'detrendedHeightMap',zeros(mapSize)), ...
                'initialRoadResult',struct('roadCellMask',false(mapSize),'roadSeedMask',false(mapSize)), ...
                'curbProbabilityParameters',struct('minimumProbability',.5), ...
                'moments',struct('count',size(points,1),'sum',sum(points,1)));
            ground.curbCellMask=false(mapSize);
            ground.curbCellMask(1,2:3)=true;
            cfg=downtownCurbConfig(spacing);
            cfg.denseNearRoadFilterEnabled=false;
            cfg.boundaryRoadFacingCellMarginMeters=Inf;
            cfg.boundaryRoadReferenceFilterEnabled=false;
            cfg.pointSupport.enabled=false;
            cfg.obstacleClearance.enabled=false;
            cfg.endpointExtension.enabled=false;
            [updated,curb]=detectDowntownCurbs(context,ground,points,cfg);
            eligible=bins(:,1)>=2 & bins(:,1)<=3;
            testCase.verifyEqual(updated,ground);
            testCase.verifyEqual(curb.acceptedMask,points(:,2)==.10 & eligible);
            testCase.verifyEqual(curb.candidateMask,eligible);
            testCase.verifyEqual(nnz(curb.cellMask),2);
            testCase.verifyTrue(all(curb.candidateMask(curb.acceptedMask)));
            testCase.verifyEqual(size(updated.curbCellMask),mapSize);
        end
        function curbContextAlignsCroppedRastersAndPoolsWholePillarMoments(testCase)
            dims=[5,6];
            ground=struct('cellOrigin',[0,0],'cellSize',[.6,.6], ...
                'stats',struct('countMap',ones(dims),'heightMap',zeros(dims), ...
                'roughnessMap',zeros(dims)), 'energyMaps',curbMaps(dims), ...
                'curbCellMask',true(dims),'roadCellMask',true(dims));
            stats=struct('pillarIndices',[1;3],'count',[2;4], ...
                'meanXYZ',[.9,1.5,1;1.5,1.5,2], ...
                'minimumXYZ',[.9,1.5,.5;1.5,1.5,1], ...
                'maximumXYZ',[.9,1.5,1.5;1.5,1.5,3], ...
                'covarianceXYZ',[zeros(2,5),[.25;.5]]);
            off=struct('columnMaps',struct('origin',[.6,1.2],'dx',.6,'dy',.6, ...
                'mapSize',[2,3],'statistics',stats));
            [values,names,ids]=measureDowntownPillarStatistics(ground,off,'curb');
            own=ids==sub2ind(dims,3,2);
            outside=ids==sub2ind(dims,2,2);
            testCase.verifyEqual(values(own,names=="offGround_r0_countSum"),2);
            testCase.verifyEqual(values(own,names=="offGround_r0_minimumAboveGround"),.5);
            testCase.verifyEqual(values(own,names=="offGround_r0_heightVariance"),.25);
            testCase.verifyEqual(values(own,names=="offGround_r1_countSum"),6);
            testCase.verifyEqual(values(own,names=="offGround_r1_meanAboveGround"),5/3,'AbsTol',1e-8);
            testCase.verifyEqual(values(own,names=="offGround_r1_heightVariance"),23/36,'AbsTol',1e-8);
            testCase.verifyEqual(values(own,names=="offGround_r1_maximumAboveGround"),3);
            testCase.verifyEqual(values(outside,names=="offGround_r1_countSum"),6);
            testCase.verifyEqual(values(outside,names=="offGround_r0_countSum"),0);
        end
    end
end

function [cells,mapSize]=pointCells(points)
% Deliberately nonsquare to exercise [Nx Ny] versus [Ny Nx] index conversion.
    dims=[50,25];mapSize=fliplr(dims);
    bins=floor((points(:,1:2)+[1,1])/.3)+1;
    cells=sub2ind(dims,bins(:,1),bins(:,2));
end

function maps=curbMaps(mapSize)
    maps=struct('extractedMask',true(mapSize),'rawExtractedMask',true(mapSize), ...
        'total',ones(mapSize),'totalBase',ones(mapSize),'heightStepMeters',.1*ones(mapSize), ...
        'linearityComponentCenterEvidence',ones(mapSize),'relativeHeightMeters',.1*ones(mapSize), ...
        'roughnessMeters',.03*ones(mapSize),'linearity',ones(mapSize));
end
