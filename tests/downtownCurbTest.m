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
        function fineSurfacesDistinguishGradeFromOutwardStep(testCase)
            cfg=downtownCurbConfig().surfaceCheck;
            [x,y]=ndgrid(-5:.12:5,2:.10:6);
            z=.04*x+.09*y;
            edge=(-3:.12:3).'; candidates=[edge,4*ones(size(edge)),.04*edge+.36];
            testCase.verifyFalse(any(checkDowntownCurbSurfaces(candidates,[x(:),y(:),z(:)],cfg)));
            raised=z+.12*(y>4);
            testCase.verifyTrue(all(checkDowntownCurbSurfaces(candidates,[x(:),y(:),raised(:)],cfg)));
            visible=y<4;
            testCase.verifyTrue(all(checkDowntownCurbSurfaces(candidates, ...
                [x(visible),y(visible),z(visible)],cfg)));
        end
        function registeredStraightCurbUsesCoverageOnEachSide(testCase)
            cfg=downtownCurbConfig().straightRun;
            s=(0:2:100).'; path=[s,zeros(size(s))]; frames=cell(numel(s),1);
            for k=1:numel(s)
                frames{k}=[s(k),-4,0];
                if s(k)>30 && s(k)<40
                    frames{k}=[frames{k};repmat([s(k),-8,0;s(k),5,0],30,1)];
                end
            end
            % The single-view endpoint has the same status as all other inliers.
            rotation=[cos(.7),-sin(.7);sin(.7),cos(.7)];offset=[114,-52];
            for k=1:numel(s),frames{k}(:,1:2)=frames{k}(:,1:2)*rotation+offset;end
            cleaned=refineDowntownCurbRuns(frames,path*rotation+offset,cfg);
            for k=1:numel(s),testCase.verifyEqual(cleaned{k},frames{k}(1,:));end
            testCase.verifyEqual(refineDowntownCurbRuns(frames(1:8),path(1:8,:),cfg),frames(1:8));
            testCase.verifyEqual(refineDowntownCurbRuns(frames,zeros(size(path)),cfg),frames);
        end
        function looseStepCannotExtendAcrossUnobservedStations(testCase)
            cfg=downtownCurbConfig().straightRun;
            s=(0:2:120).'; path=[s,zeros(size(s))]; frames=cell(numel(s),1);
            for k=1:numel(s)
                frames{k}=[s(k),5,0];
                if s(k)<=70
                    frames{k}(end+1,:)=[s(k),-4,0];
                elseif s(k)>=108
                    frames{k}(end+1,:)=[s(k),-4.8,0];
                end
            end
            cleaned=refineDowntownCurbRuns(frames,path,cfg);
            for k=1:numel(s)
                expected=frames{k}(frames{k}(:,2)~=-4.8,:);
                testCase.verifyEqual(cleaned{k},expected);
            end
        end
        function bendCheckResolvesRotatedStepAndRejectsSmoothGrade(testCase)
            cfg=downtownCurbConfig().bendStep;
            [s,v]=ndgrid(-1.5:.12:1.5,-.9:.09:.9);
            t=[cosd(37),sind(37)]; n=[-t(2),t(1)]; anchor=[5*n,-1.4];
            xy=anchor(1:2)+s(:)*t+v(:)*n;
            z=anchor(3)+.07*s(:)-.05*v(:);
            raised=z+.11*(v(:)>.10*s(:).^2);
            testCase.verifyTrue(confirmDowntownBendCurbStep(anchor,[xy,raised],cfg));
            testCase.verifyFalse(confirmDowntownBendCurbStep(anchor,[xy,z],cfg));
            occluded=v(:)<0;
            testCase.verifyFalse(confirmDowntownBendCurbStep(anchor,[xy(occluded,:),z(occluded)],cfg));
        end
        function turnUsesRawEvidenceAtObservedLocation(testCase)
            cfg=downtownCurbConfig().straightRun;
            path=[(0:2:40).',zeros(21,1);44,2;47,5;48,9;48,14;48,20;48,30;48,40;48,50];
            points=repmat({zeros(0,3)},1,size(path,1));
            % An earlier scan sees both turn candidates. The validator confirms
            % the one nearest the route and rejects the more distant candidate.
            points{16}=[44,2.1,1;43,4,0];
            mask=@(k,p) k==16 & p(:,3)==1;
            out=refineDowntownCurbRuns(points,path,cfg,mask);
            testCase.verifyEqual(out{16},points{16}(1,:));
            straight=[(0:2:56).',zeros(29,1)];
            reference=refineDowntownCurbRuns(points,straight,cfg);
            testCase.verifyEqual(refineDowntownCurbRuns(points,straight,cfg,mask),reference);
        end
        function reviewedNegativeScanPublishesNoCurb(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','processed','downtown_reference_gnss_free_20261005', ...
                'raw_data_2024-05-21-11-47-40_0','pointClouds.mat');
            testCase.assumeTrue(isfile(file),'The reviewed Downtown recording is required.');
            cfg=perceptionConfig('Downtown');cfg.featureNames="curb";
            result=perceiveFrame(loadPointCloudFrame(file,184),cfg);
            testCase.verifyEmpty(result.candidates.pillarIndices{1});
            testCase.verifyEqual(result.probabilityCloud.components.numComponents,0);
        end
        function reviewedTurnKeepsMeasuredStepAndRejectsTwoFalseLocations(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            folder=fullfile(root,'data','processed','downtown_reference_gnss_free_20261005', ...
                'raw_data_2024-05-21-11-49-00_0');
            file=fullfile(folder,'pointClouds.mat');
            testCase.assumeTrue(isfile(file),'The reviewed Downtown recording is required.');
            poses=readtable(fullfile(folder,'mapping_poses.csv'));
            ids=poses.frame_index(mod(poses.frame_index,2)==1 & poses.lidar_stamp_sec<poses.lidar_stamp_sec(end)-.1002);
            points=repmat({zeros(0,3)},1,numel(ids));
            points{ids==449}=[197.401261,-122.453025,14.413305];
            points{ids==451}=[192.934528,-118.652066,13.973596;193.715381,-117.369692,14.152745];
            p=perceptionConfig("Downtown","offline");
            data=struct('featureNames',"curb",'pointsByFeatureFrame',{points}, ...
                'frameIndices',ids,'framePoseTable',readFramePoseTable(fullfile(folder,'mapping_poses.csv'),ids), ...
                'frameCalibration',p.frameCalibration,'counts',zeros(numel(ids),1));
            loader=@(id) deskewReferenceMappingFrame(loadPointCloudFrame(file,id),poses);
            result=refineDowntownMappingCurbs(data,p,loader);
            testCase.verifyEmpty(result.pointsByFeatureFrame{ids==449});
            testCase.verifyEqual(result.pointsByFeatureFrame{ids==451},points{ids==451}(2,:));
            testCase.verifyEqual(sum(result.counts),1);
        end
        function rejectsScatteredGradeReturnsAndRetainsHeightBreaks(testCase)
            cfg=downtownCandidateConfig(.3).curbRoadPatch;
            [x,y]=meshgrid((0:24)*.3);
            stats=struct('heightMap',.08*x+.03*y,'countMap',ones(size(x)));
            scattered=false(size(x));scattered(5:4:21,5:4:21)=true;
            testCase.verifyFalse(any(rejectCurbRoadPatches(scattered,scattered,stats,.3,cfg),'all'));
            % Raised returns disagree with the surrounding grade.
            stats.heightMap(scattered)=stats.heightMap(scattered)+.15;
            testCase.verifyEqual(rejectCurbRoadPatches(scattered,scattered,stats,.3,cfg),scattered);
        end
        function retainsSparseLinearEdgesAndConnectedSegmentsOnGrades(testCase)
            cfg=downtownCandidateConfig(.3).curbRoadPatch;
            [x,y]=meshgrid((0:24)*.3);
            stats=struct('heightMap',.08*x+.03*y+.12*(y>3),'countMap',ones(size(x)));
            narrow=false(size(x));narrow(11,3:4:23)=true;
            testCase.verifyEqual(rejectCurbRoadPatches(narrow,narrow,stats,.3,cfg),narrow);
            % In a broad road patch, a connected curb and its measured endpoint survive.
            stats.heightMap=.08*x+.03*y;
            wide=false(size(x));wide(4:4:20,4:4:20)=true;wide(11,2:5)=true;wide(11,10)=true;
            proposals=wide;proposals(11,6:8)=true;
            keep=rejectCurbRoadPatches(wide,proposals,stats,.3,cfg);
            testCase.verifyTrue(all(keep(11,[2:5,10])));
            testCase.verifyFalse(keep(20,20));
            testCase.verifyFalse(any(keep(~wide)));
        end
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
