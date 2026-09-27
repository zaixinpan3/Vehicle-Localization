classdef pillarPoleValidationTest < matlab.unittest.TestCase
% pillarPoleValidationTest: Complete distributions and actual owner support.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function minorityShaftSurvivesClutterAtOtherHeights(t)
            shaft=t.shaft([.12 .16]);rng(14);
            clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
            e=t.measure([shaft;clutter],[.12 .16]);
            t.verifyTrue(e.found);t.verifyLessThan(e.ownCount,100);
        end
        function completeWideSurfaceRejectsNarrowAxisProposal(t)
            [x,z]=meshgrid(.06:.02:.54,-1:.08:3);
            p=[x(:),.3+.002*sin((1:numel(x)).'),z(:)];
            [e,h]=t.measure(p,[.3 .3]);
            t.verifyNotEmpty(h);t.verifyGreaterThan(h.maximumStd,.12);
            t.verifyGreaterThan(h.aspect,3);t.verifyFalse(any(e.found));
        end
        function translatedHeightDoesNotChangeSelection(t)
            p=t.shaft([.3 .3]);a=t.measure(p,[.3 .3]);p(:,3)=p(:,3)+.137;
            b=t.measure(p,[.3 .3]);t.verifyTrue(a.found);
            t.verifyEqual(b,a,'AbsTol',1e-12);
        end
        function boundaryShaftKeepsBothOwners(t)
            e=t.measure(t.shaft([.6 .3]),[.6 .3],[1 2]);
            t.verifyTrue(all(e.found));t.verifyGreaterThanOrEqual(e.ownCount,10*ones(2,1));
        end
        function peripheralPointsCannotBorrowOwnerConfidence(t)
            p=[t.shaft([.57 .3]);.61 .3 0;.61 .3 1;.61 .3 2];
            e=t.measure(p,[.57 .3],[1 2]);
            t.verifyTrue(e.found(1));t.verifyFalse(e.found(2));
        end
        function disconnectedBlobsDoNotSupplyContinuousSupport(t)
            z=[(0:.05:.5)';(3:.05:3.5)'];p=[.3+.01*cos(z*20),.3+.01*sin(z*20),z];
            e=t.measure(p,[.3 .3]);t.verifyFalse(any(e.found));
        end
        function brightReturnsParticipateAfterStructuralQualification(t)
            p=t.shaft([.3 .3]);mask=true(60,1);mask(1:4:end)=false;
            e=t.measure(p,[.3 .3],[1 1],mask);
            t.verifyTrue(e.found);t.verifyEqual(e.ownCount,60);
            e=t.measure(p,[.3 .3],[1 1],false(60,1));t.verifyFalse(e.found);
        end
        function shortTiltedSupportIsRejected(t)
            p=t.shaft([.3 .3]);p(:,3)=p(:,3)/3;
            upright=t.measure(p,[.3 .3]);t.verifyTrue(upright.found);
            p(:,1)=p(:,1)+tand(8)*p(:,3);
            leaning=t.measure(p,[.3 .3]);t.verifyFalse(leaning.found);
        end
        function emptyCloudIsSupported(t)
            e=t.measure(zeros(0,3),[.3 .3]);t.verifyEmpty(e.found);
        end
        function offlineRetainsOriginalDetector(t)
            a=perceptionConfig('Mississippi','offline');b=perceptionConfig();
            t.verifyEqual(a.offGroundFeatures.pole.detector,"pillar");
            t.verifyEqual(a.voxel.voxelSize,[.3 .3]);
            t.verifyEqual(b.offGroundFeatures.pole.detector,"validatedShaft");
            t.verifyEqual(b.voxel.voxelSize,[.6 .6]);
        end
        function recordedCoarseBackendsKeepTheSameCandidates(t)
            t.assumeTrue(perceptionNativeAvailable);
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');t.assumeTrue(isfile(file));
            for index=[111 301 900]
                frame=loadPointCloudFrame(file,index);cfg=perceptionConfig();
                cfg.executionBackend="matlab";a=perceiveFrame(frame,cfg);
                cfg.executionBackend="native";b=perceiveFrame(frame,cfg);
                t.verifyEqual(b.candidates,a.candidates);
                t.verifyEqual(b.probabilityCloud.components.meanXYZ,a.probabilityCloud.components.meanXYZ,'AbsTol',1e-10);
                t.verifyEqual(b.probabilityCloud.components.semanticProbability,a.probabilityCloud.components.semanticProbability,'AbsTol',1e-10);
            end
        end
    end
    methods (Static,Access=private)
        function p=shaft(center)
            z=linspace(-1,3,60).';angle=(1:60).'*2.399;
            p=[center(1)+.025*cos(angle),center(2)+.025*sin(angle),z];
        end
        function [e,h]=measure(p,center,mapSize,mask)
            if nargin<3,mapSize=[1 1];end
            if nargin<4,mask=true(size(p,1),1);end
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',mapSize);
            bins=floor(p(:,1:2)./.6)+1;ids=sub2ind(mapSize,bins(:,2),bins(:,1));
            occupied=unique(ids);n=numel(occupied);found=false(n,1);if n>0,found(1)=true;end
            modes=struct('pillarIndices',occupied,'found',found, ...
                'axisXY',repmat(center,n,1),'axisZ',zeros(n,1),'slopeXY',zeros(n,2));
            [e,h]=validatePillarPoleSupport(p,ids,geometry,modes,pillarPoleValidationConfig(),mask,ones(n,1));
        end
    end
end
