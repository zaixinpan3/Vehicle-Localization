classdef downtownStructuralTest < matlab.unittest.TestCase
% downtownStructuralTest: Original-point shape, context and sign support.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsDisconnectedAndLaterallyJumpingShafts(testCase)
            cfg=downtownStructuralConfig();
            z=(0:.05:3).'; straight=[zeros(size(z)),zeros(size(z)),z];
            gap=straight(z<1 | z>2,:);
            jumped=straight; jumped(:,1)=.23*mod(floor(z/.4),2);
            [keepStraight,~]=validateDowntownPoleShape(straight,ones(size(z)),cfg.pointShape);
            [keepGap,gapMetrics]=validateDowntownPoleShape(gap,ones(size(gap,1),1),cfg.pointShape);
            [keepJump,jumpMetrics]=validateDowntownPoleShape(jumped,ones(size(z)),cfg.pointShape);
            testCase.verifyTrue(all(keepStraight));
            testCase.verifyFalse(any(keepGap));
            testCase.verifyTrue(contains(gapMetrics.rejectReason,'verticalGap'));
            testCase.verifyFalse(any(keepJump));
            testCase.verifyTrue(contains(jumpMetrics.rejectReason,'axisProfileDiscontinuity'));
        end
        function repeatedMeasuredAttachmentsRejectOtherwiseIsolatedPole(testCase)
            cfg=downtownStructuralConfig();cfg.context.minObjectToContextPointRatio=0;
            z=(0:.05:3).';pole=[zeros(size(z)),zeros(size(z)),z];
            x=(.15:.15:1.5).';branch=[x,zeros(size(x)),.8*ones(size(x)); ...
                x,zeros(size(x)),2.2*ones(size(x))];
            raw=[pole;branch];ids=(1:size(pole,1)).';
            [isolated,~]=validateDowntownPoleContext(pole,ids,ones(size(ids)),pole,ids,cfg.context);
            [attached,metrics]=validateDowntownPoleContext(pole,ids,ones(size(ids)),raw,(1:size(raw,1)).',cfg.context);
            testCase.verifyTrue(all(isolated));
            testCase.verifyFalse(any(attached));
            testCase.verifyEqual(metrics.rejectReason,"lateralStructureConnections");
        end
        function signRequiresReliableLocalGroundAndEveryPointClearance(testCase)
            cfg=downtownStructuralConfig();
            [x,y]=ndgrid(-2:.25:2);ground=[x(:),y(:),.05*x(:)];
            sign=[-.1 0 2;0 0 2.1;.1 0 2.2];
            [high,~]=validateDowntownSignSupport(sign,ones(3,1),ground,cfg.signSupport);
            low=sign;low(1,3)=1;
            [rejected,metrics]=validateDowntownSignSupport(low,ones(3,1),ground,cfg.signSupport);
            [missing,~]=validateDowntownSignSupport(sign,ones(3,1),zeros(0,3),cfg.signSupport);
            testCase.verifyTrue(all(high));
            testCase.verifyFalse(any(rejected));
            testCase.verifyEqual(metrics.rejectReason,"lowReflector");
            testCase.verifyFalse(any(missing));
        end
        function denseRecoveryPreservesShortIsolatedPolesAndRejectsFacades(testCase)
            cfg=downtownStructuralConfig();dims=[5 5];
            z=repelem((0:.025:1.5).',2);x=repmat([-.02;.02],numel(z)/2,1);
            points=[x,zeros(size(x)),z];columns=13*ones(size(z));
            raw=false(dims);raw(13)=true;
            debug=struct('poleMaskRaw',raw,'poleSupportMask',raw);
            a=recoverDowntownDensePoles(points,columns,true(size(z)),zeros(size(z)),false(dims),debug,cfg.denseRecovery);
            b=recoverDowntownDensePoles(points,columns,true(size(z)),ones(size(z)),false(dims),debug,cfg.denseRecovery);
            testCase.verifyTrue(all(a.pointMask));
            testCase.verifyFalse(any(b.pointMask));
            testCase.verifyEqual(b.metrics.rejectReason,"otherFeatureOverlap");
        end
        function signOnlyUsesTheSamePillarAndRetainsSeparateRadiometry(testCase)
            cfg=downtownStructuralConfig(.6);cfg=rmfield(cfg,{'candidate','slice','pointShape','context','denseRecovery'});
            gridCfg=pillarGridConfig();gridCfg.exclusionHalfSize=0;
            [x,y]=ndgrid(8:.25:12,3:.25:7);ground=[x(:),y(:),zeros(numel(x),1)];
            sign=[10 5 2;10.02 5 2.1;10.04 5 2.2;10.06 5 2.3];
            frame=struct('x',[ground(:,1);sign(:,1)],'y',[ground(:,2);sign(:,2)], ...
                'z',[ground(:,3);sign(:,3)],'intensity',[zeros(size(ground,1),1);1601;1700;1800;1600]);
            full=pillarizePointCloud(frame,gridCfg);sub=frame;
            for name=string(fieldnames(sub)).',sub.(name)=sub.(name)(end-3:end);end
            sub.pointIndices=int32(size(ground,1)+(1:4).');pillars=pillarizePointCloud(sub,gridCfg);
            retainedGround=double(full.pointIndices)<=size(ground,1);
            result=detectDowntownStructures(pillars,full,retainedGround,0,struct(),cfg,"trafficSign");
            testCase.verifyFalse(isfield(result,'pole'));
            testCase.verifyEqual(result.trafficSign.pointIndices,size(ground,1)+(1:3).');
            testCase.verifyTrue(all(result.trafficSign.acceptedMask));
            testCase.verifyEqual(nnz(result.trafficSign.mask),1);
            testCase.verifyEqual(result.trafficSign.supportFraction(result.trafficSign.mask),.75,'AbsTol',1e-12);
        end
        function emptyStructuralInputsReturnEmptyDecisions(testCase)
            gridCfg=pillarGridConfig();pillars=pillarizePointCloud(zeros(0,3),gridCfg);
            facade=struct('seedLineMap',zeros(pillars.pillarGeometry.mapSize), ...
                'pointLineIds',zeros(0,1));
            result=detectDowntownStructures(pillars,pillars,false(0,1),0,facade,downtownStructuralConfig(.6));
            testCase.verifyEmpty(result.pole.pointIndices);
            testCase.verifyEmpty(result.trafficSign.pointIndices);
            testCase.verifyFalse(any(result.pole.mask,'all'));
        end
        function fineSignLabelsCannotEscapeCoarseCandidatePillars(testCase)
            cfg=downtownStructuralConfig(.6);gridCfg=pillarGridConfig();gridCfg.exclusionHalfSize=0;
            [x,y]=ndgrid(8:.25:14,3:.25:7);ground=[x(:),y(:),zeros(numel(x),1)];
            sign=[10 5 2;10.02 5 2.1;10.04 5 2.2;12 5 2;12.02 5 2.1;12.04 5 2.2];
            frame=struct('x',[ground(:,1);sign(:,1)],'y',[ground(:,2);sign(:,2)], ...
                'z',[ground(:,3);sign(:,3)],'intensity',[zeros(size(ground,1),1);1700*ones(6,1)]);
            full=pillarizePointCloud(frame,gridCfg);sub=frame;
            for name=string(fieldnames(sub)).',sub.(name)=sub.(name)(end-5:end);end
            sub.pointIndices=int32(size(ground,1)+(1:6).');pillars=pillarizePointCloud(sub,gridCfg);
            retainedGround=double(full.pointIndices)<=size(ground,1);
            candidates=struct('trafficSign',false(pillars.pillarGeometry.mapSize));
            candidates.trafficSign(pillars.pointPillarLinIdx(1))=true;
            result=detectDowntownStructures(pillars,full,retainedGround,0,struct(),cfg,"trafficSign",candidates);
            testCase.verifyEqual(result.trafficSign.pointIndices,size(ground,1)+(1:3).');
            testCase.verifyTrue(all(result.trafficSign.acceptedMask));
            testCase.verifyFalse(any(result.trafficSign.mask(~candidates.trafficSign)));
        end
        function finePoleLabelsCannotEscapeCoarseCandidatePillars(testCase)
            cfg=downtownStructuralConfig(.6);gridCfg=pillarGridConfig();gridCfg.exclusionHalfSize=0;
            z=(0:.025:3).';xyz=[10*ones(size(z)),5*ones(size(z)),z; ...
                12*ones(size(z)),5*ones(size(z)),z];
            pillars=pillarizePointCloud(xyz,gridCfg);
            facade=struct('seedLineMap',zeros(pillars.pillarGeometry.mapSize), ...
                'pointLineIds',zeros(size(xyz,1),1));
            candidates=struct('pole',false(pillars.pillarGeometry.mapSize));
            candidates.pole(pillars.pointPillarLinIdx(1))=true;
            result=detectDowntownStructures(pillars,pillars,false(size(xyz,1),1),0, ...
                facade,cfg,"pole",candidates);
            selected=result.pole.pointIndices(result.pole.acceptedMask);
            testCase.verifyNotEmpty(selected);
            testCase.verifyLessThanOrEqual(selected,numel(z));
            testCase.verifyFalse(any(result.pole.mask(~candidates.pole)));
        end
    end
end
