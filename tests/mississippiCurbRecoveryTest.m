classdef mississippiCurbRecoveryTest < matlab.unittest.TestCase
% mississippiCurbRecoveryTest: Mississippi-only precision/coverage contracts.
    properties (TestParameter)
        frameIndex={300,500,900}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'research','pillar_fine_alignment_20260926')));
        end
    end
    methods (Test)
        function overrideIsRestrictedToMississippi(testCase)
            cfg=perceptionConfig('Mississippi');alias=perceptionConfig('Missisipi');urban=perceptionConfig('Downtown');
            testCase.verifyEqual(cfg.semanticPrecision.modelFiles,alias.semanticPrecision.modelFiles);
            testCase.verifyEqual(string(fieldnames(cfg.semanticPrecision.modelFiles)),["curb";"curbRecovery"]);
            testCase.verifyEmpty(fieldnames(urban.semanticPrecision.modelFiles));
            offline=perceptionConfig('Mississippi','offline');testCase.verifyFalse(offline.semanticPrecision.enabled);
        end
        function originalFineReferencesStayExact(testCase,frameIndex)
            [frame,root]=recordedFrame(testCase,frameIndex);frozen=load(fullfile(root,'output','semantic_precision_20260927','mississippi_baseline.mat'),'frames','references','names');
            at=frozen.frames==frameIndex;cfg=perceptionConfig('Mississippi','offline');
            cfg.semanticPrecision.modelFiles.curb="MissingModelMustNotBeRead.json";fine=perceiveFrame(frame,cfg);
            for k=1:numel(frozen.names)
                testCase.verifyEqual(double(find(fine.featureMasks.(frozen.names(k)))),double(frozen.references{at,k}(:)));
            end
        end
        function currentCoarseChangesOnlyCurbDecisions(testCase,frameIndex)
            frame=recordedFrame(testCase,frameIndex);cfg=perceptionConfig('Mississippi');old=cfg;old.semanticPrecision.modelFiles=struct();
            before=perceiveFrame(frame,old);after=perceiveFrame(frame,cfg);
            testCase.verifyEqual(after.sourceSummary,before.sourceSummary);testCase.verifyEqual(after.candidates.geometry,before.candidates.geometry);
            for name=["pole","trafficSign"]
                k=after.candidates.semanticNames==name;testCase.verifyEqual(after.candidates.pillarIndices{k},before.candidates.pillarIndices{k});
            end
            unfiltered=cfg;unfiltered.semanticPrecision.enabled=false;unfiltered.coarseProbabilityCloud.storeDiagnostics=true;
            proposals=perceiveFrame(frame,unfiltered);g=proposals.diagnostics.ground;
            allowed=g.curbCellMask | (g.energyMaps.rawExtractedMask & g.energyMaps.totalBase>=.25);
            [r,c]=find(allowed);xy=g.cellOrigin+([c r]-.5).*g.cellSize;
            geometry=proposals.candidates.geometry;bin=floor((xy-geometry.origin)./geometry.cellSize)+1;
            allowedIds=sub2ind(geometry.mapSize,bin(:,2),bin(:,1));
            selected=after.candidates.pillarIndices{after.candidates.semanticNames=="curb"};
            testCase.verifyTrue(all(ismember(selected,allowedIds)));
        end
        function singleChannelAndMatlabBackendAgree(testCase)
            frame=recordedFrame(testCase,900);cfg=perceptionConfig('Mississippi');all=perceiveFrame(frame,cfg);
            cfg.featureNames="curb";single=perceiveFrame(frame,cfg);cfg.executionBackend="matlab";fallback=perceiveFrame(frame,cfg);
            expected=all.candidates.pillarIndices{all.candidates.semanticNames=="curb"};
            testCase.verifyEqual(single.candidates.pillarIndices{1},expected);testCase.verifyEqual(fallback.candidates.pillarIndices{1},expected);
        end
        function recoveryPreservesExistingSelectionsAndAddsGeometricSupport(testCase)
            frame=recordedFrame(testCase,900);cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
            old=cfg;old.semanticPrecision.modelFiles=rmfield(old.semanticPrecision.modelFiles,'curbRecovery');
            before=perceiveFrame(frame,old);after=perceiveFrame(frame,cfg);
            a=before.diagnostics.ground;b=after.diagnostics.ground;added=b.curbCellMask & ~a.curbCellMask;
            testCase.verifyTrue(all(b.curbCellMask(a.curbCellMask)));
            testCase.verifyTrue(any(added,'all'));
            testCase.verifyTrue(all(b.energyMaps.rawExtractedMask(added)));
            testCase.verifyGreaterThanOrEqual(b.curbProbability(added),single(cfg.coarseProbabilityCloud.minimumSemanticProbability));
            testCase.verifyEqual(b.moments,a.moments);
        end
        function emptyMississippiInputStaysEmpty(testCase)
            p=perceiveFrame(struct('x',NaN,'y',NaN,'z',NaN),perceptionConfig('Mississippi'));
            testCase.verifyTrue(all(cellfun(@isempty,p.candidates.pillarIndices)));
        end
        function probabilityForestPreservesFloat32Splits(testCase)
            model=smallForest();features=[NaN;.5;.50000001;1];
            testCase.verifyEqual(scoreSemanticPillarModel(features,model,.5),[.2;.2;.2;.8]);
        end
        function existingForestConsensusStillVetoesDisagreement(testCase)
            model=smallForest();model.kind="forestNeighborConsensus";
            model.neighborCenter=0;model.neighborScale=1;model.neighborVectors=[0;1];
            model.neighborPositive=[false;false];model.neighborCount=1;model.neighborThreshold=1;
            testCase.verifyEqual(scoreSemanticPillarModel([0;1],model,.5),[.2;0]);
        end
    end
end
function model=smallForest()
    model=struct('kind',"probabilityForest",'featureNames',"one",'featureScale',1e8, ...
        'roots',1,'feature',[1;0;0],'threshold',[.5;0;0],'left',[2;0;0], ...
        'right',[3;0;0],'value',[0;.2;.8],'leaf',[false;true;true]);
end
function [frame,root]=recordedFrame(testCase,index)
    root=setupVehicleLocalization();file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    testCase.assumeTrue(isfile(file));frame=loadPointCloudFrame(file,index);
end
