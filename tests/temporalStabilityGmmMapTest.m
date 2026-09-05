classdef temporalStabilityGmmMapTest < matlab.unittest.TestCase
% temporalStabilityGmmMapTest: Inference, field, provenance and schedule contracts.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'mapping')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function tangentialAppearanceChangesAreTolerated(testCase)
            [p,x,l,b]=testCase.fixture(3); tangent=x; tangent(:,1)=tangent(:,1)+0.4*(b-2);
            normal=x; normal(:,2)=0.4*(b-2);
            along=buildTemporalStabilityGmmMap(tangent,l,b,p);
            across=buildTemporalStabilityGmmMap(normal,l,b,p);
            testCase.verifyGreaterThan(along.layers.componentRepeatability,across.layers.componentRepeatability);
        end
        function distantMapGrowthDoesNotRenormalizeLocalField(testCase)
            [p,x,l,b]=testCase.fixture(3); local=buildTemporalStabilityGmmMap(x,l,b,p);
            globalMap=buildTemporalStabilityGmmMap([x;x+100],[l;l],[b;b],p);
            testCase.verifyEqual(queryTemporalStabilityGmmMap(globalMap,[1 0],"stable"), ...
                queryTemporalStabilityGmmMap(local,[1 0],"stable"),'AbsTol',1e-12);
            testCase.verifyEqual(sum(globalMap.layers.componentReferenceMasses)+globalMap.layers.backgroundReferenceMass, ...
                globalMap.layers.referenceMass,'AbsTol',1e-12);
        end
        function newCloudRetainsPerClassMassAndIndependentImportance(testCase)
            [p,x,l,b]=testCase.fixture(3); p.classes=["stable";"other"];
            map=buildTemporalStabilityGmmMap([x;x+1],[l;repmat("other",size(l))],[b;b],p);
            changed=map; changed.layers(1).classImportance=0;
            cloud=temporalMapToProbabilityCloud(map); altered=temporalMapToProbabilityCloud(changed);
            keep=cloud.components.semanticName=="stable";
            testCase.verifyEqual(sum(cloud.components.classMixtureWeight(keep)),1,'AbsTol',1e-12);
            testCase.verifyEqual(cloud.classTotalMass(1),map.layers(1).totalMass,'AbsTol',1e-12);
            testCase.verifyEqual(altered.totalMass,cloud.totalMass,'AbsTol',0);
            testCase.verifyEqual(altered.components.mass,cloud.components.mass,'AbsTol',0);
        end
        function haloObservationsHaveOneMassOwner(testCase)
            [p,x,l,b]=testCase.fixture(3); x(:,1)=x(:,1)+3;
            map=buildTemporalStabilityGmmMap(x,l,b,p);
            cells=vertcat(map.layers.tiles.referenceCells);
            testCase.verifyEqual(size(cells,1),size(unique(cells,'rows'),1));
            testCase.verifyEqual(map.layers.referenceMass,size(cells,1)*p.defaultParams.referenceResolution^2,'AbsTol',1e-12);
            testCase.verifyGreaterThan(numel(map.layers.tiles),1);
        end
        function removedSamplingConfigurationIsRejected(testCase)
            [p,x,l,b]=testCase.fixture(2); p.defaultParams.temporalReliabilitySamplingFloor=0.05;
            testCase.verifyError(@() buildTemporalStabilityGmmMap(x,l,b,p),'buildTemporalStabilityGmmMap:RemovedParameter');
        end
        function backgroundKeepsUnexplainedReferenceMass(testCase)
            [p,x,l,b]=testCase.fixture(3); map=buildTemporalStabilityGmmMap(x,l,b,p);
            testCase.verifyGreaterThan(map.layers.backgroundReferenceMass,0);
            testCase.verifyEqual(sum(map.layers.componentReferenceMasses)+map.layers.backgroundReferenceMass, ...
                map.layers.referenceMass,'AbsTol',1e-12);
        end

        function twoAgreeingBlocksAreConfirmed(testCase)
            [p,x,l,b]=testCase.fixture(2); map=buildTemporalStabilityGmmMap(x,l,b,p);
            testCase.verifyGreaterThan(map.layers.componentRepeatability,0.5);
            testCase.verifyEqual(map.layers.components.observedBlockCount,2);
            testCase.verifyTrue(map.layers.components.published);
            testCase.verifyGreaterThan(map.layers.totalMass,0);
        end
        function oneBlockRemainsUnconfirmedWithFavorablePrior(testCase)
            [p,x,l,b]=testCase.fixture(1); p.defaultParams.repeatabilityPrior=0.99;
            map=buildTemporalStabilityGmmMap(x,l,b,p);
            cloud=temporalMapToProbabilityCloud(map);
            [score,d]=queryTemporalStabilityGmmMap(map,[1 0],"stable");
            testCase.verifyEqual(map.layers.components.status,"unconfirmed");
            testCase.verifyEqual(cloud.totalMass,0,'AbsTol',0);
            testCase.verifyEmpty(cloud.components.mean);
            testCase.verifyEqual(score,0,'AbsTol',0);
            testCase.verifyEqual(d.status,"noPublishedStructure");
        end
        function denseStackedMarginalMatchesStructuredHypothesis(testCase)
            [p,x,l,b]=testCase.fixture(3); p.defaultParams.storeEmAssignments=true;
            map=buildTemporalStabilityGmmMap(x,l,b,p);
            expected=testCase.stackedProbability(map.layers,p.defaultParams);
            testCase.verifyEqual(map.layers.componentRepeatability,expected,'AbsTol',1e-10);
        end
        function objectiveAndAllParameterChangesAreTracked(testCase)
            [p,x,l,b]=testCase.fixture(3); p.defaultParams.storeEmAssignments=true;
            map=buildTemporalStabilityGmmMap(x,l,b,p); fit=map.layers.tiles.fit;
            testCase.verifyGreaterThanOrEqual(diff(fit.objectiveTrace),-1e-8*ones(fit.iterationCount,1));
            testCase.verifySize(fit.parameterChange,[fit.iterationCount,1]);
            testCase.verifyEqual(sum(fit.responsibilities,2),ones(size(x,1),1),'AbsTol',1e-12);
            testCase.verifyGreaterThan(fit.backgroundFractions,zeros(3,1));
            testCase.verifyEqual(fit.objective,"structuredVariationalLowerBoundWithParameterPriors");
        end
        function duplicatingOneFrameDoesNotChangeInference(testCase)
            [p,x,l,b]=testCase.fixture(3); rows=[(1:size(x,1))';repmat(find(b==1),12,1)];
            original=buildTemporalStabilityGmmMap(x,l,b,p);
            duplicate=buildTemporalStabilityGmmMap(x(rows,:),l(rows),b(rows),p);
            testCase.verifyEqual(duplicate.layers.componentMeans,original.layers.componentMeans,'AbsTol',0);
            testCase.verifyEqual(duplicate.layers.componentCovariances,original.layers.componentCovariances,'AbsTol',0);
            testCase.verifyEqual(duplicate.layers.componentMasses,original.layers.componentMasses,'AbsTol',0);
        end
        function inputOrderAndRandomStateDoNotChangeMap(testCase)
            [p,x,l,b]=testCase.fixture(3); before=rng;
            original=buildTemporalStabilityGmmMap(x,l,b,p);
            reordered=buildTemporalStabilityGmmMap(flipud(x),flipud(l),flipud(b),p);
            testCase.verifyEqual(rng,before);
            testCase.verifyEqual(reordered.layers.componentMeans,original.layers.componentMeans,'AbsTol',0);
            testCase.verifyEqual(reordered.layers.componentMasses,original.layers.componentMasses,'AbsTol',0);
        end
        function explicitBlocksControlTemporalEvidence(testCase)
            [p,x,l,b]=testCase.fixture(3);
            obs=struct('frameId',b,'observationBlockId',ones(size(b)));
            map=buildTemporalStabilityGmmMap(x,l,obs,p);
            testCase.verifyEqual(map.layers.components.observedBlockCount,1);
            testCase.verifyEqual(map.layers.components.status,"unconfirmed");
            testCase.verifyEqual(map.layers.representativeBlockIds,repmat("1",5,1));
        end
        function unrelatedBlocksDoNotSupplyEvidence(testCase)
            [p,x,l,b]=testCase.fixture(2);
            original=buildTemporalStabilityGmmMap(x,l,b,p);
            extra=buildTemporalStabilityGmmMap([x;100 100],[l;"other"],[b;99],p);
            testCase.verifyEqual(extra.layers.componentRepeatability,original.layers.componentRepeatability,'AbsTol',0);
        end
        function normalDisagreementReducesRepeatability(testCase)
            [p,x,l,b]=testCase.fixture(3); variable=x; variable(:,2)=b-2;
            agreeing=buildTemporalStabilityGmmMap(x,l,b,p);
            moving=buildTemporalStabilityGmmMap(variable,l,b,p);
            testCase.verifyLessThan(moving.layers.componentRepeatability,agreeing.layers.componentRepeatability/2);
            testCase.verifyFalse(moving.layers.componentPublished);
        end
        function referenceMassDoesNotGrowWithRepeatCount(testCase)
            [p,x,l,b]=testCase.fixture(2); [~,moreX,moreL,moreB]=testCase.fixture(4);
            two=buildTemporalStabilityGmmMap(x,l,b,p); four=buildTemporalStabilityGmmMap(moreX,moreL,moreB,p);
            testCase.verifyEqual(two.layers.referenceMass,four.layers.referenceMass,'AbsTol',0);
            testCase.verifyGreaterThan(four.layers.componentRepeatability,two.layers.componentRepeatability);
        end
        function splitMassLeavesQueriesAndExportedDensityUnchanged(testCase)
            [p,x,l,b]=testCase.fixture(3); map=buildTemporalStabilityGmmMap(x,l,b,p);
            split=testCase.splitComponent(map); points=[0 0;1 0;2 0.4];
            original=queryTemporalStabilityGmmMap(map,points,"stable");
            divided=queryTemporalStabilityGmmMap(split,points,"stable");
            cloud=temporalMapToProbabilityCloud(split);
            testCase.verifyEqual(divided,original,'AbsTol',1e-14);
            testCase.verifyEqual(cloud.totalMass,map.layers.totalMass,'AbsTol',1e-14);
            testCase.verifyEqual(testCase.cloudScores(cloud,points),original,'AbsTol',1e-14);
        end
        function queryAndCloudUseSameFieldAndClutter(testCase)
            [p,x,l,b]=testCase.fixture(3); map=buildTemporalStabilityGmmMap(x,l,b,p);
            points=[0.3 0;1 0.1;3 1]; cloud=temporalMapToProbabilityCloud(map);
            [score,d]=queryTemporalStabilityGmmMap(map,points,"stable");
            testCase.verifyEqual(score,testCase.cloudScores(cloud,points),'AbsTol',1e-14);
            testCase.verifyEqual(score,d.intensity./(d.intensity+cloud.clutterIntensity),'AbsTol',1e-14);
            testCase.verifyEqual(cloud.queryRelationship,"exactIntensityNormalization");
        end
        function indexRespectsDeclaredErrorAndDetectsStaleMass(testCase)
            [p,x,l,b]=testCase.fixture(3); map=buildTemporalStabilityGmmMap(x,l,b,p);
            approximate=map; approximate.layers.params.queryIntensityTolerance=0.01;
            approximate.layers.queryIndex=mappingSupport.compileFieldIndex(approximate.layers);
            points=[linspace(-3.9,3.9,100)',linspace(-3.9,3.9,100)'];
            [exact,a]=queryTemporalStabilityGmmMap(map,points,"stable");
            [score,d]=queryTemporalStabilityGmmMap(approximate,points,"stable");
            map.layers.componentMasses=map.layers.componentMasses*2;
            testCase.verifyLessThanOrEqual(max(abs(a.intensity-d.intensity)),0.01+1e-12);
            testCase.verifyLessThanOrEqual(abs(score-exact),d.scoreErrorBound+1e-12);
            testCase.verifyError(@() queryTemporalStabilityGmmMap(map,points,"stable"),'VehicleLocalization:StaleFieldIndex');
        end
        function coverageAndUnknownClassesAreExplicit(testCase)
            [p,x,l,b]=testCase.fixture(3); map=buildTemporalStabilityGmmMap(x,l,b,p);
            [score,d]=queryTemporalStabilityGmmMap(map,[100 100;1 0],["stable";"missing"]);
            testCase.verifyTrue(all(isnan(score)));
            testCase.verifyFalse(any(d.valid));
            testCase.verifyEqual(d.status,["outsideCoverage";"unknownClass"]);
        end
        function emptyLayersAndQueriesAreValidStates(testCase)
            p=temporalStabilityMapConfig(); p.classes="empty";
            map=buildTemporalStabilityGmmMap(zeros(0,2),strings(0,1),zeros(0,1),p);
            cloud=temporalMapToProbabilityCloud(map);
            [scores,d]=queryTemporalStabilityGmmMap(map,zeros(0,2));
            testCase.verifySize(scores,[0 1]); testCase.verifySize(d.valid,[0 1]);
            testCase.verifyEqual(cloud.totalMass,0,'AbsTol',0);
            testCase.verifyEqual(cloud.components.numComponents,0);
        end
        function heightMarginalAndMassArePreserved(testCase)
            [p,x,l,b]=testCase.fixture(3); xy=buildTemporalStabilityGmmMap(x,l,b,p);
            xyz=buildTemporalStabilityGmmMap([x,20+0.3*x(:,1)],l,b,p);
            cloud=temporalMapToProbabilityCloud(xyz);
            testCase.verifyEqual(xyz.layers.componentCovariances,xy.layers.componentCovariances,'AbsTol',0);
            testCase.verifyEqual(cloud.components.covarianceXYZ(1:2,1:2,:),cloud.components.covariance,'AbsTol',0);
            testCase.verifyEqual(cloud.totalMass,xy.layers.totalMass,'AbsTol',0);
            testCase.verifyEqual(cloud.components.meanXYZ(:,3),20+0.3*cloud.components.mean(:,1),'AbsTol',1e-10);
        end
        function missingAndMultimodalHeightAreNotInvented(testCase)
            [p,x,l,b]=testCase.fixture(3);
            missing=buildTemporalStabilityGmmMap([x,nan(size(b))],l,b,p);
            modes=buildTemporalStabilityGmmMap([x,zeros(size(b));x,3*ones(size(b))],[l;l],[b;b],p);
            testCase.verifyFalse(missing.layers.componentHeightAvailable);
            testCase.verifyFalse(modes.layers.componentHeightAvailable);
            testCase.verifyEqual(modes.layers.components.heightStatus,"inadequateMultipleModes");
        end
        function heldOutBlocksCanSelectTwoStructures(testCase)
            [p,x,l,b]=testCase.fixture(6); p.defaultParams.maxComponentsPerTile=2;
            x(:,1)=x(:,1)-1; x=[x+[-1.4 0];x+[1.4 0]];
            map=buildTemporalStabilityGmmMap(x,[l;l],[b;b],p);
            selection=map.layers.tiles.selection;
            testCase.verifyEqual(selection.status,"heldOutBlockPredictiveCompositeLogScore");
            testCase.verifyNotEmpty(selection.heldOutBlocks);
            testCase.verifyEqual(selection.selectedComponentCount,2);
            testCase.verifyEqual(sum(map.layers.componentReferenceMasses)+map.layers.backgroundReferenceMass, ...
                map.layers.referenceMass,'AbsTol',1e-12);
        end
        function strideChangesSchedulingOnlyAndTailIsFull(testCase)
            [data,p]=testCase.windowFixture(); first=buildSlidingWindowMap(data,p);
            p.batchFrameStride=2; second=buildSlidingWindowMap(data,p);
            testCase.verifyEqual(first.canonicalMap.layers.componentMeans,second.canonicalMap.layers.componentMeans,'AbsTol',0);
            testCase.verifyEqual(first.canonicalMap.layers.componentMasses,second.canonicalMap.layers.componentMasses,'AbsTol',0);
            testCase.verifyEqual(cellfun(@numel,first.batchFrameWindows),3*ones(3,1));
            testCase.verifyEqual([first.batchMaps.ingestedFrameIndices],1:5);
            testCase.verifyEqual(temporalMapToProbabilityCloud(first,1).totalMass, ...
                temporalMapToProbabilityCloud(first,3).totalMass,'AbsTol',0);
        end
        function invalidStrideAndConflictingProvenanceFail(testCase)
            [data,p]=testCase.windowFixture(); p.batchFrameStride=4;
            [cfg,x,l,b]=testCase.fixture(2);
            obs=struct('frameId',b,'observationBlockId',b,'sourceId',ones(size(b)));
            testCase.verifyError(@() buildSlidingWindowMap(data,p),'buildSlidingWindowMap:UncoveredFrames');
            testCase.verifyError(@() buildTemporalStabilityGmmMap(x,l,obs,cfg),'buildTemporalStabilityGmmMap:ConflictingSource');
        end
    end
    methods (Test)
        function sharedLoggingKeepsStageSwitchesDistinct(testCase)
            cfg = struct('stepLogsEnabled', true, 'logEnabled', false);
            testCase.verifyTrue(mappingSupport.isLogEnabled(cfg));
            testCase.verifyFalse(mappingSupport.isLogEnabled(cfg, "logEnabled"));
            testCase.verifyFalse(mappingSupport.isLogEnabled(struct()));
            testCase.verifyFalse(mappingSupport.isLogEnabled(struct('stepLogsEnabled', [true false])));
            testCase.verifyEqual(string(mappingSupport.formatFrameIndexSet(260:289)), "260:289");
            testCase.verifyEqual(mappingSupport.formatFrameIndexSet([260 300 326]), "[260 300 326]");
        end

        function sharedSummaryPreservesFiniteAndEmptyResults(testCase)
            [minimum, medianValue, maximum] = mappingSupport.finiteSummary([NaN 4 1 Inf 2]);
            testCase.verifyEqual([minimum medianValue maximum], [1 2 4], AbsTol=1.0e-12);
            [minimum, medianValue, maximum] = mappingSupport.finiteSummary([NaN Inf]);
            testCase.verifyTrue(all(isnan([minimum medianValue maximum])));
        end

        function sharedCovarianceChecksPreserveErrorIdentifiers(testCase)
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(nan(2), "fixture"), ...
                "buildTemporalStabilityGmmMap:InvalidCovarianceMatrix");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix([1 1; 0 1], "fixture"), ...
                "buildTemporalStabilityGmmMap:NonSymmetricCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(diag([1 -1]), "fixture"), ...
                "buildTemporalStabilityGmmMap:NonPositiveDefiniteCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(nan(2), "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:InvalidCovarianceMatrix");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix([1 1; 0 1], "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:NonSymmetricCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(diag([1 -1]), "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:NonPositiveDefiniteCovariance");
        end

        function sharedUnitChecksPreserveToleranceAndErrorIdentifiers(testCase)
            testCase.verifyEqual(mappingSupport.clipUnit([-1.0e-13 0.5 1+1.0e-13]), ...
                [0 0.5 1], AbsTol=1.0e-15);
            testCase.verifyEqual(mappingSupport.clipUnit([-1.0e-13 0.5 1+1.0e-13], ...
                "queryTemporalStabilityGmmMap"), [0 0.5 1], AbsTol=1.0e-15);
            testCase.verifyError(@() mappingSupport.clipUnit(NaN), ...
                "buildTemporalStabilityGmmMap:InvalidUnitValue");
            testCase.verifyError(@() mappingSupport.clipUnit(1+1.0e-6), ...
                "buildTemporalStabilityGmmMap:UnitValueOutOfRange");
            testCase.verifyError(@() mappingSupport.clipUnit(NaN, "queryTemporalStabilityGmmMap"), ...
                "queryTemporalStabilityGmmMap:InvalidUnitValue");
            testCase.verifyError(@() mappingSupport.clipUnit(-1.0e-6, "queryTemporalStabilityGmmMap"), ...
                "queryTemporalStabilityGmmMap:UnitValueOutOfRange");
        end
    end

    methods (Static, Access=private)
        function [p,x,l,b]=fixture(B)
            p=temporalStabilityMapConfig(); p.classes="stable"; p.classParams=p.classParams([]);
            p.defaultParams.maxComponentsPerTile=1; p.defaultParams.queryIntensityTolerance=0;
            x=[repmat((0:0.5:2)',B,1),zeros(5*B,1)];
            l=repmat("stable",5*B,1); b=repelem((1:B)',5);
        end
        function probability=stackedProbability(layer,p)
            tile=layer.tiles; c=layer.components;
            x=layer.representativePoints(tile.representativeIndices,:);
            [~,~,b]=unique(layer.representativeBlockIds(tile.representativeIndices));
            w=tile.fit.responsibilities(:,2); n=accumarray(b,w); B=numel(n);
            means=[accumarray(b,w.*x(:,1))./n,accumarray(b,w.*x(:,2))./n];
            y=reshape(means.',[],1); m0=(tile.coverage(1:2)+tile.coverage(3:4))/2;
            center=repmat(m0.',B,1); stable=zeros(2*B); variable=stable;
            Qs=c.stableOffsetCovariance;
            [v,e]=eig(c.withinBlockCovariance,'vector'); [e,order]=sort(e);
            P=eye(2);
            if e(2)/e(1)>=p.elongatedAnisotropyThreshold, normal=v(:,order(1)); P=normal*normal.'; end
            for j=1:B
                rows=2*j-1:2*j; stable(rows,rows)=c.withinBlockCovariance/n(j)+Qs;
                variable(rows,rows)=stable(rows,rows)+p.variableStandardDeviation^2*P;
            end
            prior=kron(ones(B),p.meanPriorStandardDeviation^2*eye(2));
            stable=stable+prior; variable=variable+prior; delta=y-center;
            logOdds=log(p.repeatabilityPrior/(1-p.repeatabilityPrior))- ...
                sum(log(diag(chol(stable))))+sum(log(diag(chol(variable))))- ...
                0.5*(delta.'/stable*delta-delta.'/variable*delta);
            probability=1/(1+exp(-logOdds));
        end
        function score=cloudScores(cloud,points)
            intensity=zeros(size(points,1),1);
            for k=1:cloud.components.numComponents
                C=cloud.components.covariance(:,:,k); delta=points-cloud.components.mean(k,:);
                intensity=intensity+cloud.totalMass*cloud.components.mixtureWeight(k)/ ...
                    (2*pi*sqrt(det(C)))*exp(-0.5*sum((delta/C).*delta,2));
            end
            score=intensity./(intensity+cloud.clutterIntensity);
        end
        function map=splitComponent(map)
            layer=map.layers; c=layer.components;
            c.mass=c.mass/2; c.referenceMass=c.referenceMass/2; c.referenceCellMass=c.referenceCellMass/2;
            other=c; other.id=c.id+"split"; layer.components=[c;other];
            layer.componentIds=[c.id;other.id]; layer.componentMeans=repmat(layer.componentMeans,2,1);
            layer.componentCovariances=repmat(layer.componentCovariances,1,1,2);
            layer.componentMeansXYZ=repmat(layer.componentMeansXYZ,2,1);
            layer.componentCovariancesXYZ=repmat(layer.componentCovariancesXYZ,1,1,2);
            layer.componentMasses=repmat(c.mass,2,1); layer.componentReferenceMasses=repmat(c.referenceMass,2,1);
            layer.componentRepeatability=repmat(c.repeatability,2,1); layer.componentPublished=true(2,1);
            layer.componentHeightAvailable=false(2,1); layer.componentMixtureWeights=[0.5;0.5];
            layer.queryIndex=mappingSupport.compileFieldIndex(layer); map.layers=layer;
        end
        function [data,p]=windowFixture()
            [cfg,x,~,~]=temporalStabilityGmmMapTest.fixture(1);
            p=featureMapBuildConfig(); p.temporalMap=cfg; p.batchFrameCount=3; p.batchFrameStride=1; p.logEnabled=false;
            data=struct('frameIndices',1:5,'featureNames',"stable", ...
                'pointsByFeatureFrame',{repmat({[x,zeros(5,1)]},1,5)});
        end
    end
end
