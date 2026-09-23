classdef robustPoseGraphTest < matlab.unittest.TestCase
% robustPoseGraphTest Unique evidence, marginalization and robust associations.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function exactGeometryAndMotionRemainExact(testCase)
            [r,s]=sequence("switchable",false,5,false);
            testCase.verifyEqual(r.poseXYTheta,[0 0 0],AbsTol=1e-10);
            testCase.verifyEqual(s.totalAcquisitions,8);
            testCase.verifyEqual(s.totalObservations,48);
            testCase.verifyEqual(numel(s.time),5);
        end
        function switchedFactorsLimitPersistentFalseFeature(testCase)
            robust=sequence("switchable",true,5,false);
            plain=sequence("quadratic",true,5,false);
            testCase.verifyLessThan(norm(robust.poseXYTheta(1:2)),norm(plain.poseXYTheta(1:2)));
            testCase.verifyLessThan(min(robust.correspondences.graphInfluence),.2);
        end
        function marginalizationDoesNotRecountRetainedFactors(testCase)
            full=sequence("quadratic",false,20,true);
            bounded=sequence("quadratic",false,3,true);
            testCase.verifyEqual(bounded.poseXYTheta,full.poseXYTheta,AbsTol=1e-8);
            testCase.verifyEqual(bounded.information,full.information,RelTol=1e-8);
        end
        function overlappingStacksCannotBecomeGraphFactors(testCase)
            [packet,map,cfg]=fixture();packet.source.observationScope="stackedHorizon";
            testCase.verifyError(@()updateRobustPoseGraph([],packet,map,cfg), ...
                'VehicleLocalization:GraphNeedsCurrentScan');
        end
        function duplicateAcquisitionIsRejected(testCase)
            [packet,map,cfg]=fixture();[~,s]=updateRobustPoseGraph([],packet,map,cfg);
            testCase.verifyError(@()updateRobustPoseGraph(s,packet,map,cfg), ...
                'VehicleLocalization:DuplicateGraphAcquisition');
        end
        function outputCannotBeMistakenForIndependentLidar(testCase)
            r=sequence("switchable",false,5,false);
            testCase.verifyTrue(r.containsGnss);
            testCase.verifyFalse(r.independentLidarMeasurement);
            testCase.verifyFalse(r.informationCalibrated);
            testCase.verifyEqual(r.measurementType,"fusedGraphPose");
            testCase.verifyError(@()registrationSupport.registrationPoseMeasurement(r,1), ...
                'VehicleLocalization:FusedPoseIsNotLidar');
        end
        function withoutGnssGeometryStillLocalizes(testCase)
            [packet,map,cfg]=fixture();packet.positionAid.valid=false;
            [~,s]=updateRobustPoseGraph([],packet,map,cfg);
            packet.time=.1;packet.source.acquisitionTime=.1;packet.seed=[.1 .1 .01];
            [r,~]=updateRobustPoseGraph(s,packet,map,cfg);
            testCase.verifyLessThan(norm(r.poseXYTheta),1e-4);
            testCase.verifyFalse(r.containsGnss);
        end
        function currentObservationsAreNotPooledMeans(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            shifted=distributionRegistrationTest.transform(cloud,[.2 0 0]);
            [pooled,~,~,current]=updateLocalizationSourceWindow(shifted,.1,[0 0 0],h);
            testCase.verifyEqual(current.components.mean,shifted.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(pooled.components.mean,cloud.components.mean+[.1 0],AbsTol=1e-12);
            testCase.verifyEqual(current.components.temporalStability,.4*ones(6,1),AbsTol=1e-12);
        end
        function missedCurrentScanCannotDuplicatePastObservations(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            [~,h]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,h]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],h);
            [pooled,~,~,current]=updateLocalizationSourceWindow(localizationSourceWindowTest.empty(cloud),.2,[0 0 0],h);
            testCase.verifyEqual(pooled.components.numComponents,6);
            testCase.verifyEqual(current.components.numComponents,0);
        end
        function curvedMotionAndMapGeometryAgree(testCase)
            [actual,truth]=turning([0 0]);
            testCase.verifyEqual(actual.poseXYTheta,truth,AbsTol=1e-7);
        end
        function largeMapOriginDoesNotChangeTheEstimate(testCase)
            [a,~]=turning([0 0]);[b,~]=turning([480000 4900000]);
            testCase.verifyEqual(b.poseXYTheta-[480000 4900000 0],a.poseXYTheta,AbsTol=1e-7);
            testCase.verifyEqual(b.information,a.information,RelTol=1e-7);
        end
    end
end

function [result,truth]=turning(origin)
    [packet,map,cfg]=fixture();map.components.mean=map.components.mean+origin;
    truth=[origin 0];state=[];
    for k=1:12
        R=[cos(truth(3)) -sin(truth(3));sin(truth(3)) cos(truth(3))];
        if k>1,truth=truth+[([.4 .02]*R.'),.04];end
        R=[cos(truth(3)) -sin(truth(3));sin(truth(3)) cos(truth(3))];
        source=distributionRegistrationTest.transform(map,[-truth(1:2)*R,-truth(3)]);
        source.observationScope="confirmedCurrentAcquisition";source.acquisitionTime=.1*(k-1);
        packet.source=source;packet.time=source.acquisitionTime;packet.seed=truth;
        packet.relativeMotion=[.4 .02 .04];packet.positionAid.position=truth(1:2);packet.positionAid.timestamp=packet.time;
        [result,state]=updateRobustPoseGraph(state,packet,map,cfg);
    end
end

function [packet,map,cfg]=fixture()
    map=distributionRegistrationTest.exampleCloud();cloud=map;
    cloud.observationScope="confirmedCurrentAcquisition";cloud.acquisitionTime=0;
    packet=struct('time',0,'seed',[0 0 0],'source',cloud,'relativeMotion',[0 0 0], ...
        'motionCovariance',diag([.04 .04 .003].^2), ...
        'positionAid',struct('valid',true,'timestamp',0,'position',[0 0],'covariance',.04*eye(2)));
    cfg=robustPoseGraphConfig();
end

function [result,state]=sequence(loss,outlier,maximumFrames,empty)
    [packet,map,cfg]=fixture();cfg.featureLoss=loss;cfg.maximumFrames=maximumFrames;state=[];
    if empty,packet.source=localizationSourceWindowTest.empty(packet.source);end
    for k=1:8
        packet.time=(k-1)*.1;packet.source.acquisitionTime=packet.time;packet.positionAid.timestamp=packet.time;
        if outlier && k>=3,packet.source.components.mean(2,:)=[0 1.5];end
        [result,state]=updateRobustPoseGraph(state,packet,map,cfg);packet.seed=result.poseXYTheta;
    end
end
