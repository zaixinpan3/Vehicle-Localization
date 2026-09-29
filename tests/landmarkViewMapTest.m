classdef landmarkViewMapTest < matlab.unittest.TestCase
% landmarkViewMapTest Conditional geometry, support, and offline/online separation.
    properties (TestParameter)
        invalidScale={"bandwidth","coverageScale","varianceFloor","maximumAssignmentDistance"}
    end
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function retainsBetweenAcquisitionScatter(t)
            cloud=landmarkViewMapTest.example();
            [actual,details]=conditionSemanticMapOnView(cloud,[0 0 0]);
            t.verifyTrue(details.enabled);
            t.verifyEqual(actual.components.mean(1,:),[.2 0],AbsTol=1e-14);
            t.verifyEqual(actual.components.covariance(:,:,1),diag([.05 .02]),AbsTol=1e-14);
            t.verifyEqual(actual.components.viewReliability(1),exp(-.5*4/25),AbsTol=1e-14);
            t.verifyEqual(actual.components.mean(2,:),cloud.components.mean(2,:));
            t.verifyEqual(actual.components.covariance(:,:,2),cloud.components.covariance(:,:,2));
            t.verifyEqual(actual.components.mixtureWeight,cloud.components.mixtureWeight);
        end
        function densityDoesNotReplaceAcquisitionWeight(t)
            cloud=landmarkViewMapTest.example();other=cloud;
            other.landmarkViews.observations{1}(:,9)=[3;30000];
            a=conditionSemanticMapOnView(cloud,[1 0 0]);b=conditionSemanticMapOnView(other,[1 0 0]);
            t.verifyEqual(a.components,b.components);
        end
        function unsupportedDirectionsContributeNoPoseInformation(t)
            source=geometricRegistrationTest.parallelRoad();map=source;
            source=landmarkViewMapTest.appendPole(source);map=landmarkViewMapTest.appendPole(map);
            n=map.components.numComponents;observations=cell(n,1);
            observations{n}=[1000 0 0 3 5 .01 0 .01 4];
            map.landmarkViews=struct('schemaVersion',1,'config',landmarkViewMapConfig(),'observations',{observations});
            r=registerSemanticProbabilityCloud(map,source,[.6 .2 .02]);
            t.verifyTrue(r.directionalAccepted,r.reason);t.verifyFalse(r.accepted);
            t.verifyEqual(r.observableRank,2);
            t.verifyEqual(r.poseXYTheta,[.6 0 0],AbsTol=1e-4);
            t.verifyEqual(r.information*[1;0;0],zeros(3,1),AbsTol=1e-10);
        end
        function unsupportedPointOnlyMapRejectsCleanly(t)
            cloud=distributionRegistrationTest.exampleCloud();n=cloud.components.numComponents;
            cloud.components.semanticName(:)="pole";
            cloud.landmarkViews=struct('schemaVersion',1,'config',landmarkViewMapConfig(),'observations',{cell(n,1)});
            source=rmfield(cloud,'landmarkViews');r=registerSemanticProbabilityCloud(cloud,source,[0 0 0]);
            t.verifyFalse(r.accepted||r.directionalAccepted);
            t.verifyEqual(r.information,zeros(3),AbsTol=0);
            t.verifyTrue(all(isfinite(r.poseXYTheta)));
        end
        function coordinateTransformCommutesWithConditioning(t)
            cloud=landmarkViewMapTest.example();pose=[.7 -.1 .05];angle=.8;shift=[7e5 4e6];
            R=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            transformed=landmarkViewMapTest.transform(cloud,R,shift,angle);
            a=conditionSemanticMapOnView(cloud,pose);
            b=conditionSemanticMapOnView(transformed,[pose(1:2)*R.'+shift,pose(3)+angle]);
            t.verifyEqual(b.components.mean,a.components.mean*R.'+shift,AbsTol=2e-9);
            t.verifyEqual(b.components.covariance,pagemtimes(pagemtimes(R,a.components.covariance),R.'),AbsTol=1e-10);
            t.verifyEqual(b.components.viewReliability,a.components.viewReliability,AbsTol=1e-10);
        end
        function cropAndProjectionPreserveViewIndices(t)
            cloud=landmarkViewMapTest.example();[local,ids]=selectLocalProbabilityCloud(cloud,[0 0 0],1);
            projected=registrationSupport.projectSemanticProbabilityCloud(local,2);
            t.verifyEqual(ids,1);t.verifyEqual(projected.landmarkViews.observations,cloud.landmarkViews.observations(1));
            actual=conditionSemanticMapOnView(projected,[0 0 0]);
            t.verifyEqual(actual.components.mean,[.2 0],AbsTol=1e-14);
        end
        function sparseCoverageChangesContinuously(t)
            cloud=landmarkViewMapTest.example();a=conditionSemanticMapOnView(cloud,[15-1e-5 0 0]);b=conditionSemanticMapOnView(cloud,[15+1e-5 0 0]);
            t.verifyLessThan(norm(a.components.mean-b.components.mean),1e-5);
            t.verifyLessThan(norm(a.components.viewReliability-b.components.viewReliability),1e-5);
            t.verifyGreaterThan(min(a.components.viewReliability),0);
        end
        function oppositeHeadingHasNoSupportedView(t)
            actual=conditionSemanticMapOnView(landmarkViewMapTest.example(),[0 0 pi]);
            t.verifyEqual(actual.components.viewReliability,[0;1]);
        end
        function legacyCloudIsUnchanged(t)
            cloud=rmfield(landmarkViewMapTest.example(),'landmarkViews');
            [actual,details]=conditionSemanticMapOnView(cloud,[0 0 0]);
            t.verifyEqual(actual,cloud);t.verifyFalse(details.enabled);
        end
        function builderUsesOnlySelectedOfflineAcquisitions(t)
            [base,observations]=landmarkViewMapTest.mappingFixture();
            map=buildViewConditionedLandmarkMap(base,observations,landmarkViewMapConfig(),[1 3]);
            t.verifyEqual(map.landmarkViews.observations{1}(:,1),[-2;2]);
            t.verifyEqual(map.landmarkViews.observations{1}(:,4),[0;.4],AbsTol=1e-14);
            t.verifyEqual(map.mapConstruction.mappingFrameIndices,[10 30]);
            t.verifyFalse(map.mapConstruction.onlineQueryLabelsUsed);
            t.verifyEqual(size(map.landmarkViews.observations{1},2),9);
            t.verifyEqual(map.components.mean(2,:),base.components.mean(2,:));
        end
        function builderRejectsDuplicateAcquisitions(t)
            [base,observations]=landmarkViewMapTest.mappingFixture();
            t.verifyError(@()buildViewConditionedLandmarkMap(base,observations,landmarkViewMapConfig(),[1 1]), ...
                'VehicleLocalization:InvalidMapObservationSelection');
        end
        function builderRejectsMismatchedCalibration(t)
            [base,observations]=landmarkViewMapTest.mappingFixture();observations.frameCalibration.translation(1)=1;
            t.verifyError(@()buildViewConditionedLandmarkMap(base,observations), ...
                'VehicleLocalization:CalibrationMismatch');
        end
        function rejectsInvalidScale(t,invalidScale)
            cfg=landmarkViewMapConfig();cfg.(invalidScale)=NaN;
            t.verifyError(@()validateLandmarkViewConfig(cfg),'VehicleLocalization:InvalidLandmarkViewConfig');
        end
        function rejectsMisalignedViewArrays(t)
            cloud=landmarkViewMapTest.example();cloud.landmarkViews.observations(2)=[];
            t.verifyError(@()conditionSemanticMapOnView(cloud,[0 0 0]),'VehicleLocalization:InvalidLandmarkViews');
        end
        function rejectsUncalibratedSupportOutsideUnitInterval(t)
            cloud=distributionRegistrationTest.exampleCloud();cloud.components.viewReliability=-ones(cloud.components.numComponents,1);
            t.verifyError(@()prepareSemanticRegistrationGeometry(cloud,cloud,[0 0 0],distributionRegistrationConfig()), ...
                'VehicleLocalization:InvalidViewReliability');
        end
        function preventsSecondCanonicalization(t)
            cloud=landmarkViewMapTest.example();
            t.verifyError(@()canonicalizeSemanticCloud(cloud,1.5),'VehicleLocalization:ViewMapAlreadyCanonical');
        end
    end
    methods (Static)
        function cloud=example()
            c=struct('mean',[.2 0;8 3],'covariance',cat(3,.05*eye(2),diag([2 .02])), ...
                'semanticName',["pole";"curb"],'mixtureWeight',[.4;.6],'numComponents',2);
            o={[-2 0 0 0 0 .01 0 .02 4;2 0 0 .4 0 .01 0 .02 4];zeros(0,9)};
            cloud=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig(), ...
                'landmarkViews',struct('schemaVersion',1,'config',landmarkViewMapConfig(),'observations',{o}));
        end
        function cloud=appendPole(cloud)
            c=cloud.components;c.mean(end+1,:)=[3 5];c.covariance(:,:,end+1)=.01*eye(2);
            c.semanticName(end+1)="pole";c.mixtureWeight(end+1)=.2;c.numComponents=c.numComponents+1;
            if isfield(c,'repeatability'),c.repeatability(end+1)=1;end
            cloud.components=c;
        end
        function cloud=transform(cloud,R,shift,angle)
            c=cloud.components;c.mean=c.mean*R.'+shift;c.covariance=pagemtimes(pagemtimes(R,c.covariance),R.');cloud.components=c;
            o=cloud.landmarkViews.observations{1};o(:,1:2)=o(:,1:2)*R.'+shift;o(:,3)=o(:,3)+angle;o(:,4:5)=o(:,4:5)*R.'+shift;
            S=R*diag([.01 .02])*R.';o(:,6:8)=repmat([S(1,1),S(1,2),S(2,2)],size(o,1),1);cloud.landmarkViews.observations{1}=o;
        end
        function [base,f]=mappingFixture()
            base=rmfield(landmarkViewMapTest.example(),'landmarkViews');
            p=[-.1 -.1 0;-.1 .1 1;.1 -.1 2;.1 .1 3];points={p,p+[.9 0 0],p+[.4 0 0];zeros(0,3),zeros(0,3),zeros(0,3)};
            poses=table([10;20;30],[-2;0;2],zeros(3,1),zeros(3,1),90*ones(3,1),zeros(3,1),zeros(3,1), ...
                VariableNames={'frame_index','odom_x_m','odom_y_m','odom_z_m','azimuth_deg','roll_deg','pitch_deg'});
            f=struct('featureNames',["pole";"curb"],'frameIndices',[10 20 30], ...
                'framePoseTable',poses,'pointsByFeatureFrame',{points},'frameCalibration',base.frameCalibration);
        end
    end
end
