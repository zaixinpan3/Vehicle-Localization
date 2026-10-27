classdef (SharedTestFixtures={freshObserverGainsFixture}) stationaryLocalizationObserverTest < matlab.unittest.TestCase
% Stationary LiDAR correction and local pose support with fresh MnCAV gains.
    properties (TestParameter)
        supportCase=struct( ...
            'full',struct('information',diag([1e6,2e6,3e6]),'rank',3,'heading',true), ...
            'coupled',struct('information',[1,0,1;0,2,0;1,0,1]*1e6,'rank',2,'heading',false), ...
            'tinyCoupling',struct('information',[1e-24,0,1e-12;0,2,0;1e-12,0,1]*1e6,'rank',2,'heading',false), ...
            'headingOnly',struct('information',diag([0,0,1e6]),'rank',1,'heading',true), ...
            'positionOnly',struct('information',diag([1e6,2e6,0]),'rank',2,'heading',false), ...
            'zero',struct('information',zeros(3),'rank',0,'heading',false), ...
            'largeRawScale',struct('information',diag([1,1e15,2e15]),'rank',2,'heading',true));
        invalidReliability=struct('nan',NaN,'negative',-.1,'aboveOne',1.1,'complex',1i);
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function stationaryRecordedPoseConvergesWithoutLateralAiding(testCase)
            f=fixture();r=execute(f);s=r.diagnostics.lidarPoseSupport;
            testCase.verifyLessThan(norm(r.position(end,:)),1e-3);
            testCase.verifyLessThan(abs(r.heading(end)),1e-4);
            testCase.verifyTrue(all(s.sectorQualified));
            testCase.verifyFalse(any(r.diagnostics.motionOutputValidity.bodyVelocity));
            testCase.verifyFalse(s.correctionApplied(1));
            testCase.verifyTrue(all(s.correctionApplied(2:end)));
            testCase.verifyEqual(r.internalState(1,:),f.cfg.initialState.',AbsTol=0);
            testCase.verifyFalse(r.diagnostics.zeroSpeedCertified);
            testCase.verifyFalse(any(r.diagnostics.observedSpeedWithinIssDomain));
            testCase.verifyFalse(r.diagnostics.observedMotionEnvelopeSatisfied);
        end
        function gnssPositionAndGyroLeaveStationaryYawOffset(testCase)
            f=fixture();f.data=rmfield(f.data,'lidar');t=f.data.highRate.time;n=numel(t);
            yaw=f.cfg.initialState(7);R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
            receiver=(R*f.cfg.gnss.outputPoint.bodyOffset(:)).';
            f.data.gnss=struct('time',t,'position',repmat(receiver,n,1),'information',repmat(1e6*eye(2),1,1,n), ...
                'valid',true(n,1),'delay',0);
            f.cfg.initialState([1,4])=0;r=execute(f);
            testCase.verifyEqual(r.headingUnwrapped,repmat(f.cfg.initialState(7),n,1),AbsTol=1e-12);
            testCase.verifyFalse(any(r.diagnostics.lidarPoseSupport.headingSupported));
            testCase.verifyFalse(any(r.diagnostics.lidarPoseSupport.sourceValid));
        end
        function supportRespectsCouplingAndRawAdmission(testCase,supportCase)
            cfg=mncavFullObserverConfig();
            filter=informationFilter(supportCase.information,cfg);
            s=fullObserverSupport.lidarPoseSupport(filter,cfg);
            testCase.verifyEqual(s.rank,supportCase.rank);
            testCase.verifyEqual(s.fullPoseSupported,supportCase.rank==3);
            testCase.verifyEqual(s.headingSupported,supportCase.heading);
            testCase.verifyLessThanOrEqual(s.rank,filter.rank);
        end
        function coupledYawDiagonalIsNotIndependentHeading(testCase)
            f=fixture();D=diag(f.cfg.lidar.poseScales);
            f.data.lidar.information=repmat(D\[1e6,0,1e6;0,2e6,0;1e6,0,1e6]/D,1,1,numel(f.lateral.time));
            r=execute(f);s=r.diagnostics.lidarPoseSupport;
            testCase.verifyGreaterThan(min(squeeze(r.diagnostics.poseWeight(3,3,:))),0);
            testCase.verifyFalse(any(s.headingSupported));
            testCase.verifyFalse(any(s.sectorQualified));
            testCase.verifyLessThan(max(s.headingStrengthAfterPosition),1e-20);
        end
        function headingStrengthMatchesMarginalInformation(testCase)
            cfg=mncavFullObserverConfig();I=[8,2,3;2,7,-1;3,-1,6]*1e3;
            filter=informationFilter(I,cfg);S=filter.S;s=fullObserverSupport.lidarPoseSupport(filter,cfg);
            expected=S(3,3)-S(3,1:2)*(S(1:2,1:2)\S(1:2,3));
            testCase.verifyEqual(s.headingStrengthAfterPosition,expected,AbsTol=1e-12);
            testCase.verifyGreaterThan(s.minimumStrength,0);
        end
        function numericallyFullPoseCanFailConfiguredSector(testCase)
            cfg=mncavFullObserverConfig();filter=informationFilter(eye(3),cfg);
            s=fullObserverSupport.lidarPoseSupport(filter,cfg);
            testCase.verifyTrue(s.fullPoseSupported);
            testCase.verifyFalse(s.sectorQualified);
        end
        function directionalReliabilityWithdrawsHeading(testCase)
            f=fixture();f.data.lidar.directionReliability=repmat([1,1,0],numel(f.lateral.time),1);
            r=execute(f);s=r.diagnostics.lidarPoseSupport;
            testCase.verifyEqual(s.rank,2*ones(size(r.time)));
            testCase.verifyFalse(any(s.headingSupported));
            testCase.verifyEqual(r.headingUnwrapped,repmat(f.cfg.initialState(7),numel(r.time),1),AbsTol=1e-12);
        end
        function zeroReliabilityEqualsMissingRecordedLidar(testCase)
            f=fixture();f.data.lidar.frameReliability=zeros(size(f.lateral.time));a=execute(f);
            f.data=rmfield(f.data,'lidar');b=execute(f);s=a.diagnostics.lidarPoseSupport;
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyTrue(all(s.sourceValid));
            testCase.verifyFalse(any(s.fullPoseSupported | s.headingSupported | s.correctionApplied));
        end
        function unavailableFramesSkipInvalidPayloads(testCase)
            f=fixture();f.data.lidar.valid(:)=false;f.data.lidar.pose(:)=NaN;
            f.data.lidar.information(:)=NaN;r=execute(f);
            testCase.verifyFalse(any(r.diagnostics.lidarPoseSupport.sourceValid));
            testCase.verifyTrue(all(isfinite(r.internalState),'all'));
        end
        function reliabilityZeroStillValidatesAvailablePayload(testCase)
            f=fixture();f.data.lidar.frameReliability=zeros(size(f.lateral.time));f.data.lidar.pose(:)=NaN;
            testCase.verifyError(@()execute(f),'VehicleLocalization:InvalidFullSource');
        end
        function yawInnovationCrossesTheShortWrappedArc(testCase)
            f=fixture();f.cfg.initialState([1,4])=0;f.cfg.initialState(7)=pi-.02;
            f.data.lidar.pose(:,3)=-pi+.02;r=execute(f);
            testCase.verifyGreaterThan(r.headingUnwrapped(2),f.cfg.initialState(7));
            testCase.verifyLessThan(abs(r.headingUnwrapped(end)-(pi+.02)),1e-4);
        end
        function overlapConvergesAtStandstillWithoutLateralAiding(testCase)
            % Numerical settling example only; zero speed has no ISS claim.
            % Re-synthesis changes the response time after speed substitution
            % is removed, so retain the accuracy goal over a longer horizon.
            f=overlapFixture(30);r=execute(f);
            testCase.verifyTrue(all(r.diagnostics.lidarOverlap.available));
            testCase.verifyLessThan(norm(r.position(end,:)),.01);
            testCase.verifyLessThan(abs(r.heading(end)),deg2rad(.05));
            testCase.verifyTrue(all(r.diagnostics.lidarPoseSupport.fullPoseSupported));
            testCase.verifyTrue(all(isfinite(r.internalState),'all'));
            testCase.verifyFalse(r.diagnostics.zeroSpeedCertified);
            fprintf('STATIONARY OVERLAP t=30 position=%.12g headingDeg=%.12g\n', ...
                norm(r.position(end,:)),abs(rad2deg(r.heading(end))));
        end
        function overlapReliabilityZeroEqualsMissingLidar(testCase)
            f=overlapFixture();f.data.lidarOverlap.frameReliability=zeros(size(f.lateral.time));a=execute(f);
            f.data=rmfield(f.data,'lidarOverlap');b=execute(f);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyTrue(all(a.diagnostics.lidarPoseSupport.sourceValid));
            testCase.verifyFalse(any(a.diagnostics.lidarPoseSupport.correctionApplied));
        end
        function overlapDirectionRejectionWithdrawsAllSupport(testCase)
            f=overlapFixture();f.data.lidarOverlap.directionReliability=zeros(numel(f.lateral.time),3);
            a=execute(f);f.data=rmfield(f.data,'lidarOverlap');b=execute(f);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyEqual(a.diagnostics.lidarPoseSupport.rank,zeros(size(a.time)));
        end
        function overlapRejectsInvalidReliability(testCase,invalidReliability)
            f=overlapFixture();f.data.lidarOverlap.frameReliability=repmat(invalidReliability,size(f.lateral.time));
            testCase.verifyError(@()execute(f),'VehicleLocalization:InvalidLidarReliability');
        end
        function overlapRejectsMalformedDirectionReliability(testCase)
            f=overlapFixture();f.data.lidarOverlap.directionReliability=ones(numel(f.lateral.time),2);
            testCase.verifyError(@()execute(f),'VehicleLocalization:InvalidLidarReliability');
        end
        function overlapRejectsMismatchedClock(testCase)
            f=overlapFixture();f.data.lidarOverlap.time=f.lateral.time+.01;
            testCase.verifyError(@()execute(f),'VehicleLocalization:SynchronousInputsRequired');
        end
        function overlapRejectsNonzeroDelay(testCase)
            f=overlapFixture();f.data.lidarOverlap.delay=.1;
            testCase.verifyError(@()execute(f),'VehicleLocalization:FullZeroDelayRequired');
        end
        function declaredAlignedOverlapClockIsAccepted(testCase)
            f=overlapFixture();a=execute(f);f.data.lidarOverlap.time=f.lateral.time;
            f.data.lidarOverlap.delay=0;b=execute(f);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
        end
        function synchronizationCropsRecordedQualityAndCalibration(testCase)
            f=croppedFixture(false);[d,l]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(d.lidar.frameReliability,[.2;.4;.6],AbsTol=0);
            testCase.verifyEqual(d.lidar.directionReliability,repmat([.2,.5,.9],3,1),AbsTol=0);
            testCase.verifyEqual(d.lidar.poseErrorVariance,[2,3;3,4;4,5],AbsTol=0);
            testCase.verifyFalse(any(l.velocityValid));
            r=runFullLocalizationObserver(d,struct(),f.cfg,LateralInputs=l);
            testCase.verifyTrue(all(isfinite(r.internalState),'all'));
        end
        function synchronizationCropsOverlapQualityAndSources(testCase)
            f=croppedFixture(true);[d,l]=fullObserverSupport.synchronizeLocalizationInputs(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(d.lidarOverlap.frameReliability,[.2;.4;.6],AbsTol=0);
            testCase.verifyEqual(d.lidarOverlap.directionReliability,repmat([.2,.5,.9],3,1),AbsTol=0);
            testCase.verifyEqual(numel(d.lidarOverlap.sources),3);
            testCase.verifyEqual(d.lidarOverlap.time,[0;.1;.2],AbsTol=0);
            r=runFullLocalizationObserver(d,struct(),f.cfg,LateralInputs=l);
            testCase.verifyTrue(all(isfinite(r.internalState),'all'));
        end
    end
end

function filter=informationFilter(normalized,cfg)
    D=diag(cfg.lidar.poseScales);
    filter=lidarInjectionSupport.filterLidarPoseInformation(D\normalized/D,cfg);
end

function f=fixture(duration)
    arguments
        duration (1,1) double=8
    end
    t=(0:.1:duration).';n=numel(t);zero=zeros(n,1);
    h=struct('time',t,'longitudinalSpeed',zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    lateral=struct('time',t,'lateralVelocity',nan(n,1),'sideSlipAngle',nan(n,1), ...
        'sideSlipAngleRate',nan(n,1),'velocityValid',false(n,1));
    cfg=freshObserverTestConfig();cfg.initialState=[.4;0;0;-.3;0;0;deg2rad(2);0];
    D=diag(cfg.lidar.poseScales);I=D\diag([1e6,1.5e6,2e6])/D;
    lidar=struct('time',t,'pose',zeros(n,3),'information',repmat(I,1,1,n),'valid',true(n,1),'delay',0);
    f=struct('data',struct('highRate',h,'lidar',lidar),'lateral',lateral,'cfg',cfg);
end

function r=execute(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end



function f=overlapFixture(duration)
    arguments
        duration (1,1) double=8
    end
    f=fixture(duration);f.data=rmfield(f.data,'lidar');
    means=[-6,-3;0,-3.2;6,-3;-2,4;3,4.5;7,2];
    covariance=cat(3,[2.5,0;0,.03],[2.5,0;0,.03],[2.5,0;0,.03], ...
        .04*eye(2),[.05,.01;.01,.04],[.2,.05;.05,.03]);
    c=struct('mean',means,'covariance',covariance, ...
        'semanticName',["curb";"curb";"curb";"pole";"pole";"trafficSign"], ...
        'mixtureWeight',[.2;.2;.2;.15;.15;.1],'repeatability',ones(6,1),'numComponents',6);
    cloud=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
    f.data.lidarOverlap=struct('map',cloud,'sources',{repmat({cloud},numel(f.lateral.time),1)});
end

function f=croppedFixture(overlap)
    f=fixture();t=(0:.01:.2).';n=numel(t);zero=zeros(n,1);
    f.data.highRate=struct('time',t,'longitudinalSpeed',zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    f.lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngle',zero, ...
        'sideSlipAngleRate',zero,'velocityValid',false(n,1));
    time=[-.1;0;.1;.2;.3];frame=[0;.2;.4;.6;.8];direction=repmat([.2,.5,.9],5,1);
    if overlap
        template=overlapFixture();channel=template.data.lidarOverlap;
        channel.sources=channel.sources(1:5);f.data=rmfield(f.data,'lidar');
        channel.time=time;channel.frameReliability=frame;channel.directionReliability=direction;
        f.data.lidarOverlap=channel;
    else
        channel=f.data.lidar;channel.time=time;channel.pose=channel.pose(1:5,:);
        channel.valid=channel.valid(1:5);channel.information=channel.information(:,:,1:5);
        channel.frameReliability=frame;channel.directionReliability=direction;
        channel.poseErrorVariance=[1,2;2,3;3,4;4,5;5,6];f.data.lidar=channel;
    end
end
