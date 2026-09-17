classdef fullLocalizationObserverTest < matlab.unittest.TestCase
% fullLocalizationObserverTest Public dual-source and missing-data contracts.
    properties (TestParameter)
        sourceMode={"both","gnss","lidar","none"}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function exactMotionSurvivesEachAvailabilityMode(testCase,sourceMode)
            f=fixture(4,sourceMode);r=run(f);
            testCase.verifyEqual(r.position,f.truth(:,1:2),AbsTol=1e-9);
            testCase.verifyEqual(r.heading,f.truth(:,3),AbsTol=1e-10);
            testCase.verifyEqual(r.diagnostics.stateResets,0);
        end
        function bothPositionResidualsActOnTheSameState(testCase)
            f=fixture(1,"both");f.data.gnss.position(:,2)=1;f.data.lidar.pose(:,2)=-1;
            r=run(f);c=r.diagnostics.positionCorrection(1,:);
            testCase.verifyGreaterThan(c(2),0);testCase.verifyLessThan(c(4),0);
            testCase.verifyEqual(r.diagnostics.mode,3*ones(size(r.time)));
        end
        function invalidNanPayloadImmediatelyWithdrawsTheSource(testCase)
            f=fixture(2,"both");f.data.gnss.valid(6:end)=false;
            f.data.gnss.position(6:end,:)=NaN;f.data.gnss.information(:,:,6:end)=NaN;
            r=run(f);ix=r.time>=.5;
            testCase.verifyEqual(r.diagnostics.mode(ix),2*ones(nnz(ix),1));
            testCase.verifyEqual(r.diagnostics.positionCorrection(ix,1:2),zeros(nnz(ix),2));
        end
        function absentPacketsExpireWithoutBeingReusedForever(testCase)
            f=fixture(2,"gnss");f.data.gnss=trimSource(f.data.gnss,1);
            r=run(f);ix=r.time>=f.cfg.gnss.maximumAge;
            testCase.verifyEqual(r.diagnostics.mode(ix),zeros(nnz(ix),1));
        end
        function futureMeasurementsCannotChangeThePast(testCase)
            f=fixture(4,"both");a=run(f);
            f.data.gnss.position(f.data.gnss.time>2,:)=100;
            f.data.lidar.pose(f.data.lidar.time>2,:)=100;b=run(f);
            testCase.verifyEqual(a.z(a.time<=2,:),b.z(b.time<=2,:));
        end
        function oppositeSourcesCanAlternateEveryFrame(testCase)
            f=fixture(4,"both");f.data.gnss.valid(2:2:end)=false;f.data.lidar.valid(1:2:end)=false;
            r=run(f);
            testCase.verifyEqual(r.position,f.truth(:,1:2),AbsTol=1e-9);
            testCase.verifyTrue(all(ismember(r.diagnostics.mode,[1,2])));
        end
        function movingGnssRecoversHeadingWithoutAHeadingMeasurement(testCase)
            f=fixture(15,"gnss");f.cfg.initialState(7)=.2;r=run(f);
            testCase.verifyLessThan(abs(r.heading(end)),.001);
            testCase.verifyGreaterThan(r.diagnostics.gnssCourseUpdates,0);
            testCase.verifyGreaterThan(nnz(r.diagnostics.headingMode==1),0);
        end
        function stationaryGnssDoesNotInventHeadingObservability(testCase)
            f=fixture(4,"gnss");f.data.highRate.longitudinalSpeed(:)=0;
            f.data.gnss.position(:)=0;f.cfg.initialState(:)=0;f.cfg.initialState(7)=.2;
            r=run(f);
            testCase.verifyEqual(r.heading,.2*ones(size(r.time)),AbsTol=1e-12);
            testCase.verifyEqual(r.diagnostics.gnssCourseUpdates,0);
            testCase.verifyFalse(r.diagnostics.allTheoremHypothesesVerified);
        end
        function constantTurnUsesPastMotionTransport(testCase)
            f=turnFixture(4);r=run(f);
            testCase.verifyEqual(r.pose,f.truth,AbsTol=1e-7);
        end
        function curvedGnssWindowReconstructsEndpointHeading(testCase)
            f=turnFixture(15);f.data=rmfield(f.data,'lidar');f.cfg.initialState(7)=.2;r=run(f);
            testCase.verifyLessThan(abs(r.headingUnwrapped(end)-f.truth(end,3)),.001);
            testCase.verifyLessThan(abs(atan2(sin(r.diagnostics.gnssDerivedHeading(end)-f.truth(end,3)), ...
                cos(r.diagnostics.gnssDerivedHeading(end)-f.truth(end,3)))),1e-10);
        end
        function invalidGnssDoesNotEraseLidarHeading(testCase)
            f=fixture(4,"lidar");f.cfg.initialState(7)=.1;r=run(f);
            testCase.verifyLessThan(abs(r.heading(end)),1e-7);
            testCase.verifyEqual(r.diagnostics.headingMode,2*ones(size(r.time)));
        end
        function bothMissingPredictAndThenRecover(testCase)
            f=fixture(8,"both");mask=f.data.lidar.time>=2 & f.data.lidar.time<4;
            f.data.lidar.valid(mask)=false;f.data.gnss.valid(mask)=false;
            f.data.highRate.yawRate(f.data.highRate.time>=2 & f.data.highRate.time<4)=.02;r=run(f);
            outage=r.time>=2 & r.time<4;
            testCase.verifyEqual(r.diagnostics.mode(outage),zeros(nnz(outage),1));
            testCase.verifyGreaterThan(max(abs(r.heading(outage))),.03);
            testCase.verifyLessThan(abs(r.heading(end)),1e-7);
        end
        function delayedPacketsAreRejectedExplicitly(testCase)
            f=fixture(1,"both");f.data.gnss.delay=.1;
            testCase.verifyError(@() run(f),'VehicleLocalization:FullZeroDelayRequired');
        end
        function uninformativeValidPacketIsRejected(testCase)
            f=fixture(1,"both");f.data.lidar.information(3,3,1)=0;
            testCase.verifyError(@() run(f),'VehicleLocalization:InvalidFullSource');
        end
        function missingInitialPositionNeedsExplicitInitialization(testCase)
            f=fixture(1,"none");f.cfg.initialState=[];
            testCase.verifyError(@() run(f),'VehicleLocalization:FullInitializationRequired');
        end
        function channelMatricesShareTheAdvertisedDissipation(testCase)
            cfg=fullObserverConfig;design=designFullObserverGains(cfg);
            testCase.verifyGreaterThanOrEqual(independentMargin(cfg),design.commonTranslationMargin-1e-10);
        end
        function weakGnssGainCannotClaimTheCertificate(testCase)
            cfg=fullObserverConfig;cfg.gnss.positionGain=.001;
            testCase.verifyError(@() designFullObserverGains(cfg),'VehicleLocalization:InfeasibleFullCertificate');
        end
    end
end

function f=fixture(duration,mode)
    t=(0:.01:duration).';n=numel(t);zero=zeros(n,1);m=(0:.1:duration).';
    high=struct('time',t,'longitudinalSpeed',8+zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngleRate',zero);
    gnss=struct('time',m,'position',[8*m,zeros(size(m))],'information',repmat(1e6*eye(2),1,1,numel(m)), ...
        'valid',true(size(m)),'delay',0);
    lidar=struct('time',m,'pose',[8*m,zeros(numel(m),2)],'information',repmat(1e6*eye(3),1,1,numel(m)), ...
        'valid',true(size(m)),'delay',0);
    data=struct('highRate',high);
    if ismember(mode,["gnss","both"]),data.gnss=gnss;end
    if ismember(mode,["lidar","both"]),data.lidar=lidar;end
    cfg=fullObserverConfig;cfg.timing="historical_transport";
    cfg.bias.enabled=false;cfg.initialState=[0;8;0;0;0;0;0];
    f=struct('data',data,'lateral',lateral,'cfg',cfg,'truth',[8*t,zero,zero]);
end

function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end

function s=trimSource(s,count)
    s.time=s.time(1:count);s.valid=s.valid(1:count);s.position=s.position(1:count,:);
    s.information=s.information(:,:,1:count);
end

function f=turnFixture(duration)
    f=fixture(duration,"both");t=f.data.highRate.time;m=f.data.lidar.time;r=.2;v=8;
    f.data.highRate.yawRate(:)=r;f.data.highRate.lateralAcceleration(:)=v*r;
    f.truth=[v/r*sin(r*t),v/r*(1-cos(r*t)),r*t];
    f.data.lidar.pose=[v/r*sin(r*m),v/r*(1-cos(r*m)),r*m];
    f.data.gnss.position=f.data.lidar.pose(:,1:2);f.cfg.initialState=[0;v;0;0;0;v*r;0];
end

function smallest=independentMargin(cfg)
    smallest=Inf;g=cfg.gains;J=[0,-1;1,0];
    for q=linspace(-cfg.maximumTrackAngleRate,cfg.maximumTrackAngleRate,21)
        for angle=linspace(0,pi,9)
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            for w=[cfg.gnss.positionGain*cfg.gnss.minimumPositionWeight,g(1)*cfg.lidar.minimumPoseWeight]
                W=R*diag([w,2*w])*R.';
                A=[-W,eye(2),zeros(2);zeros(2),-g(2)*eye(2),eye(2); ...
                    zeros(2),q^2*eye(2),-g(3)*eye(2)+2*q*J];
                smallest=min(smallest,min(eig(-(A+A.'))));
            end
        end
    end
end
