classdef (SharedTestFixtures={freshObserverGainsFixture}) independentHeadingTest < matlab.unittest.TestCase
% Independent absolute heading in the linear output channel.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function interpolationCrossesBranchCut(testCase)
            c=mncavFullObserverConfig();h=packet([0;.1],[pi-.02;-pi+.02]);
            p=localizationSupport.headingMeasurements(struct('gnssHeading',h),[0;.05;.1],c);
            testCase.verifyTrue(all(p.valid));
            testCase.verifyEqual(abs(p.heading(2)),pi,AbsTol=1e-12);
            testCase.verifyEqual(p.waitSeconds(2),.05,AbsTol=1e-12);
        end
        function singleFixAppliesOnlyAtAcquisition(testCase)
            p=localizationSupport.headingMeasurements(struct('gnssHeading',packet(.1,.3)),[0;.1;.2],mncavFullObserverConfig());
            testCase.verifyEqual(p.valid,[false;true;false]);
            testCase.verifyEqual(p.heading(2),.3,AbsTol=1e-15);
        end
        function rejectsInvalidEndpointsAndExtrapolation(testCase)
            c=mncavFullObserverConfig();h=packet([0;.1],[0;0]);h.valid(2)=false;
            p=localizationSupport.headingMeasurements(struct('gnssHeading',h),[-.1;0;.05;.1;.2],c);
            testCase.verifyEqual(p.valid,[false;true;false;false;false]);
            h.valid(:)=true;h.time=[0;1];
            p=localizationSupport.headingMeasurements(struct('gnssHeading',h),[0;.5;1],c);
            testCase.verifyEqual(p.valid,[true;false;true]);
        end
        function synchronizationPreservesIndependentAvailability(testCase)
            [data,c,l]=fixture();t=data.highRate.time;n=numel(t);
            data.gnssHeading.heading(:)=pi-.02;
            data.gnssHeading.heading(2:2:end)=-pi+.02;
            data.lidar=struct('time',t,'pose',zeros(n,3),'information',repmat(eye(3),1,1,n), ...
                'valid',false(n,1),'delay',0);
            [aligned,~,~]=syntheticObserver.align(data,l,c);
            testCase.verifyTrue(all(aligned.gnssHeading.valid));
            testCase.verifyTrue(all(aligned.gnss.valid));
            testCase.verifyEqual(aligned.gnssHeading.heading,data.gnssHeading.heading,AbsTol=1e-12);
            testCase.verifyFalse(isfield(aligned.gnssHeading,'waitSeconds'));
        end
        function packedPoseMatchesTwoNativeStreams(testCase)
            [data,c,l]=fixture();data.gnssHeading.heading(:)=.1;
            same=data;same.gnss.heading=data.gnssHeading.heading;same=rmfield(same,'gnssHeading');
            a=runFullLocalizationObserver(data,c,LateralInputs=l);b=runFullLocalizationObserver(same,c,LateralInputs=l);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
        end
        function missingHeadingMatchesInvalidHeading(testCase)
            [data,c,l]=fixture();data=rmfield(data,'gnssHeading');
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            data.gnssHeading=packet(data.highRate.time,nan(size(data.highRate.time)));
            data.gnssHeading.valid(:)=false;
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.headingUnwrapped,b.headingUnwrapped,AbsTol=0);
            testCase.verifyEqual(a.position,b.position,AbsTol=0);
        end
        function packedHeadingGapKeepsPositionAndRate(testCase)
            [data,c,l]=fixture();t=data.highRate.time;
            c.initialState(1)=1;c.gnss.outputPoint.bodyOffset=[0;0];
            l.yawRate=.2*ones(size(t));
            data.gnss.heading=data.gnssHeading.heading;data=rmfield(data,'gnssHeading');
            data.gnss.heading(2:end-1)=NaN;
            [aligned,l]=observerInternal.alignFrameClock(data,l,c,t);
            a=runFullLocalizationObserver(aligned,c,LateralInputs=l);
            gap=2:numel(t)-1;
            testCase.verifyTrue(all(a.diagnostics.gnssPositionValid(gap)));
            testCase.verifyTrue(all(isnan(a.diagnostics.independentHeadingInnovation(gap))));
            testCase.verifyEqual(find(a.diagnostics.gnssPoseValid),[1;numel(t)]);
            testCase.verifyTrue(all(a.diagnostics.upstreamYawRateValid(gap)));
            testCase.verifyGreaterThan(a.headingUnwrapped(gap(end)),.05);
            testCase.verifyLessThan(abs(a.position(gap(end),1)-12*t(gap(end))),.1);
        end
        function headingCorrectsHeadingWhileRateIsHeld(testCase)
            [data,c,l]=fixture();c.initialState(7)=.2;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            data=rmfield(data,'gnssHeading');b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyLessThan(abs(a.headingUnwrapped(end)),abs(b.headingUnwrapped(end)));
            testCase.verifyEqual(a.yawRate,b.yawRate,AbsTol=0);
            testCase.verifyTrue(all(a.diagnostics.independentHeadingValid));
        end
        function wrappedInnovationUsesShortArc(testCase)
            [data,c,l]=fixture();c.initialState(7)=pi-.01;
            data.gnssHeading.heading(:)=-pi+.01;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.independentHeadingInnovation(1),.02,AbsTol=1e-12);
            testCase.verifyLessThan(max(abs(diff(a.headingUnwrapped))),.02);
        end
        function reversingTurnRecoversYawWithOffsetAndSideslip(testCase)
            [data,c,l,truth]=reverseTurn();
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            actual=a.diagnostics.recoveredHeading;
            testCase.verifyTrue(isnan(actual(1)));
            testCase.verifyEqual(angle(exp(1i*(actual(2:end)-truth(2:end)))),zeros(numel(truth)-1,1),AbsTol=1e-12);
            testCase.verifyLessThan(abs(angle(exp(1i*(a.heading(end)-truth(end))))),.05);
            testCase.verifyFalse(any(a.diagnostics.independentHeadingValid));
        end
        function delayedFixUsesOnlyDeliveredPositions(testCase)
            [data,c,l,truth]=reverseTurn();t=data.highRate.time;
            data.gnss.sourceRightTime=t+.01;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyTrue(all(isnan(a.diagnostics.recoveredHeading(1:2))));
            testCase.verifyEqual(a.diagnostics.recoveredHeading(3),angle(exp(1i*truth(3))),AbsTol=1e-12);
            data.gnss.position(3:end,1)=data.gnss.position(3:end,1)+.4;
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(b.diagnostics.recoveredHeading(1:3),a.diagnostics.recoveredHeading(1:3));
        end
        function missingRateBreaksTheDisplacementBaseline(testCase)
            [data,c,l]=reverseTurn();l.yawRateValid=true(size(l.time));l.yawRateValid(5)=false;
            data.gnssHeading=packet(l.time,zeros(size(l.time)));data.gnssHeading.valid(:)=false;
            data.gnssHeading.valid(8)=true;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(isfinite(a.diagnostics.recoveredHeading(4:9)),[true;false;false;true;false;true]);
            testCase.verifyTrue(a.diagnostics.independentHeadingValid(8));
            data.highRate.longitudinalSpeed(:)=0;l.lateralVelocity(:)=0;l.yawRate(:)=0;
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyTrue(all(isnan(b.diagnostics.recoveredHeading)));
        end
        function reliabilityMetadataCannotChangeTheState(testCase)
            [data,c,l]=fixture();t=data.highRate.time;n=numel(t);
            c.initialState([1,7])=[.1;.1];
            data.gnss=struct('time',t,'position',[12*t,zeros(n,1)],'valid',true(n,1),'delay',0);
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            data.gnss.information=repmat(diag([1e-12,1e12]),1,1,n);
            data.gnssHeading.information=nan(n,1);
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            data.gnss.information=nan(2,2,n);data.gnssHeading.information=1e12*ones(n,1);
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
        end
        function synchronizesPositionWithoutInformation(testCase)
            [data,c,l]=fixture();t=data.highRate.time;n=numel(t);
            data.gnss=struct('time',t,'position',[12*t,zeros(n,1)],'valid',true(n,1),'delay',0);
            data.lidar=struct('time',t,'pose',zeros(n,3),'information',repmat(eye(3),1,1,n), ...
                'valid',false(n,1),'delay',0);
            [aligned,~,~]=syntheticObserver.align(data,l,c);
            testCase.verifyEqual(aligned.gnss.position,data.gnss.position,AbsTol=0);
            testCase.verifyFalse(isfield(aligned.gnss,'information'));
        end
        function intermittentGnssUsesOneDesign(testCase)
            [data,c,l]=fixture();d=localizationSupport.runtime(c);
            testCase.verifySize(d.linearOutputGain,[8,4]);
            g=c.invariantObserver.theta^5*c.invariantDesign.cr/c.invariantDesign.P(8,8);
            testCase.verifyEqual(d.linearOutputGain(8,:),[0,0,0,g],RelTol=1e-13);
            testCase.verifyEqual(d.linearOutputGain(1:7,4),zeros(7,1));
            t=data.highRate.time;frames=(0:numel(t)-1).';
            pos=mod(frames,4)<2;hdg=mod(frames,2)==0;
            data.gnss=struct('time',t,'position',[12*t,zeros(size(t))],'valid',pos,'delay',0);
            data.gnssHeading.valid=hdg;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.gnssPositionValid,pos);
            testCase.verifyEqual(a.diagnostics.independentHeadingValid,hdg);
            testCase.verifyEqual(a.diagnostics.gnssPoseValid,pos & hdg);
            testCase.verifyEqual(a.observer.continuousDesign,d);
            data.gnss.position(~pos,:)=NaN;
            data.gnssHeading.heading(~hdg)=NaN;
            b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(b.internalState,a.internalState,AbsTol=0);
        end
    end
end
function [data,c,l,truth]=reverseTurn()
    [data,c,l]=fixture(4);data=rmfield(data,'gnssHeading');t=l.time;
    offset=[1.2;-.4];yaw0=3;rate=.4;u=-8;vy=.6;
    truth=yaw0+rate*t;rot=exp(1i*truth);
    path=(rot-exp(1i*yaw0))*(u+1i*vy)/(1i*rate);
    antenna=path+rot*complex(offset(1),offset(2));
    data.gnss.position=[real(antenna),imag(antenna)];
    c.gnss.outputPoint.bodyOffset=offset;c.gnss.positionStdM=.04;
    velocity=exp(1i*yaw0)*(u+1i*vy);
    c.initialState=[0;real(velocity);-rate*imag(velocity);0;imag(velocity);rate*real(velocity);yaw0+pi;rate];
    data.highRate.longitudinalSpeed(:)=u;data.highRate.bodyVelocityValid(:)=true;
    l.lateralVelocity(:)=vy;l.yawRate=rate*ones(size(t));
end
function h=packet(t,heading)
    h=struct('time',t,'heading',heading,'valid',true(size(t)),'delay',0);
end
function [data,c,l]=fixture(duration)
    if nargin==0,duration=1;end
    c=freshObserverTestConfig();t=(0:.05:duration).';z=zeros(size(t));
    c.initialState=[0;12;0;0;0;0;0;0];
    h=struct('time',t,'longitudinalSpeed',12+z,'longitudinalAcceleration',z,'lateralAcceleration',z, ...
        'bodyVelocityValid',false(size(t)),'bodyAccelerationValid',false(size(t)));
    data=struct('highRate',h,'gnssHeading',packet(t,z), ...
        'gnss',struct('time',t,'position',[12*t,z],'valid',true(size(t)),'delay',0));
    l=struct('time',t,'lateralVelocity',z);
end
