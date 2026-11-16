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
            [aligned,~,~]=localizationSupport.synchronizeLocalizationInputs(data,l,c);
            testCase.verifyTrue(all(aligned.gnssHeading.valid));
            testCase.verifyFalse(isfield(aligned,'gnss'));
            testCase.verifyEqual(aligned.gnssHeading.heading,data.gnssHeading.heading,AbsTol=1e-12);
            testCase.verifyFalse(isfield(aligned.gnssHeading,'waitSeconds'));
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
        function headingCorrectsRateWithoutPositionOrLidar(testCase)
            [data,c,l]=fixture();c.initialState(7)=.2;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            data=rmfield(data,'gnssHeading');b=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyLessThan(abs(a.headingUnwrapped(end)),abs(b.headingUnwrapped(end)));
            testCase.verifyGreaterThan(max(abs(a.yawRate-b.yawRate)),1e-4);
            testCase.verifyTrue(all(a.diagnostics.independentHeadingValid));
        end
        function wrappedInnovationUsesShortArc(testCase)
            [data,c,l]=fixture();c.initialState(7)=pi-.01;
            data.gnssHeading.heading(:)=-pi+.01;
            a=runFullLocalizationObserver(data,c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.independentHeadingInnovation(1),.02,AbsTol=1e-12);
            testCase.verifyLessThan(max(abs(diff(a.headingUnwrapped))),.02);
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
            [aligned,~,~]=localizationSupport.synchronizeLocalizationInputs(data,l,c);
            testCase.verifyEqual(aligned.gnss.position,data.gnss.position,AbsTol=0);
            testCase.verifyFalse(isfield(aligned.gnss,'information'));
        end
        function headingAvailabilityChangesDesignAndCertificate(testCase)
            c=freshObserverTestConfig();d=localizationSupport.runtime(c);
            testCase.verifySize(d.LG,[8,3]);
            testCase.verifyGreaterThan(abs(d.LG(8,3)),0);
            lower=c;lower.invariantObserver.gnssHeadingAvailable=false;
            a=localizationSupport.fusionBlocks(c.invariantDesign,c);
            b=localizationSupport.fusionBlocks(c.invariantDesign,lower);
            delta=zeros(8);delta(7,7)=-2*c.invariantDesign.kappaG;
            testCase.verifyEqual(a{1}-b{1},delta,AbsTol=1e-11);
            testCase.verifyError(@()localizationSupport.runtime(lower),'VehicleLocalization:StaleInvariantDesign');
        end
    end
end
function h=packet(t,heading)
    h=struct('time',t,'heading',heading,'valid',true(size(t)),'delay',0);
end
function [data,c,l]=fixture()
    c=freshObserverTestConfig();t=(0:.05:1).';z=zeros(size(t));
    c.initialState=[0;12;0;0;0;0;0;0];
    h=struct('time',t,'longitudinalSpeed',12+z,'longitudinalAcceleration',z,'lateralAcceleration',z, ...
        'bodyVelocityValid',false(size(t)),'bodyAccelerationValid',false(size(t)));
    data=struct('highRate',h,'gnssHeading',packet(t,z));
    l=struct('time',t,'lateralVelocity',z);
end
