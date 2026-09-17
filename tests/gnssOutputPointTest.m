classdef gnssOutputPointTest < matlab.unittest.TestCase
% gnssOutputPointTest Reference-point geometry and uncertainty behavior.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function rotatedPointUsesForwardLeftSign(testCase)
            alignment=struct('bodyOffset',[2;.5],'bodyCovariance',zeros(2),'headingStdRad',0);
            [p,I]=correctGnssOutputPoint([9.5,22],eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[10,20],AbsTol=1e-12);
            testCase.verifyEqual(I,eye(2),AbsTol=1e-12);
        end
        function calibrationAndHeadingUncertaintyReduceInformation(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([.01,.04]),'headingStdRad',.1);
            [p,I]=correctGnssOutputPoint([0,2],100*eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[0,0],AbsTol=1e-12);
            testCase.verifyEqual(I,diag([1/.09,1/.02]),AbsTol=1e-11);
            testCase.verifyLessThan(max(eig(I)),100);
        end
        function invalidCalibrationRejected(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([-.01,.04]),'headingStdRad',.1);
            testCase.verifyError(@()correctGnssOutputPoint([0,2],eye(2),0,alignment), ...
                'VehicleLocalization:InvalidGnssAlignment');
        end
        function turningReceiverPointMatchesCoincidentInput(testCase)
            f=fixture();ideal=run(f);f=offsetInput(f);aligned=run(f);
            testCase.verifyEqual(aligned.z,ideal.z,AbsTol=1e-11);
            testCase.verifyEqual(aligned.diagnostics.gnssPositionAtObserverPoint, ...
                ideal.diagnostics.gnssPositionAtObserverPoint,AbsTol=1e-12);
        end
        function gnssOnlyRetainsPointCorrection(testCase)
            f=fixture();f.data=rmfield(f.data,'lidar');ideal=run(f);f=offsetInput(f);aligned=run(f);
            testCase.verifyEqual(aligned.z,ideal.z,AbsTol=1e-11);
            testCase.verifyEqual(aligned.diagnostics.mode,ones(61,1));
        end
        function absentGnssIgnoresOutputPoint(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');ideal=run(f);
            f.cfg.gnss.outputPoint.bodyOffset=[2;.5];aligned=run(f);
            testCase.verifyEqual(aligned.z,ideal.z,AbsTol=0);
            testCase.verifyTrue(all(isnan(aligned.diagnostics.gnssPositionAtObserverPoint),'all'));
        end
        function legacyConfigurationKeepsCoincidentBehavior(testCase)
            f=fixture();current=run(f);f.cfg.gnss=rmfield(f.cfg.gnss,'outputPoint');legacy=run(f);
            testCase.verifyEqual(current.z,legacy.z,AbsTol=0);
        end
        function estimatedAttitudeControlsCorrection(testCase)
            f=offsetInput(fixture());f.cfg.initialState(7)=.1;f.data=rmfield(f.data,'lidar');r=run(f);
            testCase.verifyEqual(r.diagnostics.gnssPositionAtObserverPoint(1,:), ...
                f.data.gnss.position(1,:)-([cos(.1),-sin(.1);sin(.1),cos(.1)]*[2;.5]).',AbsTol=1e-12);
        end
        function independentPointConfigurationHasCommonPositionBandwidth(testCase)
            cfg=mncavFullObserverConfig();design=designFullObserverGains(cfg);
            testCase.verifyEqual(cfg.gnss.positionGain,cfg.gains(1),AbsTol=0);
            testCase.verifyFalse(cfg.gnss.outputPoint.evaluationDriveUsed);
            testCase.verifyGreaterThan(design.commonTranslationMargin,0);
        end
        function matchedGainLimitsDriftWithGnssAlone(testCase)
            f=biasedStraightFixture();slow=run(f);f.cfg.gnss.positionGain=4;fast=run(f);
            testCase.verifyEqual(slow.position(end,1)-8*f.data.highRate.time(end),.2,AbsTol=1e-6);
            testCase.verifyEqual(fast.position(end,1)-8*f.data.highRate.time(end),.05,AbsTol=1e-6);
        end
    end
end

function f=biasedStraightFixture()
    f=fixture();t=(0:.1:30).';n=numel(t);z=zeros(n,1);
    f.data.highRate=struct('time',t,'longitudinalSpeed',8.2+z, ...
        'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z);
    f.lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
    f.data=rmfield(f.data,'lidar');
    f.data.gnss=struct('time',t,'position',[8*t,z],'valid',true(n,1), ...
        'information',repmat(1e12*eye(2),1,1,n),'delay',0);
    f.cfg.initialState=[0;8.2;0;0;0;0;0];
end

function f=fixture()
    t=(0:.1:6).';n=numel(t);z=zeros(n,1);yaw=.2*t;v=8;r=.2;
    position=[v/r*sin(yaw),v/r*(1-cos(yaw))];
    h=struct('time',t,'longitudinalSpeed',v+z,'longitudinalAcceleration',z,'lateralAcceleration',v*r+z,'yawRate',r+z);
    g=struct('time',t,'position',position,'valid',true(n,1),'information',repmat(100*eye(2),1,1,n),'delay',0);
    l=struct('time',t,'pose',[position,yaw],'valid',true(n,1),'information',repmat(100*eye(3),1,1,n),'delay',0);
    lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
    cfg=fullObserverConfig;cfg.bias.enabled=false;cfg.initialState=[0;v;0;0;0;v*r;0];
    f=struct('data',struct('highRate',h,'gnss',g,'lidar',l),'cfg',cfg,'lateral',lateral);
end

function f=offsetInput(f)
    yaw=.2*f.data.highRate.time;
    f.data.gnss.position=f.data.gnss.position+[2*cos(yaw)-.5*sin(yaw),2*sin(yaw)+.5*cos(yaw)];
    f.cfg.gnss.outputPoint.bodyOffset=[2;.5];
end

function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end
