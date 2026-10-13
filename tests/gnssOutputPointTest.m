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
            [p,I]=fullObserverSupport.correctGnssOutputPoint([9.5,22],eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[10,20],AbsTol=1e-12);
            testCase.verifyEqual(I,eye(2),AbsTol=1e-12);
        end
        function calibrationAndHeadingUncertaintyReduceInformation(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([.01,.04]),'headingStdRad',.1);
            [p,I]=fullObserverSupport.correctGnssOutputPoint([0,2],100*eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[0,0],AbsTol=1e-12);
            testCase.verifyEqual(I,diag([1/.09,1/.02]),AbsTol=1e-11);
            testCase.verifyLessThan(max(eig(I)),100);
        end
        function invalidCalibrationRejected(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([-.01,.04]),'headingStdRad',.1);
            testCase.verifyError(@()fullObserverSupport.correctGnssOutputPoint([0,2],eye(2),0,alignment), ...
                'VehicleLocalization:InvalidGnssAlignment');
        end
        function absentGnssIgnoresOutputPoint(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');ideal=run(f);
            f.cfg.gnss.outputPoint.bodyOffset=[1;.25];aligned=run(f);
            testCase.verifyEqual(aligned.z,ideal.z,AbsTol=0);
            testCase.verifyTrue(all(isnan(aligned.diagnostics.gnssPositionAtObserverPoint),'all'));
        end
        function runtimeUsesItsIntrinsicHeadingForPointCorrection(testCase)
            f=offsetInput(fixture());f.cfg.initialState(7)=.1;r=run(f);
            expected=zeros(numel(r.time),2);
            for k=1:numel(r.time)
                yaw=r.diagnostics.intrinsicPrediction(k,7);
                R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
                expected(k,:)=f.data.gnss.position(k,:)-(R*f.cfg.gnss.outputPoint.bodyOffset(:)).';
            end
            testCase.verifyEqual(r.diagnostics.gnssPositionAtObserverPoint,expected,AbsTol=1e-12);
        end
        function estimatedAttitudeControlsCorrection(testCase)
            f=offsetInput(fixture());f.cfg.initialState(7)=.1;f.data=rmfield(f.data,'lidar');r=run(f);
            testCase.verifyEqual(r.diagnostics.gnssPositionAtObserverPoint(1,:), ...
                f.data.gnss.position(1,:)-([cos(.1),-sin(.1);sin(.1),cos(.1)]*f.cfg.gnss.outputPoint.bodyOffset(:)).',AbsTol=1e-12);
        end
        function independentPointConfigurationUsesTheSolvedGnssGain(testCase)
            cfg=mncavFullObserverConfig();design=fullObserverSupport.designFullObserverGains(cfg);
            stored=jsondecode(fileread(fullfile(fileparts(fileparts(mfilename('fullpath'))),'config','mncavInvariantDesign.json')));
            Cp=zeros(2,8);Cp(1,1)=1;Cp(2,4)=1;
            testCase.verifyEqual(design.LG,design.toPhysical*stored.design.kappaG*(stored.design.P\Cp.'),RelTol=1e-12);
            testCase.verifyEqual(stored.inputs.gnssBodyOffset(:),cfg.gnss.outputPoint.bodyOffset(:),AbsTol=0);
            testCase.verifyFalse(cfg.gnss.outputPoint.evaluationDriveUsed);
            testCase.verifyTrue(design.surrogateContinuousLmiVerified);
            testCase.verifyFalse(design.fullContinuousStateCertified);
        end
    end
end

function f=fixture()
    t=(0:.1:6).';n=numel(t);z=zeros(n,1);yaw=.2*t;v=8;r=.2;
    position=[v/r*sin(yaw),v/r*(1-cos(yaw))];
    h=struct('time',t,'longitudinalSpeed',v+z,'longitudinalAcceleration',z,'lateralAcceleration',v*r+z,'yawRate',r+z);
    g=struct('time',t,'position',position,'valid',true(n,1),'information',repmat(100*eye(2),1,1,n),'delay',0);
    l=struct('time',t,'pose',[position,yaw],'valid',true(n,1),'information',repmat(100*eye(3),1,1,n),'delay',0);
    lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngle',z,'sideSlipAngleRate',z);
    cfg=mncavFullObserverConfig;cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);cfg.initialState=[0;v;0;0;0;v*r;0];
    f=struct('data',struct('highRate',h,'gnss',g,'lidar',l),'cfg',cfg,'lateral',lateral);
end

function f=offsetInput(f)
    yaw=.2*f.data.highRate.time;
    current=mncavFullObserverConfig();offset=current.gnss.outputPoint.bodyOffset(:);
    f.data.gnss.position=f.data.gnss.position+[offset(1)*cos(yaw)-offset(2)*sin(yaw), ...
        offset(1)*sin(yaw)+offset(2)*cos(yaw)];
    f.cfg.gnss.outputPoint.bodyOffset=offset;
end

function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end
