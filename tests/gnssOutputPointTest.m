classdef (SharedTestFixtures={freshObserverGainsFixture}) gnssOutputPointTest < matlab.unittest.TestCase
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
            [p,I]=localizationSupport.correctGnssOutputPoint([9.5,22],eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[10,20],AbsTol=1e-12);
            testCase.verifyEqual(I,eye(2),AbsTol=1e-12);
        end
        function calibrationAndHeadingUncertaintyReduceInformation(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([.01,.04]),'headingStdRad',.1);
            [p,I]=localizationSupport.correctGnssOutputPoint([0,2],100*eye(2),pi/2,alignment);
            testCase.verifyEqual(p,[0,0],AbsTol=1e-12);
            testCase.verifyEqual(I,diag([1/.09,1/.02]),AbsTol=1e-11);
            testCase.verifyLessThan(max(eig(I)),100);
        end
        function invalidCalibrationRejected(testCase)
            alignment=struct('bodyOffset',[2;0],'bodyCovariance',diag([-.01,.04]),'headingStdRad',.1);
            testCase.verifyError(@()localizationSupport.correctGnssOutputPoint([0,2],eye(2),0,alignment), ...
                'VehicleLocalization:InvalidGnssAlignment');
        end
        function absentGnssIgnoresPointUncertainty(testCase)
            f=fixture();f.data=rmfield(f.data,'gnss');ideal=run(f);
            f.cfg.gnss.outputPoint.bodyCovariance=eye(2);
            f.cfg.gnss.outputPoint.headingStdRad=1;aligned=run(f);
            testCase.verifyEqual(aligned.z,ideal.z,AbsTol=0);
            testCase.verifyTrue(all(isnan(aligned.diagnostics.gnssPositionAtObserverPoint),'all'));
        end
        function receiverHeadingTransportsTheAntenna(testCase)
            f=offsetInput(fixture());f.cfg.initialState(7)=.1;r=run(f);
            expected=zeros(numel(r.time),2);
            for k=1:numel(r.time)
                yaw=f.data.gnss.heading(k);
                R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
                expected(k,:)=f.data.gnss.position(k,:)-(R*f.cfg.gnss.outputPoint.bodyOffset(:)).';
            end
            testCase.verifyEqual(r.diagnostics.gnssPositionAtObserverPoint,expected,AbsTol=1e-12);
        end
        function predictedHeadingTransportsAntennaDuringGap(testCase)
            f=offsetInput(fixture());f.cfg.initialState(7)=.1;f.data=rmfield(f.data,'lidar');
            f.data.gnss.heading(2)=NaN;r=run(f);
            yaw=r.diagnostics.intrinsicPrediction(2,7);
            testCase.verifyEqual(r.diagnostics.gnssPositionAtObserverPoint(2,:), ...
                localizationSupport.correctGnssOutputPoint(f.data.gnss.position(2,:),[],yaw,f.cfg.gnss.outputPoint),AbsTol=1e-12);
            testCase.verifyTrue(isnan(r.diagnostics.independentHeadingInnovation(2)));
        end
        function independentPointConfigurationUsesTheSolvedGnssGain(testCase)
            cfg=freshObserverTestConfig();design=localizationSupport.designFullObserverGains(cfg);
            temporary=cfg.invariantDesign;E=eye(8);
            testCase.verifyEqual(design.linearOutputGain(:,1:3), ...
                design.toPhysical*(temporary.P\E(:,[1,4,7])),AbsTol=1e-12);
            testCase.verifyEqual(temporary.inputs,localizationSupport.invariantInputs(cfg));
            testCase.verifyFalse(cfg.gnss.outputPoint.evaluationDriveUsed);
            testCase.verifyTrue(design.nominalFusionLmiVerified);
        end
    end
end

function f=fixture()
    t=(0:.1:6).';n=numel(t);z=zeros(n,1);yaw=.2*t;v=8;r=.2;
    position=[v/r*sin(yaw),v/r*(1-cos(yaw))];
    h=struct('time',t,'longitudinalSpeed',v+z,'longitudinalAcceleration',z,'lateralAcceleration',v*r+z,'yawRate',r+z);
    g=struct('time',t,'position',position,'heading',yaw,'valid',true(n,1),'information',repmat(100*eye(2),1,1,n),'delay',0);
    l=struct('time',t,'pose',[position,yaw],'valid',true(n,1),'information',repmat(100*eye(3),1,1,n),'delay',0);
    lateral=struct('time',t,'lateralVelocity',z,'yawRate',z,'sideSlipAngleRate',z);
    cfg=freshObserverTestConfig();cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);cfg.initialState=[0;v;0;0;0;v*r;0];
    cfg=freshObserverTestConfig(cfg);
    f=struct('data',struct('highRate',h,'gnss',g,'lidar',l),'cfg',cfg,'lateral',lateral);
end

function f=offsetInput(f)
    yaw=.2*f.data.highRate.time;
    current=freshObserverTestConfig();offset=current.gnss.outputPoint.bodyOffset(:);
    f.data.gnss.position=f.data.gnss.position+[offset(1)*cos(yaw)-offset(2)*sin(yaw), ...
        offset(1)*sin(yaw)+offset(2)*cos(yaw)];
    f.cfg.gnss.outputPoint.bodyOffset=offset;
    f.cfg=freshObserverTestConfig(f.cfg);
end

function r=run(f)
    r=syntheticObserver.run(f.data,f.cfg,LateralInputs=f.lateral);
end
