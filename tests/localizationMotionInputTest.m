classdef (SharedTestFixtures={freshObserverGainsFixture}) localizationMotionInputTest < matlab.unittest.TestCase
% localizationMotionInputTest LiDAR translation cannot rewrite motion inputs.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function poseFeedbackDoesNotRewriteMeasuredMotion(testCase)
            f=fixture();a=run(f);before=f.data.highRate;
            f.data.lidar.pose(:,1:2)=[8*f.time,.2*f.time];b=run(f);
            testCase.verifyEqual(f.data.highRate,before);
            testCase.verifyEqual(b.lateral,f.lateral);
            testCase.verifyGreaterThan(norm(b.internalState(end,:)-a.internalState(end,:)),.01);
        end
        function removedLearnerHasNoConfigurationOrDiagnostics(testCase)
            f=fixture();r=run(f);
            testCase.verifyFalse(isfield(f.cfg,'bias'));
            testCase.verifyFalse(any(isfield(r.diagnostics, ...
                {'lidarLongitudinalVelocityBias','lidarVelocityBias','lidarBiasUpdates','motionBiasSource'})));
        end
    end
end

function f=fixture()
    t=(0:.1:6).';n=numel(t);z=zeros(n,1);
    h=struct('time',t,'longitudinalSpeed',8.4+z,'longitudinalAcceleration',z, ...
        'lateralAcceleration',z,'yawRate',z);
    l=struct('time',t,'pose',[8.4*t,-.25*t,z],'valid',true(n,1), ...
        'information',repmat(1e6*eye(3),1,1,n),'delay',0);
    cfg=freshObserverTestConfig();cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);cfg.initialState=[0;8.4;0;0;-.25;0;0];
    cfg=freshObserverTestConfig(cfg);
    f=struct('time',t,'data',struct('highRate',h,'lidar',l),'cfg',cfg, ...
        'lateral',struct('time',t,'lateralVelocity',-.25+z,'sideSlipAngle',atan2(-.25,8.4)+z,'sideSlipAngleRate',z));
end
function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end
