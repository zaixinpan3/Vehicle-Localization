function tests=invariantEkfBaselineTest
    tests=functiontests(localfunctions);
end
function testStationaryAndWithdrawnMotion(testCase)
    root=setupVehicleLocalization();addpath(fullfile(root,'scripts','baselines'));cfg=mncavSmallOffsetObserverConfig();cfg.initialState=zeros(8,1);
    t=(0:.1:3).';n=numel(t);z=zeros(n,1);h=struct('time',t,'longitudinalSpeed',z,'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z);
    l=struct('time',t,'lateralVelocity',z,'yawRate',z,'sideSlipAngleRate',z,'velocityValid',false(n,1));
    d=struct('highRate',h,'gnss',struct('position',zeros(n,2),'information',repmat(100*eye(2),1,1,n),'valid',true(n,1)));
    d.gnss.valid(12:20)=false;e=runInvariantEkfBaseline(d,l,cfg);
    verifyTrue(testCase,all(isfinite(e.z),'all'));verifyLessThan(testCase,max(abs(e.z),[],'all'),.02);
end
function testMaskedVelocityKeepsAccelerationUpdate(testCase)
    root=setupVehicleLocalization();addpath(fullfile(root,'scripts','baselines'));
    cfg=mncavFullObserverConfig();cfg.initialState=zeros(8,1);
    t=[0;.1];z=zeros(size(t));
    h=struct('time',t,'longitudinalSpeed',nan(size(t)),'bodyVelocityValid',false(size(t)), ...
        'longitudinalAcceleration',1+z,'lateralAcceleration',z);
    l=struct('velocityValid',true(size(t)));
    e=runInvariantEkfBaseline(struct('highRate',h),l,cfg);
    verifyTrue(testCase,all(isfinite(e.z),'all'));
    verifyGreaterThan(testCase,e.z(1,3),.9);
end
function testExternalPoseProviderWithoutMotion(testCase)
    root=setupVehicleLocalization();addpath(fullfile(root,'scripts','baselines'));cfg=mncavSmallOffsetObserverConfig();cfg.initialState=[.3;0;0;-.2;0;0;.05;0];
    t=(0:.1:2).';n=numel(t);z=zeros(n,1);h=struct('time',t,'longitudinalSpeed',z,'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z);
    l=struct('time',t,'lateralVelocity',z,'yawRate',z,'sideSlipAngleRate',z,'velocityValid',false(n,1));
    e=runInvariantEkfBaseline(struct('highRate',h),l,cfg,PoseProvider=@(~,~)struct('pose',zeros(3,1),'basis',eye(3),'covariance',1e-4*eye(3)));
    verifyTrue(testCase,all(isfinite(e.z),'all'));verifyLessThan(testCase,norm(e.pose(end,:)),.01);
end
