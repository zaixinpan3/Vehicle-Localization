function cfg=mncavFullObserverConfig(options)
% MnCAV intrinsic prediction with direct linear upstream yaw-rate feedback.
% Body-rate translation has polynomial drift, including at zero speed.
% This factory returns parameters only. Solve gains immediately before use
% with [design,cfg]=synthesizeMncavInvariantDesign(cfg).
    arguments
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite,mustBePositive}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-common-pose-joint-implicit-v4",'maximumBodyYawRate',.4,'initialState',[]);
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    receiver=jsondecode(fileread(fullfile(here,'mncavGnssHeading.json')));
    cfg.gnss=struct('positionStdM',receiver.positionErrorScaleM,'outputPoint', ...
        jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json'))));
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'lidarOperatorMismatch',.001);
    z=struct('theta',options.Theta,'lambda',.05,'margin',1e-7,'solver',"sedumi");
    assert(all(ismember(fieldnames(options.DesignOverride),fieldnames(z))), ...
        'VehicleLocalization:UnknownDesignOption','Unknown global observer design option.');
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
end
