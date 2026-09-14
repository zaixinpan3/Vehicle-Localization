function cfg = improvedObserverConfig(mode,profile)
% improvedObserverConfig Configure one continuous measurement mode.
% MODE is "gnss" (position only) or "lidar" (continuous delayed full pose).
% The LiDAR reference certificate has a deliberately narrow declared sector;
% runtime diagnostics report coefficient excursions without clamping inputs.
% PROFILE="lowPeaking" selects a constant GNSS gain preset validated on the
% synthetic sedan; it reduces startup peaks at the cost of slower settling.
% PROFILE="tracking" selects a LiDAR acceleration-gain preset with the same
% delay and bounds, reducing settled errors with moderately larger peaks.
    arguments
        mode (1,1) string {mustBeMember(mode,["gnss","lidar"])} = "lidar"
        profile (1,1) string {mustBeMember(profile,["reference","lowPeaking","tracking"])} = "reference"
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    reference=jsondecode(fileread(fullfile(root,'config','continuousObserverCertificate.json')));
    selected=reference.(mode);
    cfg.mode=mode;
    cfg.operating=struct('maximumSpeed',16,'maximumAcceleration',5, ...
        'maximumTrackAngleRate',selected.maximumCourseRate);
    cfg.observer=struct('theta',selected.theta, ...
        'scalingExponents',[1;2;3;1;2;3;1], ...
        'initialState',[],'initialHeading',0,'yawGain',.5);
    if profile=="lowPeaking"
        assert(mode=="gnss",'VehicleLocalization:UnsupportedObserverProfile', ...
            'The lowPeaking profile is defined only for GNSS.');
        cfg.observer.theta=8;
        cfg.observer.yawGain=.1;
        cfg.observer.gnssChainGain=[6;8;3];
    end
    if profile=="tracking"
        assert(mode=="lidar",'VehicleLocalization:UnsupportedObserverProfile', ...
            'The tracking profile is defined only for LiDAR.');
        cfg.observer.lidarGainProfile="tracking";
    end
    cfg.measurement=struct('fixedLidarDelay',.15,'maximumIntegrationStep',.005);
    cfg.lidar=struct('poseScales',[1;1;1],'gainInformationScale',5, ...
        'minimumPoseWeight',reference.lidar.minimumInformationWeight);
    cfg.gnss=struct('minimumSpeed',1,'headingErrorLimit',pi/3);
    cfg.synthesis=struct('solver',"sedumi",'rate',.05,'tolerance',1e-8, ...
        'outputFolder',"",'saveFileName',"continuousObserverDesign.mat");
    cfg.simulation=struct('sampleTime',.01,'finalTime',20, ...
        'speed',8,'courseRate',.001,'initialHeading',.2, ...
        'positionNoiseAmplitude',.01,'headingNoiseAmplitude',deg2rad(.1), ...
        'initialError',[.1;0;0;-.1;0;0;deg2rad(2)]);
end
