function designMncavReplayObserver(parameterFile,outputFile,dynamicsFactor,mode)
% designMncavReplayObserver Synthesize gains for an explicitly nominal vehicle.
% YALMIP and the configured SDP solver must already be on the MATLAB path.
% Optional input file supplies sensor calibration only; vehicle and steering
% always come from the canonical MnCAV configuration. dynamicsFactor is an
% explicit sensitivity override relative to that current nominal vehicle.
    arguments
        parameterFile (1,1) string = ""
        outputFile (1,1) string = "output/mncav_replay_design.mat"
        dynamicsFactor (1,1) double {mustBePositive} = 1
        mode (1,1) string {mustBeMember(mode,["gnss","lidar"])} = "lidar"
    end
    parameters=mncavReplayConfig(parameterFile);
    lateralCfg=lateralObserverConfig();
    lateralCfg.vehicle=parameters.vehicle;
    for name=["yawInertia","frontCorneringStiffness","rearCorneringStiffness"]
        lateralCfg.vehicle.(name)=dynamicsFactor*lateralCfg.vehicle.(name);
    end
    observerCfg=improvedObserverConfig(mode);
    observerCfg.operating.maximumSpeed=16;
    % No missing-information override: D2D must supply its actual matrix.
    timer=tic;
    lateralDesign=designLateralObserverGains(lateralCfg);
    observerDesign=designImprovedObserverGains(observerCfg);
    synthesisSeconds=toc(timer);
    save(outputFile,'parameters','dynamicsFactor','lateralCfg','observerCfg', ...
        'lateralDesign','observerDesign','synthesisSeconds');
    fprintf('Nominal factor %.3g: lateral certificate %d, global certificate %d, %.2f s.\n', ...
        dynamicsFactor,lateralDesign.certified,observerDesign.certified,synthesisSeconds);
end
