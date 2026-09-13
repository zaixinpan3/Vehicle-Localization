function designMncavReplayObserver(parameterFile,outputFile,dynamicsFactor,mode)
% designMncavReplayObserver Synthesize gains for an explicitly nominal vehicle.
% YALMIP and the configured SDP solver must already be on the MATLAB path.
% The input file records sourced stock geometry and unmeasured dynamic priors.
    arguments
        parameterFile (1,1) string
        outputFile (1,1) string
        dynamicsFactor (1,1) double {mustBePositive} = 1
        mode (1,1) string {mustBeMember(mode,["gnss","lidar"])} = "lidar"
    end
    parameters=jsondecode(fileread(parameterFile));
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
