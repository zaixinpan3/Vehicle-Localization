function cloud=buildMississippiViewConditionedMap(outputFile,options)
% buildMississippiViewConditionedMap Build the conditional map from offline data.
% Original fine observations and the original map are preserved. The saved
% product contains map observation statistics, never online fine labels.
    arguments
        outputFile (1,1) string=""
        options.ObservationIndices (1,:) double=[]
        options.Config (1,1) struct=landmarkViewMapConfig()
        options.BaseMapFile (1,1) string=""
        options.ObservationFile (1,1) string=""
    end
    setupVehicleLocalization();cfg=featureMapBuildConfig();
    if strlength(outputFile)==0,outputFile=cfg.probabilityCloudPath;end
    if strlength(options.BaseMapFile)==0,options.BaseMapFile=cfg.baseProbabilityCloudPath;end
    if strlength(options.ObservationFile)==0,options.ObservationFile=cfg.featureObservationPath;end
    assert(outputFile~=options.BaseMapFile && outputFile~=options.ObservationFile, ...
        'VehicleLocalization:MapOutputCollision','Preserve the original map and its fine observations.');
    base=load(options.BaseMapFile,'cloud');observed=load(options.ObservationFile,'featureData');
    indices=options.ObservationIndices;
    if isempty(indices),indices=1:numel(observed.featureData.frameIndices);end
    cloud=buildViewConditionedLandmarkMap(base.cloud,observed.featureData,options.Config,indices);
    cloud.mapConstruction.baseMapFile=options.BaseMapFile;
    cloud.mapConstruction.observationFile=options.ObservationFile;
    folder=fileparts(outputFile);if strlength(folder)>0 && ~isfolder(folder),mkdir(folder);end
    save(outputFile,'cloud','options','-v7.3');
end
