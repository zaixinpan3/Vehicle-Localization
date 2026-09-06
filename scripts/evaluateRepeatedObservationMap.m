function report = evaluateRepeatedObservationMap(dataRoot, outputFolder, frameIndices)
% evaluateRepeatedObservationMap: Record construction and field consistency.
% Uses registered Mississippi observations, not independent localization truth.
% No random sampling, accuracy claim, or calibrated probability claim is made.
    root=fileparts(fileparts(mfilename('fullpath')));
    if nargin<1 || strlength(string(dataRoot))==0, dataRoot=fullfile(root,'data'); end
    if nargin<2 || strlength(string(outputFolder))==0, outputFolder=fullfile(root,'output','repeated_observation_map'); end
    if nargin<3, frameIndices=260:289; end
    cfg=featureMapBuildConfig(); cfg.logEnabled=false;
    cfg.featureNames=["curb","roadMarking","pole","trafficSign"];
    poses=readFramePoseTable(fullfile(dataRoot,cfg.poseMatchCsvPath),frameIndices);
    perception=perceptionConfig(); perception.executionMode="legacyFull";
    perception.featureNames=cfg.featureNames;
    timer=tic;
    observations=collectFeatureObservations(fullfile(dataRoot,cfg.pointCloudMatPath),frameIndices,poses,perception,cfg);
    extractionSeconds=toc(timer); timer=tic;
    map=buildSlidingWindowMap(observations,cfg); buildSeconds=toc(timer);
    cloud=temporalMapToProbabilityCloud(map);
    assert(cloud.components.numComponents>0,'Recorded construction produced no published structure.');
    rows=unique(round(linspace(1,cloud.components.numComponents,min(100,cloud.components.numComponents))));
    queries=cloud.components.mean(rows,:);
    [scores,details]=queryTemporalStabilityGmmMap(map,queries);
    layers=map.canonicalMap.layers; summaries=cell(numel(layers),1);
    intensity=zeros(size(scores)); totalIterations=0;
    for j=1:numel(layers)
        layer=layers(j); keep=find(cloud.components.semanticName==layer.classLabel);
        for k=keep(:).'
            C=cloud.components.covariance(:,:,k); delta=queries-cloud.components.mean(k,:);
            intensity(:,j)=intensity(:,j)+cloud.classTotalMass(j)*cloud.components.classMixtureWeight(k)/ ...
                (2*pi*sqrt(det(C)))*exp(-0.5*sum((delta/C).*delta,2));
        end
        fits=[layer.tiles.fit]; converged=[fits.converged];
        selections=[layer.tiles.selection];
        selectionConverged=vertcat(selections.candidateConverged);
        selectionIterations=vertcat(selections.candidateIterations);
        iterations=sum([fits.iterationCount]); totalIterations=totalIterations+iterations;
        summaries{j}=struct('classLabel',layer.classLabel,'sourcePoints',layer.pointCount, ...
            'representatives',size(layer.representativePoints,1),'tiles',numel(layer.tiles), ...
            'candidates',numel(layer.components),'published',nnz(layer.componentPublished), ...
            'referenceArea',layer.referenceMass,'backgroundReferenceArea',layer.backgroundReferenceMass, ...
            'publishedMass',layer.totalMass,'convergedFits',nnz(converged), ...
            'iterationLimitedFits',nnz(~converged),'finalFitIterations',iterations, ...
            'selectionIterationLimitedFits',nnz(~selectionConverged & selectionIterations>0));
    end
    reconstructed=intensity./(intensity+cloud.clutterIntensity.');
    intensityError=max(abs(intensity-details.intensity),[],'all');
    scoreError=max(abs(reconstructed(details.valid)-scores(details.valid)),[],'all');
    assert(all(abs(intensity-details.intensity)<=details.intensityErrorBound+1e-10,'all'), ...
        'Field export exceeds the declared query intensity approximation error.');
    assert(all(abs(reconstructed(details.valid)-scores(details.valid))<=details.scoreErrorBound(details.valid)+1e-10), ...
        'Field export exceeds the declared query score approximation error.');
    report=struct('schemaVersion',2,'scenario',"MississippiRegisteredConstructionConsistency", ...
        'frameIndices',frameIndices,'matlabVersion',version,'randomSampling',false, ...
        'observationBlockSize',cfg.temporalMap.observationBlockSize, ...
        'extractionSeconds',extractionSeconds,'buildSeconds',buildSeconds, ...
        'layers',vertcat(summaries{:}),'totalPublishedMass',cloud.totalMass, ...
        'publishedComponents',cloud.components.numComponents,'finalFitIterations',totalIterations, ...
        'queryLocations',size(queries,1),'validClassQueries',nnz(details.valid), ...
        'maximumIntensityReconstructionError',intensityError,'maximumScoreReconstructionError',scoreError, ...
        'limitation',"Construction and algebraic consistency; not independent localization accuracy or probability calibration");
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    save(fullfile(outputFolder,'recorded_map.mat'),'map','cfg','report','queries','scores','details','-v7.3');
    fid=fopen(fullfile(outputFolder,'recorded_validation.json'),'w');
    assert(fid>=0,'Cannot write validation report.'); cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
