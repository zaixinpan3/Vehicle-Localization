function model=loadPillarPoleModel(modelFile)
% loadPillarPoleModel: Cache the small, versioned numeric inference artifact.
% Explicit configuration chooses the artifact; inference never opens training data.
    persistent cachedFile cachedModel
    if isempty(cachedFile)||~strcmp(cachedFile,modelFile)
        model=jsondecode(fileread(modelFile));
        model.leaf=logical(model.leaf);cachedModel=model;cachedFile=modelFile;
    else
        model=cachedModel;
    end
end
