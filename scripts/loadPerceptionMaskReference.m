function reference = loadPerceptionMaskReference(dataset, frameIndex)
% loadPerceptionMaskReference: Read immutable point-index regression evidence.
% The fixture records current outputs at the revision named in its JSON file;
% loading it never executes another implementation or reads raw point clouds.
    persistent saved
    if isempty(saved)
        root=fileparts(fileparts(mfilename('fullpath')));
        saved=jsondecode(fileread(fullfile(root,'tests','reference','perceptionMasks.json')));
    end
    dataset=lower(string(dataset));
    if any(dataset==["mississippi","missisipi"]), dataset="missisipi"; end
    match=lower(string({saved.entries.dataset}))==dataset & [saved.entries.frameIndex]==frameIndex;
    assert(nnz(match)==1,'perception:MissingReference','No unique recorded frame reference.');
    item=saved.entries(match);
    masks=struct();
    for name=string(fieldnames(item.channels)).'
        masks.(name)=false(item.numPoints,1);
        masks.(name)(item.channels.(name))=true;
    end
    reference=struct('featureMasks',masks,'revision',string(saved.revision),'candidates',item.candidates);
end
